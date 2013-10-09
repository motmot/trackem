from __future__ import division

import pkg_resources

import wx
from wx import xrc
import motmot.FastImage.FastImage as FastImage
import motmot.realtime_image_analysis.realtime_image_analysis as realtime_image_analysis
import numpy
import motmot.wxvalidatedtext.wxvalidatedtext as wxvt
import os, sys, socket, signal, time, math, threading, Queue
import motmot.imops.imops as imops
import numpy as np

# ROS stuff ------------------------------
import roslib
have_ROS = True
roslib.load_manifest('flymad')

from flymad.msg import Raw2dPositions
from geometry_msgs.msg import Pose2D
import sensor_msgs.msg
import std_msgs.msg
import rospy
import rospy.core
# -----------------------------------------

RESFILE = pkg_resources.resource_filename(__name__,"trackem.xrc") # trigger extraction
RES = xrc.EmptyXmlResource()
RES.LoadFromString(open(RESFILE).read())

def ros_ensure_valid_name(name):
    return name.replace('-','_')

class SharedValue1(object):
    def __init__(self,initial_value):
        self._val = initial_value
        self.lock = threading.Lock()
    def get(self):
        self.lock.acquire()
        try:
            val = self._val
        finally:
            self.lock.release()
        return val
    def set(self,new_value):
        self.lock.acquire()
        try:
            self._val = new_value
        finally:
            self.lock.release()


def bind_checkbox_to_threading_event( ctrl, threading_event ):
    class Handler(object):
        def __init__(self,ctrl,threading_event):
            self.ctrl = ctrl
            self.threading_event = threading_event
        def OnCheckbox(self,wx_event):
            if self.ctrl.IsChecked():
                self.threading_event.set()
            else:
                self.threading_event.clear()
    handler = Handler(ctrl,threading_event)
    ctrl.Bind( wx.EVT_CHECKBOX, handler.OnCheckbox )
    # call once to initialize properly
    handler.OnCheckbox(None)

def bind_button_to_threading_event( ctrl, threading_event ):
    class Handler(object):
        def __init__(self,ctrl,threading_event):
            self.ctrl = ctrl
            self.threading_event = threading_event
        def OnButton(self,wx_event):
            self.threading_event.set()
    handler = Handler(ctrl,threading_event)
    ctrl.Bind( wx.EVT_BUTTON, handler.OnButton )

def bind_textbox_to_sharedvalue( ctrl, shared_value, validator_func=None, conversion_func=None ):
    class Handler(object):
        def __init__(self,ctrl,shared_value,conversion_func):
            self.ctrl = ctrl
            self.shared_value = shared_value
            self.conversion_func = conversion_func
        def OnValidSet(self,wx_event):
            valid_value = self.conversion_func(self.ctrl.GetValue())
            self.shared_value.set( valid_value )
    handler = Handler(ctrl,shared_value,conversion_func)
    ctrl.SetValue( unicode(shared_value.get()) )

    if conversion_func is not None and validator_func is None:
        # create new validator func
        def my_validator_func(test_value):
            success = False
            try:
                conversion_func(test_value)
                success = True
            except ValueError:
                pass
            return success
        validator_func = my_validator_func
    if validator_func is not None:
        if conversion_func is None:
            raise ValueError("if validator_func is set, conversion_func must also be set")
        wxvt.Validator( ctrl, ctrl.GetId(), handler.OnValidSet, validator_func )
    else:
        raise NotImplementedError('not implemented')
        # XXX this codepath hasn't been tested
        #ctrl.Bind( wx.EVT_TEXTBOX, handler.OnSet )
                              
class BufferAllocator(object):
    def __call__(self, w, h):
        return FastImage.FastImage8u(FastImage.Size(w,h))


class TrackemClass(object):
    def __init__(self,wx_parent):
        self.wx_parent = wx_parent
        self.frame = RES.LoadFrame(self.wx_parent,"TRACKEM_FRAME") # make frame main panel

        self.num_points = SharedValue1(10)
        self.analysis_radius = SharedValue1(10)
        self.luminance_threshold = SharedValue1(45)
        self.max_area = SharedValue1(100)

        self.mask_center_x = SharedValue1(257)
        self.mask_center_y = SharedValue1(245)
        self.mask_radius = SharedValue1(221)

        self.cur_imagePts = SharedValue1(None)
        self.enabled = threading.Event()
        self.light_on_dark = threading.Event()
        self.wxmessage_queue = Queue.Queue() # for passing (error) messages from process_frame
        self.status_label = xrc.XRCCTRL(self.frame,"STATUS")
        self.msg = None
        self.cam_id = None

        self.view_mask_mode = threading.Event()

        self._setupGUI()

        #####
        if have_ROS:
            rospy.init_node('fview', # common name across all plugins so multiple calls to init_node() don't fail
                            anonymous=True, # allow multiple instances to run
                            disable_signals=True, # let WX intercept them
                            )
            self.pub = rospy.Publisher( '/flymad/raw_2d_positions',
                                        Raw2dPositions,
                                        tcp_nodelay=True )

    def _setupGUI(self):

        bind_checkbox_to_threading_event( xrc.XRCCTRL(self.frame,"ENABLED"), self.enabled )
        bind_checkbox_to_threading_event( xrc.XRCCTRL(self.frame,"LIGHT_ON_DARK"),  self.light_on_dark )
        bind_textbox_to_sharedvalue( xrc.XRCCTRL(self.frame,"NUM_POINTS"), self.num_points,
                                     conversion_func=int )
        bind_textbox_to_sharedvalue( xrc.XRCCTRL(self.frame,"ANALYSIS_RADIUS"), self.analysis_radius,
                                     conversion_func=int )
        bind_textbox_to_sharedvalue( xrc.XRCCTRL(self.frame,"LUMINANCE_THRESHOLD"), self.luminance_threshold,
                                     conversion_func=int )
        bind_textbox_to_sharedvalue( xrc.XRCCTRL(self.frame,"MAX_AREA"), self.max_area,
                                     conversion_func=int )
        view_mask_mode_widget = xrc.XRCCTRL(self.frame,"VIEW_MASK_CHECKBOX")
        wx.EVT_CHECKBOX(view_mask_mode_widget,
                        view_mask_mode_widget.GetId(),
                        self.OnViewMaskMode)

        for name in ["MASK_X_CENTER","MASK_Y_CENTER","MASK_RADIUS"]:
            ctrl = xrc.XRCCTRL(self.frame,name)
            wx.EVT_COMMAND_SCROLL(ctrl, ctrl.GetId(), self.update_mask )


    def OnViewMaskMode(self,event):
        widget = event.GetEventObject()
        if widget.IsChecked():
            self.view_mask_mode.set()
        else:
            self.view_mask_mode.clear()

    def get_frame(self):
        """return wxPython frame widget"""
        return self.frame

    def get_buffer_allocator(self,cam_id):
        return BufferAllocator()

    def get_plugin_name(self):
        return 'trackem'

    def get_mask(self):
        return self.mask

    def update_mask(self, _=None):
        # copy values from GUI into our SharedValue containers
        ctrl = xrc.XRCCTRL(self.frame,"MASK_X_CENTER")
        self.mask_center_x.set(ctrl.GetValue())

        ctrl = xrc.XRCCTRL(self.frame,"MASK_Y_CENTER")
        self.mask_center_y.set(ctrl.GetValue())

        ctrl = xrc.XRCCTRL(self.frame,"MASK_RADIUS")
        self.mask_radius.set(ctrl.GetValue())

        # update the actual mask image

        cx = self.mask_center_x.get()
        cy = self.mask_center_y.get()
        r =  self.mask_radius.get()

        w,h = self.image_size
        x = np.arange( w )
        y = np.arange( h )
        X,Y = np.meshgrid(x,y)
        
        dist = np.sqrt((cx-X)**2 + (cy-Y)**2)
        assert dist.shape == (h,w)
        self.mask = dist >= r
        
    def process_frame(self,cam_id,buf,buf_offset,timestamp,framenumber):
        """do work on each frame

        This function gets called on every single frame capture. It is
        called within the realtime thread, NOT the wxPython
        application mainloop's thread. Therefore, be extremely careful
        (use threading locks) when sharing data with the rest of the
        class.

        """
        assert self.pixel_format=='MONO8'
        
        now = time.time()
        if (now-self.last_image_publish) > 30.0: # every 30 seconds, publish image
            msg = sensor_msgs.msg.Image()
            msg.header.seq=framenumber
            msg.header.stamp=rospy.Time.from_sec(now) # XXX TODO: once camera trigger is ROS node, get accurate timestamp                                      
            msg.header.frame_id = "0"

            npbuf = np.array(buf)
            (height,width) = npbuf.shape

            msg.height = height
            msg.width = width
            msg.encoding = 'mono8'
            msg.step = width
            msg.data = npbuf.tostring() # let numpy convert to string           

            self.image_pub.publish(msg)
            self.last_image_publish = now

        if (now-self.last_mean_luminance_pub) > 0.5: # every 0.5 seconds, publish mean luminance
            mean_lum = np.mean(buf)
            self.mean_luminance_pub.publish(mean_lum)
            self.last_mean_luminance_pub = now

        buf = FastImage.asfastimage(buf)

        ros_list = []
        point_list = []
        draw_linesegs = [] # [ (x0,y0,x1,y1) ]
        if self.enabled.isSet():

            light_on_dark = self.light_on_dark.isSet()
            offset_x, offset_y = buf_offset
            if 1:

                # work on a copy of the image so the original is displayed unaltered
                copyview = self.copybuf.roi( offset_x, offset_y, buf.size )
                buf.get_8u_copy_put(copyview,buf.size)
                buf = copyview
            buf_view = numpy.asarray(buf) # get numpy view of data

            # step 1. extract points
            analysis_radius = self.analysis_radius.get()
            luminance_threshold = self.luminance_threshold.get()
            if light_on_dark:
                clearval = 0
            else:
                clearval = 255

            # outside mask, set values to clearval
            mask = self.get_mask()
            buf_view[ mask ] = clearval

            for pt_num in range( self.num_points.get() ):
                if light_on_dark:
                    # find brightest point
                    max_val, x,y = buf.max_index(buf.size)
                    if max_val < luminance_threshold:
                        break
                else:
                    # find darkest point
                    min_val, x,y = buf.min_index(buf.size)
                    if min_val > (255-luminance_threshold):
                        break

                # calculate region around found point
                clearxmin = max(0,x-analysis_radius)
                clearxmax = min(buf.size.w-1,x+analysis_radius)
                clearymin = max(0,y-analysis_radius)
                clearymax = min(buf.size.h-1,y+analysis_radius)

                # extract a view of this region
                this_region = buf_view[clearymin:clearymax,clearxmin:clearxmax]
                if light_on_dark:
                    binary_region = this_region > (max_val / 1.1)
                else:
                    binary_region = this_region < (min_val * 1.1)
                num_pixels_classified = np.sum(binary_region.ravel())
                #print '%d, (%d, %d)'%(num_pixels_classified,x,y)

                if num_pixels_classified > self.max_area.get():
                    # we don't want this point
                    pass
                else:
                    # compute luminance center of mass
                    if not light_on_dark:
                        # make a light-on-dark image
                        this_region2 = 255-this_region
                    else:
                        this_region2 = this_region
                    fibuf = FastImage.asfastimage(this_region2)
                    try:
                        results = realtime_image_analysis.py_fit_params(fibuf)
                    except Exception as err:
                        print '%s: error extracting image data. ignoring.'%(err,)
                    else:
                        (x0, y0, area, slope, eccentricity) = results
                        theta = np.arctan( slope )

                        x1 = x0 + clearxmin
                        y1 = y0 + clearymin

                        # save values
                        ros_list.append( (offset_x+x1,offset_y+y1,theta) )
                        point_list.append( (offset_x+x1,offset_y+y1) )

                # clear the region near the detected point
                buf_view[clearymin:clearymax,clearxmin:clearxmax]=clearval

            # send data over ROS
            if self.num_points.get():
                msg = Raw2dPositions()

                msg.header.stamp.secs = int(np.floor(timestamp))
                msg.header.stamp.nsecs = int((timestamp%1.0)*1e9)
                msg.header.frame_id = "pixels"

                msg.framenumber = framenumber

                for (x,y,theta) in ros_list:
                    pose = Pose2D()
                    pose.x = x
                    pose.y = y
                    pose.theta = theta
                    msg.points.append( pose )
                self.pub.publish(msg)

        if self.view_mask_mode.isSet():

            w,h = self.image_size
            x=self.mask_center_x.get()
            y=self.mask_center_y.get()
            radius=self.mask_radius.get()

            N = 64
            theta = numpy.arange(N)*2*math.pi/N
            xdraw = x+numpy.cos(theta)*radius
            ydraw = y+numpy.sin(theta)*radius
            for i in range(N-1):
                draw_linesegs.append(
                    (xdraw[i],ydraw[i],xdraw[i+1],ydraw[i+1]))
            draw_linesegs.append(
                (xdraw[-1],ydraw[-1],xdraw[0],ydraw[0]))

                    
        return point_list, draw_linesegs

    def set_view_flip_LR( self, val ):
        pass

    def set_view_rotate_180( self, val ):
        pass

    def quit(self):
        pass

    def camera_starting_notification(self,cam_id,
                                     pixel_format=None,
                                     max_width=None,
                                     max_height=None):
        if self.cam_id is not None:
            raise NotImplementedError("only a single camera is suppported")
        if pixel_format not in ['MONO8','YUV422']:
            raise NotImplementedError("only MONO8 and YUV422 pixel format suppported")
        self.cam_id = cam_id
        self.pixel_format = pixel_format

        self.image_size = (max_width,max_height)
        self.copybuf = FastImage.FastImage8u(FastImage.Size(max_width,max_height))

        ctrl = xrc.XRCCTRL(self.frame,"MASK_X_CENTER")
        ctrl.SetRange(0,max_width-1)
        ctrl.SetValue(self.mask_center_x.get())

        ctrl = xrc.XRCCTRL(self.frame,"MASK_Y_CENTER")
        ctrl.SetRange(0,max_height-1)
        ctrl.SetValue(self.mask_center_y.get())

        ctrl = xrc.XRCCTRL(self.frame,"MASK_RADIUS")
        ctrl.SetRange(0,max(max_width,max_height))
        ctrl.SetValue(self.mask_radius.get())

        self.update_mask()
