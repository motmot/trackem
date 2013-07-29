from __future__ import division

import pkg_resources

import wx
from wx import xrc
import motmot.FastImage.FastImage as FastImage
import numpy
import motmot.wxvalidatedtext.wxvalidatedtext as wxvt
import os, sys, socket, signal, time, math, threading, Queue
import motmot.flytrax.flytrax as flytrax
import motmot.imops.imops as imops
import numpy as np

# ROS stuff ------------------------------
import roslib
have_ROS = True
roslib.load_manifest('flymad')

from flymad.msg import Raw2dPositions
from geometry_msgs.msg import Pose2D
import rospy
import rospy.core
# -----------------------------------------

RESFILE = pkg_resources.resource_filename(__name__,"trackem.xrc") # trigger extraction
RES = xrc.EmptyXmlResource()
RES.LoadFromString(open(RESFILE).read())

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

        self.num_points = SharedValue1(7)
        self.analysis_radius = SharedValue1(25)
        self.luminance_threshold = SharedValue1(45)
        self.cur_imagePts = SharedValue1(None)
        self.enabled = threading.Event()
        self.light_on_dark = threading.Event()
        self.wxmessage_queue = Queue.Queue() # for passing (error) messages from process_frame
        self.status_label = xrc.XRCCTRL(self.frame,"STATUS")
        self.msg = None
        self.cam_id = None

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

    def get_frame(self):
        """return wxPython frame widget"""
        return self.frame

    def get_buffer_allocator(self,cam_id):
        return BufferAllocator()

    def get_plugin_name(self):
        return 'trackem'

    def process_frame(self,cam_id,buf,buf_offset,timestamp,framenumber):
        """do work on each frame

        This function gets called on every single frame capture. It is
        called within the realtime thread, NOT the wxPython
        application mainloop's thread. Therefore, be extremely careful
        (use threading locks) when sharing data with the rest of the
        class.

        """
        if self.pixel_format=='YUV422':
            buf = imops.yuv422_to_mono8( numpy.asarray(buf) ) # convert
        
        buf = FastImage.asfastimage(buf)

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

                # clear image around point
                clearxmin = max(0,x-analysis_radius)
                clearxmax = min(buf.size.w-1,x+analysis_radius)
                clearymin = max(0,y-analysis_radius)
                clearymax = min(buf.size.h-1,y+analysis_radius)
                buf_view[clearymin:clearymax,clearxmin:clearxmax]=clearval

                # save values
                point_list.append( (offset_x+x,offset_y+y) )

            # send data over ROS
            if self.num_points.get():
                msg = Raw2dPositions()

                msg.header.stamp.secs = int(np.floor(timestamp))
                msg.header.stamp.nsecs = int((timestamp%1.0)*1e9)
                msg.header.frame_id = "pixels"

                msg.framenumber = framenumber

                for point_tuple in point_list:
                    pose = Pose2D()
                    pose.x, pose.y = point_tuple[:2]
                    pose.theta = np.nan
                    msg.points.append( pose )
                self.pub.publish(msg)
                    
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
