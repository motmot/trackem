#!/usr/bin/env python
"""
Present trackem points in PointCloud2 messages, assuming zero z-coordinates.

SCL; 17 August 2013
"""

import roslib; roslib.load_manifest("trackem_ros")
import rospy
from trackem_ros.msg import MTCalPoints
from sensor_msgs.msg import PointCloud2, PointField

import sys
import struct
import numpy as np


class CloudForward(rospy.Publisher):
    def __init__(self, frame_id):
        self.ppub = rospy.Publisher("trackem_pointcloud", PointCloud2)
        self.frame_id = frame_id
        self.fresh_points = False
        self.rate = rospy.Rate(100)  # Hz

    def __call__(self, d):
        self.data = d
        self.fresh_points = True

    def run(self):
        while not rospy.is_shutdown():
            if self.fresh_points:
                points = np.array([(p.x, p.y, 0.) for p in self.data.points])
                pcm = PointCloud2()
                pcm.header.frame_id = self.frame_id
                pcm.header.stamp = rospy.Time.now()
                pcm.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                              PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                              PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
                pcm.height = 1
                pcm.width = points.shape[0]
                pcm.is_bigendian = False
                pcm.is_dense = True
                pcm.point_step = 12
                pcm.row_step = pcm.point_step*points.shape[0]
                pcm.data = struct.pack("<"+str(points.shape[0]*points.shape[1])+"f", *points.reshape(points.shape[0]*points.shape[1]))
                self.ppub.publish(pcm)
                self.fresh_points = False
            self.rate.sleep()


if __name__ == "__main__":
    if "-h" in sys.argv or "--help" in sys.argv:
        print "Usage: cloudecho.py [FRAME]"
        exit(1)

    frame_id = "/odom"
    if len(sys.argv) >= 2:
        i = 1
        while i < len(sys.argv):
            if ":=" not in sys.argv[i]:
                break
            i += 1
        if i < len(sys.argv):
            frame_id = sys.argv[i]

    rospy.init_node("trackem", anonymous=True)
    cloudf = CloudForward(frame_id)
    cfsub = rospy.Subscriber("/trackem/calpoints", MTCalPoints, cloudf)
    cloudf.run()
