#!/usr/bin/env python
"""
Present trackem points in PointCloud2 messages, assuming zero z-coordinates.

SCL; 14 August 2013
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

    def __call__(self, d):
        points = np.array([(p.x, p.y, 0.) for p in d.points])
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


if __name__ == "__main__":
    if "-h" in sys.argv or "--help" in sys.argv:
        print "Usage: cloudecho.py [FRAME]"
        exit(1)

    if len(sys.argv) >= 2:
        frame_id = sys.argv[1]
    else:
        frame_id = "/odom"

    rospy.init_node("trackem", anonymous=True)
    cfsub = rospy.Subscriber("/trackem/calpoints", MTCalPoints, CloudForward(frame_id))
    rospy.spin()
