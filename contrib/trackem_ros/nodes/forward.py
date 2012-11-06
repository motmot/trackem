#!/usr/bin/env python
"""
Forward Motmot trackem points from local UDP socket.

SCL; 14 August 2012.
"""

import roslib; roslib.load_manifest("trackem_ros")
from trackem_ros.msg import *
import rospy
import socket
import struct


if __name__ == "__main__":
    rospy.init_node("trackem")
    tpub = rospy.Publisher("~points", MTPoints)
    sockobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockobj.bind(("", 28931))
    header_dsize = struct.calcsize("<sLd")
    point_dsize = struct.calcsize("<II")
    try:
        while True:
            data = sockobj.recv(4096)
            (cam_id, framenumber, timestamp) =  struct.unpack("<sLd", data[:header_dsize])
            if len(data[header_dsize:]) % point_dsize != 0:
                raise ValueError("streamed data is corrupt; incorrect size.")
            num_points = len(data[header_dsize:])/point_dsize
            if num_points < 1:
                continue
            v = struct.unpack("<"+("I"*2*num_points), data[header_dsize:])
            tpub.publish(MTPoints(cam_id=cam_id,
                                  framenumber=framenumber, timestamp=timestamp,
                                  points=[MTPoint(v[2*i], v[2*i+1]) for i in range(len(v)/2)]))
    except KeyboardInterrupt:
        pass
    sockobj.close()
