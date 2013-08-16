#!/usr/bin/env python
"""
Perform the same map as forwardcal.py but by reading from the ROS
topic "points" provided by forward.py.  Cf. forwardcal.py for
documentation about calibration, etc.


SCL; 16 August 2013
"""

import roslib; roslib.load_manifest("trackem_ros")
import rospy
from trackem_ros.msg import MTPoints, MTCalPoint, MTCalPoints
import sys
import numpy as np

from forwardcal import normalize, pixel2m


class CalForward(rospy.Publisher):
    def __init__(self, intopic, fc, cc, kc, H):
        self.tpub = rospy.Publisher(intopic, MTCalPoints)
        self.fc = fc
        self.cc = cc
        self.kc = kc
        self.H = H

    def __call__(self, d):
        points = [pixel2m((rawpoint.u, rawpoint.v),
                          f=self.fc, pp=self.cc, kc=self.kc, H=self.H) for rawpoint in d.points]
        self.tpub.publish(MTCalPoints(cam_id=d.cam_id,
                                      framenumber=d.framenumber, timestamp=d.timestamp,
                                      points=[MTCalPoint(p[0], p[1]) for p in points]))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: calecho.py FILE"
        exit(1)

    cal_data = np.load(sys.argv[1])

    rospy.init_node("trackem", anonymous=True)
    fsub = rospy.Subscriber("/trackem/points", MTPoints,
                            CalForward("trackem_calpoints", cal_data["fc"], cal_data["cc"], cal_data["kc"], cal_data["H"]))
    rospy.spin()
