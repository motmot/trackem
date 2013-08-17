#!/usr/bin/env python
"""
Perform the same map as forwardcal.py but by reading from the ROS
topic "points" provided by forward.py.  Cf. forwardcal.py for
documentation about calibration, etc.


SCL; 17 August 2013
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
        self.fresh_points = False
        self.rate = rospy.Rate(100)  # Hz

    def __call__(self, d):
        self.data = d
        self.fresh_points = True

    def run(self):
        while not rospy.is_shutdown():
            if self.fresh_points:
                points = [pixel2m((rawpoint.u, rawpoint.v),
                                  f=self.fc, pp=self.cc, kc=self.kc, H=self.H) for rawpoint in self.data.points]
                self.tpub.publish(MTCalPoints(cam_id=self.data.cam_id,
                                              framenumber=self.data.framenumber,
                                              timestamp=self.data.timestamp,
                                              points=[MTCalPoint(p[0], p[1]) for p in points]))
                self.fresh_points = False
            self.rate.sleep()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: calecho.py FILE"
        exit(1)

    cal_data = np.load(sys.argv[1])

    rospy.init_node("trackem", anonymous=True)
    calf = CalForward("trackem_calpoints", cal_data["fc"], cal_data["cc"], cal_data["kc"], cal_data["H"])
    fsub = rospy.Subscriber("/trackem/points", MTPoints, calf)
    calf.run()
