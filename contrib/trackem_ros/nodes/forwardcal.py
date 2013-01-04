#!/usr/bin/env python
"""
Forward Motmot trackem points from local UDP socket and apply a
calibration to get "real world" coordinates.

The calibration file should be in NumPy .npz format and contain
fc -- focal length;
cc -- principal point;
kc -- radial and tangential distortion;
H -- projective transformation to recover physical coordinates.
See documentation in manifest.xml of this ROS package for details.


SCL; 6 December 2012.
"""

import roslib; roslib.load_manifest("trackem_ros")
from trackem_ros.msg import *
import rospy
import socket
import struct
import sys
import numpy as np


def normalize(x_p, f=None, pp=None, kc=None, num_it=5):
    """Normalize image coordinate.

    Equation (19) (in Section 3) of

      J. Heikkil\"{a} and O. Silv\'{e}n (1997).  A Four-step Camera
      Calibration Procedure with Implicit Image Correction.  Proceedings
      of Computer Vision and Pattern Recognition ({CVPR}).

    Also see the parameters description page of J.-Y. Bouguet's
    "Camera Calibration Toolbox for Matlab" website;
    http://vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html

    f is focal length; pp is principal point; kc is vector of radial
    and tangential distortion parameters.
    """
    if f is None:
        f = np.ones(2)
    if pp is None:
        pp = np.zeros(2)
    if kc is None:
        kc = np.zeros(5)

    x_d = np.array([(x_p[0]-pp[0])/f[0],
                    (x_p[1]-pp[1])/f[1]])

    x_n = x_d.copy()
    for i in range(num_it):
        r2 = np.sum(x_n**2)
        dx = np.array([2*kc[2]*x_n[0]*x_n[1] + kc[3]*(r2 + 2*x_n[0]**2),
                       kc[2]*(r2 + 2*x_n[1]**2) + 2*kc[3]*x_n[0]*x_n[1]])
        x_n = (x_d-dx)/(1 + kc[0]*r2 + kc[1]*r2**2 + kc[4]*r2**3)

    return x_n


def pixel2m(x_p, f, pp, kc, H):
    """Convert raw image coordinate to actual physical position."""
    x_n = np.array([0., 0, 1])
    x_n[:2] = (normalize(np.asarray(x_p), f=f, pp=pp, kc=kc)*f+pp)[:,0]
    return np.dot(H, x_n)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: forwardcal.py FILE"
        exit(1)

    cal_data = np.load(sys.argv[1])
    fc = cal_data["fc"]
    cc = cal_data["cc"]
    kc = cal_data["kc"]
    H = cal_data["H"]

    rospy.init_node("trackem")
    tpub = rospy.Publisher("~calpoints", MTCalPoints)
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
            p = [pixel2m((v[2*i], v[2*i+1]), f=fc, pp=cc, kc=kc, H=H) for i in range(len(v)/2)]
            tpub.publish(MTCalPoints(cam_id=cam_id,
                                     framenumber=framenumber, timestamp=timestamp,
                                     points=[MTCalPoint(x[0], x[1]) for x in p]))
    except KeyboardInterrupt:
        pass
    sockobj.close()
