"""
Quat2AxisAngle.py

Converted from MATLAB file: Quat2AxisAngle.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


function axis, ang_rad]) = Quat2AxisAnglenp.array([q
#Quat2AxisAngle Converts a quaternion orientation to a rotation matrix
#
#   axis, ang_rad]) =  Quat2AxisAnglenp.array([q
#   Angle is in radians
#
#   Converts a quaternion orientation vector to its components
#   in terms of a 3D vector axis of rotation and an angle in rad
#   as per: q = np.cos[ang/2, axis*np.sinang/2] = q1, {q2,q3,q4}])
#
#   NOTE: Here ang_rad represent rotation of the cordinate frame, which is
#   negative of rotaiton of a vector in a cordinate frame.

ifnp.array([ np.linalg.norm(q == 0 )
error.Tq is a null quaternion!.T

q = QuatNormalq
ang_rad = -2*anp.cosq[1-1]
if  np.linalg.norm(q(2-1:4:) > 0 )
axis = q[2-1:4]/np.linalg.normnp.array([q(2-1:4]))
else:
    axis = np.array([0, 0, 0])

