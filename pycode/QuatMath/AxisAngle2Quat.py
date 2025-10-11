"""
AxisAngle2Quat.py

Converted from MATLAB file: AxisAngle2Quat.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def AxisAngle2Quat(axis, ang_rad):
    """
    Converted from MATLAB function
    """

    #AxisAngle2Quat Converts an axis-angle orientation to a rotation quaternion
    #
    #   q = AxisAngle2Quat(axis, ang_rad)
    #   angle in radians
    #
    #   Converts an axis angle orientation pair to a quaternion rotation
    #   NOTE: This is cordinate frame rotation opposite of vector rotation
    #

    if np.linalg.norm(axis > 0 )
    axis = axis/np.linalg.normaxis

    q0 = np.cosnp.array([-ang_rad/2-1])
    q1 = -axis1*np.sinnp.array([-ang_rad/2-1])
    q2 = -axis2*np.sinnp.array([-ang_rad/2-1])
    q3 = -axis3*np.sinnp.array([-ang_rad/2-1])
    q = q0, q1, q2, q3]).T
    q = QuatNormalnp.array([ q 

