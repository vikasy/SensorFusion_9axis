"""
QuatRotate.py

Converted from MATLAB file: QuatRotate.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def QuatRotate(q, ang_deg):
    """
    Converted from MATLAB function
    """

    # Rotate given quaternion by provided angle
    # Angle is in degree
    #

    ang_mag = np.linalg.normang_deg

    EPSILON = 1e-6

    if ang_mag > EPSILON 
    rot_axis_unit = ang_deg/ang_mag
    rot_ang = Deg2Radang_mag
    rot_q = AxisAngle2Quat(rot_axis_unit, rot_ang)
else:
    rot_q = 1; 0.5*ang_deg])

    rot_q = QuatNormalnp.array([rot_q
    q_rotated = QuatProduct(q, rot_q)
    q_rotated = QuatNormalq_rotated
