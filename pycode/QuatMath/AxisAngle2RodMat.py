"""
AxisAngle2RodMat.py

Converted from MATLAB file: AxisAngle2RodMat.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def AxisAngle2RodMat(axis, ang_rad):
    """
    Converted from MATLAB function
    """

    #AxisAngle2RodMat Converts an axis-angle orientation to a rotation matrix
    #
    #   R = AxisAngle2RodMat(axis, ang_rad)
    #   Angle is in radians
    #
    #   Converts and axis-angle orientation to a Rodregues rotation matrix
    #   axis = n
    #   angle = a
    #   v is before rotation and w is after rotation by angle a about axis n,
    #   such that w = Rv, then as per Rodrigues formula:
    #   w = v + np.sin(a)nxv + np.array([1 - np.cos(a-1]))nx(nxv)
    #   i.e. w = I + [np.sin(a)N + np.array([1 - np.cos(a-1]))N^2] v = Rv
    #

    if np.linalg.norm(axis > 0 )
    axis = axis/np.linalg.normaxis

    N = CPMataxis
    R = np.eye3 + np.sin(ang_rad*N + np.array([1 - np.cos(ang_rad-1])))*N

