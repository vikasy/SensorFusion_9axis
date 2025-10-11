"""
Quat2RodMat.py

Converted from MATLAB file: Quat2RodMat.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def Quat2RodMat(q):
    """
    Converted from MATLAB function
    """

    #Quat2RodMat Converts a quaternion orientation to a Rodriguez orientation matrix
    #
    #   R = Quat2RodMatq
    #
    #   R is actually cordination frame rotation matrix.
    #   such that w = Rv is same as w = q.T*v*q
    #
    #   Converts a quaternion orientation to a Rodriguez rotation matrix.
    #

    q = QuatNormalq

    R(1,1) = 2* q[1^2 + q2^2 -1] - 1
    R(1,2) = 2* q[2*q3 + q1*q4 -1]
    R(1,3) = 2* q[2*q4 - q1*q3 -1]
    R(2,1) = 2* q[2*q3 - q1*q4 -1]
    R(2,2) = 2* q[1^2 + q3^2 -1] - 1
    R(2,3) = 2* q[3*q4 + q1*q2 -1]
    R(3,1) = 2* q[2*q4 + q1*q3 -1]
    R(3,2) = 2* q[3*q4 - q1*q2 -1]
    R(3,3) = 2* q[1^2 + q4^2 -1] - 1

