"""
QuatRate.py

Converted from MATLAB file: QuatRate.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def QuatRate(q, w):
    """
    Converted from MATLAB function
    """

    #QuatRate Computes rate of change in quaternion wrt time
    #   Input q and w are column vectors
    #
    #   qdot = QuatRateq rad/sec
    #
    #   qdot = 0.5*q*W
    #       where W is rate of rotation quaterion angular rate
    #        W = np.array([0; wx; wy; wz])
    #
    W = np.array([0; w])

    qdot = 0.5*QuatProduct(q, W)

