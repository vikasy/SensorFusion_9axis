"""
QuatRotVec.py

Converted from MATLAB file: QuatRotVec.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def QuatRotVec(q, v):
    """
    Converted from MATLAB function
    """

    #QuatRotVec Rotates a vector .Tin opposite direction.T by a quaternion
    # It basically rotates the cordinate frame and computes representation
    # of v in the rotated frame.
    #
    #   w = QuatRot(q, v) = q.T*V*q
    #
    #   where q = np.cos[ang/2, axis*np.sinang/2]
    #   and V = 0; v])
    #   Rotates the 3D column vector v in opposite direction by ang
    #   about .Taxis.T
    #
    #   q and v should be column vectors

    ifnp.array([( np.shape(v2 not = 1 ) or np.shape(q2 not = 1))
    error.Tq and v should be column vectors.T

    if np.linalg.norm(q == 0 )
    error.Tq is a null quaternion!.T

    q = QuatNormalq
    qrot = QuatProductQuatProduct(QuatConjugate(q, np.array([0; v])), q)
    w = qrot[2-1:4]

