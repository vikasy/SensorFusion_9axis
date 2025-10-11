"""
SF_Orient_Mag.py

Converted from MATLAB file: SF_Orient_Mag.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def SF_Orient_Mag(mag_in, frame):
    """
    Converted from MATLAB function
    """

    # Calculate orientation matrix based on magnetometer sensor data
    #
    #

    R = np.eye3
    mag_mag_xy = np.linalg.normnp.array([mag_in.data(1-1:2]))

    switch frame
    case .TAND.T
    if mag_mag_xy > 0 
    R(1,1) = mag_in.data2/mag_mag_xy
    R(2,2) = Rz(1,1)
    R(1,2) = mag_in.data1/mag_mag_xy
    R(2,1) = -Rz(1,2)

    #otherwise

    RotMtx = R
    q = RodMat2Qat RotMtx 
