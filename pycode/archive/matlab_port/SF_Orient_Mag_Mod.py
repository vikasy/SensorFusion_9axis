"""
SF_Orient_Mag_Mod.py

Converted from MATLAB file: SF_Orient_Mag_Mod.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def SF_Orient_Mag_Mod(mag_in, frame):
    """
    Converted from MATLAB function
    """

    # Calculate orientation matrix based on magnetometer sensor data
    #
    # In this modified algorithm, only assumption is Roll angle is 0
    # unlike other algo where both roll and np.pith are assumed to be 0.
    #

    Rx = np.eye3
    Ry = np.eye3
    Rz = np.eye3
    mag_mag_xy = np.linalg.normnp.array([mag_in.data(1-1:2]))
    mag_mag_yz_sq = np.linalg.normnp.array([mag_in.data(2-1:3]))^2

    switch frame
    case .TAND.T
    if mag_mag_xy > 0 
    Rz(1,1) = mag_in.data2/mag_mag_xy
    Rz(2,2) = Rz(1,1)
    Rz(1,2) = mag_in.data1/mag_mag_xy
    Rz(2,1) = -Rz(1,2)

    if mag_mag_yz_sq > 0 
    Rx(2,2) = mag_in.data[2^2 - mag_in.data3^2-1]/mag_mag_yz_sq
    Rx(3,3) = Rx(2,2)
    Rx(2,3) = 2*mag_in.data2*mag_in.data3/mag_mag_yz_sq
    Rx(3,2) = -Rx(2,3)

    #otherwise

    RotMtx = Rx*Ry*Rz
    q = RodMat2Qat RotMtx 
