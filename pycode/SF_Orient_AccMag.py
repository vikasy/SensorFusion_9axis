"""
SF_Orient_AccMag.py

Converted from MATLAB file: SF_Orient_AccMag.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def SF_Orient_AccMag(acc_in, mag_in, frame):
    """
    Converted from MATLAB function
    """

    # Calculate orientation matrix based on accelerometer sensor and magnetometer sensor data
    #
    #

    mag_grav = np.linalg.normacc_in.data
    mag_mag = np.linalg.normmag_in.data

    V3 = 0; 0; 1])
    V2 = np.array([0; 1; 0])
    V1 = np.array([1; 0; 0])

    switch frame
    case .TAND.T
    ifnp.array([ mag_grav > 0 
    V3 = acc_in.data/mag_grav

    if mag_mag > 0 
    temp = np.cross(mag_in.data/mag_mag, V3)
    temp_mag = np.linalg.normtemp
    if temp_mag > 0 
    V1 = temp/temp_mag


    #otherwise

    temp = np.cross(V3, V1)
    temp_mag = np.linalg.normtemp
    if temp_mag > 0 
    V2 = temp/temp_mag

    RotMtx = V1, V2, V3])
    q = RodMat2Quatnp.array([ RotMtx 

