"""
Deg2Rad.py

Converted from MATLAB file: Deg2Rad.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def deg_to_rad(deg_in):
    """
    Convert angle in degree to angle in radian
    
    Args:
        deg_in (float or array): Angle(s) in degrees
    
    Returns:
        float or array: Angle(s) in radians
    """
    return (np.pi / 180) * deg_in


# Alias for MATLAB compatibility
Deg2Rad = deg_to_rad

