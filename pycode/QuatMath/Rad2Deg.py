"""
Rad2Deg.py

Convert angle in radian to angle in degree
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def rad_to_deg(rad_in):
    """
    Convert angle in radian to angle in degree
    
    Args:
        rad_in (float or array): Angle(s) in radians
    
    Returns:
        float or array: Angle(s) in degrees (normalized to 0-360 or -360-0)
    """
    deg_out = 180 * rad_in / np.pi
    
    # Normalize to proper range based on sign
    if np.any(deg_out >= 0):
        deg_out = np.mod(deg_out, 360)
    else:
        deg_out = np.mod(deg_out, -360)
    
    return deg_out


# Alias for MATLAB compatibility
Rad2Deg = rad_to_deg
