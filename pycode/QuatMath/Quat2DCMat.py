"""
Quat2DCMat.py

Convert quaternion to direction cosine matrix (rotation matrix)
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_to_dcm(q):
    """
    Convert quaternion to direction cosine matrix
    
    Args:
        q (array): Quaternion [w, x, y, z]
    
    Returns:
        array: 3x3 rotation matrix
    """
    q = np.array(q).flatten()
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Direction cosine matrix
    dcm = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])
    
    return dcm


# Alias for MATLAB compatibility  
Quat2DCMat = quat_to_dcm

