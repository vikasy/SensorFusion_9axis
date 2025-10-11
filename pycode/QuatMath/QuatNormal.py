"""
QuatNormal.py

Normalize a quaternion
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_normalize(q):
    """
    Normalize a quaternion to unit length
    
    Args:
        q (array): Quaternion [w, x, y, z]
    
    Returns:
        array: Normalized quaternion
    """
    q = np.array(q)
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1, 0, 0, 0])  # Return identity quaternion
    return q / norm


# Alias for MATLAB compatibility
QuatNormal = quat_normalize
