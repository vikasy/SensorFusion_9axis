"""
QuatInverse.py

Calculate quaternion inverse
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
from .QuatConjugate import quat_conjugate


def quat_inverse(q):
    """
    Calculate the inverse of a quaternion
    
    Args:
        q (array): Quaternion [w, x, y, z]
    
    Returns:
        array: Inverse quaternion
    """
    q = np.array(q)
    norm_sq = np.sum(q * q)
    if norm_sq == 0:
        return np.array([1, 0, 0, 0])  # Return identity quaternion
    
    qc = quat_conjugate(q)
    return qc / norm_sq


# Alias for MATLAB compatibility
QuatInverse = quat_inverse
