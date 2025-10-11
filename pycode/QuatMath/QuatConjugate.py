"""
QuatConjugate.py

Convert a quaternion to its conjugate
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_conjugate(q):
    """
    Convert a quaternion to its conjugate
    
    Args:
        q (array): Quaternion [w, x, y, z]
    
    Returns:
        array: Conjugate quaternion [w, -x, -y, -z]
    """
    q = np.array(q)
    qc = q.copy()
    qc[1:4] = -qc[1:4]  # Negate the vector part
    return qc


# Alias for MATLAB compatibility
QuatConjugate = quat_conjugate
