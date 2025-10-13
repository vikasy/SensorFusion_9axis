"""
QuatNormal.py

Normalize a quaternion
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_normalize(q):
    """
    Normalize a quaternion to unit length
    Enforces q[0] >= 0 for consistent sign convention (matches C implementation)

    Args:
        q (array): Quaternion [w, x, y, z]

    Returns:
        array: Normalized quaternion with q[0] >= 0
    """
    q = np.array(q)
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1, 0, 0, 0])  # Return identity quaternion

    q_norm = q / norm

    # Enforce q[0] >= 0 (matches C QuatNormal function lines 152-156)
    # If q0 is negative, flip all components (q and -q represent same rotation)
    if q_norm[0] < 0.0:
        q_norm = -q_norm

    return q_norm


# Alias for MATLAB compatibility
QuatNormal = quat_normalize
