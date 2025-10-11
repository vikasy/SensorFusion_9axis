"""
CPMat.py - Cross Product Matrix

Create cross product matrix from vector
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def cp_mat(v):
    """
    Create cross product matrix from vector
    
    Args:
        v (array): 3D vector [x, y, z]
    
    Returns:
        array: 3x3 cross product matrix
    """
    v = np.array(v).flatten()
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


# Alias for MATLAB compatibility
CPMat = cp_mat

