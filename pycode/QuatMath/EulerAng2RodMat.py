"""
EulerAng2RodMat.py

Convert Euler angles to Rodriguez rotation matrix
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def euler_to_rodriguez_matrix(euler_angles):
    """
    Convert Euler angles to Rodriguez rotation matrix
    
    Args:
        euler_angles (array): [roll, pitch, yaw] in radians
    
    Returns:
        array: 3x3 rotation matrix
    """
    euler_angles = np.array(euler_angles).flatten()
    roll, pitch, yaw = euler_angles[0], euler_angles[1], euler_angles[2]
    
    # Rotation matrices for each axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation (ZYX order)
    R = Rz @ Ry @ Rx
    
    return R


# MATLAB compatibility alias
EulerAng2RodMat = euler_to_rodriguez_matrix
