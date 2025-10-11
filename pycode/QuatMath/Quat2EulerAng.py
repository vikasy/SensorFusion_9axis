"""
Quat2EulerAng.py

Convert quaternion to Euler angles
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_to_euler_angles(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    Args:
        q (array): Quaternion [w, x, y, z]
    
    Returns:
        array: Euler angles [roll, pitch, yaw] in radians
    """
    q = np.array(q).flatten()
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])


# MATLAB compatibility alias
Quat2EulerAng = quat_to_euler_angles
