"""
Quat2RodMat.py

Convert quaternion to rotation matrix (Rodriguez matrix)
Corrected Python implementation matching C code

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def quat_to_rotation_matrix(q):
    """
    Convert quaternion to rotation matrix (Rodriguez matrix)

    Args:
        q (array): Quaternion [q0, q1, q2, q3] or [w, x, y, z]
                   where q0/w is scalar component

    Returns:
        array: 3x3 rotation matrix R

    The rotation matrix R rotates vectors from body frame to inertial frame.
    R is such that w = R*v is same as w = q' * v * q
    where q' is quaternion conjugate and v is a pure quaternion
    """
    q = np.array(q).flatten()

    # Normalize quaternion
    norm = np.linalg.norm(q)
    if norm > 0:
        q = q / norm
    else:
        # Return identity matrix for zero quaternion
        return np.eye(3)

    # Extract components
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

    # Compute rotation matrix elements
    # Formula: R_ij = 2*(qi*qj + q0*qk*epsilon_ijk) - delta_ij
    # Where epsilon_ijk is Levi-Civita symbol

    # More efficient direct computation:
    R = np.zeros((3, 3))

    # Row 1
    R[0, 0] = 2.0 * (q0*q0 + q1*q1) - 1.0
    R[0, 1] = 2.0 * (q1*q2 - q0*q3)
    R[0, 2] = 2.0 * (q1*q3 + q0*q2)

    # Row 2
    R[1, 0] = 2.0 * (q1*q2 + q0*q3)
    R[1, 1] = 2.0 * (q0*q0 + q2*q2) - 1.0
    R[1, 2] = 2.0 * (q2*q3 - q0*q1)

    # Row 3
    R[2, 0] = 2.0 * (q1*q3 - q0*q2)
    R[2, 1] = 2.0 * (q2*q3 + q0*q1)
    R[2, 2] = 2.0 * (q0*q0 + q3*q3) - 1.0

    return R


# Aliases for compatibility
Quat2RodMat = quat_to_rotation_matrix
quat_to_rot_mat = quat_to_rotation_matrix
