"""
QuatProduct.py

Converted from MATLAB file: QuatProduct.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def quat_product(p, q):
    """
    Calculate product of two quaternions
    
    Args:
        p (array): First quaternion [w, x, y, z] or [x, y, z]
        q (array): Second quaternion [w, x, y, z] or [x, y, z]
    
    Returns:
        array: Product quaternion pq = [w, x, y, z]
        
    Formula: pq = [p0*q0 - P·Q, P×Q + p0*Q + q0*P]
    where p = [p0, P] and q = [q0, Q]
    """
    p = np.array(p).flatten()
    q = np.array(q).flatten()
    
    # Handle 3-element quaternions (pure quaternions)
    if len(q) == 3:
        q = np.concatenate([[0], q])
    if len(p) == 3:
        p = np.concatenate([[0], p])
    
    # Extract scalar and vector parts
    p0 = p[0]
    P = p[1:4]
    q0 = q[0] 
    Q = q[1:4]
    
    # Calculate quaternion product
    pq = np.zeros(4)
    pq[0] = p0 * q0 - np.dot(P, Q)
    pq[1:4] = np.cross(P, Q) + p0 * Q + q0 * P
    
    return pq


# Alias for MATLAB compatibility  
QuatProduct = quat_product

