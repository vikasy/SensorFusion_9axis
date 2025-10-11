"""
RodMat2AxisAngle.py

Converted from MATLAB file: RodMat2AxisAngle.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


function axis, ang]) = RodMat2AxisAnglenp.array([R
#RodMat2AxisAngle Converts a Rodriguess rotation matrix to axis angle
#
#   axis, ang]) = RodMat2AxisAnglenp.array([R
#

q = RodMat2Quat R 
axis, ang]) = Quat2AxisAnglenp.array([ q 

