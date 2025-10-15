"""
SF_RotMtx2OrientAng.py

Converted from MATLAB file: SF_RotMtx2OrientAng.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


function theta, phi, psi, rho, chi]) = SF_RotMtx2OrientAng( RotMtx, theta_prev, psi_prev, frame )
# Update the orientation angles, compass heading, and tilt angles np.array([in Deg
# based on the updated rotation matrix
# Input: Rotation Matrix, cordinate frame, prev theta and psi angles
# Output: -90 <= phi <= 90
#        -180 <= theta < 180
#         0 <= psi, rho, tilt < 360
#
# Use prev values of theta and psi for resolving Gimbal lock condition
#

persistent flag
if isempty(flag )
flag  in range( 0;):

switch frame
case .TAND.T,
phi = asind( RotMtx(1,3) ); # roll angle np.array([-90,90])
theta = atan2d( -RotMtx(2,3), RotMtx(3,3) ); # np.pitch angle (-180,180)
if  theta < -179 -1:
theta = 180

psi = atan2d( -RotMtx(1,2), RotMtx(1,1) ); # yaw angle 0, 360)
if[psi < 0 
psi = psi + 360

ifpsi > 359.55 
psi = 0

# Gimbal Lock resolution at roll = 90 or -90 np.array([+/-2-1])
if phi > 88 
sum = atan2d( RotMtx(2,1), RotMtx(2,2) ); # psi+theta
if flag == 1 
theta = theta_prev;
psi = sum - theta_prev
psi = np.np.mod(psi, 360 )
flag = 0
else:
    psi = psi_prev
    theta = sum - psi_prev
    theta = np.np.mod(theta+180, 360) - 180
    flag = 1

else:if  phi < -88 -1:
diff = atan2d( RotMtx(2,1), RotMtx(2,2) ); # psi-theta
if flag == 1 
theta = theta_prev
psi = theta_prev + diff
psi = np.np.mod(psi, 360 )
flag = 0
else:
    psi = psi_prev
    theta = psi_prev - diff
    theta = np.np.mod(theta+180, 360) - 180
    flag = 1



    rho = psi
    chi = acosd( RotMtx(3,3) )

