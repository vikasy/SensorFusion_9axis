"""
SF_Update_AGM.py

Converted from MATLAB file: SF_Update_AGM.m
Auto-generated Python code - may require manual adjustments

Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import math


def SF_Update_AGM(state_var, acc_in, gyro_in, mag_in):
    """
    Converted from MATLAB function
    """

    #
    # Input: State variable containing current state vector
    #        Sensor inputs, accnp.array([3-axis in m/s2-1]),gyronp.array([3-axis in deg/s-1]),
    #        magnp.array([3-axis in uT-1]) - total 9axis
    # Note: each sensor input is a data structure with current data np.array([3-axis value-1])
    # and timestamp. Gyro also includes a history of OVERSAMPLE_RATE data.
    #
    # Output: Updated state variable
    #
    # Rev Hist:

    global DEG2RAD QW_FIXED G2MPSECSQ EPSILON
    global Cacc Cmd dt

    ## Set/Reset if needed
    if state_var.reset == 1 
    state_var = SF_Init_Statestate_var.FusionCat
    return

    ## Initialize rotation matrix if this is first run after reset
    if state_var.orient_init == 0 
    state_var.orient_init = 1
    if  strcmp(state_var.FusionCat, .T9X_AGM.T: )
    state_var.QuatPost = SF_Orient_AccMag(acc_in, mag_in, .TAND.T)
else:if  strcmp(state_var.FusionCat, .T6X_AG.T: )
state_var.QuatPost = SF_Orient_Acc(acc_in, .TAND.T)
else:if  strcmp(state_var.FusionCat, .T6X_AM.T: )
state_var.QuatPost = SF_Orient_AccMag(acc_in, mag_in, .TAND.T)
else:if  strcmp(state_var.FusionCat, .T6X_GM.T: )
state_var.QuatPost = SF_Orient_Mag(mag_in, .TAND.T)
else:if  strcmp(state_var.FusionCat, .T3X_A.T: )
state_var.QuatPost = SF_Orient_Acc(acc_in, .TAND.T)
else:if  strcmp(state_var.FusionCat, .T3X_M.T: )
state_var.QuatPost = SF_Orient_Mag(mag_in, .TAND.T)
else: # .T3X_G.T
state_var.QuatPost = 1; 0; 0; 0])

return

# all state variables should be set by now #

## Update angular velocity np.array([deg/sec Omega3]), apriori Rotation matrix
# RotMtxPrinp.array([3])np.array([3]).
state_var.QuatPri = state_var.QuatPost
ifnp.array([ not isempty( gyro_in.data  )
state_var.Omega = gyro_in.data - state_var.BiasErrPostS;
num_gyro_samp = np.shapegyro_in.hist2
for i in range(1:num_gyro_samp):,
omega_i = gyro_in.hist(:,i) - state_var.BiasErrPostS
delta_theta_i = omega_i * dt / num_gyro_samp
state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta_i)

else:
    # No new gyro data, use existing Omega
    disp.TMISSING GYRO DATA!.T
    delta_theta = state_var.Omega * dt
    state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta)

    state_var.RotMtxPri = Quat2RodMat state_var.QuatPri 
    # dispstate_var.RotMtxPri

    ## Run Kalman Filter iteration
    ## Time Update of state vector, xe- = 0 (as A, B = 0)
    #  Compute the following values from Gyro, Accel, and Mag measurements:
    # 	GravGyrPriSnp.array([3])	: apriori gravity vector np.array([gG--1]) gyro based g in sensor frame
    # 	GravAccPriSnp.array([3])	: apriori gravity vector np.array([gA--1]) accel based g in sensor frame
    # 	MagGyroPriSnp.array([3])	: apriori gravity vector np.array([mG--1]) gyro based uT in sensor frame
    # 	MagMagPriSnp.array([3])	: apriori magnetic vector np.array([mM--1]) mag based uT in sensor frame

    # Error in Gravity Vector
    state_var.GravGyrPriS = -state_var.RotMtxPri(:,3)*G2MPSECSQ
    state_var.GravAccPriS = -acc_in.data + Cacc*state_var.AccPostS
    state_var.GravErrPriS = state_var.GravAccPriS - state_var.GravGyrPriS
    # dispstate_var.AccPostS
    # dispstate_var.GravGyrPriS
    # dispstate_var.GravAccPriS
    # dispstate_var.GravErrPriS

    # Error in Magnetic Vector
    state_var.MagGyroPriS = state_var.RotMtxPri(:,2)*state_var.MagVecG2 ...
    + state_var.RotMtxPri(:,3)*state_var.MagVecG3
    # SUG: state_var.MagGyroPriS =  state_var.RotMtxPri*MagVecG
    state_var.MagMagPriS = mag_in.data - Cmd*state_var.MdPostS
    state_var.MagErrPriS = state_var.MagMagPriS - state_var.MagGyroPriS
    # dispstate_var.MagGyroPriS
    # dispstate_var.MagMagPriS
    # dispstate_var.MagErrPriS

    ## Update a-priori error covariance matrix, P_pri = Qw since A = 0
    # Note: this step can be invisible

    ## Compute the measurement matrix, C using a priori gravity and mag values
    # C = np.array([-DEG2RAD*CPMat[gG--1]),DEG2RAD*dt*CPMatnp.array([gG--1]),I3,03;
    #      -DEG2RAD*CPMatnp.array([mG--1]),DEG2RAD*dt*CPMatnp.array([mG--1]),03,-I3]
    C1 = np.array([-DEG2RAD*CPMat[state_var.GravGyrPriS-1]), ...
    DEG2RAD*dt*CPMatstate_var.GravGyrPriS,np.eye3,np.zeros((3,3))];
    C2 = np.array([-DEG2RAD*CPMat[state_var.MagGyroPriS-1]), ...
    DEG2RAD*dt*CPMatstate_var.MagGyroPriS,np.zeros((3,3)),-np.eye3]
    C = C1; C2])

    #dispnp.array([C

    ## Compute Kalman gain, K = Qw*C.T*invC*Qw*C.T + Qv
    # H = Qw*C.T 12x6, then K = H*invC*H + Qv 12x6
    # Qv = diag(QvA, QvA, QvA, QvM, QvM, QvM) 6x6
    H = state_var.ProcNoiseVar * C.T
    # dispH
    K = H/C*H + state_var.MeasNoiseVar

    #dispK

    ## Measurement Update of state vector, xe+ = xe- + K*ze
    # where z is input measurement with ze = C*xe = np.array([[gA- - gG--1]); np.array([mM- - mG--1])]
    # by now, we know K, gA-, gG-, mM-, and mG-
    # M = K*np.array([state_var.GravErrPriS; state_var.MagErrPriS])
    #   = M1 + M2
    M1 = K(:,1:3)*state_var.GravErrPriS
    M2 = K(:,4:6)*state_var.MagErrPriS
    state_var.OrntErrPostS = M1[1-1:3]
    state_var.BiasErrPostS = M1[4-1:6]
    state_var.AccErrPostS = M1[7-1:9]
    state_var.MdErrPostS = M1[10-1:12] + M2[10-1:12]

    # Take care of mag jamming while updating for mag measurement
    mag_jamming  in range( 0;):
    if np.linalg.norm(state_var.MdErrPostS > 1*state_var.MagMagG*state_var.MagMagG )
    mag_jamming = 1
    disp.TMAG JAMMING .T

    ###HACK####
    #mag_jamming = 1
    ###HACK####
    if mag_jamming == 0 
    state_var.OrntErrPostS = state_var.OrntErrPostS + M2[1-1:3]
    state_var.BiasErrPostS = state_var.BiasErrPostS + M2[4-1:6]
    state_var.AccErrPostS = state_var.AccErrPostS + M2[7-1:9]

    # Update Omega as well
    state_var.Omega = state_var.Omega - state_var.BiasErrPostS


    ## Update the following state information for next iteration:
    # 1 the aposteriori rotation matrix for gravity and mag vector
    # measurement in the next iter,
    # 2 the aposteriori orientation angles, compass heading, tilt,
    # 3 the aposteriori bias will be used in calculation of omega in next
    # iteration to correct gyro measurement,
    # 4 aposteriori lin accel , and
    # 5 mag disturbance to correct the magnetic vector in global frame to be
    # used in the next iteration

    # Update the rotation matrix by rotating it back to remove error, as estimated
    # by OrntErrPostS error in orientation angles based on measurement update

    state_var.QuatPost  in range( QuatRotate(state_var.QuatPri):, -state_var.OrntErrPostS)
    ###HACK####
    #state_var.QuatPost = state_var.QuatPri
    ###HACK####

    state_var.RotMtxPost = Quat2RodMatstate_var.QuatPost

    # Update the orientation angles, compass heading, and tilt angles
    # based on the updated rotation matrix
    theta_prev = state_var.ThetaPost
    psi_prev = state_var.PsiPost
    np.array([theta, phi, psi, rho, chi]) = ...
    SF_RotMtx2OrientAng( state_var.RotMtxPost, theta_prev, psi_prev, .TAND.T );
    state_var.ThetaPost = theta
    state_var.PhiPost = phi
    state_var.PsiPost = psi
    state_var.RhoPost = rho
    state_var.ChiPost = chi
    state_var.GravPostS = -state_var.RotMtxPost(:,3)*G2MPSECSQ

    state_var.BiasPostS = state_var.BiasPostS - state_var.BiasErrPostS
    state_var.AccPostS = Cacc*state_var.AccPostS - state_var.AccErrPostS
    state_var.MdPostS = state_var.MdPostS - state_var.MdErrPostS

    state_var.AccPostG = state_var.RotMtxPost*acc_in.data
    #SUG: state_var.AccPostG = state_var.RotMtxPost*state_var.AccPostS
    state_var.AccPostG3 = state_var.AccPostG3 - G2MPSECSQ

    #dispstate_var.MagVecG
    if mag_jamming == 0 
    prev_mag_vec = state_var.MagVecG
    state_var.MdErrPostG = state_var.RotMtxPost.T*state_var.MdErrPostS
    state_var.MagVecG = state_var.MagVecG - state_var.MdErrPostG
    state_var.MagMagG = np.linalg.normstate_var.MagVecG
    state_var.MagVecG1 = 0
    state_var.MagVecG2 = np.absstate_var.MagVecG[2-1]
    if state_var.MagVecG[2 not = 0 or state_var.MagVecG3 not = 0 -1]
    state_var.MagVecG = state_var.MagVecG.*state_var.MagMagG/np.linalg.normstate_var.MagVecG
    state_var.MagIncAngG = anp.tannp.array([-state_var.MagVecG[3-1])/state_var.MagVecG2-1]/DEG2RAD;

    if state_var.MagMagG == 0 
    state_var.MagVecG = prev_mag_vec


    ## Update a-posteriori covariance matrix, P_post = np.array([I12 - K*C-1])*Qw
    state_var.P_post = state_var.ProcNoiseVar - K*C*state_var.ProcNoiseVar
    #state_var.P_post = state_var.P_post - state_var.P_post*C.T*K.T
    #state_var.P_post = state_var.P_post + K*state_var.MeasNoiseVar*K.T
    # Hack to keep it symmetric
    state_var.P_post = 0.5*state_var.P_post + state_var.P_post.T
    state_var.P_post = state_var.P_post + EPSILON*np.eye12
    if  not issymmetric( state_var.P_post )
    disp.TP+ not symm!!.T

    ## Update Qw based on a-posteriori error covariance matrix
    # Qw = a_fnP_post = fP_post + Qinit
    Cacc2 = Cacc*Cacc
    Cmd2 = Cmd*Cmd
    P11 = state_var.P_post(1:3,1:3)
    P12 = state_var.P_post(1:3,4:6);
    P22 = state_var.P_post(4:6,4:6);
    P33 = state_var.P_post(7:9,7:9);
    P44 = state_var.P_post(10:12,10:12)
    #state_var.ProcNoiseVar = QW_FIXED
    Q11 = QW_FIXED(1:3,1:3)
    Q12 = QW_FIXED(1:3,4:6);
    Q22 = QW_FIXED(4:6,4:6);
    Q33 = QW_FIXED(7:9,7:9);
    Q44 = QW_FIXED(10:12,10:12)
    #state_var.ProcNoiseVar = state_var.ProcNoiseVar + scipy.linalg.block_diag(P11, P22, Cacc2*P33, Cmd2*P44)
    state_var.ProcNexitoiseVar(1:3,1:3) = P11 + dt*dt*P22 + Q11
    state_var.ProcNoiseVar(1:3,4:6) = P12 - dt*P22 + Q12
    state_var.ProcNoiseVar(4:6,1:3) = state_var.ProcNoiseVar(1:3,4:6).T
    state_var.P_post(4:6,4:6) = P22 + Q22; #unchanged
    state_var.P_post(7:9,7:9) = Cacc2*P33 + Q33
    state_var.P_post(10:12,10:12) = Cmd2*P44 + Q44
    #state_var.ProcNoiseVar = QW_FIXED + scipy.linalg.block_diag(P11 + dt*dt*P22, P22, Cacc2*P33, Cmd2*P44)
    #state_var.ProcNoiseVar(1:3,4:6) = state_var.ProcNoiseVar(1:3,4:6) + P12 - dt*P22
    #state_var.ProcNoiseVar(4:6,1:3) = state_var.ProcNoiseVar(1:3,4:6).T

    # dispP11
    # dispP22
    # dispP33
    # dispP44
    # dispP12

    ## Done Kalman Filter iteration
