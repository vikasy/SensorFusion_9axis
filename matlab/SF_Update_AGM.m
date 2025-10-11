function state_var = SF_Update_AGM(state_var, acc_in, gyro_in, mag_in)
%
% Input: State variable containing current state vector
%        Sensor inputs, acc(3-axis in m/s2),gyro(3-axis in deg/s), 
%        mag(3-axis in uT) - total 9axis
% Note: each sensor input is a data structure with current data (3-axis value) 
% and timestamp. Gyro also includes a history of OVERSAMPLE_RATE data.
%
% Output: Updated state variable
%
% Rev Hist:

global DEG2RAD QW_FIXED G2MPSECSQ EPSILON;
global Cacc Cmd dt;
    
%% Set/Reset if needed
if( state_var.reset == 1 )
    state_var = SF_Init_State(state_var.FusionCat);
    return;
end

%% Initialize rotation matrix if this is first run after reset
if( state_var.orient_init == 0 )
    state_var.orient_init = 1;
    if( strcmp(state_var.FusionCat, '9X_AGM') )
        state_var.QuatPost = SF_Orient_AccMag(acc_in, mag_in, 'AND');
    elseif( strcmp(state_var.FusionCat, '6X_AG') )
        state_var.QuatPost = SF_Orient_Acc(acc_in, 'AND');
    elseif( strcmp(state_var.FusionCat, '6X_AM') )
        state_var.QuatPost = SF_Orient_AccMag(acc_in, mag_in, 'AND');
    elseif( strcmp(state_var.FusionCat, '6X_GM') )
        state_var.QuatPost = SF_Orient_Mag(mag_in, 'AND');
    elseif( strcmp(state_var.FusionCat, '3X_A') )
        state_var.QuatPost = SF_Orient_Acc(acc_in, 'AND');
    elseif( strcmp(state_var.FusionCat, '3X_M') )
        state_var.QuatPost = SF_Orient_Mag(mag_in, 'AND');
    else % '3X_G'
        state_var.QuatPost = [1; 0; 0; 0];
    end
    return;
end    
% all state variables should be set by now %

%% Update angular velocity (deg/sec) Omega[3], apriori Rotation matrix 
% RotMtxPri[3][3]. 
state_var.QuatPri = state_var.QuatPost;
if( ~isempty( gyro_in.data ) )
    state_var.Omega = gyro_in.data - state_var.BiasErrPostS; 
    num_gyro_samp = size(gyro_in.hist,2);
    for i=1:num_gyro_samp,
        omega_i = gyro_in.hist(:,i) - state_var.BiasErrPostS;
        delta_theta_i = omega_i * dt / num_gyro_samp;
        state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta_i);
    end
else
    % No new gyro data, use existing Omega
    disp('MISSING GYRO DATA!');
    delta_theta = state_var.Omega * dt;
    state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta);
end
state_var.RotMtxPri = Quat2RodMat( state_var.QuatPri );
% disp(state_var.RotMtxPri)

%% Run Kalman Filter iteration
%% Time Update of state vector, xe- = 0 (as A, B = 0)
%  Compute the following values from Gyro, Accel, and Mag measurements:
% 	GravGyrPriS[3]	: apriori gravity vector (gG-) gyro based (g) in sensor frame
% 	GravAccPriS[3]	: apriori gravity vector (gA-) accel based (g) in sensor frame
% 	MagGyroPriS[3]	: apriori gravity vector (mG-) gyro based (uT) in sensor frame
% 	MagMagPriS[3]	: apriori magnetic vector (mM-) mag based (uT) in sensor frame

% Error in Gravity Vector
state_var.GravGyrPriS = -state_var.RotMtxPri(:,3)*G2MPSECSQ;
state_var.GravAccPriS = -acc_in.data + Cacc*state_var.AccPostS;
state_var.GravErrPriS = state_var.GravAccPriS - state_var.GravGyrPriS;
% disp(state_var.AccPostS)
% disp(state_var.GravGyrPriS)
% disp(state_var.GravAccPriS)
% disp(state_var.GravErrPriS)

% Error in Magnetic Vector
state_var.MagGyroPriS = state_var.RotMtxPri(:,2)*state_var.MagVecG(2) ... 
                          + state_var.RotMtxPri(:,3)*state_var.MagVecG(3);
% SUG: state_var.MagGyroPriS =  state_var.RotMtxPri*MagVecG;
state_var.MagMagPriS = mag_in.data - Cmd*state_var.MdPostS;
state_var.MagErrPriS = state_var.MagMagPriS - state_var.MagGyroPriS;
% disp(state_var.MagGyroPriS)
% disp(state_var.MagMagPriS)
% disp(state_var.MagErrPriS)

%% Update a-priori error covariance matrix, P_pri = Qw (since A = 0)
% Note: this step can be invisible

%% Compute the measurement matrix, C using a priori gravity and mag values
% C = [-DEG2RAD*CPMat(gG-),DEG2RAD*dt*CPMat(gG-),I3,03; 
%      -DEG2RAD*CPMat(mG-),DEG2RAD*dt*CPMat(mG-),03,-I3]
C1 = [-DEG2RAD*CPMat(state_var.GravGyrPriS), ...
                DEG2RAD*dt*CPMat(state_var.GravGyrPriS),eye(3),zeros(3,3)]; 
C2 = [-DEG2RAD*CPMat(state_var.MagGyroPriS), ...
                DEG2RAD*dt*CPMat(state_var.MagGyroPriS),zeros(3,3),-eye(3)];
C = [C1; C2];

%disp(C)

%% Compute Kalman gain, K = Qw*C'*inv(C*Qw*C' + Qv)
% H = Qw*C' (12x6), then K = H*inv(C*H + Qv) (12x6)
% Qv = diag(QvA, QvA, QvA, QvM, QvM, QvM) (6x6)
H = state_var.ProcNoiseVar * C';
% disp(H)
K = H/(C*H + state_var.MeasNoiseVar);

%disp(K)

%% Measurement Update of state vector, xe+ = xe- + K*ze
% where z is input measurement with ze = C*xe = [(gA- - gG-); (mM- - mG-)]
% by now, we know K, gA-, gG-, mM-, and mG-
% M = K*[state_var.GravErrPriS; state_var.MagErrPriS];
%   = M1 + M2
M1 = K(:,1:3)*state_var.GravErrPriS;
M2 = K(:,4:6)*state_var.MagErrPriS;
state_var.OrntErrPostS = M1(1:3);
state_var.BiasErrPostS = M1(4:6);
state_var.AccErrPostS = M1(7:9);
state_var.MdErrPostS = M1(10:12) + M2(10:12);

% Take care of mag jamming while updating for mag measurement
mag_jamming = 0;
if( norm(state_var.MdErrPostS) > 1*state_var.MagMagG*state_var.MagMagG )
    mag_jamming = 1;
    disp('MAG JAMMING ');
end
%%%HACK%%%%
%mag_jamming = 1;
%%%HACK%%%%
if( mag_jamming == 0 )
    state_var.OrntErrPostS = state_var.OrntErrPostS + M2(1:3);
    state_var.BiasErrPostS = state_var.BiasErrPostS + M2(4:6);
    state_var.AccErrPostS = state_var.AccErrPostS + M2(7:9);
end

% Update Omega as well
state_var.Omega = state_var.Omega - state_var.BiasErrPostS;


%% Update the following state information for next iteration:
% (1) the aposteriori rotation matrix (for gravity and mag vector 
% measurement in the next iter), 
% (2) the aposteriori orientation angles, compass heading, tilt, 
% (3) the aposteriori bias (will be used in calculation of omega in next 
% iteration to correct gyro measurement), 
% (4) aposteriori lin accel , and 
% (5) mag disturbance to correct the magnetic vector in global frame to be 
% used in the next iteration

% Update the rotation matrix by rotating it back to remove error, as estimated 
% by OrntErrPostS (error in orientation angles based on measurement update)

state_var.QuatPost = QuatRotate(state_var.QuatPri, -state_var.OrntErrPostS);
%%%HACK%%%%
%state_var.QuatPost = state_var.QuatPri;
%%%HACK%%%%

state_var.RotMtxPost = Quat2RodMat(state_var.QuatPost);
    
% Update the orientation angles, compass heading, and tilt angles
% based on the updated rotation matrix
theta_prev = state_var.ThetaPost;
psi_prev = state_var.PsiPost;
[theta, phi, psi, rho, chi] = ...
      SF_RotMtx2OrientAng( state_var.RotMtxPost, theta_prev, psi_prev, 'AND' );  
state_var.ThetaPost = theta;
state_var.PhiPost = phi;
state_var.PsiPost = psi;
state_var.RhoPost = rho;
state_var.ChiPost = chi;
state_var.GravPostS = -state_var.RotMtxPost(:,3)*G2MPSECSQ;

state_var.BiasPostS = state_var.BiasPostS - state_var.BiasErrPostS;
state_var.AccPostS = Cacc*state_var.AccPostS - state_var.AccErrPostS;
state_var.MdPostS = state_var.MdPostS - state_var.MdErrPostS;

state_var.AccPostG = state_var.RotMtxPost*acc_in.data;
%SUG: state_var.AccPostG = state_var.RotMtxPost*state_var.AccPostS;
state_var.AccPostG(3) = state_var.AccPostG(3) - G2MPSECSQ;

%disp(state_var.MagVecG);
if( mag_jamming == 0 )
    prev_mag_vec = state_var.MagVecG;
    state_var.MdErrPostG = state_var.RotMtxPost'*state_var.MdErrPostS;
    state_var.MagVecG = state_var.MagVecG - state_var.MdErrPostG;
    state_var.MagMagG = norm(state_var.MagVecG);
    state_var.MagVecG(1) = 0;
    state_var.MagVecG(2) = abs(state_var.MagVecG(2));
    if( state_var.MagVecG(2) ~= 0 || state_var.MagVecG(3) ~= 0 )
        state_var.MagVecG = state_var.MagVecG.*state_var.MagMagG/norm(state_var.MagVecG);
        state_var.MagIncAngG = atan(-state_var.MagVecG(3)/state_var.MagVecG(2))/DEG2RAD; 
    end
    if( state_var.MagMagG == 0 )
        state_var.MagVecG = prev_mag_vec;
    end
end


%% Update a-posteriori covariance matrix, P_post = (I12 - K*C)*Qw
state_var.P_post = state_var.ProcNoiseVar - K*C*state_var.ProcNoiseVar;
%state_var.P_post = state_var.P_post - state_var.P_post*C'*K';
%state_var.P_post = state_var.P_post + K*state_var.MeasNoiseVar*K';
% Hack to keep it symmetric
state_var.P_post = 0.5*(state_var.P_post + state_var.P_post');
state_var.P_post = state_var.P_post + EPSILON*eye(12);
if ( ~issymmetric( state_var.P_post) )
    disp('P+ not symm!!');
end

%% Update Qw based on a-posteriori error covariance matrix
% Qw = a_fn(P_post) = f(P_post) + Qinit
Cacc2 = Cacc*Cacc;
Cmd2 = Cmd*Cmd;
P11 = state_var.P_post(1:3,1:3);
P12 = state_var.P_post(1:3,4:6); 
P22 = state_var.P_post(4:6,4:6); 
P33 = state_var.P_post(7:9,7:9); 
P44 = state_var.P_post(10:12,10:12);
%state_var.ProcNoiseVar = QW_FIXED;
Q11 = QW_FIXED(1:3,1:3);
Q12 = QW_FIXED(1:3,4:6); 
Q22 = QW_FIXED(4:6,4:6); 
Q33 = QW_FIXED(7:9,7:9); 
Q44 = QW_FIXED(10:12,10:12);
%state_var.ProcNoiseVar = state_var.ProcNoiseVar + blkdiag(P11, P22, Cacc2*P33, Cmd2*P44);
state_var.ProcNexitoiseVar(1:3,1:3) = P11 + dt*dt*P22 + Q11;
state_var.ProcNoiseVar(1:3,4:6) = P12 - dt*P22 + Q12;
state_var.ProcNoiseVar(4:6,1:3) = state_var.ProcNoiseVar(1:3,4:6)';
state_var.P_post(4:6,4:6) = P22 + Q22; %unchanged 
state_var.P_post(7:9,7:9) = Cacc2*P33 + Q33;
state_var.P_post(10:12,10:12) = Cmd2*P44 + Q44;
%state_var.ProcNoiseVar = QW_FIXED + blkdiag(P11 + dt*dt*P22, P22, Cacc2*P33, Cmd2*P44);
%state_var.ProcNoiseVar(1:3,4:6) = state_var.ProcNoiseVar(1:3,4:6) + P12 - dt*P22;
%state_var.ProcNoiseVar(4:6,1:3) = state_var.ProcNoiseVar(1:3,4:6)';

% disp(P11);
% disp(P22);
% disp(P33);
% disp(P44);
% disp(P12);

%% Done Kalman Filter iteration
