function state_var = SF_Update_6X_AG(state_var, acc_in, gyro_in)
% 6-Axis (Accel + Gyro) Sensor Fusion Update
% Simplified version of SF_Update_AGM for 6-axis only

global DEG2RAD QW_FIXED G2MPSECSQ EPSILON;
global Cacc dt;

% Reset if needed
if state_var.reset == 1
    state_var = SF_Init_State(state_var.FusionCat);
    return;
end

% Initialize orientation if first run
if state_var.orient_init == 0
    state_var.orient_init = 1;
    state_var.QuatPost = SF_Orient_Acc(acc_in, 'AND');
    return;
end

% Time Update - integrate gyro
state_var.QuatPri = state_var.QuatPost;
if ~isempty(gyro_in.data)
    state_var.Omega = gyro_in.data - state_var.BiasErrPostS;
    num_gyro_samp = size(gyro_in.hist, 2);
    for i = 1:num_gyro_samp
        omega_i = gyro_in.hist(:,i) - state_var.BiasErrPostS;
        delta_theta_i = omega_i * dt / num_gyro_samp;
        state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta_i);
    end
else
    delta_theta = state_var.Omega * dt;
    state_var.QuatPri = QuatRotate(state_var.QuatPri, delta_theta);
end

state_var.RotMtxPri = Quat2RodMat(state_var.QuatPri);

% Measurement Update
% Compute gravity vectors
state_var.GravGyrPriS = -state_var.RotMtxPri(:,3) * G2MPSECSQ;
state_var.GravAccPriS = -acc_in.data + Cacc * state_var.AccPostS;
state_var.GravErrPriS = state_var.GravAccPriS - state_var.GravGyrPriS;

% Measurement matrix for 6-axis (orientation + bias only)
C = [-DEG2RAD*CPMat(state_var.GravGyrPriS), ...
     DEG2RAD*dt*CPMat(state_var.GravGyrPriS)];

% Kalman gain
H = state_var.ProcNoiseVar * C';
K = H / (C * H + state_var.MeasNoiseVar);

% State update
M = K * state_var.GravErrPriS;
state_var.OrntErrPostS = M(1:3);
state_var.BiasErrPostS = M(4:6);

% Update omega
state_var.Omega = state_var.Omega - state_var.BiasErrPostS;

% Update quaternion
state_var.QuatPost = QuatRotate(state_var.QuatPri, -state_var.OrntErrPostS);

% Normalize quaternion to prevent drift
quat_norm = norm(state_var.QuatPost);
if quat_norm > 0
    state_var.QuatPost = state_var.QuatPost / quat_norm;
end

state_var.RotMtxPost = Quat2RodMat(state_var.QuatPost);

% Update orientation angles
theta_prev = state_var.ThetaPost;
psi_prev = state_var.PsiPost;
[theta, phi, psi, rho, chi] = SF_RotMtx2OrientAng(state_var.RotMtxPost, theta_prev, psi_prev, 'AND');
state_var.ThetaPost = theta;
state_var.PhiPost = phi;
state_var.PsiPost = psi;
state_var.RhoPost = rho;
state_var.ChiPost = chi;
state_var.GravPostS = -state_var.RotMtxPost(:,3) * G2MPSECSQ;

% Update bias and acceleration
state_var.BiasPostS = state_var.BiasPostS - state_var.BiasErrPostS;
state_var.AccPostS = Cacc * state_var.AccPostS;  % No AccErrPostS in 6-axis

state_var.AccPostG = state_var.RotMtxPost * acc_in.data;
state_var.AccPostG(3) = state_var.AccPostG(3) - G2MPSECSQ;

% Update error covariance
state_var.P_post = state_var.ProcNoiseVar - K * C * state_var.ProcNoiseVar;
state_var.P_post = 0.5 * (state_var.P_post + state_var.P_post');
state_var.P_post = state_var.P_post + EPSILON * eye(6);

% Update process noise covariance
P11 = state_var.P_post(1:3, 1:3);
P12 = state_var.P_post(1:3, 4:6);
P22 = state_var.P_post(4:6, 4:6);

Q11 = QW_FIXED(1:3, 1:3);
Q12 = QW_FIXED(1:3, 4:6);
Q22 = QW_FIXED(4:6, 4:6);

state_var.ProcNoiseVar(1:3, 1:3) = P11 + dt*dt*P22 + Q11;
state_var.ProcNoiseVar(1:3, 4:6) = P12 - dt*P22 + Q12;
state_var.ProcNoiseVar(4:6, 1:3) = state_var.ProcNoiseVar(1:3, 4:6)';
state_var.ProcNoiseVar(4:6, 4:6) = P22 + Q22;

end
