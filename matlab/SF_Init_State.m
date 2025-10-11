function state_var = SF_Init_State( FusionCat )

% Initialize the state information of the filter
    global QV_INIT_AGM QW_INIT_AGM B;
    
% Disable reset and setup orientation initialization
    state_var.reset = 0;
    state_var.orient_init = 0;
    
% 	PhiPost         : aposteriori roll (deg)
% 	ThetaPost       : aposteriori pitch (deg)
% 	PsiPost         : aposteriori	yaw (deg)
% 	RhoPost         : aposteriori compass (deg)
% 	ChiPost         : aposteriori tilt from vertical (deg)
    state_var.ThetaPost = 0;
    state_var.PhiPost = 0;
    state_var.PsiPost = 0;
    state_var.RhoPost = 0;
    state_var.ChiPost = 0;
    
% 	Omega[3]	 	: angular velocity (deg/sec)
% 	RotMtxPri[3][3] : a priori orientation matrix
% 	QuatPri         : apriori orientation quaternion
% 	RotMtxPost[3][3]: a posteriori rotation matrix
% 	QuatPost		: a posteriori orientation quaternion
    state_var.Omega = zeros(3,1); 
    state_var.QuatPri = zeros(4,1);
    state_var.QuatPost = zeros(4,1);     
    state_var.RotMtxPri = zeros(3,3);
    state_var.RotMtxPost = zeros(3,3);
    
%   OrntErrPostS[3] : aposteriori orientation error (deg)
%   BiasErrPostS[3] : aposteriori gyro offset error (deg/sec)
% 	MdErrPostS[3]   : aposteriori magnetic disturbance error (uT) in sensor frame
% 	AccErrPostS[3]	: linear acceleration error (g) in sensor frame
    state_var.OrntErrPostS = zeros(3,1);
    state_var.BiasErrPostS = zeros(3,1);
    state_var.AccErrPostS = zeros(3,1);
    state_var.MdErrPostS = zeros(3,1);
    
% 	GravGyrPriS[3]	: apriori gravity vector gyro based (g) in sensor frame
% 	GravAccPriS[3]	: apriori gravity vector accel based (g) in sensor frame
% 	MagGyroPriS[3]	: apriori gravity vector gyro based (uT) in sensor frame
% 	MagMagPriS[3]	: apriori magnetic vector mag based (uT) in sensor frame
% 	GravErrPriS[3]	: apriori difference between accel based value and gyro based value (g) in sensor frame
% 	MagErrPriS[3]	: apriori difference between mag based value and gyro based value (uT) in sensor frame
    state_var.GravGyrPriS = zeros(3,1);
    state_var.GravAccPriS = zeros(3,1);
    state_var.GravErrPriS = zeros(3,1);
    state_var.MagGyroPriS = zeros(3,1);
    state_var.MagMagPriS = zeros(3,1);
    state_var.MagErrPriS = zeros(3,1);
    
%   BiasPostS[3]    : aposteriori estimate of gyro bias (deg/sec)
% 	AccPostS[3]		: aposteriori linear acceleration (g) sensor frame
% 	MdPostG[3]	    : aposteriori magnetic disturbance (uT) in sensor frame
    state_var.BiasPostS = zeros(3,1);
    state_var.AccPostS = zeros(3,1);
    state_var.MdPostS = zeros(3,1);
    state_var.GravPostS = zeros(3,1);
    state_var.AccPostG = zeros(3,1);
    
% 	MagVecG[3]   	: magnetic vector (uT) in global frame
% 	MagMagG[3]   	: magnetic vector magnitude (uT) in global frame
% 	MagIncAngG 	 	: magnetic inclination angle (deg) in global frame
    state_var.MagVecG = [0; B/sqrt(2); -B/sqrt(2)];
    state_var.MagMagG = B;
    state_var.MagIncAngG = 45.0; 
    
%   FusionCat       : Category of sensor fusion (3X_A, 3X_M, 3X_G, 6X_AM, 6X_AG, 6X_GM, 9X_AGM)   
    state_var.FusionCat = FusionCat;

    if( strcmp(FusionCat, '9X_AGM') )
        % 	MeasNoiseVar   	: Measurement error variance
        state_var.MeasNoiseVar = QV_INIT_AGM;
        % 	ProcNoiseVar   	: Filter model error variance
        state_var.ProcNoiseVar = QW_INIT_AGM;
        %   ErrCov          : Filter error covariance matrix
        state_var.ErrCov = QW_INIT_AGM;
        state_var.P_post = QW_INIT_AGM;
    else      
        state_var.ProcNoiseVar = eye(3);
        state_var.ErrCov = eye(3);
    end





