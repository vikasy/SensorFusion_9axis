"""
SF_Init_State.py

Initialize the state information of the sensor fusion filter
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np


def SF_Init_State(FusionCat):
    """
    Initialize the state information of the filter
    
    Args:
        FusionCat (str): Category of sensor fusion 
                        ('3X_A', '3X_M', '3X_G', '6X_AM', '6X_AG', '6X_GM', '9X_AGM')
    
    Returns:
        dict: Initialized state variables
    """
    
    # Initialize state variable structure
    state_var = {}
    
    # Disable reset and setup orientation initialization
    state_var['reset'] = 0
    state_var['orient_init'] = 0
    
    # Orientation angles (degrees)
    state_var['ThetaPost'] = 0.0  # aposteriori pitch
    state_var['PhiPost'] = 0.0    # aposteriori roll 
    state_var['PsiPost'] = 0.0    # aposteriori yaw
    state_var['RhoPost'] = 0.0    # aposteriori compass
    state_var['ChiPost'] = 0.0    # aposteriori tilt from vertical
    
    # Angular velocity and orientation matrices
    state_var['Omega'] = np.zeros((3, 1))
    state_var['QuatPri'] = np.array([1, 0, 0, 0])     # a priori quaternion
    state_var['QuatPost'] = np.array([1, 0, 0, 0])    # a posteriori quaternion
    state_var['RotMtxPri'] = np.eye(3)                # a priori rotation matrix
    state_var['RotMtxPost'] = np.eye(3)               # a posteriori rotation matrix
    
    # Error states
    state_var['OrntErrPostS'] = np.zeros((3, 1))      # orientation error
    state_var['BiasErrPostS'] = np.zeros((3, 1))      # gyro bias error
    state_var['AccErrPostS'] = np.zeros((3, 1))       # acceleration error
    state_var['MdErrPostS'] = np.zeros((3, 1))        # magnetic disturbance error
    
    # Gravity and magnetic vectors
    state_var['GravGyrPriS'] = np.zeros((3, 1))       # gyro-based gravity
    state_var['GravAccPriS'] = np.zeros((3, 1))       # accel-based gravity
    state_var['GravErrPriS'] = np.zeros((3, 1))       # gravity error
    state_var['MagGyroPriS'] = np.zeros((3, 1))       # gyro-based magnetic
    state_var['MagMagPriS'] = np.zeros((3, 1))        # mag-based magnetic
    state_var['MagErrPriS'] = np.zeros((3, 1))        # magnetic error
    
    # Posteriori estimates
    state_var['BiasPostS'] = np.zeros((3, 1))         # gyro bias
    state_var['AccPostS'] = np.zeros((3, 1))          # linear acceleration
    state_var['MdPostS'] = np.zeros((3, 1))           # magnetic disturbance
    state_var['GravPostS'] = np.array([[0], [0], [1]]) # gravity vector (down)
    state_var['AccPostG'] = np.zeros((3, 1))          # acceleration in global frame
    
    # Magnetic field parameters (typical Earth field ~50 µT)
    B = 50.0  # µT
    state_var['MagVecG'] = np.array([[B/np.sqrt(2)], [0], [-B/np.sqrt(2)]])  # NED frame
    state_var['MagMagG'] = B
    state_var['MagIncAngG'] = 45.0  # inclination angle (degrees)
    
    # Fusion category
    state_var['FusionCat'] = FusionCat
    
    # Noise parameters based on fusion type
    if FusionCat == '9X_AGM':
        # 9-axis fusion noise parameters
        state_var['MeasNoiseVar'] = np.eye(9) * 0.01
        state_var['ProcNoiseVar'] = np.eye(12) * 0.001
        state_var['ErrCov'] = np.eye(12) * 0.001
        state_var['P_post'] = np.eye(12) * 0.001
    else:
        # Default noise parameters
        state_var['ProcNoiseVar'] = np.eye(6) * 0.001
        state_var['ErrCov'] = np.eye(6) * 0.001
        state_var['P_post'] = np.eye(6) * 0.001
    
    return state_var


# MATLAB compatibility alias
SF_Init_State_MATLAB = SF_Init_State
