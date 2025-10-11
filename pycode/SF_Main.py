"""
SF_Main.py

Main sensor fusion algorithm
Original MATLAB sensor fusion algorithms converted for Python/NumPy
"""

import numpy as np
from .SF_Init_State import SF_Init_State
from .SF_Update_AGM import SF_Update_AGM
from .SF_Orient_Acc import SF_Orient_Acc
from .SF_Orient_AccMag import SF_Orient_AccMag
from .SF_RotMtx2OrientAng import SF_RotMtx2OrientAng


def SF_Main(accel, gyro, mag=None, dt=0.02, fusion_type='9X_AGM'):
    """
    Main sensor fusion algorithm
    
    Args:
        accel (array): Accelerometer data [ax, ay, az] in g
        gyro (array): Gyroscope data [gx, gy, gz] in rad/s
        mag (array, optional): Magnetometer data [mx, my, mz] in µT
        dt (float): Time step in seconds
        fusion_type (str): Type of fusion ('6X_AG', '9X_AGM', etc.)
    
    Returns:
        dict: Updated state with orientation estimates
    """
    
    # Initialize or get existing state
    if not hasattr(SF_Main, 'state'):
        SF_Main.state = SF_Init_State(fusion_type)
    
    state = SF_Main.state
    
    # Convert inputs to numpy arrays
    accel = np.array(accel).reshape(3, 1)
    gyro = np.array(gyro).reshape(3, 1)
    if mag is not None:
        mag = np.array(mag).reshape(3, 1)
    
    # Update the sensor fusion filter
    if fusion_type == '9X_AGM' and mag is not None:
        # 9-axis fusion with accelerometer, gyro, and magnetometer
        state = SF_Update_AGM(state, accel, gyro, mag, dt)
    elif fusion_type == '6X_AG':
        # 6-axis fusion with accelerometer and gyro
        state = SF_Orient_Acc(state, accel, gyro, dt)
    elif fusion_type == '6X_AM' and mag is not None:
        # 6-axis fusion with accelerometer and magnetometer
        state = SF_Orient_AccMag(state, accel, mag, dt)
    else:
        raise ValueError(f"Unsupported fusion type: {fusion_type}")
    
    # Convert rotation matrix to orientation angles
    state = SF_RotMtx2OrientAng(state)
    
    # Store state for next iteration
    SF_Main.state = state
    
    return state


def reset_fusion():
    """Reset the sensor fusion state"""
    if hasattr(SF_Main, 'state'):
        delattr(SF_Main, 'state')


# MATLAB compatibility alias
SF_Main_MATLAB = SF_Main
