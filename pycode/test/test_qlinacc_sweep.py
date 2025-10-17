#!/usr/bin/env python3
"""Test different QLinAcc values on walking dataset"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis, SensorID
from sensor_platform_config import SensorPlatform
import sensor_fusion_6axis
import pandas as pd
import numpy as np

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, dot)
    return 2.0 * np.arccos(dot) * 180.0 / np.pi

def test_qlinacc(qlinacc_value):
    """Test walking dataset with specific QLinAcc value"""
    
    # Override QLinAcc
    sensor_fusion_6axis.SF_6XAG_QLinAcc = qlinacc_value
    
    # Load data
    df = pd.read_csv('../../test/data/datasets/realistic/walking.csv')
    
    # Initialize fusion
    config = SensorPlatform()
    sf = SensorFusion9Axis(SensorID.PLATFORM_9DOF_SENSOR, config)
    
    errors = []
    
    for idx in range(min(500, len(df))):  # Test first 500 samples
        row = df.iloc[idx]
        
        accel = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.int16)
        gyro = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.int16)
        mag = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.int16)
        
        sf.process_sample(accel, gyro, mag)
        
        if sf.is_orientation_initialized():
            py_quat = sf.get_orientation_quaternion()
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            error = quaternion_angular_distance(gt_quat, py_quat)
            errors.append(error)
    
    mean_error = np.mean(errors) if errors else 999.0
    max_error = np.max(errors) if errors else 999.0
    
    return mean_error, max_error

# Test different QLinAcc values
print("QLinAcc | Mean Error | Max Error")
print("--------|------------|----------")

for qlinacc in [1.0, 5.0, 10.0, 20.0, 50.0, 100.0]:
    mean_err, max_err = test_qlinacc(qlinacc)
    print(f"{qlinacc:7.1f} | {mean_err:10.2f}° | {max_err:9.2f}°")
