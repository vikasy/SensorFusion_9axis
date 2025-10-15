#!/usr/bin/env python3
"""
Debug rotation tracking - check if Python is correctly tracking rotation.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("DEBUG ROTATION TRACKING")
    print("="*80)
    print()

    # Load rotation dataset
    dataset_path = "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples from {os.path.basename(dataset_path)}")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    print("Processing first 10 fusion cycles...")
    print()

    fusion_count = 0
    max_fusions = 10

    for idx in range(len(df)):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([
            row['accel_x_counts'],
            row['accel_y_counts'],
            row['accel_z_counts']
        ], dtype=np.float64)

        gyro_counts = np.array([
            row['gyro_x_counts'],
            row['gyro_y_counts'],
            row['gyro_z_counts']
        ], dtype=np.float64)

        gt_quat = np.array([
            row['gt_quat_w'],
            row['gt_quat_x'],
            row['gt_quat_y'],
            row['gt_quat_z']
        ])

        gt_euler = np.array([
            row['gt_roll_deg'],
            row['gt_pitch_deg'],
            row['gt_yaw_deg']
        ])

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        if ready_gyro:
            output = sf.run()
            fusion_count += 1

            # Calculate quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            # Get estimated euler angles
            est_euler = output.orientation  # [yaw, pitch, roll]

            print(f"Fusion #{fusion_count}:")
            print(f"  Gyro raw (counts): [{int(gyro_counts[0]):5d}, {int(gyro_counts[1]):5d}, {int(gyro_counts[2]):5d}]")

            # Calculate gyro in dps
            gyro_dps = gyro_counts * platform.gyro_scale_factor
            print(f"  Gyro (dps):        [{gyro_dps[0]:7.2f}, {gyro_dps[1]:7.2f}, {gyro_dps[2]:7.2f}]")

            print(f"  Bias estimate:     [{sf.bias_post_s[0]:7.2f}, {sf.bias_post_s[1]:7.2f}, {sf.bias_post_s[2]:7.2f}] dps")

            # Bias-corrected gyro
            omega_corrected = gyro_dps - sf.bias_post_s
            print(f"  Omega (corrected): [{omega_corrected[0]:7.2f}, {omega_corrected[1]:7.2f}, {omega_corrected[2]:7.2f}] dps")

            print(f"  GT Euler (RPY):    [{gt_euler[0]:7.2f}, {gt_euler[1]:7.2f}, {gt_euler[2]:7.2f}]°")
            print(f"  Est Euler (YPR):   [{est_euler[0]:7.2f}, {est_euler[1]:7.2f}, {est_euler[2]:7.2f}]°")
            print(f"  Quat error:        {quat_error_deg:7.3f}°")
            print()

            if fusion_count >= max_fusions:
                break

    print("="*80)
    print("ANALYSIS")
    print("="*80)
    print()
    print("Expected behavior:")
    print("  - Gyro X should be ~80 dps (20 dps rotation + 60 dps bias)")
    print("  - Bias should converge to ~60 dps on X")
    print("  - Omega (corrected) should be ~20 dps on X")
    print("  - Roll should increase by ~20 dps * 0.04s = 0.8° per fusion")
    print()

if __name__ == '__main__':
    main()
