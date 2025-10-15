#!/usr/bin/env python3
"""
Debug yaw drift in rotation_sequence.
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
    print("DEBUGGING YAW DRIFT")
    print("="*80)
    print()

    # Load rotation_sequence dataset
    dataset_path = "../test/data/datasets/synthetic/rotation_sequence_15s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    print("Tracking yaw evolution...")
    print()

    fusion_count = 0

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

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        if ready_gyro:
            output = sf.run()
            fusion_count += 1

            # Focus on the yaw drift period (fusion 175-250)
            if fusion_count in [175, 200, 225, 250]:
                # Get gyro readings in dps
                gyro_dps = gyro_counts * platform.gyro_scale_factor

                # Calculate quaternion error
                est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
                quat_dot = np.abs(np.dot(est_quat, gt_quat))
                quat_dot = np.clip(quat_dot, 0.0, 1.0)
                quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

                # Calculate yaw error
                gt_yaw = row['gt_yaw_deg']
                est_yaw = output.orientation[0]
                yaw_error = est_yaw - gt_yaw

                # Handle wrap-around
                if yaw_error > 180:
                    yaw_error -= 360
                elif yaw_error < -180:
                    yaw_error += 360

                print(f"Fusion #{fusion_count:3d} (t={fusion_count*0.04:.1f}s):")
                print(f"  Gyro (dps):      [{gyro_dps[0]:7.2f}, {gyro_dps[1]:7.2f}, {gyro_dps[2]:7.2f}]")
                print(f"  Bias est (dps):  [{sf.bias_post_s[0]:7.2f}, {sf.bias_post_s[1]:7.2f}, {sf.bias_post_s[2]:7.2f}]")
                print(f"  Omega (corrected): [{(gyro_dps[0]-sf.bias_post_s[0]):7.2f}, {(gyro_dps[1]-sf.bias_post_s[1]):7.2f}, {(gyro_dps[2]-sf.bias_post_s[2]):7.2f}]")
                print(f"  GT Yaw:  {gt_yaw:7.2f}°")
                print(f"  Est Yaw: {est_yaw:7.2f}°")
                print(f"  Yaw error: {yaw_error:7.2f}°")
                print(f"  Quat error: {quat_error_deg:7.2f}°")
                print()

    print()
    print("ANALYSIS:")
    print("-" * 80)
    print("6-axis sensor fusion (accel + gyro only) CANNOT determine absolute yaw.")
    print("Gyroscopes measure rotation RATE, not absolute heading.")
    print("Any integration error in yaw accumulates unbounded over time.")
    print()
    print("This is a fundamental limitation that requires a magnetometer (9-axis fusion)")
    print("to provide absolute heading reference.")

if __name__ == '__main__':
    main()
