#!/usr/bin/env python3
"""
Debug 9-axis rotation failures - focus on first 5 samples of rotation_x
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("=" * 80)
    print("DEBUGGING 9-AXIS ROTATION X FAILURE")
    print("=" * 80)
    print()

    # Load rotation X dataset
    dataset_path = "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"Dataset: {dataset_path}")
    print(f"Samples: {len(df)}")
    print(f"Expected rotation: 20 dps around X-axis")
    print()

    # Process first 10 samples and print detailed output
    fusion_count = 0
    max_samples = 10

    for idx in range(min(max_samples, len(df))):
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

        mag_counts = np.array([
            row['mag_x_counts'],
            row['mag_y_counts'],
            row['mag_z_counts']
        ], dtype=np.float64)

        gt_quat = np.array([
            row['gt_quat_w'],
            row['gt_quat_x'],
            row['gt_quat_y'],
            row['gt_quat_z']
        ])

        # Convert to physical units for debugging
        accel_g = accel_counts * platform.accel_scale_factor
        gyro_dps = gyro_counts * platform.gyro_scale_factor
        mag_ut = mag_counts * platform.mag_scale_factor

        print(f"Sample {idx}:")
        print(f"  Accel (g):    [{accel_g[0]:7.3f}, {accel_g[1]:7.3f}, {accel_g[2]:7.3f}]")
        print(f"  Gyro (dps):   [{gyro_dps[0]:7.2f}, {gyro_dps[1]:7.2f}, {gyro_dps[2]:7.2f}]")
        print(f"  Mag (µT):     [{mag_ut[0]:7.2f}, {mag_ut[1]:7.2f}, {mag_ut[2]:7.2f}]")
        print(f"  GT Quat:      [{gt_quat[0]:7.4f}, {gt_quat[1]:7.4f}, {gt_quat[2]:7.4f}, {gt_quat[3]:7.4f}]")
        print(f"  GT Euler:     roll={row['gt_roll_deg']:6.2f}° pitch={row['gt_pitch_deg']:6.2f}° yaw={row['gt_yaw_deg']:6.2f}°")

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready
            output = sf.run()
            fusion_count += 1

            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            # Check both quaternion and its negative (q and -q represent same rotation)
            quat_dot = np.dot(est_quat, gt_quat)
            quat_dot_neg = np.dot(-est_quat, gt_quat)

            if abs(quat_dot_neg) > abs(quat_dot):
                quat_dot = quat_dot_neg
                sign_used = "-q"
            else:
                sign_used = "+q"

            quat_dot = np.clip(abs(quat_dot), 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            print(f"  Est Quat:     [{est_quat[0]:7.4f}, {est_quat[1]:7.4f}, {est_quat[2]:7.4f}, {est_quat[3]:7.4f}] ({sign_used})")
            print(f"  Est Euler:    roll={output.orientation[2]:6.2f}° pitch={output.orientation[1]:6.2f}° yaw={output.orientation[0]:6.2f}°")
            print(f"  Quat Error:   {quat_error_deg:6.2f}°")
            print(f"  Gyro Bias:    [{sf.bias_post_s[0]:7.3f}, {sf.bias_post_s[1]:7.3f}, {sf.bias_post_s[2]:7.3f}] dps")
            print(f"  Mag Ref:      [{sf.mag_field_ref[0]:7.3f}, {sf.mag_field_ref[1]:7.3f}, {sf.mag_field_ref[2]:7.3f}]")
            print()

    print("=" * 80)
    print(f"Processed {fusion_count} fusions")
    print("=" * 80)

if __name__ == '__main__':
    main()
