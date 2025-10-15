#!/usr/bin/env python3
"""
Debug magnetometer reference initialization in 9-axis fusion
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
    print("DEBUGGING MAGNETOMETER REFERENCE INITIALIZATION")
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

    print(f"Dataset: rotation_x_20dps (first 100 samples)")
    print()

    # Process first 100 samples and check mag reference
    fusion_count = 0
    mag_ref_initialized = False

    for idx in range(min(100, len(df))):
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

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready
            output = sf.run()
            fusion_count += 1

            # Check if mag ref was initialized
            mag_ref_norm = np.linalg.norm(sf.mag_field_ref)

            if mag_ref_norm > 0 and not mag_ref_initialized:
                mag_ref_initialized = True
                print(f"Mag Ref INITIALIZED at fusion {fusion_count} (sample {idx}):")
                print(f"  mag_field_ref: [{sf.mag_field_ref[0]:7.4f}, {sf.mag_field_ref[1]:7.4f}, {sf.mag_field_ref[2]:7.4f}]")
                print(f"  mag_ref_norm: {mag_ref_norm:.4f}")
                print()

            # Print detailed info every 10 fusions for first 50
            if fusion_count <= 50 and fusion_count % 10 == 0:
                est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
                quat_dot = np.abs(np.dot(est_quat, gt_quat))
                quat_dot = np.clip(quat_dot, 0.0, 1.0)
                quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

                print(f"Fusion {fusion_count} (sample {idx}):")
                print(f"  GT roll={row['gt_roll_deg']:6.2f}°, Est roll={output.orientation[2]:6.2f}°")
                print(f"  Quat error: {quat_error_deg:6.2f}°")
                print(f"  Mag ref: [{sf.mag_field_ref[0]:7.4f}, {sf.mag_field_ref[1]:7.4f}, {sf.mag_field_ref[2]:7.4f}] (norm={mag_ref_norm:.4f})")
                print()

    print("=" * 80)
    print(f"Processed {fusion_count} fusions from 100 samples")
    print(f"Mag ref initialized: {mag_ref_initialized}")
    print(f"Final mag_field_ref: [{sf.mag_field_ref[0]:7.4f}, {sf.mag_field_ref[1]:7.4f}, {sf.mag_field_ref[2]:7.4f}]")
    print("=" * 80)

if __name__ == '__main__':
    main()
