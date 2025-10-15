#!/usr/bin/env python3
"""
Debug magnetometer update during rotation - trace first 20 samples of rotation_x
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
    print("DEBUGGING MAGNETOMETER UPDATE DURING ROTATION X")
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
    print(f"Expected: Constant 20 dps rotation around X-axis")
    print(f"Magnetometer should see rotation of its field direction in YZ plane")
    print()

    # Process first 20 fusion cycles
    fusion_count = 0
    max_fusions = 20

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

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

        # Convert to physical units
        mag_ut = mag_counts * platform.mag_scale_factor

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready
            # Get state before fusion
            quat_before = np.array([sf.quat_post.q0, sf.quat_post.q1, sf.quat_post.q2, sf.quat_post.q3])
            mag_ref_before = sf.mag_field_ref.copy()

            output = sf.run()
            fusion_count += 1

            # Get state after fusion
            quat_after = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            mag_ref_after = sf.mag_field_ref.copy()

            # Compute errors
            quat_dot = np.abs(np.dot(quat_after, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            # Check if mag reference changed (indicates mag update ran)
            mag_ref_changed = not np.allclose(mag_ref_before, mag_ref_after)

            print(f"Fusion {fusion_count} (sample {idx}):")
            print(f"  GT:   roll={row['gt_roll_deg']:6.2f}° pitch={row['gt_pitch_deg']:6.2f}° yaw={row['gt_yaw_deg']:6.2f}°")
            print(f"  Est:  roll={output.orientation[2]:6.2f}° pitch={output.orientation[1]:6.2f}° yaw={output.orientation[0]:6.2f}°")
            print(f"  Quat error: {quat_error_deg:6.2f}°")
            print(f"  Mag (µT): [{mag_ut[0]:7.2f}, {mag_ut[1]:7.2f}, {mag_ut[2]:7.2f}]")
            print(f"  Mag ref:  [{mag_ref_after[0]:7.4f}, {mag_ref_after[1]:7.4f}, {mag_ref_after[2]:7.4f}] (changed={mag_ref_changed})")
            if quat_error_deg > 10.0:
                print(f"  *** LARGE ERROR ***")
            print()

    print("=" * 80)
    print(f"Processed {fusion_count} fusions")
    print("=" * 80)

if __name__ == '__main__':
    main()
