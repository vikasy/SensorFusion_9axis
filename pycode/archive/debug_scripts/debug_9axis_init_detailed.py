#!/usr/bin/env python3
"""
Detailed line-by-line debugging of 9-axis initialization (first 5 samples)
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def print_state(sf, label=""):
    """Print current state of sensor fusion"""
    print(f"  {label}")
    print(f"    quat_post: [{sf.quat_post.q0:.4f}, {sf.quat_post.q1:.4f}, {sf.quat_post.q2:.4f}, {sf.quat_post.q3:.4f}]")
    print(f"    rot_mtx_post:\n{sf.rot_mtx_post}")
    print(f"    bias_post_s: [{sf.bias_post_s[0]:.4f}, {sf.bias_post_s[1]:.4f}, {sf.bias_post_s[2]:.4f}] dps")
    print(f"    mag_field_ref: [{sf.mag_field_ref[0]:.4f}, {sf.mag_field_ref[1]:.4f}, {sf.mag_field_ref[2]:.4f}]")
    print(f"    orient_init: {sf.orient_init}")
    print()

def main():
    print("=" * 80)
    print("DETAILED DEBUGGING: 9-AXIS INITIALIZATION (First 5 Samples)")
    print("=" * 80)
    print()

    # Load rotation X dataset
    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"  # Start with static
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"Dataset: {dataset_path}")
    print(f"Testing: Static scenario (no rotation, should be easy)")
    print()

    # Process first 5 samples with detailed debugging
    for idx in range(5):
        print(f"{'=' * 80}")
        print(f"SAMPLE {idx}")
        print(f"{'=' * 80}")

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
        accel_g = accel_counts * platform.accel_scale_factor
        gyro_dps = gyro_counts * platform.gyro_scale_factor
        mag_ut = mag_counts * platform.mag_scale_factor

        print(f"Input:")
        print(f"  timestamp_ns: {timestamp}")
        print(f"  accel_counts: [{accel_counts[0]:7.0f}, {accel_counts[1]:7.0f}, {accel_counts[2]:7.0f}]")
        print(f"  accel_g:      [{accel_g[0]:7.3f}, {accel_g[1]:7.3f}, {accel_g[2]:7.3f}]")
        print(f"  gyro_counts:  [{gyro_counts[0]:7.0f}, {gyro_counts[1]:7.0f}, {gyro_counts[2]:7.0f}]")
        print(f"  gyro_dps:     [{gyro_dps[0]:7.2f}, {gyro_dps[1]:7.2f}, {gyro_dps[2]:7.2f}]")
        print(f"  mag_counts:   [{mag_counts[0]:7.0f}, {mag_counts[1]:7.0f}, {mag_counts[2]:7.0f}]")
        print(f"  mag_ut:       [{mag_ut[0]:7.2f}, {mag_ut[1]:7.2f}, {mag_ut[2]:7.2f}]")
        print(f"  GT quat:      [{gt_quat[0]:7.4f}, {gt_quat[1]:7.4f}, {gt_quat[2]:7.4f}, {gt_quat[3]:7.4f}]")
        print(f"  GT euler:     roll={row['gt_roll_deg']:6.2f}° pitch={row['gt_pitch_deg']:6.2f}° yaw={row['gt_yaw_deg']:6.2f}°")
        print()

        print("State BEFORE processing:")
        print_state(sf, "Before")

        # Call preprocess_sensor_data for each sensor
        print("Step 1: preprocess_sensor_data(ACC)")
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        print(f"  ready_acc = 0x{ready_acc:02X}")
        print(f"  acc_data.count_avg: [{sf.acc_data.count_avg[0]:.1f}, {sf.acc_data.count_avg[1]:.1f}, {sf.acc_data.count_avg[2]:.1f}]")
        print()

        print("Step 2: preprocess_sensor_data(GYRO)")
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        print(f"  ready_gyro = 0x{ready_gyro:02X}")
        print(f"  gyro_data.count_avg: [{sf.gyro_data.count_avg[0]:.1f}, {sf.gyro_data.count_avg[1]:.1f}, {sf.gyro_data.count_avg[2]:.1f}]")
        print()

        print("Step 3: preprocess_sensor_data(MAG)")
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)
        print(f"  ready_mag = 0x{ready_mag:02X}")
        print(f"  mag_data.count_avg: [{sf.mag_data.count_avg[0]:.1f}, {sf.mag_data.count_avg[1]:.1f}, {sf.mag_data.count_avg[2]:.1f}]")
        print()

        if ready_gyro & 0x2:  # Gyro ready
            print("Step 4: run() - Gyro ready, calling fusion")

            # Check if orientation needs initialization
            if not sf.orient_init:
                print("  Orientation NOT initialized - will initialize in run()")
                print(f"  acc_data.count_avg: [{sf.acc_data.count_avg[0]:.1f}, {sf.acc_data.count_avg[1]:.1f}, {sf.acc_data.count_avg[2]:.1f}]")
                print(f"  mag_data.count_avg: [{sf.mag_data.count_avg[0]:.1f}, {sf.mag_data.count_avg[1]:.1f}, {sf.mag_data.count_avg[2]:.1f}]")
                accel_avg_mps2 = sf.acc_data.count_avg * sf.acc_data.scale_factor * 9.81
                mag_avg_ut = sf.mag_data.count_avg * sf.mag_data.scale_factor
                print(f"  accel_avg (m/s²): [{accel_avg_mps2[0]:.3f}, {accel_avg_mps2[1]:.3f}, {accel_avg_mps2[2]:.3f}]")
                print(f"  mag_avg (µT):     [{mag_avg_ut[0]:.3f}, {mag_avg_ut[1]:.3f}, {mag_avg_ut[2]:.3f}]")

            output = sf.run()

            print(f"  Output quat:  [{output.quat.q0:.4f}, {output.quat.q1:.4f}, {output.quat.q2:.4f}, {output.quat.q3:.4f}]")
            print(f"  Output euler: roll={output.orientation[2]:6.2f}° pitch={output.orientation[1]:6.2f}° yaw={output.orientation[0]:6.2f}°")

            # Compute error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))
            print(f"  Quat error:   {quat_error_deg:6.2f}°")
            print()

        print("State AFTER processing:")
        print_state(sf, "After")

        print()

    print("=" * 80)
    print("INITIALIZATION COMPLETE")
    print("=" * 80)

if __name__ == '__main__':
    main()
