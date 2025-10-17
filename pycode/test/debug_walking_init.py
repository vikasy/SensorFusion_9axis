#!/usr/bin/env python3
"""
Debug script to understand walking.csv initialization issue

Author: Vikas Yadav
Date: 2025-10-15
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi

def main():
    # Load walking dataset
    dataset_path = "../../test/data/datasets/realistic/walking.csv"
    df = pd.read_csv(dataset_path)

    print("=" * 80)
    print("DEBUGGING WALKING.CSV INITIALIZATION")
    print("=" * 80)

    # Check first few samples
    print("\nFirst 5 samples - Raw Sensor Data:")
    print("-" * 80)
    for i in range(5):
        row = df.iloc[i]
        print(f"\nSample {i}:")
        print(f"  Accel counts: [{row['accel_x_counts']:6.0f}, {row['accel_y_counts']:6.0f}, {row['accel_z_counts']:6.0f}]")
        print(f"  Gyro counts:  [{row['gyro_x_counts']:6.0f}, {row['gyro_y_counts']:6.0f}, {row['gyro_z_counts']:6.0f}]")
        print(f"  Mag counts:   [{row['mag_x_counts']:6.0f}, {row['mag_y_counts']:6.0f}, {row['mag_z_counts']:6.0f}]")
        print(f"  GT Quat:      [{row['gt_quat_w']:.4f}, {row['gt_quat_x']:.4f}, {row['gt_quat_y']:.4f}, {row['gt_quat_z']:.4f}]")
        print(f"  GT RPY:       [{row['gt_roll_deg']:.2f}°, {row['gt_pitch_deg']:.2f}°, {row['gt_yaw_deg']:.2f}°]")

    # Convert to physical units
    platform = SensorPlatform(INVENSENSE)
    print(f"\n" + "=" * 80)
    print("SENSOR SCALE FACTORS:")
    print("=" * 80)
    print(f"Accel scale: {platform.accel_scale_factor} (counts to m/s²)")
    print(f"Gyro scale:  {platform.gyro_scale_factor} (counts to dps)")
    print(f"Mag scale:   {platform.mag_scale_factor} (counts to uT)")

    print(f"\nFirst sample in physical units:")
    row0 = df.iloc[0]
    accel_phys = np.array([row0['accel_x_counts'], row0['accel_y_counts'], row0['accel_z_counts']]) * platform.accel_scale_factor
    gyro_phys = np.array([row0['gyro_x_counts'], row0['gyro_y_counts'], row0['gyro_z_counts']]) * platform.gyro_scale_factor
    mag_phys = np.array([row0['mag_x_counts'], row0['mag_y_counts'], row0['mag_z_counts']]) * platform.mag_scale_factor

    print(f"  Accel: [{accel_phys[0]:8.3f}, {accel_phys[1]:8.3f}, {accel_phys[2]:8.3f}] m/s²")
    print(f"  Gyro:  [{gyro_phys[0]:8.3f}, {gyro_phys[1]:8.3f}, {gyro_phys[2]:8.3f}] dps")
    print(f"  Mag:   [{mag_phys[0]:8.3f}, {mag_phys[1]:8.3f}, {mag_phys[2]:8.3f}] uT")

    accel_mag = np.linalg.norm(accel_phys)
    mag_mag = np.linalg.norm(mag_phys)
    print(f"\n  Accel magnitude: {accel_mag:.3f} m/s² (expected ~9.81)")
    print(f"  Mag magnitude:   {mag_mag:.3f} uT (expected ~25-65 uT)")

    # Now run fusion and see what happens
    print(f"\n" + "=" * 80)
    print("RUNNING FUSION:")
    print("=" * 80)

    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print("\nProcessing first 10 samples:")
    for idx in range(10):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            quat_error = quaternion_angular_distance(gt_quat, py_quat)

            print(f"\nSample {idx}:")
            print(f"  GT Quat:  [{gt_quat[0]:7.4f}, {gt_quat[1]:7.4f}, {gt_quat[2]:7.4f}, {gt_quat[3]:7.4f}]")
            print(f"  Py Quat:  [{py_quat[0]:7.4f}, {py_quat[1]:7.4f}, {py_quat[2]:7.4f}, {py_quat[3]:7.4f}]")
            print(f"  GT RPY:   [{row['gt_roll_deg']:7.2f}°, {row['gt_pitch_deg']:7.2f}°, {row['gt_yaw_deg']:7.2f}°]")
            print(f"  Py RPY:   [{output.orientation[2]:7.2f}°, {output.orientation[1]:7.2f}°, {output.orientation[0]:7.2f}°]")
            print(f"  Error:    {quat_error:.2f}°")
            print(f"  Bias:     [{sf.bias_post_s[0]:.4f}, {sf.bias_post_s[1]:.4f}, {sf.bias_post_s[2]:.4f}] dps (mag: {np.linalg.norm(sf.bias_post_s):.4f})")

    # Compare with synthetic data
    print(f"\n" + "=" * 80)
    print("COMPARISON WITH SYNTHETIC DATA:")
    print("=" * 80)

    synthetic_path = "../../test/data/datasets/synthetic/rotation_sequence_15s.csv"
    df_syn = pd.read_csv(synthetic_path)

    print("\nSynthetic first sample:")
    row_syn = df_syn.iloc[0]
    print(f"  Accel counts: [{row_syn['accel_x_counts']:6.0f}, {row_syn['accel_y_counts']:6.0f}, {row_syn['accel_z_counts']:6.0f}]")
    print(f"  Gyro counts:  [{row_syn['gyro_x_counts']:6.0f}, {row_syn['gyro_y_counts']:6.0f}, {row_syn['gyro_z_counts']:6.0f}]")
    print(f"  Mag counts:   [{row_syn['mag_x_counts']:6.0f}, {row_syn['mag_y_counts']:6.0f}, {row_syn['mag_z_counts']:6.0f}]")

    accel_syn_phys = np.array([row_syn['accel_x_counts'], row_syn['accel_y_counts'], row_syn['accel_z_counts']]) * platform.accel_scale_factor
    mag_syn_phys = np.array([row_syn['mag_x_counts'], row_syn['mag_y_counts'], row_syn['mag_z_counts']]) * platform.mag_scale_factor

    print(f"\n  Accel: [{accel_syn_phys[0]:8.3f}, {accel_syn_phys[1]:8.3f}, {accel_syn_phys[2]:8.3f}] m/s²")
    print(f"  Mag:   [{mag_syn_phys[0]:8.3f}, {mag_syn_phys[1]:8.3f}, {mag_syn_phys[2]:8.3f}] uT")
    print(f"  Accel magnitude: {np.linalg.norm(accel_syn_phys):.3f} m/s²")
    print(f"  Mag magnitude:   {np.linalg.norm(mag_syn_phys):.3f} uT")

    print(f"\nRealistic first sample:")
    print(f"  Accel: [{accel_phys[0]:8.3f}, {accel_phys[1]:8.3f}, {accel_phys[2]:8.3f}] m/s²")
    print(f"  Mag:   [{mag_phys[0]:8.3f}, {mag_phys[1]:8.3f}, {mag_phys[2]:8.3f}] uT")
    print(f"  Accel magnitude: {accel_mag:.3f} m/s²")
    print(f"  Mag magnitude:   {mag_mag:.3f} uT")

    # Key difference analysis
    print(f"\n" + "=" * 80)
    print("KEY OBSERVATIONS:")
    print("=" * 80)

    # Normalize and compare directions
    accel_syn_norm = accel_syn_phys / np.linalg.norm(accel_syn_phys)
    accel_real_norm = accel_phys / np.linalg.norm(accel_phys)
    mag_syn_norm = mag_syn_phys / np.linalg.norm(mag_syn_phys)
    mag_real_norm = mag_phys / np.linalg.norm(mag_phys)

    print(f"\nAccel direction (normalized):")
    print(f"  Synthetic:  [{accel_syn_norm[0]:7.4f}, {accel_syn_norm[1]:7.4f}, {accel_syn_norm[2]:7.4f}]")
    print(f"  Realistic:  [{accel_real_norm[0]:7.4f}, {accel_real_norm[1]:7.4f}, {accel_real_norm[2]:7.4f}]")
    print(f"  Dot product: {np.dot(accel_syn_norm, accel_real_norm):.4f}")

    print(f"\nMag direction (normalized):")
    print(f"  Synthetic:  [{mag_syn_norm[0]:7.4f}, {mag_syn_norm[1]:7.4f}, {mag_syn_norm[2]:7.4f}]")
    print(f"  Realistic:  [{mag_real_norm[0]:7.4f}, {mag_real_norm[1]:7.4f}, {mag_real_norm[2]:7.4f}]")
    print(f"  Dot product: {np.dot(mag_syn_norm, mag_real_norm):.4f}")

    print("\n" + "=" * 80)

if __name__ == '__main__':
    main()
