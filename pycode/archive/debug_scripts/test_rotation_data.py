#!/usr/bin/env python3
"""
Test Python sensor fusion on rotation synthetic data.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def test_dataset(dataset_path, max_samples=None):
    """Test on a specific dataset"""

    print(f"Testing: {os.path.basename(dataset_path)}")
    print("-" * 80)

    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples")

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Calculate actual bias
    gyro_x_mean = df['gyro_x_counts'].mean() * platform.gyro_scale_factor
    gyro_y_mean = df['gyro_y_counts'].mean() * platform.gyro_scale_factor
    gyro_z_mean = df['gyro_z_counts'].mean() * platform.gyro_scale_factor
    actual_bias = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])

    print(f"Actual bias: [{actual_bias[0]:.4f}, {actual_bias[1]:.4f}, {actual_bias[2]:.4f}] dps")

    fusion_count = 0
    quat_errors = []
    bias_errors = []

    if max_samples is None:
        max_samples = len(df)

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

            # Calculate quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            # Calculate bias error
            bias_estimate_dps = sf.bias_post_s.copy()
            bias_error = np.linalg.norm(bias_estimate_dps - actual_bias)

            quat_errors.append(quat_error_deg)
            bias_errors.append(bias_error)

            # Print every 25 fusions
            if fusion_count % 25 == 0 or fusion_count <= 5:
                print(f"Fusion #{fusion_count:3d}: "
                      f"Quat error = {quat_error_deg:6.3f}°, "
                      f"Bias error = {bias_error:6.3f} dps")

    print()
    print(f"Processed {fusion_count} fusion cycles")
    print()

    # Statistics
    quat_errors = np.array(quat_errors)
    bias_errors = np.array(bias_errors)

    print("Quaternion Error Statistics:")
    print(f"  Initial:  {quat_errors[0]:.3f}°")
    print(f"  Final:    {quat_errors[-1]:.3f}°")
    print(f"  Mean:     {quat_errors.mean():.3f}°")
    print(f"  Max:      {quat_errors.max():.3f}°")
    print(f"  Std Dev:  {quat_errors.std():.3f}°")

    print()
    print("Bias Error Statistics:")
    print(f"  Initial:  {bias_errors[0]:.3f} dps")
    print(f"  Final:    {bias_errors[-1]:.3f} dps")
    print(f"  Improvement: {(bias_errors[0] - bias_errors[-1]) / bias_errors[0] * 100:.1f}%")

    print()

    # Pass/Fail criteria
    if quat_errors[-1] < 5.0:
        print(f"✓ PASS: Final quaternion error < 5° ({quat_errors[-1]:.3f}°)")
    else:
        print(f"❌ FAIL: Final quaternion error >= 5° ({quat_errors[-1]:.3f}°)")

    if quat_errors.mean() < 5.0:
        print(f"✓ PASS: Mean quaternion error < 5° ({quat_errors.mean():.3f}°)")
    else:
        print(f"❌ FAIL: Mean quaternion error >= 5° ({quat_errors.mean():.3f}°)")

    print()
    return quat_errors, bias_errors

def main():
    print("="*80)
    print("TESTING PYTHON SENSOR FUSION ON ROTATION DATA")
    print("="*80)
    print()

    datasets = [
        "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv",
        "../test/data/datasets/synthetic/rotation_y_15dps_10s.csv",
        "../test/data/datasets/synthetic/rotation_z_30dps_10s.csv",
        "../test/data/datasets/synthetic/rotation_sequence_15s.csv",
        "../test/data/datasets/synthetic/complex_motion_20s.csv",
    ]

    for dataset in datasets:
        if os.path.exists(dataset):
            test_dataset(dataset)
            print("="*80)
            print()
        else:
            print(f"Dataset not found: {dataset}")
            print()

if __name__ == '__main__':
    main()
