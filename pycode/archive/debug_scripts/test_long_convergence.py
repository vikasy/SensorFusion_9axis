#!/usr/bin/env python3
"""
Test bias convergence over extended time (100+ samples).
"""

import numpy as np
import pandas as pd
import sys
import os
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("TESTING LONG-TERM BIAS CONVERGENCE")
    print("="*80)
    print()

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples")
    print()

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
    print()

    fusion_count = 0
    bias_errors = []
    quat_errors = []
    biases_x = []
    biases_y = []
    biases_z = []
    fusions = []

    max_samples = 500  # Process first 125 fusions

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

            # Calculate bias error
            bias_estimate_dps = sf.bias_post_s.copy()
            bias_error = np.linalg.norm(bias_estimate_dps - actual_bias)

            # Calculate quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            bias_errors.append(bias_error)
            quat_errors.append(quat_error_deg)
            biases_x.append(bias_estimate_dps[0])
            biases_y.append(bias_estimate_dps[1])
            biases_z.append(bias_estimate_dps[2])
            fusions.append(fusion_count)

    print(f"Processed {fusion_count} fusion cycles")
    print()
    print("Final bias estimate:")
    print(f"  X: {biases_x[-1]:.4f} dps (actual: {actual_bias[0]:.4f}, error: {abs(biases_x[-1] - actual_bias[0]):.4f})")
    print(f"  Y: {biases_y[-1]:.4f} dps (actual: {actual_bias[1]:.4f}, error: {abs(biases_y[-1] - actual_bias[1]):.4f})")
    print(f"  Z: {biases_z[-1]:.4f} dps (actual: {actual_bias[2]:.4f}, error: {abs(biases_z[-1] - actual_bias[2]):.4f})")
    print(f"  Total error: {bias_errors[-1]:.4f} dps")
    print()
    print(f"Final quaternion error: {quat_errors[-1]:.4f} deg")
    print()

    improvement = (bias_errors[0] - bias_errors[-1]) / bias_errors[0] * 100
    print(f"Bias error improvement: {improvement:.1f}%")
    print()

    if bias_errors[-1] < 0.5:
        print("✓ Bias converged to < 0.5 dps!")
    elif bias_errors[-1] < 1.0:
        print("⚠️  Bias partially converged (< 1.0 dps)")
    else:
        print("❌ Bias did not converge")
    print()

if __name__ == '__main__':
    main()
