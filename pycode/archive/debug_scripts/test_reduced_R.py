#!/usr/bin/env python3
"""
Test reducing measurement noise R to improve bias convergence.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def test_with_r_factor(r_factor, num_samples=100):
    """Test with scaled measurement noise R"""

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Scale measurement noise R
    sf.meas_noise_var_acc *= r_factor

    # Calculate actual bias
    gyro_x_mean = df['gyro_x_counts'].mean() * platform.gyro_scale_factor
    gyro_y_mean = df['gyro_y_counts'].mean() * platform.gyro_scale_factor
    gyro_z_mean = df['gyro_z_counts'].mean() * platform.gyro_scale_factor
    actual_bias = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])

    fusion_count = 0
    final_bias_error = None
    final_quat_error = None

    for idx in range(min(num_samples, len(df))):
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

            final_bias_error = bias_error
            final_quat_error = quat_error_deg

    return {
        'r_factor': r_factor,
        'fusion_count': fusion_count,
        'final_bias_error': final_bias_error,
        'final_quat_error': final_quat_error
    }

def main():
    print("="*80)
    print("TESTING REDUCED MEASUREMENT NOISE R")
    print("="*80)
    print()

    # Test various R scaling factors
    r_factors = [1.0, 0.5, 0.1, 0.05, 0.01, 0.005, 0.001]

    print(f"{'R Factor':>10} {'Fusions':>8} {'Bias Error':>12} {'Quat Error':>12}")
    print(f"{'':>10} {'':>8} {'(dps)':>12} {'(deg)':>12}")
    print("-"*80)

    results = []
    for r_factor in r_factors:
        result = test_with_r_factor(r_factor, num_samples=120)
        results.append(result)

        print(f"{result['r_factor']:>10.4f} "
              f"{result['fusion_count']:>8d} "
              f"{result['final_bias_error']:>12.4f} "
              f"{result['final_quat_error']:>12.4f}")

    print()
    print("="*80)
    print("ANALYSIS")
    print("="*80)
    print()

    # Find best result
    best = min(results, key=lambda r: r['final_bias_error'])

    print(f"Best result:")
    print(f"  R factor: {best['r_factor']}")
    print(f"  Final bias error: {best['final_bias_error']:.4f} dps")
    print(f"  Final quat error: {best['final_quat_error']:.4f} deg")
    print()

    # Check if any achieved good convergence
    good_results = [r for r in results if r['final_bias_error'] < 0.5]
    if good_results:
        print(f"✓ Found {len(good_results)} configurations with bias error < 0.5 dps")
        print(f"  Recommended R factor: {min(r['r_factor'] for r in good_results):.4f}")
    else:
        print("⚠️  No configuration achieved bias error < 0.5 dps with 30 samples")
        print("   Try longer convergence time or further R reduction")
    print()

if __name__ == '__main__':
    main()
