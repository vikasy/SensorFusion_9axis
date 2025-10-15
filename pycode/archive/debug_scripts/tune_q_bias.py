#!/usr/bin/env python3
"""
Test different Q_bias values to see if increasing process noise helps convergence.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis, SF_6XAG_QBias
from sensor_platform_config import SensorPlatform, INVENSENSE

def test_q_bias_value(q_bias_multiplier, max_samples=100):
    """Test a specific Q_bias value and return final bias error."""

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)

    # Initialize sensor fusion with modified Q_bias
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Override Q_bias
    original_q_bias = sf.proc_noise_var_bias
    sf.proc_noise_var_bias = original_q_bias * q_bias_multiplier

    # Calculate actual bias from data
    gyro_x_mean = df['gyro_x_counts'].mean() * platform.gyro_scale_factor
    gyro_y_mean = df['gyro_y_counts'].mean() * platform.gyro_scale_factor
    gyro_z_mean = df['gyro_z_counts'].mean() * platform.gyro_scale_factor
    actual_bias = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])

    fusion_count = 0
    final_bias_error = None
    final_p_trace = None
    final_quat_error = None

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

            # Get P trace
            P_trace = np.trace(sf.err_cov_mtx_post[1, 1])

            final_bias_error = bias_error
            final_p_trace = P_trace
            final_quat_error = quat_error_deg

    return {
        'q_bias_multiplier': q_bias_multiplier,
        'q_bias_value': sf.proc_noise_var_bias,
        'fusion_count': fusion_count,
        'final_bias_error_dps': final_bias_error,
        'final_p_trace': final_p_trace,
        'final_quat_error_deg': final_quat_error
    }

def main():
    print("="*80)
    print("TUNING Q_BIAS FOR BIAS CONVERGENCE")
    print("="*80)
    print()

    print(f"Original SF_6XAG_QBias = {SF_6XAG_QBias}")
    print()

    # Test various multipliers
    multipliers = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

    print(f"{'Multiplier':>10} {'Q_bias':>12} {'Fusions':>8} {'Bias Error':>12} {'P Trace':>12} {'Quat Error':>12}")
    print(f"{'':>10} {'':>12} {'':>8} {'(dps)':>12} {'':>12} {'(deg)':>12}")
    print("-"*80)

    results = []
    for mult in multipliers:
        result = test_q_bias_value(mult, max_samples=100)
        results.append(result)

        print(f"{result['q_bias_multiplier']:>10.1f} "
              f"{result['q_bias_value']:>12.2e} "
              f"{result['fusion_count']:>8d} "
              f"{result['final_bias_error_dps']:>12.4f} "
              f"{result['final_p_trace']:>12.4e} "
              f"{result['final_quat_error_deg']:>12.4f}")

    print()
    print("="*80)
    print("ANALYSIS")
    print("="*80)
    print()

    # Find best result (lowest bias error)
    best = min(results, key=lambda r: r['final_bias_error_dps'])

    print(f"Best result:")
    print(f"  Q_bias multiplier: {best['q_bias_multiplier']}")
    print(f"  Q_bias value: {best['q_bias_value']:.2e}")
    print(f"  Final bias error: {best['final_bias_error_dps']:.4f} dps")
    print(f"  Final quat error: {best['final_quat_error_deg']:.4f} deg")
    print(f"  Final P trace: {best['final_p_trace']:.4e}")
    print()

    # Check if any configuration achieved good convergence
    good_results = [r for r in results if r['final_bias_error_dps'] < 0.5]
    if good_results:
        print(f"✓ Found {len(good_results)} configurations with bias error < 0.5 dps")
        print(f"  Recommended Q_bias multiplier range: {min(r['q_bias_multiplier'] for r in good_results):.1f} to {max(r['q_bias_multiplier'] for r in good_results):.1f}")
    else:
        print("⚠️  No configuration achieved bias error < 0.5 dps")
        print("   This suggests a fundamental issue beyond just Q_bias tuning")
    print()

if __name__ == '__main__':
    main()
