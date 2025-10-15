#!/usr/bin/env python3
"""
Test different values of SF_6XAG_QBias to find optimal convergence behavior.
Tests: 100.0 (current), 10.0, 1.0, 0.1, 0.01
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

# We need to modify the constant before importing
import sensor_fusion_6axis

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi

def test_qbias_value(qbias_value, num_samples=100):
    """Test a specific QBias value and return convergence metrics"""

    # Temporarily modify the constant
    original_value = sensor_fusion_6axis.SF_6XAG_QBias
    sensor_fusion_6axis.SF_6XAG_QBias = qbias_value

    # Reload the 9-axis module to pick up the new value
    import importlib
    if 'sensor_fusion_9axis' in sys.modules:
        del sys.modules['sensor_fusion_9axis']

    from sensor_fusion_9axis import SensorFusion9Axis
    from sensor_platform_config import SensorPlatform, INVENSENSE

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    fusion_data = []
    fusion_count = 0

    for idx in range(min(num_samples, len(df))):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            fusion_count += 1
            output = sf.run()

            # Get ground truth
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])

            # Get Python output
            python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            # Compute error
            quat_error = quaternion_angular_distance(gt_quat, python_quat)

            # Get bias
            bias = sf.bias_post_s.copy()
            bias_mag = np.linalg.norm(bias)

            fusion_data.append({
                'fusion': fusion_count,
                'quat_error': quat_error,
                'bias_mag': bias_mag
            })

    # Restore original value
    sensor_fusion_6axis.SF_6XAG_QBias = original_value

    # Compute metrics
    if len(fusion_data) == 0:
        return None

    errors = np.array([d['quat_error'] for d in fusion_data])
    bias_mags = np.array([d['bias_mag'] for d in fusion_data])

    metrics = {
        'qbias': qbias_value,
        'fusion_count': fusion_count,
        'quat_error_initial': errors[0],
        'quat_error_final': errors[-1],
        'quat_error_mean': errors.mean(),
        'quat_error_std': errors.std(),
        'quat_error_max': errors.max(),
        'bias_initial': bias_mags[0],
        'bias_final': bias_mags[-1],
        'bias_mean': bias_mags.mean(),
        'bias_max': bias_mags.max(),
        'converged': errors[-1] < 0.5,
        'diverged': bias_mags[-1] > 10.0
    }

    return metrics, fusion_data

def main():
    print("="*80)
    print("QBIAS TUNING TEST - Finding Optimal Bias Process Noise")
    print("="*80)

    # Test different QBias values
    qbias_values = [100.0, 50.0, 10.0, 5.0, 1.0, 0.5, 0.1, 0.01]

    print(f"\nTesting {len(qbias_values)} different QBias values on first 100 samples")
    print(f"Dataset: static_10s.csv (ground truth: all zeros)\n")

    results = []

    for qbias in qbias_values:
        print(f"Testing QBias = {qbias:7.2f}...", end=" ", flush=True)

        metrics, fusion_data = test_qbias_value(qbias, num_samples=100)

        if metrics is None:
            print("FAILED")
            continue

        results.append(metrics)

        # Quick summary
        print(f"Quat: {metrics['quat_error_final']:6.3f}°, Bias: {metrics['bias_final']:8.5f} dps", end="")

        if metrics['diverged']:
            print(" ✗ DIVERGED")
        elif metrics['converged']:
            print(" ✓ CONVERGED")
        else:
            print(" ⚠ PARTIAL")

    # Detailed results
    print("\n" + "="*80)
    print("DETAILED RESULTS")
    print("="*80)

    print(f"\n{'QBias':>8} {'Quat Err':>10} {'Quat Err':>10} {'Quat Err':>10} {'Bias':>10} {'Bias':>10} {'Status':>12}")
    print(f"{'':>8} {'Initial':>10} {'Final':>10} {'Mean':>10} {'Final':>10} {'Max':>10} {'':>12}")
    print(f"{'':>8} {'(deg)':>10} {'(deg)':>10} {'(deg)':>10} {'(dps)':>10} {'(dps)':>10} {'':>12}")
    print("-"*80)

    for m in results:
        status = "DIVERGED" if m['diverged'] else ("CONVERGED" if m['converged'] else "PARTIAL")
        print(f"{m['qbias']:8.2f} {m['quat_error_initial']:10.4f} {m['quat_error_final']:10.4f} "
              f"{m['quat_error_mean']:10.4f} {m['bias_final']:10.5f} {m['bias_max']:10.5f} {status:>12}")

    # Find best value
    print("\n" + "="*80)
    print("RECOMMENDATION")
    print("="*80)

    # Filter out diverged cases
    good_results = [m for m in results if not m['diverged']]

    if len(good_results) == 0:
        print("\n✗ All tested values diverged! Need to test even smaller values.")
    else:
        # Find the one with best (lowest) final quaternion error
        best = min(good_results, key=lambda m: m['quat_error_final'])

        print(f"\n✓ Best QBias value: {best['qbias']:.2f}")
        print(f"  - Final quaternion error: {best['quat_error_final']:.4f}°")
        print(f"  - Final bias magnitude: {best['bias_final']:.6f} dps")
        print(f"  - Mean quaternion error: {best['quat_error_mean']:.4f}°")
        print(f"  - Converged: {'Yes' if best['converged'] else 'Partial'}")

        # Show comparison with original
        original = next((m for m in results if m['qbias'] == 100.0), None)
        if original and best['qbias'] != 100.0:
            print(f"\n  Improvement vs. QBias=100.0:")
            print(f"  - Quaternion error: {original['quat_error_final']:.4f}° → {best['quat_error_final']:.4f}° "
                  f"({100*(1 - best['quat_error_final']/original['quat_error_final']):.1f}% better)")
            print(f"  - Bias magnitude: {original['bias_final']:.5f} → {best['bias_final']:.5f} dps "
                  f"({100*(1 - best['bias_final']/original['bias_final']):.1f}% better)")

    print("\n" + "="*80)
    print("TEST COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
