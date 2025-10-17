#!/usr/bin/env python3
"""
Regression Test Suite for 6-Axis Sensor Fusion

This suite consists of two regression test groups based on synthetic data:
1. Core Regression: Original regression test (motion-adaptive bias limiting validation)
2. Extended Regression: All 10 synthetic datasets validation

These tests establish baseline performance and prevent parameter degradation.

Author: Vikas Yadav
Date: 2025-10-15
"""

import numpy as np
import pandas as pd
import sys
import os
import json
from pathlib import Path

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# ============================================================================
# Regression Test Configuration
# ============================================================================

# Baseline results (established with optimized parameters R=10.0)
REGRESSION_BASELINE_FILE = "regression_baseline_optimized.json"

# Performance thresholds (allow 5% degradation tolerance)
ERROR_TOLERANCE = 1.05  # 5% worse than baseline
BIAS_TOLERANCE = 1.10   # 10% worse than baseline (bias can vary more)

# Test datasets for extended regression
EXTENDED_REGRESSION_DATASETS = [
    "static_10s",
    "static_60s",
    "static_high_noise_10s",
    "static_high_bias_10s",
    "rotation_x_20dps_10s",
    "rotation_y_15dps_10s",
    "rotation_z_30dps_10s",
    "rotation_sequence_15s",
    "complex_motion_20s",
    "vibration_5hz_10s"
]

# ============================================================================
# Core Regression Test (Motion-Adaptive Bias Limiting)
# ============================================================================

def test_core_regression():
    """
    Core regression test: Validate motion-adaptive bias limiting

    Tests the key scenarios that validate bias limiting works correctly:
    - rotation_sequence_15s: Multi-axis rotation (moderate motion)
    - complex_motion_20s: Complex motion profile (fast motion)

    Returns:
        dict: Test results with pass/fail status
    """
    print("=" * 80)
    print("CORE REGRESSION TEST: Motion-Adaptive Bias Limiting")
    print("=" * 80)

    results = {
        'test_name': 'core_regression',
        'datasets_tested': [],
        'passed': True,
        'failures': []
    }

    # Load baseline if exists
    baseline = load_baseline()

    # Test key scenarios
    test_cases = [
        ('rotation_sequence_15s', 'Multi-axis rotation - moderate motion'),
        ('complex_motion_20s', 'Complex motion - fast motion with bias limiting')
    ]

    platform = SensorPlatform(INVENSENSE)

    for dataset_name, description in test_cases:
        print(f"\nTesting: {dataset_name}")
        print(f"  Description: {description}")

        dataset_path = f"../../test/data/datasets/synthetic/{dataset_name}.csv"

        if not os.path.exists(dataset_path):
            print(f"  ⚠ Dataset not found: {dataset_path}")
            results['failures'].append(f"{dataset_name}: Dataset not found")
            results['passed'] = False
            continue

        # Run test
        metrics = run_single_test(dataset_path, platform)

        # Check against baseline
        if baseline and dataset_name in baseline:
            baseline_metrics = baseline[dataset_name]

            # Check quaternion error
            error_ratio = metrics['quat_error_mean'] / baseline_metrics['quat_error_mean']
            if error_ratio > ERROR_TOLERANCE:
                fail_msg = (f"{dataset_name}: Error degraded by {(error_ratio-1)*100:.1f}% "
                           f"({metrics['quat_error_mean']:.3f}° vs baseline {baseline_metrics['quat_error_mean']:.3f}°)")
                print(f"  ✗ FAILED: {fail_msg}")
                results['failures'].append(fail_msg)
                results['passed'] = False
            else:
                print(f"  ✓ Error: {metrics['quat_error_mean']:.3f}° (baseline: {baseline_metrics['quat_error_mean']:.3f}°)")

            # Check bias
            bias_ratio = metrics['bias_final'] / baseline_metrics['bias_final']
            if bias_ratio > BIAS_TOLERANCE:
                fail_msg = (f"{dataset_name}: Bias degraded by {(bias_ratio-1)*100:.1f}% "
                           f"({metrics['bias_final']:.4f} vs baseline {baseline_metrics['bias_final']:.4f} dps)")
                print(f"  ✗ FAILED: {fail_msg}")
                results['failures'].append(fail_msg)
                results['passed'] = False
            else:
                print(f"  ✓ Bias: {metrics['bias_final']:.4f} dps (baseline: {baseline_metrics['bias_final']:.4f} dps)")
        else:
            print(f"  ℹ No baseline - Error: {metrics['quat_error_mean']:.3f}°, Bias: {metrics['bias_final']:.4f} dps")

        results['datasets_tested'].append(dataset_name)

    return results


# ============================================================================
# Extended Regression Test (All Synthetic Datasets)
# ============================================================================

def test_extended_regression():
    """
    Extended regression test: Validate all 10 synthetic datasets

    Ensures optimized parameters work well across all scenarios:
    - Static scenarios (convergence, noise, bias)
    - Single-axis rotations (X, Y, Z)
    - Multi-axis rotation
    - Complex motion
    - Vibration

    Returns:
        dict: Test results with pass/fail status
    """
    print("\n" + "=" * 80)
    print("EXTENDED REGRESSION TEST: All Synthetic Datasets")
    print("=" * 80)

    results = {
        'test_name': 'extended_regression',
        'datasets_tested': [],
        'passed': True,
        'failures': [],
        'summary': {}
    }

    # Load baseline
    baseline = load_baseline()

    platform = SensorPlatform(INVENSENSE)

    total_error = 0.0
    total_bias = 0.0

    for dataset_name in EXTENDED_REGRESSION_DATASETS:
        print(f"\nTesting: {dataset_name}")

        dataset_path = f"../../test/data/datasets/synthetic/{dataset_name}.csv"

        if not os.path.exists(dataset_path):
            print(f"  ⚠ Dataset not found: {dataset_path}")
            results['failures'].append(f"{dataset_name}: Dataset not found")
            results['passed'] = False
            continue

        # Run test
        metrics = run_single_test(dataset_path, platform)

        total_error += metrics['quat_error_mean']
        total_bias += metrics['bias_final']

        # Check against baseline
        if baseline and dataset_name in baseline:
            baseline_metrics = baseline[dataset_name]

            # Check quaternion error
            error_ratio = metrics['quat_error_mean'] / baseline_metrics['quat_error_mean']
            bias_ratio = metrics['bias_final'] / baseline_metrics['bias_final']

            error_status = "✓" if error_ratio <= ERROR_TOLERANCE else "✗"
            bias_status = "✓" if bias_ratio <= BIAS_TOLERANCE else "✗"

            print(f"  {error_status} Error: {metrics['quat_error_mean']:.3f}° (baseline: {baseline_metrics['quat_error_mean']:.3f}°)")
            print(f"  {bias_status} Bias: {metrics['bias_final']:.4f} dps (baseline: {baseline_metrics['bias_final']:.4f} dps)")

            if error_ratio > ERROR_TOLERANCE or bias_ratio > BIAS_TOLERANCE:
                fail_msg = f"{dataset_name}: Performance degraded"
                results['failures'].append(fail_msg)
                results['passed'] = False
        else:
            print(f"  ℹ Error: {metrics['quat_error_mean']:.3f}°, Bias: {metrics['bias_final']:.4f} dps")

        results['datasets_tested'].append(dataset_name)

    # Summary statistics
    num_datasets = len(results['datasets_tested'])
    if num_datasets > 0:
        results['summary'] = {
            'average_error': total_error / num_datasets,
            'average_bias': total_bias / num_datasets,
            'datasets_count': num_datasets
        }

    return results


# ============================================================================
# Helper Functions
# ============================================================================

def run_single_test(dataset_path, platform):
    """
    Run fusion on a single dataset and return metrics

    Args:
        dataset_path: Path to CSV dataset
        platform: SensorPlatform instance

    Returns:
        dict: Performance metrics
    """
    df = pd.read_csv(dataset_path)

    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    quat_errors = []
    bias_values = []

    # Process all samples
    for idx in range(len(df)):
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

            # Get ground truth
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            # Compute quaternion angular distance
            quat_error = quaternion_angular_distance(gt_quat, python_quat)
            quat_errors.append(quat_error)

            # Get bias
            bias_mag = np.linalg.norm(sf.bias_post_s)
            bias_values.append(bias_mag)

    # Compute metrics
    quat_errors = np.array(quat_errors)
    bias_values = np.array(bias_values)

    return {
        'quat_error_mean': quat_errors.mean(),
        'quat_error_std': quat_errors.std(),
        'quat_error_max': quat_errors.max(),
        'bias_initial': bias_values[0] if len(bias_values) > 0 else 0.0,
        'bias_final': bias_values[-1] if len(bias_values) > 0 else 0.0,
        'bias_mean': bias_values.mean()
    }


def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi


def load_baseline():
    """Load baseline results from file"""
    if os.path.exists(REGRESSION_BASELINE_FILE):
        with open(REGRESSION_BASELINE_FILE, 'r') as f:
            return json.load(f)
    return None


def save_baseline(results):
    """Save current results as baseline"""
    baseline = {}
    for dataset in results['datasets_tested']:
        # This would need to be populated with actual metrics
        # For now, placeholder
        pass

    with open(REGRESSION_BASELINE_FILE, 'w') as f:
        json.dump(baseline, f, indent=2)

    print(f"\n✓ Baseline saved to {REGRESSION_BASELINE_FILE}")


# ============================================================================
# Main Test Runner
# ============================================================================

def main():
    """Run all regression tests"""
    print("\n" + "=" * 80)
    print("SENSOR FUSION REGRESSION TEST SUITE")
    print("=" * 80)
    print("\nThis suite validates that optimized parameters maintain performance")
    print("across all synthetic test scenarios.")
    print("\nTest Groups:")
    print("  1. Core Regression: Motion-adaptive bias limiting validation")
    print("  2. Extended Regression: All 10 synthetic datasets")
    print("=" * 80)

    all_passed = True

    # Run core regression
    core_results = test_core_regression()
    if not core_results['passed']:
        all_passed = False

    # Run extended regression
    extended_results = test_extended_regression()
    if not extended_results['passed']:
        all_passed = False

    # Summary
    print("\n" + "=" * 80)
    print("REGRESSION TEST SUMMARY")
    print("=" * 80)

    print(f"\nCore Regression: {'✓ PASSED' if core_results['passed'] else '✗ FAILED'}")
    print(f"  Datasets tested: {len(core_results['datasets_tested'])}")
    if core_results['failures']:
        print(f"  Failures: {len(core_results['failures'])}")
        for failure in core_results['failures']:
            print(f"    - {failure}")

    print(f"\nExtended Regression: {'✓ PASSED' if extended_results['passed'] else '✗ FAILED'}")
    print(f"  Datasets tested: {len(extended_results['datasets_tested'])}")
    if extended_results['failures']:
        print(f"  Failures: {len(extended_results['failures'])}")
        for failure in extended_results['failures']:
            print(f"    - {failure}")

    if 'summary' in extended_results and extended_results['summary']:
        print(f"\nAverage Performance:")
        print(f"  Mean Error: {extended_results['summary']['average_error']:.3f}°")
        print(f"  Mean Bias: {extended_results['summary']['average_bias']:.4f} dps")

    print("\n" + "=" * 80)
    print(f"OVERALL: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    print("=" * 80)

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
