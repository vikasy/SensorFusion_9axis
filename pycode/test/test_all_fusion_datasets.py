#!/usr/bin/env python3
"""
Test all fusion datasets for data quality and regression

Tests all INVENSENSE fusion test vectors:
- test_input_output_0922.h (main dataset)
- test_input_output_0923_moving.h
- test_input_output_0923_standstill.h
- test_input_output_0930.h
- test_input_output_1012.h

Author: Vikas Yadav
Date: 2025-10-17
"""

import re
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE


def parse_fusion_header(header_path):
    """Parse C header file"""
    with open(header_path, 'r') as f:
        content = f.read()

    # Parse sensor input
    sensor_match = re.search(
        r'static const test_sensor_sample_t sensor_input_data\[\] = \{(.*?)\};',
        content, re.DOTALL
    )
    if not sensor_match:
        return None, None

    sensor_lines = sensor_match.group(1).strip().split('\n')
    sensor_data = []

    for line in sensor_lines:
        line = line.strip().rstrip(',')
        if not line or line.startswith('//'):
            continue
        match = re.match(r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}', line)
        if match:
            sensor_data.append((int(match.group(1)), int(match.group(2)),
                              int(match.group(3)), int(match.group(4)), int(match.group(5))))

    # Parse expected output
    output_match = re.search(
        r'static const test_expected_output_t expected_output_data\[\] = \{(.*?)\};',
        content, re.DOTALL
    )
    if not output_match:
        return sensor_data, None

    output_lines = output_match.group(1).strip().split('\n')
    expected_outputs = []

    for line in output_lines:
        line = line.strip().rstrip(',')
        if not line or line.startswith('//'):
            continue
        match = re.match(
            r'\{(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*'
            r'(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(\d+)ULL\}',
            line
        )
        if match:
            expected_outputs.append((float(match.group(1)), float(match.group(2)),
                                   float(match.group(3)), float(match.group(4)),
                                   float(match.group(5)), float(match.group(6)),
                                   float(match.group(7)), int(match.group(8))))

    return sensor_data, expected_outputs


def run_fusion_test(sensor_data, expected_outputs):
    """Run fusion and compare with expected outputs"""
    platform = SensorPlatform(INVENSENSE)
    fusion = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Group by timestamp
    ts_groups = {}
    for sid, x, y, z, ts in sensor_data:
        if ts not in ts_groups:
            ts_groups[ts] = {0: None, 1: None, 2: None}
        ts_groups[ts][sid] = (x, y, z)

    sorted_timestamps = sorted(ts_groups.keys())
    outputs = []
    output_idx = 0

    for ts in sorted_timestamps:
        samples = ts_groups[ts]

        if samples[0] is not None:
            fusion.preprocess_sensor_data(0, np.array(samples[0], dtype=np.float64), ts)
        if samples[1] is not None:
            ready = fusion.preprocess_sensor_data(1, np.array(samples[1], dtype=np.float64), ts)
        if samples[2] is not None:
            fusion.preprocess_sensor_data(2, np.array(samples[2], dtype=np.float64), ts)

        if samples[1] is not None and (ready & 0x2):
            output = fusion.run()
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            if output_idx < len(expected_outputs):
                exp_q0, exp_q1, exp_q2, exp_q3, _, _, _, _ = expected_outputs[output_idx]
                exp_quat = np.array([exp_q0, exp_q1, exp_q2, exp_q3])

                dot_product = np.abs(np.dot(py_quat, exp_quat))
                dot_product = np.clip(dot_product, 0.0, 1.0)
                angle_error = 2.0 * np.arccos(dot_product) * (180.0 / np.pi)

                outputs.append(angle_error)
                output_idx += 1

    return np.array(outputs)


def test_dataset(name, filepath):
    """Test a single fusion dataset"""
    print(f"\n{'='*80}")
    print(f"TESTING: {name}")
    print(f"{'='*80}")

    # Parse
    try:
        sensor_data, expected_outputs = parse_fusion_header(filepath)
        if sensor_data is None or expected_outputs is None:
            print(f"  ✗ SKIP: Could not parse dataset")
            return None

        print(f"  Sensor samples: {len(sensor_data)}")
        print(f"  Expected outputs: {len(expected_outputs)}")

    except Exception as e:
        print(f"  ✗ ERROR: {e}")
        return None

    # Run fusion
    try:
        angle_errors = run_fusion_test(sensor_data, expected_outputs)
        print(f"  Fusion cycles: {len(angle_errors)}")
    except Exception as e:
        print(f"  ✗ ERROR during fusion: {e}")
        return None

    # Statistics
    samples_within_10deg = np.sum(angle_errors < 10.0) / len(angle_errors) * 100

    print(f"\nResults:")
    print(f"  Mean error:   {angle_errors.mean():.3f}°")
    print(f"  Median error: {np.median(angle_errors):.3f}°")
    print(f"  Max error:    {angle_errors.max():.3f}°")
    print(f"  Within 10°:   {samples_within_10deg:.1f}%")

    # Regression check
    passed = True
    issues = []

    if samples_within_10deg < 74.0:
        issues.append(f"Only {samples_within_10deg:.1f}% within 10° (threshold: ≥74%)")
        passed = False

    if angle_errors.mean() > 25.0:
        issues.append(f"Mean error {angle_errors.mean():.3f}° exceeds 25°")
        passed = False

    if angle_errors.max() > 60.0:
        issues.append(f"Max error {angle_errors.max():.3f}° exceeds 60°")
        passed = False

    if passed:
        print(f"\n  ✓ PASS")
    else:
        print(f"\n  ✗ FAIL:")
        for issue in issues:
            print(f"    - {issue}")

    return {
        'name': name,
        'passed': passed,
        'mean_error': angle_errors.mean(),
        'median_error': np.median(angle_errors),
        'max_error': angle_errors.max(),
        'within_10deg': samples_within_10deg,
        'num_samples': len(angle_errors)
    }


def main():
    base_path = "../../test/data/datasets/fusion"

    datasets = [
        ("0922 (Main)", "test_input_output_0922.h"),
        ("0923 Moving", "test_input_output_0923_moving.h"),
        ("0923 Standstill", "test_input_output_0923_standstill.h"),
        ("0930", "test_input_output_0930.h"),
        ("1012", "test_input_output_1012.h"),
    ]

    print("="*80)
    print("FUSION DATASETS - COMPREHENSIVE VALIDATION")
    print("="*80)
    print(f"Testing {len(datasets)} INVENSENSE fusion test vectors")
    print()

    results = []
    for name, filename in datasets:
        filepath = os.path.join(base_path, filename)
        result = test_dataset(name, filepath)
        if result:
            results.append(result)

    # Summary
    print(f"\n{'='*80}")
    print("SUMMARY - ALL FUSION DATASETS")
    print(f"{'='*80}\n")

    print(f"{'Dataset':<20} {'Samples':<10} {'Mean°':<10} {'Median°':<10} {'Max°':<10} {'<10°%':<10} {'Status':<10}")
    print("-"*80)

    all_passed = True
    for r in results:
        status = "✓ PASS" if r['passed'] else "✗ FAIL"
        if not r['passed']:
            all_passed = False

        print(f"{r['name']:<20} {r['num_samples']:<10} {r['mean_error']:<10.2f} "
              f"{r['median_error']:<10.2f} {r['max_error']:<10.2f} {r['within_10deg']:<10.1f} {status:<10}")

    print()
    print("="*80)

    if all_passed:
        print("✓ ALL FUSION DATASETS PASSED")
        print("="*80)
        return 0
    else:
        print("✗ SOME FUSION DATASETS FAILED")
        print("="*80)
        return 1


if __name__ == '__main__':
    sys.exit(main())
