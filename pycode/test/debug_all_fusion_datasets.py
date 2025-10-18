#!/usr/bin/env python3
"""
Debug all fusion datasets to understand failures

Author: Vikas Yadav
Date: 2025-10-17
"""

import re
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_platform_config import SensorPlatform, INVENSENSE


def parse_first_samples(header_path, num_samples=5):
    """Parse first few sensor samples and expected outputs"""
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
    count = 0

    for line in sensor_lines:
        line = line.strip().rstrip(',')
        if not line or line.startswith('//'):
            continue
        match = re.match(r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}', line)
        if match:
            sensor_data.append((int(match.group(1)), int(match.group(2)),
                              int(match.group(3)), int(match.group(4)), int(match.group(5))))
            count += 1
            if count >= num_samples * 3:  # 3 sensors per sample
                break

    # Parse expected output
    output_match = re.search(
        r'static const test_expected_output_t expected_output_data\[\] = \{(.*?)\};',
        content, re.DOTALL
    )
    if not output_match:
        return sensor_data, None

    output_lines = output_match.group(1).strip().split('\n')
    expected_outputs = []
    count = 0

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
            count += 1
            if count >= num_samples:
                break

    return sensor_data, expected_outputs


def analyze_dataset(name, filepath):
    """Analyze initial conditions of a dataset"""
    print(f"\n{'='*80}")
    print(f"{name}")
    print(f"{'='*80}")

    sensor_data, expected_outputs = parse_first_samples(filepath, 10)

    if sensor_data is None:
        print("  ✗ Could not parse sensor data")
        return

    if expected_outputs is None:
        print("  ✗ Could not parse expected outputs")
        return

    # Find first accel sample
    first_accel = None
    for sid, x, y, z, ts in sensor_data:
        if sid == 0:
            first_accel = (x, y, z)
            break

    if first_accel:
        platform = SensorPlatform(INVENSENSE)
        accel_g = np.array(first_accel) * platform.accel_scale_factor
        accel_mag = np.linalg.norm(accel_g)

        print(f"\nFirst Accelerometer Sample:")
        print(f"  Counts: [{first_accel[0]:6d}, {first_accel[1]:6d}, {first_accel[2]:6d}]")
        print(f"  G's:    [{accel_g[0]:7.3f}, {accel_g[1]:7.3f}, {accel_g[2]:7.3f}]")
        print(f"  Magnitude: {accel_mag:.3f}g")

        if abs(accel_mag - 9.81) > 2.0:
            print(f"  ⚠ WARNING: Large linear acceleration ({abs(accel_mag - 9.81):.2f} m/s² deviation)")
        else:
            print(f"  ✓ Acceleration reasonable for initialization")

    # Show first expected output
    if expected_outputs:
        q0, q1, q2, q3, roll, pitch, yaw, ts = expected_outputs[0]
        qmag = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)

        print(f"\nFirst Expected Output:")
        print(f"  Quat: [{q0:.6f}, {q1:.6f}, {q2:.6f}, {q3:.6f}]")
        print(f"  Magnitude: {qmag:.6f}")
        print(f"  Euler: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

        if abs(qmag - 1.0) > 0.01:
            print(f"  ⚠ WARNING: Quaternion not normalized (mag={qmag:.6f})")

        # Check if identity quaternion
        if abs(q0 - 1.0) < 0.01 and abs(q1) < 0.01 and abs(q2) < 0.01 and abs(q3) < 0.01:
            print(f"  ✓ Starts near identity (no rotation)")
        else:
            print(f"  ⚠ Initial rotation: q=[{q0:.3f}, {q1:.3f}, {q2:.3f}, {q3:.3f}]")


def main():
    base_path = "../../test/data/datasets/fusion"

    datasets = [
        ("0922 (Main - PASS)", "test_input_output_0922.h"),
        ("0923 Moving (FAIL)", "test_input_output_0923_moving.h"),
        ("0923 Standstill (FAIL)", "test_input_output_0923_standstill.h"),
        ("0930 (CATASTROPHIC FAIL)", "test_input_output_0930.h"),
        ("1012 (Parse Error)", "test_input_output_1012.h"),
    ]

    print("="*80)
    print("FUSION DATASETS - INITIALIZATION ANALYSIS")
    print("="*80)

    for name, filename in datasets:
        filepath = os.path.join(base_path, filename)
        if os.path.exists(filepath):
            analyze_dataset(name, filepath)
        else:
            print(f"\n{name}: File not found")


if __name__ == '__main__':
    main()
