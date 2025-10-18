#!/usr/bin/env python3
"""
Debug fusion dataset to understand why errors are high

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


def main():
    header_path = "../../test/data/datasets/fusion/test_input_output_0922.h"

    print("Parsing dataset...")
    sensor_data, expected_outputs = parse_fusion_header(header_path)
    print(f"Loaded {len(sensor_data)} sensor samples, {len(expected_outputs)} expected outputs\n")

    # Check first sensor sample
    print("First 10 sensor samples:")
    for i in range(min(10, len(sensor_data))):
        sid, x, y, z, ts = sensor_data[i]
        sensor_name = ['Accel', 'Gyro', 'Mag'][sid]
        print(f"  {i}: {sensor_name} [{x:6d}, {y:6d}, {z:6d}] @ {ts}")

    # Convert first accel to g's
    platform = SensorPlatform(INVENSENSE)
    first_accel = sensor_data[0]
    accel_g = np.array([first_accel[1], first_accel[2], first_accel[3]]) * platform.accel_scale_factor
    accel_mag = np.linalg.norm(accel_g)

    print(f"\nFirst accelerometer sample in g's: [{accel_g[0]:.3f}, {accel_g[1]:.3f}, {accel_g[2]:.3f}]")
    print(f"Magnitude: {accel_mag:.3f}g (expected ~9.81 m/s² = 1.0g)")

    if abs(accel_mag - 9.81) > 2.0:
        print("⚠ WARNING: Large linear acceleration at start - initialization may be corrupted!")
    else:
        print("✓ Acceleration magnitude looks reasonable for initialization")

    print("\nFirst 5 expected outputs:")
    for i in range(min(5, len(expected_outputs))):
        q0, q1, q2, q3, roll, pitch, yaw, ts = expected_outputs[i]
        print(f"  {i}: Quat=[{q0:.6f}, {q1:.6f}, {q2:.6f}, {q3:.6f}] Euler=[{roll:.1f}, {pitch:.1f}, {yaw:.1f}]")

    # Run fusion and check first few outputs
    print("\nRunning fusion...")
    fusion = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    ts_groups = {}
    for sid, x, y, z, ts in sensor_data:
        if ts not in ts_groups:
            ts_groups[ts] = {0: None, 1: None, 2: None}
        ts_groups[ts][sid] = (x, y, z)

    sorted_timestamps = sorted(ts_groups.keys())
    output_idx = 0

    print("\nFirst 5 fusion outputs vs expected:")
    for ts in sorted_timestamps:
        if output_idx >= 5:
            break

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

                print(f"\n  Output {output_idx}:")
                print(f"    Expected: [{exp_q0:.6f}, {exp_q1:.6f}, {exp_q2:.6f}, {exp_q3:.6f}]")
                print(f"    Python:   [{py_quat[0]:.6f}, {py_quat[1]:.6f}, {py_quat[2]:.6f}, {py_quat[3]:.6f}]")
                print(f"    Error: {angle_error:.3f}°")

                output_idx += 1


if __name__ == '__main__':
    main()
