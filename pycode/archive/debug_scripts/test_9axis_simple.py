#!/usr/bin/env python3
"""
Test 9-axis sensor fusion on simple synthetic scenarios
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def angle_diff(a, b):
    """Calculate smallest angle difference (handles wraparound)"""
    diff = a - b
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def test_dataset(dataset_path, name):
    """Test a single dataset with 9-axis fusion"""

    df = pd.read_csv(dataset_path)
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    fusion_count = 0
    roll_errors = []
    pitch_errors = []
    yaw_errors = []
    tilt_errors = []
    quat_errors = []

    for idx in range(len(df)):
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

        mag_counts = np.array([
            row['mag_x_counts'],
            row['mag_y_counts'],
            row['mag_z_counts']
        ], dtype=np.float64)

        gt_quat = np.array([
            row['gt_quat_w'],
            row['gt_quat_x'],
            row['gt_quat_y'],
            row['gt_quat_z']
        ])

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready
            output = sf.run()
            fusion_count += 1

            # Quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))
            quat_errors.append(quat_error_deg)

            # Euler angle errors
            roll_err = abs(angle_diff(output.orientation[2], row['gt_roll_deg']))
            pitch_err = abs(angle_diff(output.orientation[1], row['gt_pitch_deg']))
            yaw_err = abs(angle_diff(output.orientation[0], row['gt_yaw_deg']))

            roll_errors.append(roll_err)
            pitch_errors.append(pitch_err)
            yaw_errors.append(yaw_err)

            # Tilt error (roll + pitch)
            tilt_err = np.sqrt(roll_err**2 + pitch_err**2)
            tilt_errors.append(tilt_err)

    roll_errors = np.array(roll_errors)
    pitch_errors = np.array(pitch_errors)
    yaw_errors = np.array(yaw_errors)
    tilt_errors = np.array(tilt_errors)
    quat_errors = np.array(quat_errors)

    duration_s = fusion_count * 0.04

    print(f"{name}")
    print(f"  Duration: {duration_s:.1f}s ({fusion_count} fusions)")
    print(f"  Tilt Error:  mean={tilt_errors.mean():5.2f}°  max={tilt_errors.max():6.2f}°  final={tilt_errors[-1]:5.2f}°")
    print(f"  Roll Error:  mean={roll_errors.mean():5.2f}°  max={roll_errors.max():6.2f}°")
    print(f"  Pitch Error: mean={pitch_errors.mean():5.2f}°  max={pitch_errors.max():6.2f}°")
    print(f"  Yaw Error:   mean={yaw_errors.mean():5.2f}°  max={yaw_errors.max():6.2f}°")
    print(f"  Quat Error:  mean={quat_errors.mean():5.2f}°  max={quat_errors.max():6.2f}°")

    # Pass criteria: tilt < 5°, yaw < 5° (9-axis should NOT drift!)
    if tilt_errors.mean() < 5.0 and yaw_errors.mean() < 5.0:
        status = "✓ PASS"
    elif tilt_errors.mean() < 10.0 and yaw_errors.mean() < 10.0:
        status = "⚠️  PARTIAL"
    else:
        status = "❌ FAIL"

    print(f"  {status}")
    print()

    return {
        'name': name,
        'tilt_mean': tilt_errors.mean(),
        'tilt_max': tilt_errors.max(),
        'roll_mean': roll_errors.mean(),
        'pitch_mean': pitch_errors.mean(),
        'yaw_mean': yaw_errors.mean(),
        'quat_mean': quat_errors.mean(),
        'status': status,
        'duration': duration_s,
    }

def main():
    print("="*80)
    print("TESTING 9-AXIS SENSOR FUSION ON SIMPLE SCENARIOS")
    print("="*80)
    print()

    test_cases = [
        ("Static 10s", "../test/data/datasets/synthetic/static_10s.csv"),
        ("Rotation X (20 dps)", "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv"),
        ("Rotation Y (15 dps)", "../test/data/datasets/synthetic/rotation_y_15dps_10s.csv"),
        ("Rotation Z (30 dps)", "../test/data/datasets/synthetic/rotation_z_30dps_10s.csv"),
        ("Rotation Sequence", "../test/data/datasets/synthetic/rotation_sequence_15s.csv"),
        ("Vibration 5Hz", "../test/data/datasets/synthetic/vibration_5hz_10s.csv"),
        ("High Noise", "../test/data/datasets/synthetic/static_high_noise_10s.csv"),
        ("High Bias", "../test/data/datasets/synthetic/static_high_bias_10s.csv"),
    ]

    results = []

    for name, path in test_cases:
        if os.path.exists(path):
            result = test_dataset(path, name)
            results.append(result)
        else:
            print(f"⚠️  Dataset not found: {path}")
            print()

    print("="*80)
    print("SUMMARY - 9-AXIS FUSION")
    print("="*80)
    print(f"{'Scenario':<20s} {'Duration':>8s} {'Tilt':>8s} {'Yaw':>8s} {'Quat':>8s} {'Status'}")
    print("-" * 80)

    for r in results:
        print(f"{r['name']:<20s} {r['duration']:7.1f}s {r['tilt_mean']:7.2f}° "
              f"{r['yaw_mean']:7.2f}° {r['quat_mean']:7.2f}° {r['status']}")

    print()
    passed = sum(1 for r in results if "PASS" in r['status'])
    total = len(results)
    print(f"Passed: {passed}/{total} ({100*passed/total:.0f}%)")
    print()

    # Compare with 6-axis baseline (expected to have yaw drift)
    print("EXPECTED IMPROVEMENT OVER 6-AXIS:")
    print("-" * 80)
    print("- Yaw drift should be eliminated (6-axis has ~10-30° yaw error)")
    print("- Absolute heading should be accurate")
    print("- Tilt accuracy should be similar to 6-axis (<5°)")

if __name__ == '__main__':
    main()
