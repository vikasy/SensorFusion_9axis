#!/usr/bin/env python3
"""
Test tilt (roll/pitch) accuracy, ignoring yaw drift.
6-axis fusion cannot determine absolute yaw without magnetometer.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
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
    """Test a single dataset, measuring tilt (roll/pitch) accuracy"""

    df = pd.read_csv(dataset_path)
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    fusion_count = 0
    roll_errors = []
    pitch_errors = []
    yaw_errors = []
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

            # Quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))
            quat_errors.append(quat_error_deg)

            # Euler angle errors (with wraparound handling)
            # output.orientation is [yaw, pitch, roll]
            roll_err = abs(angle_diff(output.orientation[2], row['gt_roll_deg']))
            pitch_err = abs(angle_diff(output.orientation[1], row['gt_pitch_deg']))
            yaw_err = abs(angle_diff(output.orientation[0], row['gt_yaw_deg']))

            roll_errors.append(roll_err)
            pitch_errors.append(pitch_err)
            yaw_errors.append(yaw_err)

    roll_errors = np.array(roll_errors)
    pitch_errors = np.array(pitch_errors)
    yaw_errors = np.array(yaw_errors)
    quat_errors = np.array(quat_errors)

    # Calculate tilt error (combined roll + pitch)
    tilt_errors = np.sqrt(roll_errors**2 + pitch_errors**2)

    print(f"{name}")
    print(f"  Fusions: {fusion_count}")
    print(f"  Tilt (Roll+Pitch) Error:")
    print(f"    Mean: {tilt_errors.mean():.3f}°  Max: {tilt_errors.max():.3f}°")
    print(f"  Roll Error:")
    print(f"    Mean: {roll_errors.mean():.3f}°  Max: {roll_errors.max():.3f}°")
    print(f"  Pitch Error:")
    print(f"    Mean: {pitch_errors.mean():.3f}°  Max: {pitch_errors.max():.3f}°")
    print(f"  Yaw Error (expected to drift):")
    print(f"    Mean: {yaw_errors.mean():.3f}°  Max: {yaw_errors.max():.3f}°")
    print(f"  Quaternion Error:")
    print(f"    Mean: {quat_errors.mean():.3f}°  Max: {quat_errors.max():.3f}°")

    # Pass criteria: tilt error < 3°
    if tilt_errors.mean() < 3.0:
        status = "✓ PASS"
    else:
        status = "❌ FAIL"

    print(f"  {status} (tilt accuracy)")
    print()

    return {
        'name': name,
        'tilt_mean': tilt_errors.mean(),
        'roll_mean': roll_errors.mean(),
        'pitch_mean': pitch_errors.mean(),
        'yaw_mean': yaw_errors.mean(),
        'quat_mean': quat_errors.mean(),
        'status': status,
    }

def main():
    print("="*80)
    print("TILT ACCURACY TEST (ROLL/PITCH ONLY)")
    print("="*80)
    print("6-axis fusion (no magnetometer) cannot determine absolute yaw.")
    print("This test focuses on tilt (roll/pitch) accuracy, which is reliable.")
    print()

    test_cases = [
        ("Static 60s", "../test/data/datasets/synthetic/static_60s.csv"),
        ("Rotation X", "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv"),
        ("Rotation Y", "../test/data/datasets/synthetic/rotation_y_15dps_10s.csv"),
        ("Rotation Z", "../test/data/datasets/synthetic/rotation_z_30dps_10s.csv"),
        ("Vibration", "../test/data/datasets/synthetic/vibration_5hz_10s.csv"),
        ("High Noise", "../test/data/datasets/synthetic/static_high_noise_10s.csv"),
        ("High Bias", "../test/data/datasets/synthetic/static_high_bias_10s.csv"),
    ]

    results = []

    for name, path in test_cases:
        if os.path.exists(path):
            result = test_dataset(path, name)
            results.append(result)

    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"{'Dataset':<20s} {'Tilt':>8s} {'Roll':>8s} {'Pitch':>8s} {'Yaw':>8s} {'Status'}")
    print("-" * 80)

    for r in results:
        print(f"{r['name']:<20s} {r['tilt_mean']:7.3f}° {r['roll_mean']:7.3f}° "
              f"{r['pitch_mean']:7.3f}° {r['yaw_mean']:7.3f}° {r['status']}")

    print()
    passed = sum(1 for r in results if "PASS" in r['status'])
    total = len(results)
    print(f"Passed: {passed}/{total} ({100*passed/total:.0f}%)")

if __name__ == '__main__':
    main()
