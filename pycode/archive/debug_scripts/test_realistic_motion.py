#!/usr/bin/env python3
"""
Test Python sensor fusion on realistic motion scenarios.
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
    """Test a single realistic motion dataset"""

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

    # Pass criteria: tilt < 5° mean
    if tilt_errors.mean() < 5.0:
        status = "✓ PASS"
    elif tilt_errors.mean() < 10.0:
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
    print("TESTING REALISTIC MOTION SCENARIOS")
    print("="*80)
    print()

    test_cases = [
        ("Climbing Stairs", "../test/data/datasets/realistic/climbing_stairs.csv"),
        ("Driving in Car", "../test/data/datasets/realistic/driving_in_car.csv"),
        ("Flying Drone", "../test/data/datasets/realistic/flying_drone.csv"),
        ("Walking", "../test/data/datasets/realistic/walking.csv"),
        ("Handheld Device", "../test/data/datasets/realistic/handheld_device.csv"),
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
    print("SUMMARY")
    print("="*80)
    print(f"{'Scenario':<20s} {'Duration':>8s} {'Tilt':>8s} {'Roll':>8s} {'Pitch':>8s} {'Yaw':>8s} {'Status'}")
    print("-" * 80)

    for r in results:
        print(f"{r['name']:<20s} {r['duration']:7.1f}s {r['tilt_mean']:7.2f}° "
              f"{r['roll_mean']:7.2f}° {r['pitch_mean']:7.2f}° {r['yaw_mean']:7.2f}° {r['status']}")

    print()
    passed = sum(1 for r in results if "PASS" in r['status'])
    total = len(results)
    print(f"Passed: {passed}/{total} ({100*passed/total:.0f}%)")

if __name__ == '__main__':
    main()
