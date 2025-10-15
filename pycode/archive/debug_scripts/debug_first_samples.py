#!/usr/bin/env python3
"""
Debug Python sensor fusion by examining first few samples in detail.
Compare against ground truth from synthetic static dataset.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def quaternion_distance(q1, q2):
    """Compute minimum distance between quaternions (handles q/-q ambiguity)."""
    d1 = np.linalg.norm(q1 - q2)
    d2 = np.linalg.norm(q1 + q2)
    return min(d1, d2)

def quaternion_angle_difference(q1, q2):
    """Compute angular difference in degrees."""
    q1_norm = q1 / np.linalg.norm(q1)
    q2_norm = q2 / np.linalg.norm(q2)
    dot = np.abs(np.dot(q1_norm, q2_norm))
    dot = np.clip(dot, 0.0, 1.0)
    angle_rad = 2 * np.arccos(dot)
    return np.degrees(angle_rad)

def main():
    print("="*80)
    print("DEBUGGING PYTHON SENSOR FUSION - FIRST SAMPLES")
    print("="*80)
    print()

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    print(f"Loading: {dataset_path}")
    df = pd.read_csv(dataset_path)
    print(f"  Loaded {len(df)} samples")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    print(f"Platform: {platform.platform_name}")
    print(f"  Accel scale: {platform.accel_scale_factor:.10f} g/count")
    print(f"  Gyro scale:  {platform.gyro_scale_factor:.10f} dps/count")
    print()

    print("="*80)
    print("PROCESSING FIRST 20 SAMPLES")
    print("="*80)
    print()

    fusion_count = 0
    max_samples = 100  # Process enough to get ~25 fusion outputs

    for idx in range(min(max_samples, len(df))):
        row = df.iloc[idx]

        # Get sensor data
        timestamp = int(row['timestamp_ns'])

        # Accelerometer
        accel_counts = np.array([
            row['accel_x_counts'],
            row['accel_y_counts'],
            row['accel_z_counts']
        ], dtype=np.float64)

        # Gyroscope
        gyro_counts = np.array([
            row['gyro_x_counts'],
            row['gyro_y_counts'],
            row['gyro_z_counts']
        ], dtype=np.float64)

        # Ground truth
        gt_quat = np.array([
            row['gt_quat_w'],
            row['gt_quat_x'],
            row['gt_quat_y'],
            row['gt_quat_z']
        ])

        # Process accelerometer
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)

        # Process gyroscope
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        # Run fusion if ready
        if ready_gyro:
            output = sf.run()
            python_quat = np.array([output.quat.q0, output.quat.q1,
                                   output.quat.q2, output.quat.q3])

            dist = quaternion_distance(python_quat, gt_quat)
            angle = quaternion_angle_difference(python_quat, gt_quat)

            fusion_count += 1

            print(f"Fusion #{fusion_count} (sample idx={idx}, timestamp={timestamp}):")
            print(f"  Accel counts: [{accel_counts[0]:6.0f}, {accel_counts[1]:6.0f}, {accel_counts[2]:6.0f}]")
            print(f"  Gyro counts:  [{gyro_counts[0]:6.0f}, {gyro_counts[1]:6.0f}, {gyro_counts[2]:6.0f}]")
            print(f"  Ground Truth: [{gt_quat[0]:11.8f}, {gt_quat[1]:11.8f}, {gt_quat[2]:11.8f}, {gt_quat[3]:11.8f}]")
            print(f"  Python:       [{python_quat[0]:11.8f}, {python_quat[1]:11.8f}, {python_quat[2]:11.8f}, {python_quat[3]:11.8f}]")
            print(f"  Distance: {dist:.6e}")
            print(f"  Angle:    {angle:.6f}°")

            # Check if quaternion is negated
            if dist > 1.0:  # Large distance suggests opposite sign
                print(f"  ⚠️  WARNING: Large distance - checking quaternion sign")
                dist_neg = np.linalg.norm(python_quat + gt_quat)
                print(f"  Distance (negated): {dist_neg:.6e}")
                if dist_neg < dist:
                    print(f"  ⚠️  QUATERNION SIGN ISSUE DETECTED!")

            # Check magnitude
            py_mag = np.linalg.norm(python_quat)
            gt_mag = np.linalg.norm(gt_quat)
            print(f"  Magnitude: Python={py_mag:.8f}, GT={gt_mag:.8f}")

            # Show orientation
            print(f"  Euler (output): Yaw={output.orientation[0]:.2f}°, Pitch={output.orientation[1]:.2f}°, Roll={output.orientation[2]:.2f}°")
            print(f"  GT Euler: Roll={row['gt_roll_deg']:.2f}°, Pitch={row['gt_pitch_deg']:.2f}°, Yaw={row['gt_yaw_deg']:.2f}°")

            if angle > 10.0:
                print(f"  ❌ ERROR: Angular difference > 10°!")
            elif angle > 1.0:
                print(f"  ⚠️  WARNING: Angular difference > 1°")
            else:
                print(f"  ✓ OK: Angular difference < 1°")
            print()

            if fusion_count >= 20:
                break

    print("="*80)
    print("SUMMARY OF FIRST 20 FUSION OUTPUTS")
    print("="*80)
    print()
    print(f"Total fusion outputs: {fusion_count}")
    print()
    print("Key observations to look for:")
    print("  1. Is initial quaternion correct? Should be close to [1, 0, 0, 0] for static")
    print("  2. Are quaternions consistently negated? (distance > 1.0)")
    print("  3. Are quaternion magnitudes normalized? (should be 1.0)")
    print("  4. Does error grow over time or stay constant?")
    print("  5. Are sensor readings being interpreted correctly?")
    print()

if __name__ == '__main__':
    main()
