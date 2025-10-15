#!/usr/bin/env python3
"""
Debug complex motion failures.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("DEBUGGING COMPLEX MOTION FAILURES")
    print("="*80)
    print()

    # Load rotation_sequence dataset
    dataset_path = "../test/data/datasets/synthetic/rotation_sequence_15s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples from {os.path.basename(dataset_path)}")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    print("Processing fusion cycles...")
    print()

    fusion_count = 0
    quat_errors = []
    roll_values = []
    pitch_values = []
    yaw_values = []
    gt_roll = []
    gt_pitch = []
    gt_yaw = []
    timestamps_sec = []

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

            # Calculate quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            quat_errors.append(quat_error_deg)
            roll_values.append(output.orientation[2])  # [yaw, pitch, roll]
            pitch_values.append(output.orientation[1])
            yaw_values.append(output.orientation[0])
            gt_roll.append(row['gt_roll_deg'])
            gt_pitch.append(row['gt_pitch_deg'])
            gt_yaw.append(row['gt_yaw_deg'])
            timestamps_sec.append(fusion_count * 0.04)

            # Print key moments
            if fusion_count in [1, 25, 50, 75, 100, 125, 150, 175, 200, 250, 300, 350]:
                print(f"Fusion #{fusion_count:3d} (t={timestamps_sec[-1]:.1f}s):")
                print(f"  GT Euler:  R={row['gt_roll_deg']:6.1f}° P={row['gt_pitch_deg']:6.1f}° Y={row['gt_yaw_deg']:6.1f}°")
                print(f"  Est Euler: R={output.orientation[2]:6.1f}° P={output.orientation[1]:6.1f}° Y={output.orientation[0]:6.1f}°")
                print(f"  Quat error: {quat_error_deg:6.2f}°")
                print()

    print(f"\nProcessed {fusion_count} fusion cycles over {timestamps_sec[-1]:.1f} seconds")
    print()

    # Convert to numpy arrays
    quat_errors = np.array(quat_errors)
    roll_values = np.array(roll_values)
    pitch_values = np.array(pitch_values)
    yaw_values = np.array(yaw_values)
    gt_roll = np.array(gt_roll)
    gt_pitch = np.array(gt_pitch)
    gt_yaw = np.array(gt_yaw)

    # Find where error starts to explode
    print("ERROR EXPLOSION ANALYSIS:")
    print("-" * 80)

    for i in range(1, len(quat_errors)):
        if quat_errors[i] > 10.0 and quat_errors[i-1] < 10.0:
            print(f"\nError explosion at fusion #{i+1} (t={timestamps_sec[i]:.1f}s):")
            print(f"  Previous error: {quat_errors[i-1]:.2f}°")
            print(f"  Current error:  {quat_errors[i]:.2f}°")
            print(f"  GT Euler:  R={gt_roll[i]:6.1f}° P={gt_pitch[i]:6.1f}° Y={gt_yaw[i]:6.1f}°")
            print(f"  Est Euler: R={roll_values[i]:6.1f}° P={pitch_values[i]:6.1f}° Y={yaw_values[i]:6.1f}°")

            # Check for gimbal lock
            if abs(gt_pitch[i]) > 80.0:
                print(f"  ⚠️  GIMBAL LOCK: Pitch = {gt_pitch[i]:.1f}° (near ±90°)")

            break

    # Check pitch angle range
    print()
    print("PITCH ANGLE ANALYSIS:")
    print("-" * 80)
    print(f"GT Pitch range: [{gt_pitch.min():.1f}°, {gt_pitch.max():.1f}°]")
    print(f"Est Pitch range: [{pitch_values.min():.1f}°, {pitch_values.max():.1f}°]")

    if gt_pitch.max() > 60.0 or gt_pitch.min() < -60.0:
        print("⚠️  Large pitch angles detected (>60°) - potential gimbal lock issues")

    # Statistics by time segments
    print()
    print("ERROR BY TIME SEGMENT:")
    print("-" * 80)

    segments = [
        (0, 3, "0-3s"),
        (3, 6, "3-6s"),
        (6, 9, "6-9s"),
        (9, 12, "9-12s"),
        (12, 15, "12-15s"),
    ]

    for t_start, t_end, label in segments:
        mask = (np.array(timestamps_sec) >= t_start) & (np.array(timestamps_sec) < t_end)
        if np.any(mask):
            seg_errors = quat_errors[mask]
            print(f"{label}: mean={seg_errors.mean():6.2f}°, max={seg_errors.max():6.2f}°")

if __name__ == '__main__':
    main()
