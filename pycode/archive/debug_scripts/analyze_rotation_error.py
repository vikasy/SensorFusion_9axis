#!/usr/bin/env python3
"""
Analyze rotation error progression over time to identify root cause.
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
    print("ANALYZING ROTATION ERROR PROGRESSION")
    print("="*80)
    print()

    # Load rotation_x dataset
    dataset_path = "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples from {os.path.basename(dataset_path)}")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Calculate actual bias
    gyro_x_mean = df['gyro_x_counts'].mean() * platform.gyro_scale_factor
    gyro_y_mean = df['gyro_y_counts'].mean() * platform.gyro_scale_factor
    gyro_z_mean = df['gyro_z_counts'].mean() * platform.gyro_scale_factor
    actual_bias = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])

    print(f"Actual bias: [{actual_bias[0]:.4f}, {actual_bias[1]:.4f}, {actual_bias[2]:.4f}] dps")
    print(f"Expected rotation: 20 dps around X-axis")
    print()

    fusion_count = 0
    quat_errors = []
    bias_errors = []
    bias_x_estimates = []
    roll_estimates = []
    roll_gt = []
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

            # Calculate bias error
            bias_estimate_dps = sf.bias_post_s.copy()
            bias_error = np.linalg.norm(bias_estimate_dps - actual_bias)

            quat_errors.append(quat_error_deg)
            bias_errors.append(bias_error)
            bias_x_estimates.append(bias_estimate_dps[0])
            roll_estimates.append(output.orientation[2])  # Roll is 3rd element in [yaw, pitch, roll]
            roll_gt.append(row['gt_roll_deg'])
            timestamps_sec.append(fusion_count * 0.04)  # 25 Hz fusion rate

    print(f"Processed {fusion_count} fusion cycles over {timestamps_sec[-1]:.1f} seconds")
    print()

    # Convert to numpy arrays
    quat_errors = np.array(quat_errors)
    bias_errors = np.array(bias_errors)
    bias_x_estimates = np.array(bias_x_estimates)
    roll_estimates = np.array(roll_estimates)
    roll_gt = np.array(roll_gt)
    timestamps_sec = np.array(timestamps_sec)

    # Analyze error progression in phases
    print("ERROR PROGRESSION BY PHASE:")
    print("-" * 80)

    phases = [
        ("Initial (0-1s)", 0, 1),
        ("Early (1-3s)", 1, 3),
        ("Mid (3-6s)", 3, 6),
        ("Late (6-10s)", 6, 10),
    ]

    for phase_name, t_start, t_end in phases:
        mask = (timestamps_sec >= t_start) & (timestamps_sec < t_end)
        if np.any(mask):
            phase_quat_errors = quat_errors[mask]
            phase_bias_errors = bias_errors[mask]
            phase_bias_x = bias_x_estimates[mask]

            print(f"\n{phase_name} (t={t_start:.1f}s to {t_end:.1f}s):")
            print(f"  Quaternion error: mean={phase_quat_errors.mean():.3f}°, "
                  f"std={phase_quat_errors.std():.3f}°, "
                  f"max={phase_quat_errors.max():.3f}°")
            print(f"  Bias error: mean={phase_bias_errors.mean():.3f} dps, "
                  f"final={phase_bias_errors[-1]:.3f} dps")
            print(f"  Bias X estimate: mean={phase_bias_x.mean():.3f} dps "
                  f"(actual: {actual_bias[0]:.3f} dps)")

    # Check if error is growing or steady
    print()
    print("ERROR TREND:")
    print("-" * 80)

    early_error = quat_errors[25:50].mean()  # 1-2 seconds
    late_error = quat_errors[-25:].mean()     # Last second

    print(f"Early error (1-2s):    {early_error:.3f}°")
    print(f"Late error (9-10s):    {late_error:.3f}°")
    print(f"Change:                {late_error - early_error:+.3f}°")

    if late_error > early_error + 1.0:
        print("⚠️  Error is GROWING - suggests drift or instability")
    elif late_error < early_error - 1.0:
        print("✓ Error is DECREASING - filter is converging")
    else:
        print("→ Error is STEADY - filter has reached equilibrium")

    # Analyze roll angle tracking specifically
    print()
    print("ROLL ANGLE TRACKING:")
    print("-" * 80)

    roll_error = np.abs(roll_estimates - roll_gt)
    print(f"Roll error: mean={roll_error.mean():.3f}°, max={roll_error.max():.3f}°")
    print(f"Final roll: GT={roll_gt[-1]:.2f}°, Est={roll_estimates[-1]:.2f}°, "
          f"error={roll_error[-1]:.2f}°")

    # Check rotation rate
    expected_rotation_rate = 20.0  # dps
    actual_roll_change = roll_gt[-1] - roll_gt[0]
    time_elapsed = timestamps_sec[-1]
    measured_rate = actual_roll_change / time_elapsed

    print()
    print(f"Expected rotation rate: {expected_rotation_rate:.2f} dps")
    print(f"Measured rate (GT): {measured_rate:.2f} dps")
    print(f"Total rotation: {actual_roll_change:.2f}° over {time_elapsed:.2f}s")

    # Check if bias estimate is stable
    print()
    print("BIAS STABILITY:")
    print("-" * 80)

    bias_x_late = bias_x_estimates[-25:].mean()
    bias_x_std_late = bias_x_estimates[-25:].std()
    bias_x_error = abs(bias_x_late - actual_bias[0])

    print(f"Bias X (last 1s): {bias_x_late:.3f} ± {bias_x_std_late:.3f} dps")
    print(f"Actual bias X:    {actual_bias[0]:.3f} dps")
    print(f"Error:            {bias_x_error:.3f} dps")

    if bias_x_std_late > 5.0:
        print("⚠️  High bias variance - filter is unstable")
    elif bias_x_error > 5.0:
        print("⚠️  Large bias error - filter hasn't converged")
    else:
        print("✓ Bias is relatively stable and accurate")

    # Summary
    print()
    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Mean quaternion error: {quat_errors.mean():.3f}° (target: < 5°)")
    print(f"Final quaternion error: {quat_errors[-1]:.3f}°")
    print(f"Mean roll error: {roll_error.mean():.3f}°")
    print(f"Final bias error: {bias_errors[-1]:.3f} dps")
    print()

    if quat_errors.mean() > 10.0:
        print("❌ Large error - likely fundamental algorithm issue")
    elif quat_errors.mean() > 5.0:
        print("⚠️  Moderate error - needs tuning or convergence time")
    else:
        print("✓ Error within acceptable range")

if __name__ == '__main__':
    main()
