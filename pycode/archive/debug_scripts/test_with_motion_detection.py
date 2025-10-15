#!/usr/bin/env python3
"""
Test sensor fusion with motion-adaptive Q_bias.
When motion is detected, reduce Q_bias to prevent wild bias swings.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def detect_motion(gyro_dps, threshold=5.0):
    """Detect if significant motion is present"""
    gyro_mag = np.linalg.norm(gyro_dps)
    return gyro_mag > threshold

def test_with_adaptive_q(dataset_path):
    """Test with motion-adaptive Q_bias"""

    df = pd.read_csv(dataset_path)
    print(f"Testing: {os.path.basename(dataset_path)}")
    print(f"Samples: {len(df)}")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    Q_BIAS_STATIC = 100.0   # High Q during static for fast convergence
    Q_BIAS_DYNAMIC = 10.0   # Low Q during motion for stability

    fusion_count = 0
    quat_errors = []
    motion_states = []

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
            # Adapt Q_bias based on motion
            gyro_dps = gyro_counts * platform.gyro_scale_factor
            in_motion = detect_motion(gyro_dps, threshold=5.0)

            if in_motion:
                sf.proc_noise_var_bias = Q_BIAS_DYNAMIC
            else:
                sf.proc_noise_var_bias = Q_BIAS_STATIC

            output = sf.run()
            fusion_count += 1

            # Calculate quaternion error
            est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_dot = np.abs(np.dot(est_quat, gt_quat))
            quat_dot = np.clip(quat_dot, 0.0, 1.0)
            quat_error_deg = 2 * np.degrees(np.arccos(quat_dot))

            quat_errors.append(quat_error_deg)
            motion_states.append(in_motion)

            if fusion_count % 50 == 0 or fusion_count <= 5:
                motion_str = "MOTION" if in_motion else "STATIC"
                q_bias = sf.proc_noise_var_bias
                print(f"Fusion #{fusion_count:3d}: Q_bias={q_bias:6.1f}, {motion_str:6s}, "
                      f"Quat error={quat_error_deg:6.2f}°")

    quat_errors = np.array(quat_errors)
    motion_states = np.array(motion_states)

    print()
    print(f"Processed {fusion_count} fusions")
    print(f"Motion detected: {motion_states.sum()}/{len(motion_states)} fusions ({100*motion_states.sum()/len(motion_states):.1f}%)")
    print()
    print(f"Mean quat error: {quat_errors.mean():.2f}°")
    print(f"Max quat error:  {quat_errors.max():.2f}°")
    print()

    if quat_errors.mean() < 5.0:
        print("✓ PASS")
    else:
        print("❌ FAIL")

    return quat_errors.mean()

def main():
    print("="*80)
    print("TESTING MOTION-ADAPTIVE Q_BIAS")
    print("="*80)
    print()

    datasets = [
        "../test/data/datasets/synthetic/rotation_x_20dps_10s.csv",
        "../test/data/datasets/synthetic/rotation_sequence_15s.csv",
        "../test/data/datasets/synthetic/complex_motion_20s.csv",
    ]

    results = {}
    for dataset in datasets:
        if os.path.exists(dataset):
            mean_error = test_with_adaptive_q(dataset)
            results[os.path.basename(dataset)] = mean_error
            print("="*80)
            print()

    print("SUMMARY:")
    print("-" * 80)
    for name, error in results.items():
        status = "✓ PASS" if error < 5.0 else "❌ FAIL"
        print(f"{name:40s}: {error:6.2f}° {status}")

if __name__ == '__main__':
    main()
