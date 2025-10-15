#!/usr/bin/env python3
"""
Track quaternion and Euler angle errors over first 100 samples
to observe filter convergence behavior
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi

def wrap_angle(angle):
    """Wrap angle to [-180, 180] range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def main():
    print("="*80)
    print("FILTER CONVERGENCE TRACKING - First 100 Samples")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nDataset: {dataset_path}")
    print(f"Tracking first 100 samples to observe convergence\n")

    # Storage for plotting
    fusion_data = []

    fusion_count = 0
    max_samples = 100

    print(f"{'Fusion':>6} {'Sample':>6} {'Quat Err':>10} {'Yaw Err':>10} {'Pitch Err':>10} {'Roll Err':>10}")
    print(f"{'#':>6} {'Index':>6} {'(deg)':>10} {'(deg)':>10} {'(deg)':>10} {'(deg)':>10}")
    print("-"*66)

    for idx in range(len(df)):
        if idx >= max_samples:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready - fusion will run
            fusion_count += 1

            # Run fusion
            output = sf.run()

            # Get ground truth
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            gt_yaw = row['gt_yaw_deg']
            gt_pitch = row['gt_pitch_deg']
            gt_roll = row['gt_roll_deg']

            # Get Python output
            python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            python_yaw = output.orientation[0]
            python_pitch = output.orientation[1]
            python_roll = output.orientation[2]

            # Compute errors
            quat_error = quaternion_angular_distance(gt_quat, python_quat)
            yaw_err = wrap_angle(python_yaw - gt_yaw)
            pitch_err = wrap_angle(python_pitch - gt_pitch)
            roll_err = wrap_angle(python_roll - gt_roll)

            # Get bias
            bias = sf.bias_post_s.copy()
            bias_mag = np.linalg.norm(bias)

            # Store for analysis
            fusion_data.append({
                'fusion': fusion_count,
                'sample': idx,
                'quat_error': quat_error,
                'yaw_error': yaw_err,
                'pitch_error': pitch_err,
                'roll_error': roll_err,
                'gt_quat': gt_quat,
                'python_quat': python_quat,
                'gt_yaw': gt_yaw,
                'gt_pitch': gt_pitch,
                'gt_roll': gt_roll,
                'python_yaw': python_yaw,
                'python_pitch': python_pitch,
                'python_roll': python_roll,
                'bias': bias,
                'bias_mag': bias_mag
            })

            # Print every fusion for first 10, then every 5th
            if fusion_count <= 10 or fusion_count % 5 == 0:
                print(f"{fusion_count:6d} {idx:6d} {quat_error:10.4f} {yaw_err:+10.4f} {pitch_err:+10.4f} {roll_err:+10.4f}")

    print("-"*66)
    print(f"\nTotal fusion cycles: {fusion_count}")

    # Statistical summary
    print("\n" + "="*80)
    print("CONVERGENCE ANALYSIS")
    print("="*80)

    if len(fusion_data) > 0:
        errors = np.array([d['quat_error'] for d in fusion_data])
        yaw_errors = np.array([d['yaw_error'] for d in fusion_data])
        pitch_errors = np.array([d['pitch_error'] for d in fusion_data])
        roll_errors = np.array([d['roll_error'] for d in fusion_data])
        bias_mags = np.array([d['bias_mag'] for d in fusion_data])

        print(f"\nQuaternion Angular Error (degrees):")
        print(f"  First 10 samples: mean={errors[:10].mean():.4f}°, max={errors[:10].max():.4f}°")
        print(f"  Last 10 samples:  mean={errors[-10:].mean():.4f}°, max={errors[-10:].max():.4f}°")
        print(f"  Overall:          mean={errors.mean():.4f}°, std={errors.std():.4f}°, max={errors.max():.4f}°")

        print(f"\nYaw Error (degrees):")
        print(f"  First 10 samples: mean={yaw_errors[:10].mean():+.4f}°, std={abs(yaw_errors[:10]).std():.4f}°")
        print(f"  Last 10 samples:  mean={yaw_errors[-10:].mean():+.4f}°, std={abs(yaw_errors[-10:]).std():.4f}°")
        print(f"  Overall:          mean={yaw_errors.mean():+.4f}°, std={abs(yaw_errors).std():.4f}°")

        print(f"\nPitch Error (degrees):")
        print(f"  First 10 samples: mean={pitch_errors[:10].mean():+.4f}°, std={abs(pitch_errors[:10]).std():.4f}°")
        print(f"  Last 10 samples:  mean={pitch_errors[-10:].mean():+.4f}°, std={abs(pitch_errors[-10:]).std():.4f}°")
        print(f"  Overall:          mean={pitch_errors.mean():+.4f}°, std={abs(pitch_errors).std():.4f}°")

        print(f"\nRoll Error (degrees):")
        print(f"  First 10 samples: mean={roll_errors[:10].mean():+.4f}°, std={abs(roll_errors[:10]).std():.4f}°")
        print(f"  Last 10 samples:  mean={roll_errors[-10:].mean():+.4f}°, std={abs(roll_errors[-10:]).std():.4f}°")
        print(f"  Overall:          mean={roll_errors.mean():+.4f}°, std={abs(roll_errors).std():.4f}°")

        print(f"\nGyro Bias Magnitude (dps):")
        print(f"  First 10 samples: mean={bias_mags[:10].mean():.6f}, max={bias_mags[:10].max():.6f}")
        print(f"  Last 10 samples:  mean={bias_mags[-10:].mean():.6f}, max={bias_mags[-10:].max():.6f}")
        print(f"  Overall:          mean={bias_mags.mean():.6f}, std={bias_mags.std():.6f}, max={bias_mags.max():.6f}")
        print(f"  Initial:          {bias_mags[0]:.6f} dps")
        print(f"  Final:            {bias_mags[-1]:.6f} dps")
        print(f"  Growth:           {bias_mags[-1] - bias_mags[0]:+.6f} dps")

        # Convergence check
        print("\n" + "="*80)
        print("CONVERGENCE VERDICT")
        print("="*80)

        improvement_quat = errors[0] - errors[-1]
        improvement_pct = (improvement_quat / errors[0]) * 100 if errors[0] > 0 else 0

        print(f"\nQuaternion error:")
        print(f"  Initial (fusion #1):  {errors[0]:.4f}°")
        print(f"  Final (fusion #{fusion_count}): {errors[-1]:.4f}°")
        print(f"  Improvement: {improvement_quat:.4f}° ({improvement_pct:.1f}%)")

        if errors[-1] < 0.1:
            print(f"\n✓ EXCELLENT: Final error < 0.1° - filter has converged well")
        elif errors[-1] < 0.5:
            print(f"\n✓ GOOD: Final error < 0.5° - filter showing good convergence")
        elif errors[-1] < 1.0:
            print(f"\n⚠ ACCEPTABLE: Final error < 1.0° - filter partially converged")
        else:
            print(f"\n✗ POOR: Final error >= 1.0° - filter not converging properly")

        # Check if errors are decreasing
        first_half_mean = errors[:len(errors)//2].mean()
        second_half_mean = errors[len(errors)//2:].mean()

        if second_half_mean < first_half_mean * 0.8:
            print(f"✓ Filter is converging (second half errors 20%+ lower than first half)")
        elif second_half_mean < first_half_mean:
            print(f"⚠ Filter showing slow convergence (second half errors slightly lower)")
        else:
            print(f"✗ Filter not converging (second half errors not improving)")

    print("\n" + "="*80)
    print("TRACKING COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
