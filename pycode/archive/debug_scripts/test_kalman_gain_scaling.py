#!/usr/bin/env python3
"""
Test effect of artificially scaling Kalman gain on filter convergence
Compare baseline (1x) vs 2x vs 10x gain scaling
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

def run_fusion_with_gain_scale(gain_scale: float, max_samples: int = 100):
    """
    Run sensor fusion with scaled Kalman gain

    Args:
        gain_scale: Multiplier for Kalman gain (1.0 = baseline, 2.0 = double, etc.)
        max_samples: Number of samples to process

    Returns:
        List of error dictionaries for each fusion cycle
    """
    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Monkey-patch to scale Kalman gain
    original_meas_update = sf.measurement_update
    original_mag_update = sf.measurement_update_mag

    def scaled_meas_update(debug=False):
        """Measurement update with scaled Kalman gain"""
        original_meas_update(debug=debug)
        # Scale the Kalman gain after computation
        sf.kalman_gain *= gain_scale

    def scaled_mag_update():
        """Mag measurement update with scaled Kalman gain"""
        original_mag_update()
        # Scale the Kalman gain after computation
        sf.kalman_gain *= gain_scale

    # Only apply scaling if not baseline
    if abs(gain_scale - 1.0) > 1e-6:
        sf.measurement_update = scaled_meas_update
        sf.measurement_update_mag = scaled_mag_update

    fusion_data = []
    fusion_count = 0

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

            fusion_data.append({
                'fusion': fusion_count,
                'sample': idx,
                'quat_error': quat_error,
                'yaw_error': yaw_err,
                'pitch_error': pitch_err,
                'roll_error': roll_err
            })

    return fusion_data

def main():
    print("="*80)
    print("KALMAN GAIN SCALING TEST")
    print("="*80)
    print("\nTesting effect of artificially scaling Kalman gain on convergence")
    print("Static dataset - errors should converge to near-zero\n")

    gain_scales = [1.0, 2.0, 10.0]
    max_samples = 100

    results = {}

    for gain_scale in gain_scales:
        print(f"\n{'='*80}")
        print(f"Running with gain scale = {gain_scale}x")
        print(f"{'='*80}")

        fusion_data = run_fusion_with_gain_scale(gain_scale, max_samples)
        results[gain_scale] = fusion_data

        if len(fusion_data) > 0:
            errors = np.array([d['quat_error'] for d in fusion_data])
            yaw_errors = np.array([d['yaw_error'] for d in fusion_data])
            pitch_errors = np.array([d['pitch_error'] for d in fusion_data])
            roll_errors = np.array([d['roll_error'] for d in fusion_data])

            print(f"\nQuaternion Angular Error:")
            print(f"  Initial (fusion #1):  {errors[0]:.4f}°")
            print(f"  Final (fusion #{len(fusion_data)}): {errors[-1]:.4f}°")
            print(f"  Improvement: {errors[0] - errors[-1]:.4f}° ({(errors[0] - errors[-1])/errors[0]*100:.1f}%)")
            print(f"  Mean: {errors.mean():.4f}°, Std: {errors.std():.4f}°, Max: {errors.max():.4f}°")

            print(f"\nEuler Angle Errors (final):")
            print(f"  Yaw:   {yaw_errors[-1]:+.4f}° (drift from initial: {abs(yaw_errors[-1] - yaw_errors[0]):.4f}°)")
            print(f"  Pitch: {pitch_errors[-1]:+.4f}° (drift from initial: {abs(pitch_errors[-1] - pitch_errors[0]):.4f}°)")
            print(f"  Roll:  {roll_errors[-1]:+.4f}° (drift from initial: {abs(roll_errors[-1] - roll_errors[0]):.4f}°)")

            # Convergence check
            first_half_mean = errors[:len(errors)//2].mean()
            second_half_mean = errors[len(errors)//2:].mean()

            if errors[-1] < 0.1:
                verdict = "✓ EXCELLENT: < 0.1°"
            elif errors[-1] < 0.5:
                verdict = "✓ GOOD: < 0.5°"
            elif errors[-1] < 1.0:
                verdict = "⚠ ACCEPTABLE: < 1.0°"
            else:
                verdict = "✗ POOR: >= 1.0°"

            if second_half_mean < first_half_mean * 0.8:
                conv_status = "✓ Converging"
            elif second_half_mean < first_half_mean:
                conv_status = "⚠ Slow convergence"
            else:
                conv_status = "✗ Diverging"

            print(f"\nVerdict: {verdict}, {conv_status}")

    # Comparison summary
    print("\n" + "="*80)
    print("COMPARISON SUMMARY")
    print("="*80)

    print(f"\n{'Gain Scale':>12} {'Initial Error':>15} {'Final Error':>15} {'Improvement':>15} {'Verdict':>20}")
    print("-"*80)

    for gain_scale in gain_scales:
        data = results[gain_scale]
        if len(data) > 0:
            errors = np.array([d['quat_error'] for d in data])
            initial = errors[0]
            final = errors[-1]
            improvement = initial - final
            improvement_pct = (improvement / initial) * 100

            if final < 0.5:
                verdict = "✓ Good"
            elif final < 1.0:
                verdict = "⚠ Acceptable"
            else:
                verdict = "✗ Poor"

            print(f"{gain_scale:>12.1f}x {initial:>14.4f}° {final:>14.4f}° {improvement:>+14.4f}° {verdict:>20}")

    # Recommendation
    print("\n" + "="*80)
    print("ANALYSIS")
    print("="*80)

    best_scale = min(gain_scales, key=lambda s: np.array([d['quat_error'] for d in results[s]])[-1])
    best_final_error = np.array([d['quat_error'] for d in results[best_scale]])[-1]

    print(f"\nBest performing gain scale: {best_scale}x (final error: {best_final_error:.4f}°)")

    if best_scale == 1.0:
        print("\nConclusion: Baseline gain is optimal.")
        print("The divergence issue is NOT due to weak measurement corrections.")
        print("Need to investigate:")
        print("  1. Gyro bias estimation errors")
        print("  2. Magnetometer reference initialization")
        print("  3. Process/measurement noise tuning")
    elif best_scale > 1.0:
        print(f"\nConclusion: Increasing gain by {best_scale}x improves performance.")
        print("This suggests measurement corrections are too weak (gains too conservative).")
        print("Recommendations:")
        print(f"  1. Increase measurement trust (reduce measurement noise variance)")
        print(f"  2. Decrease process trust (increase process noise variance)")
        print(f"  3. Optimal scaling factor appears to be ~{best_scale}x")

    print("\n" + "="*80)
    print("TEST COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
