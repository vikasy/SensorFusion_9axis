#!/usr/bin/env python3
"""
Test Python 9-axis implementation against ground truth for ALL samples in the dataset.
Generates comprehensive comparison and plots.
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
    print("FULL DATASET TEST - Python vs Ground Truth")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    print(f"\nDataset: {dataset_path}")
    print(f"Total samples in dataset: {len(df)}")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Storage for results
    results = []
    fusion_count = 0

    print(f"\nProcessing all {len(df)} samples...")

    for idx in range(len(df)):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Fusion runs when gyro buffer is ready
            fusion_count += 1
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

            results.append({
                'fusion': fusion_count,
                'sample': idx,
                'timestamp_ns': timestamp,
                'quat_error': quat_error,
                'yaw_error': yaw_err,
                'pitch_error': pitch_err,
                'roll_error': roll_err,
                'gt_quat_w': gt_quat[0],
                'gt_quat_x': gt_quat[1],
                'gt_quat_y': gt_quat[2],
                'gt_quat_z': gt_quat[3],
                'py_quat_w': python_quat[0],
                'py_quat_x': python_quat[1],
                'py_quat_y': python_quat[2],
                'py_quat_z': python_quat[3],
                'gt_yaw': gt_yaw,
                'gt_pitch': gt_pitch,
                'gt_roll': gt_roll,
                'py_yaw': python_yaw,
                'py_pitch': python_pitch,
                'py_roll': python_roll,
                'bias_x': bias[0],
                'bias_y': bias[1],
                'bias_z': bias[2],
                'bias_mag': bias_mag
            })

            # Progress indicator
            if fusion_count % 100 == 0:
                print(f"  Processed {fusion_count} fusion cycles (sample {idx}/{len(df)})...")

    print(f"\nCompleted! Total fusion cycles: {fusion_count}")

    # Convert to DataFrame for analysis
    df_results = pd.DataFrame(results)

    # Save detailed results to CSV
    output_csv = "full_dataset_comparison_results.csv"
    df_results.to_csv(output_csv, index=False)
    print(f"\nDetailed results saved to: {output_csv}")

    # Statistical Analysis
    print("\n" + "="*80)
    print("STATISTICAL ANALYSIS")
    print("="*80)

    quat_errors = df_results['quat_error'].values
    yaw_errors = df_results['yaw_error'].values
    pitch_errors = df_results['pitch_error'].values
    roll_errors = df_results['roll_error'].values
    bias_mags = df_results['bias_mag'].values

    print(f"\n{'Metric':<30} {'Mean':<12} {'Std':<12} {'Min':<12} {'Max':<12}")
    print("-"*80)
    print(f"{'Quaternion Error (deg)':<30} {quat_errors.mean():11.4f}° {quat_errors.std():11.4f}° {quat_errors.min():11.4f}° {quat_errors.max():11.4f}°")
    print(f"{'Yaw Error (deg)':<30} {yaw_errors.mean():+11.4f}° {np.abs(yaw_errors).std():11.4f}° {yaw_errors.min():+11.4f}° {yaw_errors.max():+11.4f}°")
    print(f"{'Pitch Error (deg)':<30} {pitch_errors.mean():+11.4f}° {np.abs(pitch_errors).std():11.4f}° {pitch_errors.min():+11.4f}° {pitch_errors.max():+11.4f}°")
    print(f"{'Roll Error (deg)':<30} {roll_errors.mean():+11.4f}° {np.abs(roll_errors).std():11.4f}° {roll_errors.min():+11.4f}° {roll_errors.max():+11.4f}°")
    print(f"{'Gyro Bias (dps)':<30} {bias_mags.mean():11.6f}  {bias_mags.std():11.6f}  {bias_mags.min():11.6f}  {bias_mags.max():11.6f} ")

    # Percentile analysis
    print("\n" + "="*80)
    print("PERCENTILE ANALYSIS")
    print("="*80)

    percentiles = [50, 90, 95, 99, 99.9]
    print(f"\n{'Metric':<30}", end="")
    for p in percentiles:
        print(f" {f'p{p}':<10}", end="")
    print()
    print("-"*80)

    print(f"{'Quaternion Error (deg)':<30}", end="")
    for p in percentiles:
        print(f" {np.percentile(quat_errors, p):10.4f}°", end="")
    print()

    print(f"{'|Yaw Error| (deg)':<30}", end="")
    for p in percentiles:
        print(f" {np.percentile(np.abs(yaw_errors), p):10.4f}°", end="")
    print()

    print(f"{'|Pitch Error| (deg)':<30}", end="")
    for p in percentiles:
        print(f" {np.percentile(np.abs(pitch_errors), p):10.4f}°", end="")
    print()

    print(f"{'|Roll Error| (deg)':<30}", end="")
    for p in percentiles:
        print(f" {np.percentile(np.abs(roll_errors), p):10.4f}°", end="")
    print()

    # Convergence analysis
    print("\n" + "="*80)
    print("CONVERGENCE ANALYSIS")
    print("="*80)

    # Split into quarters
    n = len(quat_errors)
    q1_errors = quat_errors[:n//4]
    q2_errors = quat_errors[n//4:n//2]
    q3_errors = quat_errors[n//2:3*n//4]
    q4_errors = quat_errors[3*n//4:]

    print(f"\n{'Period':<20} {'Mean Quat Error':<20} {'Mean Bias':<20}")
    print("-"*60)
    print(f"{'First Quarter':<20} {q1_errors.mean():18.4f}° {bias_mags[:n//4].mean():18.6f} dps")
    print(f"{'Second Quarter':<20} {q2_errors.mean():18.4f}° {bias_mags[n//4:n//2].mean():18.6f} dps")
    print(f"{'Third Quarter':<20} {q3_errors.mean():18.4f}° {bias_mags[n//2:3*n//4].mean():18.6f} dps")
    print(f"{'Fourth Quarter':<20} {q4_errors.mean():18.4f}° {bias_mags[3*n//4:].mean():18.6f} dps")

    improvement = (q1_errors.mean() - q4_errors.mean()) / q1_errors.mean() * 100
    print(f"\nImprovement from first to last quarter: {improvement:+.1f}%")

    # Quality metrics
    print("\n" + "="*80)
    print("QUALITY ASSESSMENT")
    print("="*80)

    excellent = np.sum(quat_errors < 0.1)
    good = np.sum((quat_errors >= 0.1) & (quat_errors < 0.5))
    acceptable = np.sum((quat_errors >= 0.5) & (quat_errors < 1.0))
    poor = np.sum(quat_errors >= 1.0)

    print(f"\nQuaternion Error Distribution:")
    print(f"  Excellent (<0.1°):  {excellent:5d} samples ({100*excellent/n:5.1f}%)")
    print(f"  Good (0.1-0.5°):    {good:5d} samples ({100*good/n:5.1f}%)")
    print(f"  Acceptable (0.5-1°): {acceptable:5d} samples ({100*acceptable/n:5.1f}%)")
    print(f"  Poor (≥1.0°):       {poor:5d} samples ({100*poor/n:5.1f}%)")

    if poor == 0:
        print("\n✓ EXCELLENT: All samples within 1.0° accuracy!")
    elif poor < n * 0.01:
        print("\n✓ GOOD: >99% of samples within 1.0° accuracy")
    elif poor < n * 0.05:
        print("\n⚠ ACCEPTABLE: >95% of samples within 1.0° accuracy")
    else:
        print("\n✗ NEEDS IMPROVEMENT: Significant number of samples exceed 1.0° error")

    # Final verdict
    print("\n" + "="*80)
    print("FINAL VERDICT")
    print("="*80)

    mean_error = quat_errors.mean()
    max_error = quat_errors.max()
    final_bias = bias_mags[-1]

    print(f"\nMean Quaternion Error: {mean_error:.4f}°")
    print(f"Max Quaternion Error:  {max_error:.4f}°")
    print(f"Final Gyro Bias:       {final_bias:.6f} dps")

    if mean_error < 0.5 and max_error < 1.5 and final_bias < 1.0:
        print("\n✓✓✓ EXCELLENT PERFORMANCE ✓✓✓")
        print("The filter converges well and maintains high accuracy!")
    elif mean_error < 1.0 and max_error < 2.0 and final_bias < 5.0:
        print("\n✓ GOOD PERFORMANCE ✓")
        print("The filter shows acceptable accuracy for most applications.")
    else:
        print("\n⚠ PERFORMANCE NEEDS IMPROVEMENT ⚠")
        print("Consider further tuning or investigation.")

    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)
    print(f"\nResults saved to: {output_csv}")
    print("Run plot_comparison.py to generate visualizations.")

if __name__ == '__main__':
    main()
