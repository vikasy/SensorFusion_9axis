#!/usr/bin/env python3
"""
Batch test Python 9-axis implementation on ALL 10 synthetic datasets.
Generates comprehensive comparison and analysis for each dataset.
"""

import numpy as np
import pandas as pd
import sys
import os
import time
from pathlib import Path

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

def test_dataset(dataset_path, output_dir):
    """Test a single dataset and save results"""

    dataset_name = Path(dataset_path).stem
    print(f"\n{'='*80}")
    print(f"Testing: {dataset_name}")
    print(f"{'='*80}")

    # Load dataset
    df = pd.read_csv(dataset_path)
    print(f"Total samples: {len(df)}")

    # Initialize fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Storage for results
    results = []
    fusion_count = 0
    start_time = time.time()

    # Process all samples
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

    elapsed_time = time.time() - start_time
    print(f"Processed {fusion_count} fusion cycles in {elapsed_time:.2f}s")

    # Convert to DataFrame
    df_results = pd.DataFrame(results)

    # Save detailed results
    output_csv = os.path.join(output_dir, f"{dataset_name}_results.csv")
    df_results.to_csv(output_csv, index=False)
    print(f"Results saved to: {output_csv}")

    # Compute statistics
    quat_errors = df_results['quat_error'].values
    yaw_errors = df_results['yaw_error'].values
    pitch_errors = df_results['pitch_error'].values
    roll_errors = df_results['roll_error'].values
    bias_mags = df_results['bias_mag'].values

    stats = {
        'dataset': dataset_name,
        'samples': len(df),
        'fusion_cycles': fusion_count,
        'processing_time_s': elapsed_time,
        'quat_error_mean': quat_errors.mean(),
        'quat_error_std': quat_errors.std(),
        'quat_error_min': quat_errors.min(),
        'quat_error_max': quat_errors.max(),
        'quat_error_p50': np.percentile(quat_errors, 50),
        'quat_error_p95': np.percentile(quat_errors, 95),
        'quat_error_p99': np.percentile(quat_errors, 99),
        'yaw_error_mean': yaw_errors.mean(),
        'yaw_error_std': np.abs(yaw_errors).std(),
        'yaw_error_rms': np.sqrt(np.mean(yaw_errors**2)),
        'pitch_error_mean': pitch_errors.mean(),
        'pitch_error_std': np.abs(pitch_errors).std(),
        'pitch_error_rms': np.sqrt(np.mean(pitch_errors**2)),
        'roll_error_mean': roll_errors.mean(),
        'roll_error_std': np.abs(roll_errors).std(),
        'roll_error_rms': np.sqrt(np.mean(roll_errors**2)),
        'bias_initial': bias_mags[0] if len(bias_mags) > 0 else 0,
        'bias_final': bias_mags[-1] if len(bias_mags) > 0 else 0,
        'bias_mean': bias_mags.mean(),
        'bias_max': bias_mags.max(),
        'excellent_pct': 100 * np.sum(quat_errors < 0.1) / len(quat_errors),
        'good_pct': 100 * np.sum((quat_errors >= 0.1) & (quat_errors < 0.5)) / len(quat_errors),
        'acceptable_pct': 100 * np.sum((quat_errors >= 0.5) & (quat_errors < 1.0)) / len(quat_errors),
        'poor_pct': 100 * np.sum(quat_errors >= 1.0) / len(quat_errors),
    }

    # Print summary
    print(f"\nQuick Summary:")
    print(f"  Quat Error: {stats['quat_error_mean']:.3f}° ± {stats['quat_error_std']:.3f}° (max: {stats['quat_error_max']:.3f}°)")
    print(f"  Yaw Error:  {stats['yaw_error_mean']:+.3f}° (RMS: {stats['yaw_error_rms']:.3f}°)")
    print(f"  Pitch Error: {stats['pitch_error_mean']:+.3f}° (RMS: {stats['pitch_error_rms']:.3f}°)")
    print(f"  Roll Error:  {stats['roll_error_mean']:+.3f}° (RMS: {stats['roll_error_rms']:.3f}°)")
    print(f"  Bias: {stats['bias_initial']:.4f} → {stats['bias_final']:.4f} dps")
    print(f"  Quality: {stats['good_pct']+stats['excellent_pct']:.1f}% within 0.5°")

    return stats

def main():
    print("="*80)
    print("BATCH TEST - ALL SYNTHETIC DATASETS")
    print("="*80)

    # Define all synthetic datasets
    base_path = "../test/data/datasets/synthetic"
    datasets = [
        "static_10s.csv",
        "static_60s.csv",
        "static_high_bias_10s.csv",
        "static_high_noise_10s.csv",
        "rotation_x_20dps_10s.csv",
        "rotation_y_15dps_10s.csv",
        "rotation_z_30dps_10s.csv",
        "rotation_sequence_15s.csv",
        "complex_motion_20s.csv",
        "vibration_5hz_10s.csv"
    ]

    # Create output directory
    output_dir = "synthetic_test_results"
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nOutput directory: {output_dir}/")

    # Test each dataset
    all_stats = []
    for dataset_file in datasets:
        dataset_path = os.path.join(base_path, dataset_file)

        if not os.path.exists(dataset_path):
            print(f"\n⚠ WARNING: {dataset_file} not found, skipping...")
            continue

        try:
            stats = test_dataset(dataset_path, output_dir)
            all_stats.append(stats)
        except Exception as e:
            print(f"\n✗ ERROR testing {dataset_file}: {e}")
            import traceback
            traceback.print_exc()
            continue

    # Save summary statistics
    df_summary = pd.DataFrame(all_stats)
    summary_csv = os.path.join(output_dir, "summary_all_datasets.csv")
    df_summary.to_csv(summary_csv, index=False)

    print("\n" + "="*80)
    print("BATCH TEST COMPLETE")
    print("="*80)
    print(f"\nTested {len(all_stats)} datasets successfully")
    print(f"Summary saved to: {summary_csv}")
    print(f"Individual results in: {output_dir}/")

    # Print comparison table
    print("\n" + "="*80)
    print("COMPARISON ACROSS ALL DATASETS")
    print("="*80)

    print(f"\n{'Dataset':<30} {'Cycles':<8} {'Mean Error':<12} {'Max Error':<12} {'Final Bias':<12} {'Quality':<10}")
    print("-"*90)
    for stats in all_stats:
        quality = f"{stats['good_pct']+stats['excellent_pct']:.1f}%"
        print(f"{stats['dataset']:<30} {stats['fusion_cycles']:<8} {stats['quat_error_mean']:>10.3f}° {stats['quat_error_max']:>10.3f}° {stats['bias_final']:>10.4f}dps {quality:>9}")

    # Identify best and worst performers
    print("\n" + "="*80)
    print("PERFORMANCE RANKING")
    print("="*80)

    sorted_by_error = sorted(all_stats, key=lambda x: x['quat_error_mean'])
    print(f"\n✓ BEST Accuracy: {sorted_by_error[0]['dataset']}")
    print(f"  Mean error: {sorted_by_error[0]['quat_error_mean']:.3f}°")

    print(f"\n✗ WORST Accuracy: {sorted_by_error[-1]['dataset']}")
    print(f"  Mean error: {sorted_by_error[-1]['quat_error_mean']:.3f}°")

    sorted_by_bias = sorted(all_stats, key=lambda x: x['bias_final'])
    print(f"\n✓ BEST Bias Control: {sorted_by_bias[0]['dataset']}")
    print(f"  Final bias: {sorted_by_bias[0]['bias_final']:.4f} dps")

    print(f"\n✗ WORST Bias Control: {sorted_by_bias[-1]['dataset']}")
    print(f"  Final bias: {sorted_by_bias[-1]['bias_final']:.4f} dps")

    print("\n" + "="*80)
    print("Next step: Run plot_all_synthetic_datasets.py to generate visualizations")
    print("="*80)

if __name__ == '__main__':
    main()
