#!/usr/bin/env python3
"""
Validate Python sensor fusion against synthetic datasets with known ground truth.

Uses test/data/datasets/synthetic/*.csv which contain:
- Realistic sensor data (with noise and bias)
- Known ground truth quaternions and Euler angles
- Multiple motion scenarios
"""

import numpy as np
import pandas as pd
import sys
import os
from pathlib import Path

# Add pycode to path
sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def load_synthetic_dataset(filepath):
    """
    Load synthetic dataset CSV file.

    Returns:
        DataFrame with columns: timestamp_ns, accel_x/y/z_counts, gyro_x/y/z_counts,
                               mag_x/y/z_counts, gt_roll/pitch/yaw_deg, gt_quat_w/x/y/z
    """
    df = pd.read_csv(filepath)
    print(f"  Loaded {len(df)} samples")
    print(f"  Columns: {list(df.columns[:5])}... (showing first 5)")
    return df

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

def run_fusion_on_dataset(df, platform):
    """
    Run Python sensor fusion on dataset.

    Returns:
        List of dicts with: timestamp, python_quat, ground_truth_quat, sample_idx
    """
    # Initialize sensor fusion
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    results = []
    fusion_count = 0

    for idx, row in df.iterrows():
        # Get sensor data
        timestamp = int(row['timestamp_ns'])

        # Accelerometer (sensor_id = 0)
        accel_counts = np.array([
            row['accel_x_counts'],
            row['accel_y_counts'],
            row['accel_z_counts']
        ], dtype=np.float64)

        ready = sf.preprocess_sensor_data(0, accel_counts, timestamp)

        # Gyroscope (sensor_id = 1)
        gyro_counts = np.array([
            row['gyro_x_counts'],
            row['gyro_y_counts'],
            row['gyro_z_counts']
        ], dtype=np.float64)

        ready = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        # Run fusion if ready
        if ready:
            output = sf.run()
            python_quat = np.array([output.quat.q0, output.quat.q1,
                                   output.quat.q2, output.quat.q3])

            # Ground truth quaternion (w, x, y, z)
            gt_quat = np.array([
                row['gt_quat_w'],
                row['gt_quat_x'],
                row['gt_quat_y'],
                row['gt_quat_z']
            ])

            results.append({
                'timestamp': timestamp,
                'python_quat': python_quat,
                'gt_quat': gt_quat,
                'sample_idx': idx,
                'gt_roll': row['gt_roll_deg'],
                'gt_pitch': row['gt_pitch_deg'],
                'gt_yaw': row['gt_yaw_deg']
            })
            fusion_count += 1

            if fusion_count % 100 == 0:
                print(f"    Processed {fusion_count} fusion runs...")

    return results

def analyze_results(results, dataset_name):
    """
    Analyze fusion results against ground truth.
    """
    print(f"\n{'='*80}")
    print(f"ANALYSIS: {dataset_name}")
    print(f"{'='*80}\n")

    if len(results) == 0:
        print("ERROR: No fusion outputs generated!")
        return None

    distances = []
    angles = []

    for result in results:
        py_quat = result['python_quat']
        gt_quat = result['gt_quat']

        dist = quaternion_distance(py_quat, gt_quat)
        angle = quaternion_angle_difference(py_quat, gt_quat)

        distances.append(dist)
        angles.append(angle)

    distances = np.array(distances)
    angles = np.array(angles)

    print(f"Samples analyzed: {len(results)}")
    print()

    print(f"Quaternion Distance Statistics:")
    print(f"  Mean:        {np.mean(distances):.6e}")
    print(f"  Median:      {np.median(distances):.6e}")
    print(f"  Std Dev:     {np.std(distances):.6e}")
    print(f"  Min:         {np.min(distances):.6e}")
    print(f"  Max:         {np.max(distances):.6e}")
    print(f"  95th %ile:   {np.percentile(distances, 95):.6e}")
    print(f"  99th %ile:   {np.percentile(distances, 99):.6e}")
    print()

    print(f"Angular Difference Statistics (degrees):")
    print(f"  Mean:        {np.mean(angles):.6f}°")
    print(f"  Median:      {np.median(angles):.6f}°")
    print(f"  Std Dev:     {np.std(angles):.6f}°")
    print(f"  Min:         {np.min(angles):.6f}°")
    print(f"  Max:         {np.max(angles):.6f}°")
    print(f"  95th %ile:   {np.percentile(angles, 95):.6f}°")
    print(f"  99th %ile:   {np.percentile(angles, 99):.6f}°")
    print()

    # Accuracy distribution
    thresholds = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.1, 0.5]
    print("Accuracy Distribution (quaternion distance):")
    for thresh in thresholds:
        count = np.sum(distances <= thresh)
        percent = 100.0 * count / len(distances)
        print(f"  Distance <= {thresh:.0e}: {count:4d} / {len(distances)} ({percent:.1f}%)")
    print()

    # Angular accuracy
    angle_thresholds = [0.1, 0.5, 1.0, 5.0, 10.0]
    print("Angular Accuracy Distribution (degrees):")
    for thresh in angle_thresholds:
        count = np.sum(angles <= thresh)
        percent = 100.0 * count / len(angles)
        print(f"  Angle <= {thresh:5.1f}°: {count:4d} / {len(angles)} ({percent:.1f}%)")
    print()

    # Show worst mismatches
    worst_indices = np.argsort(angles)[-5:][::-1]
    print("Top 5 Worst Mismatches:")
    for idx in worst_indices:
        result = results[idx]
        print(f"  Sample {result['sample_idx']:4d}: angle={angles[idx]:.6f}°, dist={distances[idx]:.6e}")
        print(f"    Ground Truth: [{result['gt_quat'][0]:.10f}, {result['gt_quat'][1]:.10f}, "
              f"{result['gt_quat'][2]:.10f}, {result['gt_quat'][3]:.10f}]")
        print(f"    Python:       [{result['python_quat'][0]:.10f}, {result['python_quat'][1]:.10f}, "
              f"{result['python_quat'][2]:.10f}, {result['python_quat'][3]:.10f}]")
        print(f"    GT Euler: Roll={result['gt_roll']:.2f}°, Pitch={result['gt_pitch']:.2f}°, Yaw={result['gt_yaw']:.2f}°")
    print()

    # Assessment
    mean_angle = np.mean(angles)
    max_angle = np.max(angles)

    print(f"{'='*80}")
    print(f"ASSESSMENT: {dataset_name}")
    print(f"{'='*80}")
    print()

    if max_angle < 1.0:
        status = "✓ EXCELLENT"
        assessment = "Python implementation matches ground truth within acceptable tolerance."
    elif max_angle < 5.0:
        status = "✓ GOOD"
        assessment = "Python implementation is reasonably accurate but could be improved."
    elif max_angle < 20.0:
        status = "⚠ FAIR"
        assessment = "Python implementation has noticeable errors. Investigation recommended."
    else:
        status = "✗ POOR"
        assessment = "Python implementation has significant errors. Debugging required."

    print(f"{status}: Mean={mean_angle:.2f}°, Max={max_angle:.2f}°")
    print(f"  {assessment}")
    print()

    return {
        'dataset': dataset_name,
        'samples': len(results),
        'mean_distance': np.mean(distances),
        'mean_angle': mean_angle,
        'max_angle': max_angle,
        'distances': distances,
        'angles': angles
    }

def main():
    print("="*80)
    print("PYTHON SENSOR FUSION VALIDATION - SYNTHETIC DATASETS")
    print("="*80)
    print()

    # Initialize platform
    platform = SensorPlatform(INVENSENSE)
    print(f"Platform: {platform.platform_name}")
    print(f"  Accel scale: {platform.accel_scale_factor:.10f} g/count")
    print(f"  Gyro scale:  {platform.gyro_scale_factor:.10f} dps/count")
    print()

    # Find synthetic datasets
    datasets_dir = Path("../test/data/datasets/synthetic")
    if not datasets_dir.exists():
        print(f"ERROR: Datasets directory not found: {datasets_dir}")
        return

    # Select datasets to test
    test_datasets = [
        "static_60s.csv",           # Simple static case
        "rotation_x_20dps_10s.csv", # Single axis rotation
        "rotation_y_15dps_10s.csv",
        "rotation_z_30dps_10s.csv",
        "rotation_sequence_15s.csv", # Multi-axis rotations
    ]

    all_results = {}

    for dataset_file in test_datasets:
        dataset_path = datasets_dir / dataset_file

        if not dataset_path.exists():
            print(f"⚠ Skipping {dataset_file} (not found)")
            continue

        print(f"\n{'='*80}")
        print(f"TESTING: {dataset_file}")
        print(f"{'='*80}")

        # Load dataset
        print(f"\nLoading dataset...")
        df = load_synthetic_dataset(dataset_path)

        # Run fusion
        print(f"\nRunning sensor fusion...")
        results = run_fusion_on_dataset(df, platform)
        print(f"  Generated {len(results)} fusion outputs")

        # Analyze
        analysis = analyze_results(results, dataset_file)
        if analysis:
            all_results[dataset_file] = analysis

    # Summary across all datasets
    print(f"\n{'='*80}")
    print("OVERALL SUMMARY")
    print(f"{'='*80}\n")

    if not all_results:
        print("No results to summarize.")
        return

    print(f"{'Dataset':<35} {'Samples':>8} {'Mean Angle':>12} {'Max Angle':>12}")
    print("-"*80)

    for dataset_name, analysis in all_results.items():
        print(f"{dataset_name:<35} {analysis['samples']:>8} "
              f"{analysis['mean_angle']:>11.2f}° {analysis['max_angle']:>11.2f}°")

    print("-"*80)

    # Overall statistics
    all_angles = np.concatenate([r['angles'] for r in all_results.values()])
    print(f"{'OVERALL':<35} {len(all_angles):>8} "
          f"{np.mean(all_angles):>11.2f}° {np.max(all_angles):>11.2f}°")
    print()

    print(f"Overall Assessment:")
    print(f"  Mean angular error: {np.mean(all_angles):.2f}° (target: <0.5°)")
    print(f"  Max angular error:  {np.max(all_angles):.2f}° (target: <1.0°)")
    print(f"  Median angular error: {np.median(all_angles):.2f}°")
    print()

if __name__ == '__main__':
    main()
