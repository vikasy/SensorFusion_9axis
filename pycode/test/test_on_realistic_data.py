#!/usr/bin/env python3
"""
Real-World Dataset Testing for Sensor Fusion

Tests the sensor fusion algorithm on realistic motion datasets:
- walking.csv
- handheld_device.csv
- climbing_stairs.csv
- flying_drone.csv
- driving_in_car.csv

Author: Vikas Yadav
Date: 2025-10-15
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import argparse
from pathlib import Path

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# ============================================================================
# Configuration
# ============================================================================

DATASETS = {
    'walking': {
        'name': 'Walking',
        'description': 'Person walking - periodic gait motion',
        'expected_duration_s': 37,
        'difficulty': 'Easy'
    },
    'handheld_device': {
        'name': 'Handheld Device',
        'description': 'User manipulating handheld device',
        'expected_duration_s': 38,
        'difficulty': 'Medium'
    },
    'climbing_stairs': {
        'name': 'Climbing Stairs',
        'description': 'Person climbing stairs - vertical motion',
        'expected_duration_s': 35,
        'difficulty': 'Medium'
    },
    'flying_drone': {
        'name': 'Flying Drone',
        'description': 'Quadcopter flight - high rotation rates',
        'expected_duration_s': 42,
        'difficulty': 'Hard'
    },
    'driving_in_car': {
        'name': 'Driving in Car',
        'description': 'Vehicle motion - long duration, magnetic interference',
        'expected_duration_s': 100,
        'difficulty': 'Hard'
    }
}

# ============================================================================
# Test Functions
# ============================================================================

def test_dataset(dataset_name, plot=False, save_results=False):
    """
    Test sensor fusion on a realistic dataset

    Args:
        dataset_name: Name of dataset (without .csv extension)
        plot: If True, generate plots
        save_results: If True, save detailed results to CSV

    Returns:
        dict: Test results and metrics
    """
    if dataset_name not in DATASETS:
        print(f"Error: Unknown dataset '{dataset_name}'")
        print(f"Available datasets: {', '.join(DATASETS.keys())}")
        return None

    dataset_info = DATASETS[dataset_name]
    dataset_path = f"../../test/data/datasets/realistic/{dataset_name}.csv"

    print("=" * 80)
    print(f"TESTING: {dataset_info['name']}")
    print("=" * 80)
    print(f"Description: {dataset_info['description']}")
    print(f"Difficulty: {dataset_info['difficulty']}")
    print(f"Expected Duration: ~{dataset_info['expected_duration_s']}s")
    print()

    # Load dataset
    if not os.path.exists(dataset_path):
        print(f"Error: Dataset file not found: {dataset_path}")
        return None

    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples")

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

    print(f"Processing...")

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
            gt_roll = row['gt_roll_deg']
            gt_pitch = row['gt_pitch_deg']
            gt_yaw = row['gt_yaw_deg']

            # Get fusion output
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            py_roll = output.orientation[2]  # [yaw, pitch, roll]
            py_pitch = output.orientation[1]
            py_yaw = output.orientation[0]

            # Compute errors
            quat_error = quaternion_angular_distance(gt_quat, py_quat)
            roll_error = wrap_angle(py_roll - gt_roll)
            pitch_error = wrap_angle(py_pitch - gt_pitch)
            yaw_error = wrap_angle(py_yaw - gt_yaw)

            # Get bias
            bias = sf.bias_post_s.copy()
            bias_mag = np.linalg.norm(bias)

            # Get rotation magnitude (for analysis)
            rotation_mag = np.linalg.norm(sf.omega)

            results.append({
                'sample': idx,
                'fusion': fusion_count,
                'timestamp_ns': timestamp,
                'quat_error': quat_error,
                'roll_error': roll_error,
                'pitch_error': pitch_error,
                'yaw_error': yaw_error,
                'bias_x': bias[0],
                'bias_y': bias[1],
                'bias_z': bias[2],
                'bias_mag': bias_mag,
                'rotation_mag': rotation_mag,
                'gt_quat_w': gt_quat[0],
                'gt_quat_x': gt_quat[1],
                'gt_quat_y': gt_quat[2],
                'gt_quat_z': gt_quat[3],
                'py_quat_w': py_quat[0],
                'py_quat_x': py_quat[1],
                'py_quat_y': py_quat[2],
                'py_quat_z': py_quat[3]
            })

    print(f"Processed {fusion_count} fusion cycles")

    # Convert to DataFrame
    df_results = pd.DataFrame(results)

    # Compute statistics
    metrics = compute_metrics(df_results, dataset_name)

    # Print summary
    print_summary(metrics)

    # Save results if requested
    if save_results:
        output_file = f"realistic_results_{dataset_name}.csv"
        df_results.to_csv(output_file, index=False)
        print(f"\n✓ Detailed results saved to: {output_file}")

    # Generate plots if requested
    if plot:
        generate_plots(df_results, dataset_name, metrics)

    return metrics


def compute_metrics(df_results, dataset_name):
    """Compute performance metrics from results"""
    quat_errors = df_results['quat_error'].values
    roll_errors = df_results['roll_error'].values
    pitch_errors = df_results['pitch_error'].values
    yaw_errors = df_results['yaw_error'].values
    bias_mags = df_results['bias_mag'].values
    rotation_mags = df_results['rotation_mag'].values

    return {
        'dataset': dataset_name,
        'num_samples': len(df_results),
        'quat_error_mean': quat_errors.mean(),
        'quat_error_std': quat_errors.std(),
        'quat_error_min': quat_errors.min(),
        'quat_error_max': quat_errors.max(),
        'quat_error_p50': np.percentile(quat_errors, 50),
        'quat_error_p95': np.percentile(quat_errors, 95),
        'quat_error_p99': np.percentile(quat_errors, 99),
        'roll_error_rms': np.sqrt(np.mean(roll_errors**2)),
        'pitch_error_rms': np.sqrt(np.mean(pitch_errors**2)),
        'yaw_error_rms': np.sqrt(np.mean(yaw_errors**2)),
        'bias_initial': bias_mags[0],
        'bias_final': bias_mags[-1],
        'bias_mean': bias_mags.mean(),
        'bias_max': bias_mags.max(),
        'rotation_mean': rotation_mags.mean(),
        'rotation_max': rotation_mags.max(),
        'excellent_pct': 100 * np.sum(quat_errors < 1.0) / len(quat_errors),
        'good_pct': 100 * np.sum((quat_errors >= 1.0) & (quat_errors < 5.0)) / len(quat_errors),
        'acceptable_pct': 100 * np.sum((quat_errors >= 5.0) & (quat_errors < 10.0)) / len(quat_errors),
        'poor_pct': 100 * np.sum(quat_errors >= 10.0) / len(quat_errors)
    }


def print_summary(metrics):
    """Print performance summary"""
    print("\n" + "=" * 80)
    print("PERFORMANCE SUMMARY")
    print("=" * 80)

    print(f"\nQuaternion Error:")
    print(f"  Mean:   {metrics['quat_error_mean']:.3f}°")
    print(f"  Std:    {metrics['quat_error_std']:.3f}°")
    print(f"  Min:    {metrics['quat_error_min']:.3f}°")
    print(f"  Max:    {metrics['quat_error_max']:.3f}°")
    print(f"  Median: {metrics['quat_error_p50']:.3f}°")
    print(f"  95th:   {metrics['quat_error_p95']:.3f}°")
    print(f"  99th:   {metrics['quat_error_p99']:.3f}°")

    print(f"\nAngle Errors (RMS):")
    print(f"  Roll:  {metrics['roll_error_rms']:.3f}°")
    print(f"  Pitch: {metrics['pitch_error_rms']:.3f}°")
    print(f"  Yaw:   {metrics['yaw_error_rms']:.3f}°")

    print(f"\nBias Convergence:")
    print(f"  Initial: {metrics['bias_initial']:.4f} dps")
    print(f"  Final:   {metrics['bias_final']:.4f} dps")
    print(f"  Mean:    {metrics['bias_mean']:.4f} dps")
    print(f"  Max:     {metrics['bias_max']:.4f} dps")

    print(f"\nMotion Characteristics:")
    print(f"  Mean Rotation Rate: {metrics['rotation_mean']:.2f} dps")
    print(f"  Max Rotation Rate:  {metrics['rotation_max']:.2f} dps")

    print(f"\nError Distribution:")
    print(f"  Excellent (< 1°):  {metrics['excellent_pct']:.1f}%")
    print(f"  Good (1-5°):       {metrics['good_pct']:.1f}%")
    print(f"  Acceptable (5-10°): {metrics['acceptable_pct']:.1f}%")
    print(f"  Poor (> 10°):      {metrics['poor_pct']:.1f}%")

    # Assessment
    print(f"\nAssessment:")
    if metrics['quat_error_mean'] < 5.0:
        print(f"  ✓ EXCELLENT: Mean error < 5°")
    elif metrics['quat_error_mean'] < 10.0:
        print(f"  ✓ GOOD: Mean error < 10°")
    else:
        print(f"  ⚠ NEEDS IMPROVEMENT: Mean error >= 10°")

    if metrics['quat_error_max'] < 20.0:
        print(f"  ✓ Robust: Max error < 20°")
    else:
        print(f"  ⚠ Check robustness: Max error >= 20°")

    if metrics['bias_final'] < 2.0:
        print(f"  ✓ Good bias convergence: < 2 dps")
    else:
        print(f"  ⚠ Bias needs investigation: >= 2 dps")


def generate_plots(df_results, dataset_name, metrics):
    """Generate analysis plots"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Realistic Data Test: {DATASETS[dataset_name]["name"]}', fontsize=14, fontweight='bold')

    # Time axis (convert fusion cycles to seconds, assuming 100Hz / 4 oversampling = 25Hz fusion rate)
    time_s = df_results['fusion'].values * 0.04  # 25Hz = 0.04s per fusion cycle

    # Plot 1: Quaternion Error over Time
    ax1 = axes[0, 0]
    ax1.plot(time_s, df_results['quat_error'], linewidth=0.8, alpha=0.7)
    ax1.axhline(y=metrics['quat_error_mean'], color='r', linestyle='--', label=f'Mean: {metrics["quat_error_mean"]:.2f}°')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Quaternion Error (°)')
    ax1.set_title('Quaternion Error vs Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Bias Convergence
    ax2 = axes[0, 1]
    ax2.plot(time_s, df_results['bias_x'], label='Bias X', linewidth=0.8)
    ax2.plot(time_s, df_results['bias_y'], label='Bias Y', linewidth=0.8)
    ax2.plot(time_s, df_results['bias_z'], label='Bias Z', linewidth=0.8)
    ax2.plot(time_s, df_results['bias_mag'], label='Bias Magnitude', linewidth=1.2, color='black')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Gyro Bias (dps)')
    ax2.set_title('Bias Convergence')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: Error Distribution
    ax3 = axes[1, 0]
    ax3.hist(df_results['quat_error'], bins=50, alpha=0.7, edgecolor='black')
    ax3.axvline(x=metrics['quat_error_mean'], color='r', linestyle='--', label=f'Mean: {metrics["quat_error_mean"]:.2f}°')
    ax3.axvline(x=metrics['quat_error_p95'], color='orange', linestyle='--', label=f'95th: {metrics["quat_error_p95"]:.2f}°')
    ax3.set_xlabel('Quaternion Error (°)')
    ax3.set_ylabel('Count')
    ax3.set_title('Error Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3, axis='y')

    # Plot 4: Error vs Rotation Rate
    ax4 = axes[1, 1]
    scatter = ax4.scatter(df_results['rotation_mag'], df_results['quat_error'],
                         alpha=0.5, s=10, c=df_results['quat_error'], cmap='viridis')
    ax4.set_xlabel('Rotation Rate (dps)')
    ax4.set_ylabel('Quaternion Error (°)')
    ax4.set_title('Error vs Rotation Rate')
    ax4.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax4, label='Error (°)')

    plt.tight_layout()

    # Save plot
    plot_file = f"realistic_results_{dataset_name}_plots.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Plots saved to: {plot_file}")

    plt.show()


# ============================================================================
# Helper Functions
# ============================================================================

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


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Test sensor fusion on realistic datasets')
    parser.add_argument('--dataset', type=str, default='walking',
                       help=f'Dataset to test: {", ".join(DATASETS.keys())}')
    parser.add_argument('--plot', action='store_true',
                       help='Generate analysis plots')
    parser.add_argument('--save', action='store_true',
                       help='Save detailed results to CSV')
    parser.add_argument('--list', action='store_true',
                       help='List available datasets')

    args = parser.parse_args()

    if args.list:
        print("\nAvailable Realistic Datasets:")
        print("=" * 80)
        for key, info in DATASETS.items():
            print(f"\n{key}:")
            print(f"  Name: {info['name']}")
            print(f"  Description: {info['description']}")
            print(f"  Duration: ~{info['expected_duration_s']}s")
            print(f"  Difficulty: {info['difficulty']}")
        print("\n" + "=" * 80)
        return 0

    # Run test
    metrics = test_dataset(args.dataset, plot=args.plot, save_results=args.save)

    if metrics is None:
        return 1

    print("\n" + "=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)

    return 0


if __name__ == '__main__':
    sys.exit(main())
