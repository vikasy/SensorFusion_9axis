#!/usr/bin/env python3
"""
Debug gyro bias estimation in Python sensor fusion.
Track how bias estimates evolve and compare with actual sensor biases.
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
    print("DEBUGGING GYRO BIAS ESTIMATION")
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

    # Calculate actual sensor biases from raw data
    print("="*80)
    print("ACTUAL SENSOR BIASES (from raw data)")
    print("="*80)
    print()

    gyro_x_mean = df['gyro_x_counts'].mean()
    gyro_y_mean = df['gyro_y_counts'].mean()
    gyro_z_mean = df['gyro_z_counts'].mean()

    gyro_x_bias_dps = gyro_x_mean * platform.gyro_scale_factor
    gyro_y_bias_dps = gyro_y_mean * platform.gyro_scale_factor
    gyro_z_bias_dps = gyro_z_mean * platform.gyro_scale_factor

    print(f"Average gyro readings (counts): [{gyro_x_mean:.2f}, {gyro_y_mean:.2f}, {gyro_z_mean:.2f}]")
    print(f"Actual gyro bias (dps): [{gyro_x_bias_dps:.6f}, {gyro_y_bias_dps:.6f}, {gyro_z_bias_dps:.6f}]")
    print(f"Actual gyro bias (rad/s): [{np.radians(gyro_x_bias_dps):.6f}, {np.radians(gyro_y_bias_dps):.6f}, {np.radians(gyro_z_bias_dps):.6f}]")
    print()

    print("="*80)
    print("BIAS ESTIMATION EVOLUTION (first 30 fusion cycles)")
    print("="*80)
    print()

    fusion_count = 0
    max_samples = 150  # Process enough to get ~30 fusion outputs
    bias_history = []

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
            fusion_count += 1

            # Get bias estimate from sensor fusion object
            # bias_err_post_s is the estimated bias correction in dps
            # The filter estimates bias as a correction to apply to gyro readings
            bias_estimate_dps = sf.bias_err_post_s.copy()

            # Compute bias error
            bias_error_dps = bias_estimate_dps - np.array([gyro_x_bias_dps, gyro_y_bias_dps, gyro_z_bias_dps])

            # Store history
            bias_history.append({
                'fusion': fusion_count,
                'timestamp': timestamp,
                'bias_x_dps': bias_estimate_dps[0],
                'bias_y_dps': bias_estimate_dps[1],
                'bias_z_dps': bias_estimate_dps[2],
                'error_x_dps': bias_error_dps[0],
                'error_y_dps': bias_error_dps[1],
                'error_z_dps': bias_error_dps[2],
            })

            if fusion_count <= 30:
                print(f"Fusion #{fusion_count:3d} (idx={idx:3d}):")
                print(f"  Bias estimate (dps): [{bias_estimate_dps[0]:8.4f}, {bias_estimate_dps[1]:8.4f}, {bias_estimate_dps[2]:8.4f}]")
                print(f"  Bias error (dps):    [{bias_error_dps[0]:8.4f}, {bias_error_dps[1]:8.4f}, {bias_error_dps[2]:8.4f}]")
                print(f"  |Bias error|:         {np.linalg.norm(bias_error_dps):.4f} dps")

                # Check covariance matrix diagonal (uncertainty estimates)
                # err_cov_mtx_post[1][1] is the bias error covariance (3x3 block)
                P_bias_block = sf.err_cov_mtx_post[1, 1]
                bias_uncertainty = np.sqrt(np.diag(P_bias_block))  # Std dev of bias estimates (rad/s)
                bias_uncertainty_dps = np.degrees(bias_uncertainty)
                print(f"  Bias uncertainty (1σ): [{bias_uncertainty_dps[0]:8.4f}, {bias_uncertainty_dps[1]:8.4f}, {bias_uncertainty_dps[2]:8.4f}] dps")
                print()

            if fusion_count >= 30:
                break

    print("="*80)
    print("BIAS CONVERGENCE SUMMARY")
    print("="*80)
    print()

    bias_df = pd.DataFrame(bias_history)

    print(f"Actual bias (dps):      [{gyro_x_bias_dps:8.4f}, {gyro_y_bias_dps:8.4f}, {gyro_z_bias_dps:8.4f}]")
    print()
    print(f"Initial estimate (fusion #1):")
    print(f"  Bias (dps):           [{bias_df.iloc[0]['bias_x_dps']:8.4f}, {bias_df.iloc[0]['bias_y_dps']:8.4f}, {bias_df.iloc[0]['bias_z_dps']:8.4f}]")
    print(f"  Error (dps):          [{bias_df.iloc[0]['error_x_dps']:8.4f}, {bias_df.iloc[0]['error_y_dps']:8.4f}, {bias_df.iloc[0]['error_z_dps']:8.4f}]")
    print(f"  |Error|:              {np.linalg.norm(bias_df.iloc[0][['error_x_dps', 'error_y_dps', 'error_z_dps']]):.4f} dps")
    print()
    print(f"Final estimate (fusion #{fusion_count}):")
    print(f"  Bias (dps):           [{bias_df.iloc[-1]['bias_x_dps']:8.4f}, {bias_df.iloc[-1]['bias_y_dps']:8.4f}, {bias_df.iloc[-1]['bias_z_dps']:8.4f}]")
    print(f"  Error (dps):          [{bias_df.iloc[-1]['error_x_dps']:8.4f}, {bias_df.iloc[-1]['error_y_dps']:8.4f}, {bias_df.iloc[-1]['error_z_dps']:8.4f}]")
    print(f"  |Error|:              {np.linalg.norm(bias_df.iloc[-1][['error_x_dps', 'error_y_dps', 'error_z_dps']]):.4f} dps")
    print()

    # Compute convergence rate
    error_initial = np.linalg.norm(bias_df.iloc[0][['error_x_dps', 'error_y_dps', 'error_z_dps']])
    error_final = np.linalg.norm(bias_df.iloc[-1][['error_x_dps', 'error_y_dps', 'error_z_dps']])
    improvement = (error_initial - error_final) / error_initial * 100

    print(f"Convergence:")
    print(f"  Initial error: {error_initial:.4f} dps")
    print(f"  Final error:   {error_final:.4f} dps")
    print(f"  Improvement:   {improvement:.1f}%")
    print()

    if error_final > 0.1:
        print("⚠️  WARNING: Bias estimate did not converge!")
        print(f"   Expected final error: <0.1 dps")
        print(f"   Actual final error:   {error_final:.4f} dps")
        print()
        print("Possible causes:")
        print("  1. Q matrix (process noise) for bias is too small")
        print("  2. Initial P matrix (covariance) for bias is too small")
        print("  3. Kalman gain computation has bugs")
        print("  4. Bias update step has bugs")
    else:
        print("✓ Bias estimate converged successfully")
    print()

if __name__ == '__main__':
    main()
