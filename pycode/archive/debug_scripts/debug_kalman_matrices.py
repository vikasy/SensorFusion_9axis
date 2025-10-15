#!/usr/bin/env python3
"""
Debug Kalman filter matrices - inspect Q, R, K, P evolution.
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
    print("DEBUGGING KALMAN FILTER MATRICES")
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

    print("="*80)
    print("KALMAN FILTER PARAMETER VALUES")
    print("="*80)
    print()

    print(f"Process noise parameters:")
    print(f"  SF_6XAG_QOrient = {sf.proc_noise_var_orient:.6e}")
    print(f"  SF_6XAG_QBias = {sf.proc_noise_var_bias:.6e}")
    print(f"  SF_6XAG_QLinAcc = {sf.proc_noise_var_lin_acc:.6e}")
    print(f"  SF_6XAG_QBiasOrient = {sf.proc_noise_var_bias_orient:.6e}")
    print()
    print(f"Measurement noise:")
    print(f"  meas_noise_var_acc = {sf.meas_noise_var_acc:.6e}")
    print()

    fusion_count = 0
    max_samples = 20  # Process first 5 fusion cycles

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

        # Process accelerometer
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)

        # Process gyroscope
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        # Run fusion if ready
        if ready_gyro:
            output = sf.run()
            fusion_count += 1

            print("="*80)
            print(f"FUSION #{fusion_count} (sample idx={idx})")
            print("="*80)
            print()

            # Process noise covariance matrix Q
            print("Q[1][1] (Bias process noise covariance - should be ~1e1):")
            print(sf.proc_noise_var[1, 1])
            print()

            # Error covariance matrix P
            print("P[1][1] (Bias error covariance AFTER update):")
            print(sf.err_cov_mtx_post[1, 1])
            print(f"  Diagonal: [{sf.err_cov_mtx_post[1, 1][0, 0]:.6e}, {sf.err_cov_mtx_post[1, 1][1, 1]:.6e}, {sf.err_cov_mtx_post[1, 1][2, 2]:.6e}]")
            print()

            # Kalman gain
            print("K[1] (Kalman gain for bias - should be small initially, increase over time):")
            print(sf.kalman_gain[1])
            print(f"  Norm: {np.linalg.norm(sf.kalman_gain[1]):.6e}")
            print()

            # Bias estimate
            bias_est = sf.bias_err_post_s
            print(f"Bias estimate (dps): [{bias_est[0]:.6f}, {bias_est[1]:.6f}, {bias_est[2]:.6f}]")
            print()

            # Check if P is growing or shrinking
            P_trace = np.trace(sf.err_cov_mtx_post[1, 1])
            Q_trace = np.trace(sf.proc_noise_var[1, 1])
            print(f"Trace(P[1][1]) = {P_trace:.6e}  (sum of diagonal elements)")
            print(f"Trace(Q[1][1]) = {Q_trace:.6e}")
            print()

            if P_trace > 1e6:
                print("⚠️  WARNING: P[1][1] is EXPLODING! This should DECREASE over time.")
                print("   This indicates either:")
                print("   1. Q (process noise) is too large")
                print("   2. R (measurement noise) is too large")
                print("   3. Kalman gain computation has bugs")
                print("   4. Covariance update P = (I - K*C)*Q has bugs")
                print()

            if fusion_count >= 5:
                break

    print("="*80)
    print("DIAGNOSIS")
    print("="*80)
    print()
    print("For a working Kalman filter on static data:")
    print("  1. P (error covariance) should DECREASE over time")
    print("  2. K (Kalman gain) should be moderate (not tiny, not huge)")
    print("  3. Bias estimate should converge to actual bias (~1.96 dps)")
    print()
    print("If P is increasing:")
    print("  - Process noise Q is too large (filter assumes system is very noisy)")
    print("  - Measurement noise R is too large (filter doesn't trust measurements)")
    print("  - Update equation P = (I - K*C)*Q has a bug")
    print()

if __name__ == '__main__':
    main()
