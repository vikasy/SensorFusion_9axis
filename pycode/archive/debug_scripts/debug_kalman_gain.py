#!/usr/bin/env python3
"""
Debug Kalman gain computation in detail.
Check if there's a bug in how K is calculated.
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
    print("DEBUGGING KALMAN GAIN COMPUTATION")
    print("="*80)
    print()

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    print("Running first few fusion cycles...")
    print()

    fusion_count = 0
    max_fusions = 3

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

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        if ready_gyro:
            fusion_count += 1
            print("="*80)
            print(f"FUSION #{fusion_count}")
            print("="*80)
            print()

            # Just look at the key values after running
            # Get current state before run
            Q11_before = sf.proc_noise_var[1, 1].copy()

            # Run fusion
            output = sf.run()

            # After fusion, get Kalman gain
            K1 = sf.kalman_gain[1]

            print(f"Q[1][1] trace: {np.trace(Q11_before):.6e}")
            print(f"K[1] Frobenius norm: {np.linalg.norm(K1, 'fro'):.6e}")
            print(f"R (meas noise): {sf.meas_noise_var_acc:.6e}")
            print()

            # Manually compute what C[1] should be
            grav_gyr = -sf.rot_mtx_post[:, 2] * 9.80665
            from sensor_fusion_6axis import cross_product_matrix, DEG2RAD, SF_DELTA_T
            cp_mat = cross_product_matrix(grav_gyr)
            C1 = (DEG2RAD * SF_DELTA_T) * cp_mat

            print("C[1] (bias → gravity error):")
            print(C1)
            print(f"  Frobenius norm: {np.linalg.norm(C1, 'fro'):.6e}")
            print()

            # Compute innovation covariance S for bias only: S = C1*Q11*C1' + R*I
            C1_Q11_C1T = C1 @ Q11_before @ C1.T
            R_matrix = np.eye(3) * sf.meas_noise_var_acc
            S = C1_Q11_C1T + R_matrix

            print("Innovation covariance S = C[1]*Q[1][1]*C[1]' + R*I:")
            print(f"  Trace(C[1]*Q[1][1]*C[1]'): {np.trace(C1_Q11_C1T):.6e}")
            print(f"  Trace(R*I):                {np.trace(R_matrix):.6e}")
            print(f"  Trace(S):                  {np.trace(S):.6e}")
            print(f"  Ratio Trace(R) / Trace(C*Q*C'): {np.trace(R_matrix) / np.trace(C1_Q11_C1T):.2f}")
            print()

            if np.trace(R_matrix) > 100 * np.trace(C1_Q11_C1T):
                print("  ⚠️  R dominates S (ratio > 100)")
                print("     This means bias signal is unobservable due to noise")
            elif np.trace(R_matrix) > 10 * np.trace(C1_Q11_C1T):
                print("  ⚠️  R >> C*Q*C' (ratio > 10)")
                print("     Bias signal is weakly observable")
            else:
                print("  ✓ R and C*Q*C' are comparable, bias should be observable")
            print()

            if fusion_count >= max_fusions:
                break

    print("="*80)
    print("SUMMARY")
    print("="*80)
    print()
    print("Key findings:")
    print("1. Check if Kalman gain computation matches expected formula")
    print("2. Identify which term (C, Q, or R) is causing K to be tiny")
    print("3. The issue is likely:")
    print("   - C[1] is too small (factor of ~0.0007)")
    print("   - R is too large relative to C*Q*C'")
    print("   - Or both")
    print()

if __name__ == '__main__':
    main()
