#!/usr/bin/env python3
"""
Trace the exact mathematical computation of bias updates for first 3 fusion cycles.
Print every single step to find where the divergence starts.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("BIAS UPDATE MATHEMATICS - First 3 Fusion Cycles")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Monkey-patch to trace bias updates in measurement_update
    original_measurement_update = sf.measurement_update
    original_mag_update = sf.measurement_update_mag

    fusion_count = 0
    mag_update_count = 0

    def traced_measurement_update(debug=False):
        nonlocal fusion_count
        fusion_count += 1

        if fusion_count <= 3:
            print(f"\n{'='*80}")
            print(f"FUSION CYCLE #{fusion_count} - ACCELEROMETER MEASUREMENT UPDATE")
            print(f"{'='*80}")

            print(f"\n--- STATE BEFORE MEASUREMENT UPDATE ---")
            print(f"bias_post_s: [{sf.bias_post_s[0]:+.10e}, {sf.bias_post_s[1]:+.10e}, {sf.bias_post_s[2]:+.10e}] dps")
            print(f"quat_post: [{sf.quat_post.q0:.10e}, {sf.quat_post.q1:.10e}, {sf.quat_post.q2:.10e}, {sf.quat_post.q3:.10e}]")

        # Call original (which computes innovation, K, and updates state)
        original_measurement_update(debug=False)

        if fusion_count <= 3:
            print(f"\n--- INNOVATION (Gravity Error) ---")
            print(f"grav_err_pri_s: [{sf.grav_err_pri_s[0]:+.10e}, {sf.grav_err_pri_s[1]:+.10e}, {sf.grav_err_pri_s[2]:+.10e}]")

            print(f"\n--- KALMAN GAIN K[1] (Bias Block) ---")
            for i in range(3):
                print(f"  K[1][{i},:] = [{sf.kalman_gain[1][i,0]:+.10e}, {sf.kalman_gain[1][i,1]:+.10e}, {sf.kalman_gain[1][i,2]:+.10e}]")

            print(f"\n--- BIAS ERROR COMPUTATION ---")
            print(f"bias_err_post_s = K[1] @ grav_err_pri_s")

            # Manual computation
            bias_err_manual = sf.kalman_gain[1] @ sf.grav_err_pri_s
            print(f"  Manual computation: [{bias_err_manual[0]:+.10e}, {bias_err_manual[1]:+.10e}, {bias_err_manual[2]:+.10e}]")
            print(f"  From filter:        [{sf.bias_err_post_s[0]:+.10e}, {sf.bias_err_post_s[1]:+.10e}, {sf.bias_err_post_s[2]:+.10e}]")

            # Check if they match
            diff = bias_err_manual - sf.bias_err_post_s
            if np.linalg.norm(diff) > 1e-12:
                print(f"  ⚠ MISMATCH! Difference: [{diff[0]:+.10e}, {diff[1]:+.10e}, {diff[2]:+.10e}]")
            else:
                print(f"  ✓ Match")

            print(f"\n--- BIAS UPDATE ---")
            print(f"bias_new = bias_old - bias_err_post_s")
            print(f"         = [{sf.bias_post_s[0]:+.10e}, {sf.bias_post_s[1]:+.10e}, {sf.bias_post_s[2]:+.10e}] dps")

            bias_mag = np.linalg.norm(sf.bias_post_s)
            print(f"\nBias magnitude: {bias_mag:.10e} dps")

    def traced_mag_update():
        nonlocal mag_update_count
        mag_update_count += 1

        # Only trace mag updates between fusion cycles 2 and 3
        if fusion_count == 2 or (fusion_count == 3 and mag_update_count == 1):
            print(f"\n{'-'*80}")
            print(f"MAG UPDATE #{mag_update_count} (after fusion cycle #{fusion_count})")
            print(f"{'-'*80}")

            bias_before = sf.bias_post_s.copy()
            print(f"\nBias BEFORE mag update: [{bias_before[0]:+.10e}, {bias_before[1]:+.10e}, {bias_before[2]:+.10e}] dps")

        # Call original
        original_mag_update()

        if fusion_count == 2 or (fusion_count == 3 and mag_update_count == 1):
            bias_after = sf.bias_post_s.copy()
            print(f"Bias AFTER mag update:  [{bias_after[0]:+.10e}, {bias_after[1]:+.10e}, {bias_after[2]:+.10e}] dps")

            bias_change = bias_after - bias_before
            bias_change_mag = np.linalg.norm(bias_change)
            print(f"Bias CHANGE:            [{bias_change[0]:+.10e}, {bias_change[1]:+.10e}, {bias_change[2]:+.10e}] (mag: {bias_change_mag:.10e})")

            if bias_change_mag > 0.001:
                print(f"\n⚠⚠⚠ LARGE BIAS CHANGE FROM MAG UPDATE! ⚠⚠⚠")
                print(f"\nMag update details:")
                print(f"  K[1] (bias Kalman gain):")
                for i in range(3):
                    print(f"    K[1][{i},:] = [{sf.kalman_gain[1][i,0]:+.10e}, {sf.kalman_gain[1][i,1]:+.10e}, {sf.kalman_gain[1][i,2]:+.10e}]")
                print(f"  bias_err_post_s: [{sf.bias_err_post_s[0]:+.10e}, {sf.bias_err_post_s[1]:+.10e}, {sf.bias_err_post_s[2]:+.10e}]")

                print(f"\n  Process noise Q[1,0] (bias-orientation coupling):")
                for i in range(3):
                    print(f"    Q[1,0][{i},:] = [{sf.proc_noise_var[1,0][i,0]:+.10e}, {sf.proc_noise_var[1,0][i,1]:+.10e}, {sf.proc_noise_var[1,0][i,2]:+.10e}]")

                print(f"\n  Process noise Q[1,1] (bias variance):")
                for i in range(3):
                    print(f"    Q[1,1][{i},:] = [{sf.proc_noise_var[1,1][i,0]:+.10e}, {sf.proc_noise_var[1,1][i,1]:+.10e}, {sf.proc_noise_var[1,1][i,2]:+.10e}]")

                print(f"\n  Process noise Q[1,3] (bias-mag coupling):")
                for i in range(3):
                    print(f"    Q[1,3][{i},:] = [{sf.proc_noise_var[1,3][i,0]:+.10e}, {sf.proc_noise_var[1,3][i,1]:+.10e}, {sf.proc_noise_var[1,3][i,2]:+.10e}]")

    sf.measurement_update = traced_measurement_update
    sf.measurement_update_mag = traced_mag_update

    print(f"\nDataset: {dataset_path}")
    print(f"Processing first 20 samples (should capture 3-4 fusion cycles)...\n")

    for idx in range(min(20, len(df))):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

            if fusion_count >= 3:
                break

    print("\n" + "="*80)
    print("BIAS TRACING COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
