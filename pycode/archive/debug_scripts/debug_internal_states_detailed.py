#!/usr/bin/env python3
"""
Generate detailed internal state logging for first 10 fusion cycles
to compare with C implementation.

Logs:
- Kalman gain matrices K[0], K[1], K[2], K[3]
- Innovation vectors (gravity error, mag error)
- Error covariance matrix P (all 16 blocks)
- Bias state before and after updates
- All intermediate calculation steps
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def print_3x3_matrix(name, M):
    """Pretty print a 3x3 matrix"""
    print(f"  {name}:")
    for i in range(3):
        print(f"    [{M[i,0]:+.10e}, {M[i,1]:+.10e}, {M[i,2]:+.10e}]")

def print_vector(name, v):
    """Pretty print a vector"""
    print(f"  {name}: [{v[0]:+.10e}, {v[1]:+.10e}, {v[2]:+.10e}]")

def main():
    print("="*80)
    print("DETAILED INTERNAL STATE LOGGING - First 10 Fusion Cycles")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Monkey-patch to intercept measurement updates
    original_accel_update = sf.measurement_update
    original_mag_update = sf.measurement_update_mag

    fusion_logs = []

    def logged_accel_update():
        """Log detailed state during accelerometer measurement update"""
        log = {
            'type': 'accel',
            'fusion_count': len([l for l in fusion_logs if 'fusion_count' in l]) + 1,
        }

        # State before update
        log['bias_before'] = sf.bias_post_s.copy()
        log['quat_before'] = sf.quat_post.to_array().copy()

        # IMPORTANT: Innovation will be computed INSIDE measurement_update()
        # So we can't capture it here. We'll capture it after the update.

        # Call original update
        original_accel_update()

        # Innovation (computed inside update)
        log['grav_err_pri_s'] = sf.grav_err_pri_s.copy()

        # Kalman gains (after update)
        log['K'] = [sf.kalman_gain[i].copy() for i in range(4)]

        # Error states
        log['ornt_err_post_s'] = sf.ornt_err_post_s.copy()
        log['bias_err_post_s'] = sf.bias_err_post_s.copy()
        log['acc_err_post_s'] = sf.acc_err_post_s.copy()
        log['mag_dist_err_post_s'] = sf.mag_dist_err_post_s.copy()

        # State after update
        log['bias_after'] = sf.bias_post_s.copy()
        log['quat_after'] = sf.quat_post.to_array().copy()

        # Error covariance P (all 16 blocks)
        log['P'] = [[sf.err_cov_mtx_post[i, j].copy() for j in range(4)] for i in range(4)]

        fusion_logs.append(log)

    def logged_mag_update():
        """Log detailed state during magnetometer measurement update"""
        log = {
            'type': 'mag',
        }

        # State before update
        log['bias_before'] = sf.bias_post_s.copy()
        log['quat_before'] = sf.quat_post.to_array().copy()

        # Innovation (mag error)
        mag_measured_raw = sf.mag_data.count_avg.copy()
        mag_norm = np.linalg.norm(mag_measured_raw)
        mag_measured = mag_measured_raw / mag_norm if mag_norm > 1e-12 else mag_measured_raw
        mag_expected = sf.rot_mtx_post @ sf.mag_field_ref if np.linalg.norm(sf.mag_field_ref) > 1e-12 else np.zeros(3)
        log['mag_err'] = mag_measured - mag_expected

        # Call original update
        original_mag_update()

        # Kalman gains (after update)
        log['K'] = [sf.kalman_gain[i].copy() for i in range(4)]

        # Error states
        log['ornt_err_post_s'] = sf.ornt_err_post_s.copy()
        log['bias_err_post_s'] = sf.bias_err_post_s.copy()
        log['acc_err_post_s'] = sf.acc_err_post_s.copy()
        log['mag_dist_err_post_s'] = sf.mag_dist_err_post_s.copy()

        # State after update
        log['bias_after'] = sf.bias_post_s.copy()
        log['quat_after'] = sf.quat_post.to_array().copy()

        # Error covariance P (all 16 blocks)
        log['P'] = [[sf.err_cov_mtx_post[i, j].copy() for j in range(4)] for i in range(4)]

        fusion_logs.append(log)

    sf.measurement_update = logged_accel_update
    sf.measurement_update_mag = logged_mag_update

    print(f"\nDataset: {dataset_path}")
    print(f"Logging first 10 fusion cycles with detailed internal states\n")

    fusion_count = 0
    max_fusions = 10

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            fusion_count += 1
            output = sf.run()

    # Print logs
    print("="*80)
    print("DETAILED LOGS")
    print("="*80)

    accel_fusion_count = 0
    for log_idx, log in enumerate(fusion_logs):
        if log['type'] == 'accel':
            accel_fusion_count += 1
            print(f"\n{'='*80}")
            print(f"FUSION CYCLE #{accel_fusion_count} - ACCELEROMETER MEASUREMENT UPDATE")
            print(f"{'='*80}")

            print("\n--- STATE BEFORE UPDATE ---")
            print_vector("bias_before (dps)", log['bias_before'])
            print(f"  quat_before: [{log['quat_before'][0]:.10e}, {log['quat_before'][1]:.10e}, {log['quat_before'][2]:.10e}, {log['quat_before'][3]:.10e}]")

            if log['grav_err_pri_s'] is not None:
                print("\n--- INNOVATION (Gravity Error) ---")
                print_vector("grav_err_pri_s", log['grav_err_pri_s'])

            print("\n--- KALMAN GAINS ---")
            for i in range(4):
                block_names = ['K[0] (orientation)', 'K[1] (bias)', 'K[2] (linear_acc)', 'K[3] (mag_dist)']
                print_3x3_matrix(block_names[i], log['K'][i])

            print("\n--- ERROR STATES (Posterior) ---")
            print_vector("ornt_err_post_s", log['ornt_err_post_s'])
            print_vector("bias_err_post_s", log['bias_err_post_s'])
            print_vector("acc_err_post_s", log['acc_err_post_s'])
            print_vector("mag_dist_err_post_s", log['mag_dist_err_post_s'])

            print("\n--- STATE AFTER UPDATE ---")
            print_vector("bias_after (dps)", log['bias_after'])
            print(f"  quat_after: [{log['quat_after'][0]:.10e}, {log['quat_after'][1]:.10e}, {log['quat_after'][2]:.10e}, {log['quat_after'][3]:.10e}]")

            print_vector("bias_change (dps)", log['bias_after'] - log['bias_before'])

            print("\n--- ERROR COVARIANCE P (Diagonal blocks only) ---")
            for i in range(4):
                block_names = ['P[0,0] (ornt)', 'P[1,1] (bias)', 'P[2,2] (acc)', 'P[3,3] (mag)']
                print_3x3_matrix(block_names[i], log['P'][i][i])

            print("\n--- KEY CROSS-COVARIANCES ---")
            print_3x3_matrix("P[0,1] (ornt-bias coupling)", log['P'][0][1])
            print_3x3_matrix("P[1,0] (bias-ornt coupling)", log['P'][1][0])

        elif log['type'] == 'mag':
            print(f"\n{'-'*80}")
            print(f"MAG UPDATE (fusion cycle #{accel_fusion_count})")
            print(f"{'-'*80}")

            print("\n--- INNOVATION (Mag Error) ---")
            print_vector("mag_err", log['mag_err'])

            print("\n--- STATE BEFORE UPDATE ---")
            print_vector("bias_before (dps)", log['bias_before'])

            print("\n--- STATE AFTER UPDATE ---")
            print_vector("bias_after (dps)", log['bias_after'])
            print_vector("bias_change (dps)", log['bias_after'] - log['bias_before'])

    print("\n" + "="*80)
    print("LOGGING COMPLETE")
    print("="*80)
    print(f"\nTotal fusion cycles logged: {accel_fusion_count}")
    print(f"Total updates logged: {len(fusion_logs)}")

if __name__ == '__main__':
    main()
