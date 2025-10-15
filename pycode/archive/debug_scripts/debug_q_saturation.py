#!/usr/bin/env python3
"""
Debug Q saturation to see if it's preventing convergence.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis, DEG2RAD
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("DEBUGGING Q SATURATION")
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

    MAX_BIAS_VAR = (10.0 * DEG2RAD) ** 2
    print(f"MAX_BIAS_VAR = {MAX_BIAS_VAR:.6e}")
    print()

    fusion_count = 0
    max_fusions = 5

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

            # Run fusion
            output = sf.run()

            # Check Q[1][1]
            Q11 = sf.proc_noise_var[1, 1]
            print(f"Q[1][1] after fusion:")
            print(Q11)
            print(f"  Diagonal: [{Q11[0,0]:.6e}, {Q11[1,1]:.6e}, {Q11[2,2]:.6e}]")
            print(f"  Are any saturated? [{Q11[0,0] >= MAX_BIAS_VAR*0.99}, {Q11[1,1] >= MAX_BIAS_VAR*0.99}, {Q11[2,2] >= MAX_BIAS_VAR*0.99}]")
            print()

            # Check P[1][1]
            P11 = sf.err_cov_mtx_post[1, 1]
            print(f"P[1][1] after fusion:")
            print(f"  Diagonal: [{P11[0,0]:.6e}, {P11[1,1]:.6e}, {P11[2,2]:.6e}]")
            print()

            # Check K[1]
            K1 = sf.kalman_gain[1]
            print(f"K[1] Frobenius norm: {np.linalg.norm(K1, 'fro'):.6e}")
            print()

            if fusion_count >= max_fusions:
                break

    print("="*80)
    print("DIAGNOSIS")
    print("="*80)
    print()
    print("If Q is saturated early, it prevents the filter from building up")
    print("enough uncertainty to make meaningful corrections.")
    print()

if __name__ == '__main__':
    main()
