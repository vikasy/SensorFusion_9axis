#!/usr/bin/env python3
"""
Trace Kalman filter step-by-step to understand why P is growing.
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
    print("TRACING KALMAN FILTER STEP-BY-STEP")
    print("="*80)
    print()

    # Load static dataset
    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)
    print(f"Loaded {len(df)} samples from {dataset_path}")
    print()

    # Initialize sensor fusion
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    fusion_count = 0
    max_samples = 20

    for idx in range(min(max_samples, len(df))):
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

        # Process sensors
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        if ready_gyro:
            fusion_count += 1

            print("="*80)
            print(f"FUSION #{fusion_count} (sample idx={idx})")
            print("="*80)

            # Capture state BEFORE run
            P_before = sf.err_cov_mtx_post[1, 1].copy()
            bias_before = sf.bias_post_s.copy()

            # Run fusion
            output = sf.run()

            # Capture state AFTER run
            Q_after = sf.proc_noise_var[1, 1].copy()
            P_after = sf.err_cov_mtx_post[1, 1].copy()
            K_after = sf.kalman_gain[1].copy()
            bias_after = sf.bias_post_s.copy()

            print(f"\nBias state:")
            print(f"  Before: [{bias_before[0]:.6f}, {bias_before[1]:.6f}, {bias_before[2]:.6f}] dps")
            print(f"  After:  [{bias_after[0]:.6f}, {bias_after[1]:.6f}, {bias_after[2]:.6f}] dps")
            print(f"  Change: [{bias_after[0]-bias_before[0]:.6f}, {bias_after[1]-bias_before[1]:.6f}, {bias_after[2]-bias_before[2]:.6f}] dps")

            print(f"\nP[1][1] diagonal BEFORE run:")
            print(f"  [{P_before[0,0]:.6e}, {P_before[1,1]:.6e}, {P_before[2,2]:.6e}]")

            print(f"\nQ[1][1] diagonal (after time update):")
            print(f"  [{Q_after[0,0]:.6e}, {Q_after[1,1]:.6e}, {Q_after[2,2]:.6e}]")

            print(f"\nKalman Gain K[1] norm:")
            print(f"  {np.linalg.norm(K_after):.6e}")

            print(f"\nP[1][1] diagonal AFTER run:")
            print(f"  [{P_after[0,0]:.6e}, {P_after[1,1]:.6e}, {P_after[2,2]:.6e}]")

            # Check if P grew or shrank
            trace_before = np.trace(P_before)
            trace_after = np.trace(P_after)
            trace_Q = np.trace(Q_after)

            print(f"\nTrace analysis:")
            print(f"  Trace(P_before) = {trace_before:.6e}")
            print(f"  Trace(Q)        = {trace_Q:.6e}")
            print(f"  Trace(P_after)  = {trace_after:.6e}")
            print(f"  Change:         = {trace_after - trace_before:.6e}")

            if trace_after > trace_before:
                print(f"  ⚠️  P GREW by {(trace_after/trace_before - 1)*100:.1f}%")
            else:
                print(f"  ✓ P SHRANK by {(1 - trace_after/trace_before)*100:.1f}%")

            # Key insight: P should shrink after measurement update
            # If P ≈ Q, it means (I-K*C) ≈ 1, which means K ≈ 0
            ratio = trace_after / trace_Q if trace_Q > 0 else 0
            print(f"\n  Ratio P_after/Q = {ratio:.4f}")
            if ratio > 0.95:
                print(f"  ⚠️  P ≈ Q suggests Kalman gain is too small!")

            print()

            if fusion_count >= 5:
                break

    print("="*80)
    print("DIAGNOSIS")
    print("="*80)
    print()
    print("Key observations:")
    print("1. If P grows after each cycle, the filter is diverging")
    print("2. If P ≈ Q, then (I-K*C) ≈ I, meaning K ≈ 0 (no correction)")
    print("3. K should be moderate (not tiny, not huge) for filter to work")
    print("4. For static case, bias should converge and P should decrease")
    print()

if __name__ == '__main__':
    main()
