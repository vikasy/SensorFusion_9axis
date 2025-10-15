#!/usr/bin/env python3
"""
Compare convergence curves for different QBias values.
Shows how quaternion error and bias evolve over time.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

import sensor_fusion_6axis

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi

def run_with_qbias(qbias_value, num_samples=100):
    """Run fusion with specific QBias and return time series"""

    original_value = sensor_fusion_6axis.SF_6XAG_QBias
    sensor_fusion_6axis.SF_6XAG_QBias = qbias_value

    if 'sensor_fusion_9axis' in sys.modules:
        del sys.modules['sensor_fusion_9axis']

    from sensor_fusion_9axis import SensorFusion9Axis
    from sensor_platform_config import SensorPlatform, INVENSENSE

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    results = []
    fusion_count = 0

    for idx in range(min(num_samples, len(df))):
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

            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            quat_error = quaternion_angular_distance(gt_quat, python_quat)
            bias = sf.bias_post_s.copy()
            bias_mag = np.linalg.norm(bias)

            results.append({
                'fusion': fusion_count,
                'quat_error': quat_error,
                'bias_mag': bias_mag,
                'bias_x': bias[0],
                'bias_y': bias[1],
                'bias_z': bias[2]
            })

    sensor_fusion_6axis.SF_6XAG_QBias = original_value

    return results

def main():
    print("="*80)
    print("QBIAS CONVERGENCE COMPARISON")
    print("="*80)

    # Compare a few key values
    qbias_values = [100.0, 10.0, 1.0, 0.1]

    print(f"\nComparing {len(qbias_values)} QBias values: {qbias_values}")
    print(f"Processing first 100 samples...\n")

    all_results = {}

    for qbias in qbias_values:
        print(f"Running QBias = {qbias:6.2f}...", end=" ", flush=True)
        results = run_with_qbias(qbias, num_samples=100)
        all_results[qbias] = results
        print(f"{len(results)} fusion cycles")

    # Print convergence table
    print("\n" + "="*80)
    print("CONVERGENCE COMPARISON - First 25 Fusion Cycles")
    print("="*80)

    print(f"\n{'Fusion':>6}", end="")
    for qbias in qbias_values:
        print(f" | QBias={qbias:6.2f}", end="")
    print()

    print(f"{'#':>6}", end="")
    for _ in qbias_values:
        print(f" | {'Quat(°)':>7} {'Bias':>7}", end="")
    print()

    print("-" * (6 + len(qbias_values) * 18))

    for i in range(min(25, min(len(r) for r in all_results.values()))):
        print(f"{i+1:6d}", end="")
        for qbias in qbias_values:
            r = all_results[qbias][i]
            print(f" | {r['quat_error']:7.3f} {r['bias_mag']:7.4f}", end="")
        print()

    # Print final values
    print("\n" + "="*80)
    print("FINAL VALUES (Fusion #25)")
    print("="*80)

    print(f"\n{'QBias':>8} {'Quat Error':>12} {'Bias Mag':>12} {'Bias X':>10} {'Bias Y':>10} {'Bias Z':>10}")
    print(f"{'':>8} {'(deg)':>12} {'(dps)':>12} {'(dps)':>10} {'(dps)':>10} {'(dps)':>10}")
    print("-"*72)

    for qbias in qbias_values:
        if len(all_results[qbias]) >= 25:
            r = all_results[qbias][24]  # 25th fusion (index 24)
            print(f"{qbias:8.2f} {r['quat_error']:12.4f} {r['bias_mag']:12.6f} "
                  f"{r['bias_x']:+10.6f} {r['bias_y']:+10.6f} {r['bias_z']:+10.6f}")

    # Show late convergence (samples 75-100)
    print("\n" + "="*80)
    print("LATE CONVERGENCE (Fusion #75-100)")
    print("="*80)

    print(f"\n{'QBias':>8} {'Quat Error':>15} {'Bias Magnitude':>18}")
    print(f"{'':>8} {'Mean':>7} {'Std':>7} {'Mean':>9} {'Std':>8}")
    print("-"*50)

    for qbias in qbias_values:
        results = all_results[qbias]
        if len(results) >= 75:
            late_results = results[74:]  # From 75th onward
            quat_errors = np.array([r['quat_error'] for r in late_results])
            bias_mags = np.array([r['bias_mag'] for r in late_results])

            print(f"{qbias:8.2f} {quat_errors.mean():7.4f} {quat_errors.std():7.4f} "
                  f"{bias_mags.mean():9.6f} {bias_mags.std():8.6f}")

    # Recommendation
    print("\n" + "="*80)
    print("ANALYSIS")
    print("="*80)

    print("\nConvergence behavior:")
    for qbias in qbias_values:
        results = all_results[qbias]
        if len(results) >= 25:
            early_quat = results[4]['quat_error']  # Fusion #5
            late_quat = results[24]['quat_error']  # Fusion #25
            improvement = (early_quat - late_quat) / early_quat * 100

            final_bias = results[-1]['bias_mag']

            print(f"\nQBias = {qbias:6.2f}:")
            print(f"  Quaternion: {early_quat:.3f}° → {late_quat:.3f}° ({improvement:+.1f}% change)")
            print(f"  Final bias: {final_bias:.6f} dps", end="")

            if final_bias > 2.0:
                print(" ✗ DIVERGING")
            elif final_bias < 0.5:
                print(" ✓ WELL BEHAVED")
            else:
                print(" ⚠ MARGINAL")

    print("\n" + "="*80)
    print("COMPARISON COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
