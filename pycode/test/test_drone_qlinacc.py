#!/usr/bin/env python3
"""
Test flying_drone dataset with increased QLinAcc parameter

Test if increasing linear acceleration process noise (QLinAcc) helps
with the drone dataset which starts with +1.5g takeoff acceleration.

Author: Vikas Yadav
Date: 2025-10-17
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE


def test_drone_with_qlinacc(qlinacc_value):
    """Test flying_drone with specific QLinAcc value"""

    dataset_path = "../../test/data/datasets/realistic/flying_drone.csv"
    df = pd.read_csv(dataset_path)

    # Initialize fusion with custom QLinAcc
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Override QLinAcc
    sf.proc_noise_var_lin_acc = qlinacc_value

    # Storage for results
    quat_errors = []
    bias_mags = []

    # Process all samples
    for idx in range(len(df)):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

            # Get ground truth
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            # Compute error
            dot_product = np.abs(np.dot(gt_quat, py_quat))
            dot_product = np.clip(dot_product, 0.0, 1.0)
            quat_error = 2.0 * np.arccos(dot_product) * (180.0 / np.pi)

            quat_errors.append(quat_error)
            bias_mags.append(np.linalg.norm(sf.bias_post_s))

    # Compute statistics
    quat_errors = np.array(quat_errors)
    bias_mags = np.array(bias_mags)

    return {
        'qlinacc': qlinacc_value,
        'mean_error': quat_errors.mean(),
        'max_error': quat_errors.max(),
        'final_bias': bias_mags[-1],
        'poor_pct': 100 * np.sum(quat_errors >= 10.0) / len(quat_errors)
    }


def main():
    print("="*80)
    print("FLYING DRONE DATASET - QLinAcc Parameter Test")
    print("="*80)
    print("\nDataset starts with +1.5g takeoff acceleration")
    print("Testing if increasing QLinAcc (linear acceleration uncertainty) helps\n")

    # Test different QLinAcc values
    test_values = [1.0, 5.0, 10.0, 20.0, 50.0]

    results = []
    for qlinacc in test_values:
        print(f"Testing QLinAcc = {qlinacc}...")
        result = test_drone_with_qlinacc(qlinacc)
        results.append(result)
        print(f"  Mean error: {result['mean_error']:.3f}°, Max: {result['max_error']:.3f}°, "
              f"Final bias: {result['final_bias']:.3f} dps, Poor (>10°): {result['poor_pct']:.1f}%")

    print("\n" + "="*80)
    print("RESULTS SUMMARY")
    print("="*80)
    print(f"\n{'QLinAcc':<10} {'Mean Error':<12} {'Max Error':<12} {'Final Bias':<12} {'Poor %':<10}")
    print("-"*80)

    for r in results:
        print(f"{r['qlinacc']:<10.1f} {r['mean_error']:<12.3f} {r['max_error']:<12.3f} {r['final_bias']:<12.3f} {r['poor_pct']:<10.1f}")

    # Find best
    best = min(results, key=lambda x: x['mean_error'])
    print("\n" + "="*80)
    print(f"BEST RESULT: QLinAcc = {best['qlinacc']}")
    print(f"  Mean error: {best['mean_error']:.3f}° (baseline: 13.735°)")
    print(f"  Improvement: {13.735 - best['mean_error']:.3f}° ({100*(13.735 - best['mean_error'])/13.735:.1f}%)")
    print("="*80)


if __name__ == '__main__':
    main()
