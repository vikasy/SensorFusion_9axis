#!/usr/bin/env python3
"""
QVACC Parameter Sweep Test

Tests the impact of varying SF_6XAG_QVACC (accelerometer quantization noise
in R matrix) on walking dataset performance.

R matrix = QVACC + QWACC + ((QVGYRO + QWGYRO) * DELTA_T²)

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
import sensor_fusion_6axis as sf6

def test_qvacc_value(qvacc_value):
    """Test walking dataset with specific QVACC value"""

    # Temporarily override QVACC
    original_qvacc = sf6.SF_6XAG_QVACC
    sf6.SF_6XAG_QVACC = qvacc_value

    # Load walking dataset
    dataset_path = '../../test/data/datasets/realistic/walking.csv'
    df = pd.read_csv(dataset_path)

    # Initialize fusion
    platform = SensorPlatform(INVENSENSE)
    fusion = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Override the meas_noise_var_acc with new QVACC
    fusion.meas_noise_var_acc = (qvacc_value + sf6.SF_6XAG_QWACC +
                                  ((sf6.SF_6XAG_QVGYRO + sf6.SF_6XAG_QWGYRO) * sf6.SF_DELTA_T_SQ))

    errors = []
    biases = []

    # Process samples
    for idx in range(len(df)):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        fusion.preprocess_sensor_data(0, accel_counts, timestamp)
        ready = fusion.preprocess_sensor_data(1, gyro_counts, timestamp)
        fusion.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready & 0x2:  # Gyro buffer ready
            output = fusion.run()

            # Compute error
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            dot_product = np.abs(np.dot(gt_quat, py_quat))
            dot_product = np.clip(dot_product, 0.0, 1.0)
            angle_error = 2.0 * np.arccos(dot_product) * (180.0 / np.pi)

            errors.append(angle_error)
            biases.append(np.linalg.norm(fusion.bias_post_s))

    # Restore original value
    sf6.SF_6XAG_QVACC = original_qvacc

    # Compute metrics
    errors = np.array(errors)
    biases = np.array(biases)

    return {
        'qvacc': qvacc_value,
        'R_total': fusion.meas_noise_var_acc,
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'final_bias': biases[-1] if len(biases) > 0 else 0.0
    }


def main():
    print("=" * 80)
    print("QVACC PARAMETER SWEEP - Walking Dataset")
    print("=" * 80)
    print("\nTesting impact of accelerometer quantization noise (QVACC) in R matrix")
    print(f"Current QWACC: {sf6.SF_6XAG_QWACC}")
    print(f"Current QVGYRO: {sf6.SF_6XAG_QVGYRO}")
    print(f"Current QWGYRO: {sf6.SF_6XAG_QWGYRO}")
    print(f"DELTA_T²: {sf6.SF_DELTA_T_SQ}")
    print()

    # Test range: from very small to very large QVACC
    qvacc_values = [
        2e-6,    # Current value (baseline)
        1e-5,    # 5× larger
        1e-4,    # 50× larger
        1e-3,    # 500× larger
        1e-2,    # 5000× larger
        0.1,     # 50000× larger
        1.0,     # 500000× larger
        10.0,    # Equal to QWACC
        100.0,   # 10× QWACC
    ]

    results = []

    for qvacc in qvacc_values:
        print(f"Testing QVACC = {qvacc:.2e}...", end=" ", flush=True)
        result = test_qvacc_value(qvacc)
        results.append(result)
        print(f"Mean error: {result['mean_error']:.2f}°, R_total: {result['R_total']:.2e}")

    # Print summary table
    print("\n" + "=" * 80)
    print("RESULTS SUMMARY")
    print("=" * 80)
    print(f"{'QVACC':<12} {'R Total':<12} {'Mean Error':<12} {'Max Error':<12} {'Final Bias':<12}")
    print("-" * 80)

    for r in results:
        qvacc_str = f"{r['qvacc']:.2e}" if r['qvacc'] < 0.01 else f"{r['qvacc']:.2f}"
        print(f"{qvacc_str:<12} {r['R_total']:<12.2e} {r['mean_error']:<12.2f} {r['max_error']:<12.2f} {r['final_bias']:<12.4f}")

    # Find best
    best = min(results, key=lambda x: x['mean_error'])
    print("\n" + "=" * 80)
    print(f"BEST: QVACC = {best['qvacc']:.2e}, Mean Error = {best['mean_error']:.2f}°")
    print("=" * 80)


if __name__ == '__main__':
    main()
