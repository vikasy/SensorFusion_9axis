#!/usr/bin/env python3
"""
QVACC Parameter Test on All Realistic Datasets

Tests QVACC = 1.0 vs baseline (2e-6) on all realistic datasets

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

DATASETS = ['walking', 'handheld_device', 'climbing_stairs', 'flying_drone', 'driving_in_car']

def test_dataset_with_qvacc(dataset_name, qvacc_value):
    """Test dataset with specific QVACC value"""

    # Temporarily override QVACC
    original_qvacc = sf6.SF_6XAG_QVACC
    sf6.SF_6XAG_QVACC = qvacc_value

    # Load dataset
    dataset_path = f'../../test/data/datasets/realistic/{dataset_name}.csv'
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
        'dataset': dataset_name,
        'qvacc': qvacc_value,
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'final_bias': biases[-1] if len(biases) > 0 else 0.0
    }


def main():
    print("=" * 80)
    print("QVACC COMPARISON - All Realistic Datasets")
    print("=" * 80)
    print("\nTesting QVACC = 1.0 (best from sweep) vs 2e-6 (baseline)")
    print()

    qvacc_values = [2e-6, 1.0]  # Baseline vs best from sweep

    results = []

    for dataset in DATASETS:
        print(f"\n{dataset}:")
        print("-" * 40)
        for qvacc in qvacc_values:
            print(f"  QVACC = {qvacc:.2e}...", end=" ", flush=True)
            result = test_dataset_with_qvacc(dataset, qvacc)
            results.append(result)
            print(f"Mean: {result['mean_error']:.2f}°, Max: {result['max_error']:.2f}°, Bias: {result['final_bias']:.4f}")

    # Print comparison table
    print("\n" + "=" * 80)
    print("COMPARISON TABLE")
    print("=" * 80)

    for dataset in DATASETS:
        print(f"\n{dataset.upper()}:")
        baseline = [r for r in results if r['dataset'] == dataset and r['qvacc'] == 2e-6][0]
        modified = [r for r in results if r['dataset'] == dataset and r['qvacc'] == 1.0][0]

        print(f"  Baseline (QVACC=2e-6):  Mean={baseline['mean_error']:.2f}°, Max={baseline['max_error']:.2f}°")
        print(f"  Modified (QVACC=1.0):   Mean={modified['mean_error']:.2f}°, Max={modified['max_error']:.2f}°")

        delta = modified['mean_error'] - baseline['mean_error']
        if abs(delta) < 0.05:
            print(f"  Change: ≈0° (no significant difference)")
        elif delta > 0:
            print(f"  Change: +{delta:.2f}° (worse)")
        else:
            print(f"  Change: {delta:.2f}° (better)")

    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
