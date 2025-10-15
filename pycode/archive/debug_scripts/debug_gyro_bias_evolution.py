#!/usr/bin/env python3
"""
Track gyro bias estimation over time to see if it's diverging
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
    print("GYRO BIAS EVOLUTION TRACKING")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nDataset: {dataset_path}")
    print(f"Expected: Gyro bias should be ~0 dps for static data\n")

    fusion_count = 0
    max_samples = 100

    print(f"{'Fusion':>6} {'Sample':>6} {'Bias X':>12} {'Bias Y':>12} {'Bias Z':>12} {'|Bias|':>12} {'ΔBias':>12}")
    print(f"{'#':>6} {'Index':>6} {'(dps)':>12} {'(dps)':>12} {'(dps)':>12} {'(dps)':>12} {'(dps)':>12}")
    print("-"*84)

    prev_bias = None
    bias_history = []

    for idx in range(len(df)):
        if idx >= max_samples:
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

            # Get current bias
            bias = sf.bias_post_s.copy()
            bias_mag = np.linalg.norm(bias)

            # Compute change from previous
            if prev_bias is not None:
                delta_bias = np.linalg.norm(bias - prev_bias)
            else:
                delta_bias = 0.0

            bias_history.append({
                'fusion': fusion_count,
                'sample': idx,
                'bias': bias.copy(),
                'bias_mag': bias_mag,
                'delta': delta_bias
            })

            # Print every fusion for first 10, then every 5th
            if fusion_count <= 10 or fusion_count % 5 == 0:
                print(f"{fusion_count:6d} {idx:6d} {bias[0]:+12.6f} {bias[1]:+12.6f} {bias[2]:+12.6f} {bias_mag:12.6f} {delta_bias:12.6f}")

            prev_bias = bias

    print("-"*84)

    # Statistical analysis
    print("\n" + "="*80)
    print("BIAS EVOLUTION ANALYSIS")
    print("="*80)

    if len(bias_history) > 0:
        biases = np.array([h['bias'] for h in bias_history])
        bias_mags = np.array([h['bias_mag'] for h in bias_history])
        deltas = np.array([h['delta'] for h in bias_history])

        print(f"\nBias Magnitude (|bias|):")
        print(f"  Initial: {bias_mags[0]:.6f} dps")
        print(f"  Final:   {bias_mags[-1]:.6f} dps")
        print(f"  Mean:    {bias_mags.mean():.6f} dps")
        print(f"  Std:     {bias_mags.std():.6f} dps")
        print(f"  Max:     {bias_mags.max():.6f} dps")

        print(f"\nBias Change Per Update (Δbias):")
        print(f"  Mean:    {deltas[1:].mean():.6f} dps")
        print(f"  Std:     {deltas[1:].std():.6f} dps")
        print(f"  Max:     {deltas[1:].max():.6f} dps")

        print(f"\nBias Components:")
        print(f"  X-axis: {biases[0,0]:+.6f} → {biases[-1,0]:+.6f} dps (change: {biases[-1,0]-biases[0,0]:+.6f})")
        print(f"  Y-axis: {biases[0,1]:+.6f} → {biases[-1,1]:+.6f} dps (change: {biases[-1,1]-biases[0,1]:+.6f})")
        print(f"  Z-axis: {biases[0,2]:+.6f} → {biases[-1,2]:+.6f} dps (change: {biases[-1,2]-biases[0,2]:+.6f})")

        # Check for divergence
        print("\n" + "="*80)
        print("VERDICT")
        print("="*80)

        # Check if bias is growing
        first_half_mean = bias_mags[:len(bias_mags)//2].mean()
        second_half_mean = bias_mags[len(bias_mags)//2:].mean()
        growth = second_half_mean - first_half_mean

        print(f"\nBias magnitude:")
        print(f"  First half mean:  {first_half_mean:.6f} dps")
        print(f"  Second half mean: {second_half_mean:.6f} dps")
        print(f"  Growth:           {growth:+.6f} dps")

        if abs(growth) < 0.01:
            print(f"\n✓ Bias is STABLE (growth < 0.01 dps)")
        elif abs(growth) < 0.1:
            print(f"\n⚠ Bias showing SLOW DRIFT (growth < 0.1 dps)")
        else:
            print(f"\n✗ Bias is DIVERGING (growth >= 0.1 dps)")

        # Check actual gyro data for reference
        print("\n" + "="*80)
        print("REFERENCE: Actual Gyro Readings")
        print("="*80)

        # Calculate mean gyro from first 20 samples (static)
        gyro_samples = []
        for idx in range(min(20, len(df))):
            row = df.iloc[idx]
            gyro = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']]) * platform.gyro_scale_factor
            gyro_samples.append(gyro)

        gyro_samples = np.array(gyro_samples)
        gyro_mean = gyro_samples.mean(axis=0)
        gyro_std = gyro_samples.std(axis=0)

        print(f"\nActual gyro measurements (first 20 samples):")
        print(f"  Mean:  [{gyro_mean[0]:+.6f}, {gyro_mean[1]:+.6f}, {gyro_mean[2]:+.6f}] dps")
        print(f"  Std:   [{gyro_std[0]:.6f}, {gyro_std[1]:.6f}, {gyro_std[2]:.6f}] dps")

        print(f"\nEstimated bias vs actual gyro mean:")
        print(f"  Bias - Mean: [{biases[-1,0]-gyro_mean[0]:+.6f}, {biases[-1,1]-gyro_mean[1]:+.6f}, {biases[-1,2]-gyro_mean[2]:+.6f}] dps")

        # The estimated bias should roughly equal the mean gyro for static data
        bias_error = np.linalg.norm(biases[-1] - gyro_mean)
        print(f"  |Bias - Mean|: {bias_error:.6f} dps")

        if bias_error < 0.1:
            print(f"\n✓ Bias estimate is ACCURATE (matches actual gyro mean)")
        elif bias_error < 0.5:
            print(f"\n⚠ Bias estimate has MODERATE ERROR")
        else:
            print(f"\n✗ Bias estimate is INACCURATE (large discrepancy from actual)")

    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
