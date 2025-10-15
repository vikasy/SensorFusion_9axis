#!/usr/bin/env python3
"""
Super detailed debug showing count_buff, count_avg, and quaternion integration
for first TWO fusion cycles of 9-axis fusion
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def print_buffer_state(sf, row=None, platform=None):
    """Print detailed buffer and average states"""
    print(f"\n{'─'*80}")
    print("BUFFER STATE:")
    print(f"{'─'*80}")

    # Print count_buff for all sensors
    print("\ncount_buff (raw buffered samples):")
    print(f"  accel count_buff:")
    for i in range(4):
        buf = sf.acc_data.count_buff[i]
        print(f"    [{i}]: [{buf[0]:7.0f}, {buf[1]:7.0f}, {buf[2]:7.0f}]")

    print(f"  gyro count_buff:")
    for i in range(4):
        buf = sf.gyro_data.count_buff[i]
        print(f"    [{i}]: [{buf[0]:7.0f}, {buf[1]:7.0f}, {buf[2]:7.0f}]")

    print(f"  mag count_buff:")
    buf = sf.mag_data.count_buff[0]
    print(f"    [0]: [{buf[0]:7.0f}, {buf[1]:7.0f}, {buf[2]:7.0f}]")

    # Print count_avg (scaled values)
    print("\ncount_avg (scaled/computed values):")
    acc_avg = sf.acc_data.count_avg
    print(f"  accel count_avg: [{acc_avg[0]:10.6f}, {acc_avg[1]:10.6f}, {acc_avg[2]:10.6f}] g")

    gyro_avg = sf.gyro_data.count_avg
    print(f"  gyro count_avg:  [{gyro_avg[0]:10.6f}, {gyro_avg[1]:10.6f}, {gyro_avg[2]:10.6f}] dps (unused)")

    mag_avg = sf.mag_data.count_avg
    print(f"  mag count_avg:   [{mag_avg[0]:10.6f}, {mag_avg[1]:10.6f}, {mag_avg[2]:10.6f}] µT")

    # Verify against input if provided
    if row is not None and platform is not None:
        print("\nVERIFICATION (input CSV vs buffered):")
        accel_csv = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']])
        gyro_csv = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']])
        mag_csv = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']])

        # Latest buffered sample should match CSV input
        accel_buf = sf.acc_data.count_buff[0]
        gyro_buf = sf.gyro_data.count_buff[0]
        mag_buf = sf.mag_data.count_buff[0]

        print(f"  Accel CSV:    [{accel_csv[0]:7.0f}, {accel_csv[1]:7.0f}, {accel_csv[2]:7.0f}]")
        print(f"  Accel buf[0]: [{accel_buf[0]:7.0f}, {accel_buf[1]:7.0f}, {accel_buf[2]:7.0f}] ✓" if np.allclose(accel_csv, accel_buf, atol=1) else f"  Accel buf[0]: [{accel_buf[0]:7.0f}, {accel_buf[1]:7.0f}, {accel_buf[2]:7.0f}] ✗")

        print(f"  Gyro CSV:     [{gyro_csv[0]:7.0f}, {gyro_csv[1]:7.0f}, {gyro_csv[2]:7.0f}]")
        print(f"  Gyro buf[0]:  [{gyro_buf[0]:7.0f}, {gyro_buf[1]:7.0f}, {gyro_buf[2]:7.0f}] ✓" if np.allclose(gyro_csv, gyro_buf, atol=1) else f"  Gyro buf[0]:  [{gyro_buf[0]:7.0f}, {gyro_buf[1]:7.0f}, {gyro_buf[2]:7.0f}] ✗")

        print(f"  Mag CSV:      [{mag_csv[0]:7.0f}, {mag_csv[1]:7.0f}, {mag_csv[2]:7.0f}]")
        print(f"  Mag buf[0]:   [{mag_buf[0]:7.0f}, {mag_buf[1]:7.0f}, {mag_buf[2]:7.0f}] ✓" if np.allclose(mag_csv, mag_buf, atol=1) else f"  Mag buf[0]:   [{mag_buf[0]:7.0f}, {mag_buf[1]:7.0f}, {mag_buf[2]:7.0f}] ✗")

        # Verify scaled values
        accel_g_csv = accel_csv * platform.accel_scale_factor
        gyro_dps_csv = gyro_csv * platform.gyro_scale_factor
        mag_ut_csv = mag_csv * platform.mag_scale_factor

        print(f"\n  Accel (g) CSV:      [{accel_g_csv[0]:10.6f}, {accel_g_csv[1]:10.6f}, {accel_g_csv[2]:10.6f}]")
        print(f"  Gyro (dps) CSV:     [{gyro_dps_csv[0]:10.6f}, {gyro_dps_csv[1]:10.6f}, {gyro_dps_csv[2]:10.6f}]")
        print(f"  Mag (µT) CSV:       [{mag_ut_csv[0]:10.6f}, {mag_ut_csv[1]:10.6f}, {mag_ut_csv[2]:10.6f}]")

def print_quaternion_state(sf, label=""):
    """Print quaternion state"""
    print(f"\n{label}")
    print(f"  quat: [{sf.quat_post.q0:10.8f}, {sf.quat_post.q1:10.8f}, {sf.quat_post.q2:10.8f}, {sf.quat_post.q3:10.8f}]")
    # Compute norm to verify normalization
    norm = np.sqrt(sf.quat_post.q0**2 + sf.quat_post.q1**2 + sf.quat_post.q2**2 + sf.quat_post.q3**2)
    print(f"  norm: {norm:.10f}")

def main():
    print("=" * 80)
    print("DETAILED BUFFER AND QUATERNION INTEGRATION DEBUG")
    print("First TWO fusion cycles")
    print("=" * 80)

    # Load static dataset for simplicity
    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nDataset: {dataset_path}")
    print(f"Platform: INVENSENSE")
    print(f"  accel_scale: {platform.accel_scale_factor:.10f} g/count")
    print(f"  gyro_scale: {platform.gyro_scale_factor:.10f} dps/count")
    print(f"  mag_scale: {platform.mag_scale_factor:.10f} µT/count")

    print_quaternion_state(sf, "INITIAL QUATERNION STATE (after __init__):")
    print_buffer_state(sf)

    fusion_count = 0
    max_fusions = 2

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

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

        mag_counts = np.array([
            row['mag_x_counts'],
            row['mag_y_counts'],
            row['mag_z_counts']
        ], dtype=np.float64)

        print(f"\n{'#'*80}")
        print(f"SAMPLE {idx} (timestamp: {timestamp} ns)")
        print(f"{'#'*80}")
        print(f"Input from CSV:")
        print(f"  accel_counts: [{accel_counts[0]:7.0f}, {accel_counts[1]:7.0f}, {accel_counts[2]:7.0f}]")
        print(f"  gyro_counts:  [{gyro_counts[0]:7.0f}, {gyro_counts[1]:7.0f}, {gyro_counts[2]:7.0f}]")
        print(f"  mag_counts:   [{mag_counts[0]:7.0f}, {mag_counts[1]:7.0f}, {mag_counts[2]:7.0f}]")

        # Step 1: Preprocess sensors
        print(f"\n--- Preprocessing sensors ---")
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        print(f"ready_acc:  0x{ready_acc:02X}")
        print(f"ready_gyro: 0x{ready_gyro:02X}")
        print(f"ready_mag:  0x{ready_mag:02X}")

        print_buffer_state(sf, row, platform)

        if ready_gyro & 0x2:  # Gyro ready - will trigger fusion
            print(f"\n{'='*80}")
            print(f"FUSION CYCLE {fusion_count + 1} - GYRO READY, CALLING run()")
            print(f"{'='*80}")

            print_quaternion_state(sf, "Quaternion BEFORE run():")

            # Need to instrument time_update to see quaternion changes
            # For now, just call run() and show before/after
            output = sf.run()
            fusion_count += 1

            print_quaternion_state(sf, "Quaternion AFTER run():")

            print(f"\nOutput:")
            print(f"  quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
            print(f"  orientation: [yaw={output.orientation[0]:7.3f}°, pitch={output.orientation[1]:7.3f}°, roll={output.orientation[2]:7.3f}°]")

    print(f"\n{'='*80}")
    print(f"DEBUG COMPLETE - Processed {fusion_count} fusions")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
