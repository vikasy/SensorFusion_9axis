#!/usr/bin/env python3
"""
Super detailed debug of first TWO fusion cycles for 9-axis
Print ALL internal variables after every step
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def print_full_state(sf, label=""):
    """Print complete internal state"""
    print(f"\n{'='*80}")
    print(f"{label}")
    print(f"{'='*80}")
    print(f"orient_init: {sf.orient_init}")
    print(f"quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
    print(f"rot_mtx_post:\n{sf.rot_mtx_post}")
    print(f"bias_post_s: [{sf.bias_post_s[0]:.8f}, {sf.bias_post_s[1]:.8f}, {sf.bias_post_s[2]:.8f}] dps")
    print(f"mag_field_ref: [{sf.mag_field_ref[0]:.8f}, {sf.mag_field_ref[1]:.8f}, {sf.mag_field_ref[2]:.8f}]")
    print(f"mag_meas_updt_ts: {sf.mag_meas_updt_ts}")
    print(f"nom_updt_ts: {sf.nom_updt_ts}")
    print(f"meas_updt_ts: {sf.meas_updt_ts}")
    print(f"acc_data.timestamp: {sf.acc_data.timestamp}")
    print(f"gyro_data.timestamp: {sf.gyro_data.timestamp}")
    print(f"mag_data.timestamp: {sf.mag_data.timestamp}")
    print(f"acc_data.count_avg: [{sf.acc_data.count_avg[0]:.6f}, {sf.acc_data.count_avg[1]:.6f}, {sf.acc_data.count_avg[2]:.6f}]")
    print(f"gyro_data.count_avg: [{sf.gyro_data.count_avg[0]:.6f}, {sf.gyro_data.count_avg[1]:.6f}, {sf.gyro_data.count_avg[2]:.6f}]")
    print(f"mag_data.count_avg: [{sf.mag_data.count_avg[0]:.6f}, {sf.mag_data.count_avg[1]:.6f}, {sf.mag_data.count_avg[2]:.6f}]")

def main():
    print("=" * 80)
    print("SUPER DETAILED DEBUG: FIRST TWO FUSION CYCLES")
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

    print_full_state(sf, "INITIAL STATE (after __init__)")

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
        print(f"SAMPLE {idx}")
        print(f"{'#'*80}")
        print(f"timestamp_ns: {timestamp}")
        print(f"accel_counts: [{accel_counts[0]:.0f}, {accel_counts[1]:.0f}, {accel_counts[2]:.0f}]")
        print(f"gyro_counts:  [{gyro_counts[0]:.0f}, {gyro_counts[1]:.0f}, {gyro_counts[2]:.0f}]")
        print(f"mag_counts:   [{mag_counts[0]:.0f}, {mag_counts[1]:.0f}, {mag_counts[2]:.0f}]")

        # Step 1: Preprocess accelerometer
        print(f"\n--- Step 1: preprocess_sensor_data(ACC) ---")
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        print(f"ready_acc = 0x{ready_acc:02X}")
        print(f"acc_data.count_avg: [{sf.acc_data.count_avg[0]:.6f}, {sf.acc_data.count_avg[1]:.6f}, {sf.acc_data.count_avg[2]:.6f}]")
        print(f"acc_data.timestamp: {sf.acc_data.timestamp}")

        # Step 2: Preprocess gyroscope
        print(f"\n--- Step 2: preprocess_sensor_data(GYRO) ---")
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        print(f"ready_gyro = 0x{ready_gyro:02X}")
        print(f"gyro_data.count_avg: [{sf.gyro_data.count_avg[0]:.6f}, {sf.gyro_data.count_avg[1]:.6f}, {sf.gyro_data.count_avg[2]:.6f}]")
        print(f"gyro_data.timestamp: {sf.gyro_data.timestamp}")

        # Step 3: Preprocess magnetometer
        print(f"\n--- Step 3: preprocess_sensor_data(MAG) ---")
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)
        print(f"ready_mag = 0x{ready_mag:02X}")
        print(f"mag_data.count_avg: [{sf.mag_data.count_avg[0]:.6f}, {sf.mag_data.count_avg[1]:.6f}, {sf.mag_data.count_avg[2]:.6f}]")
        print(f"mag_data.timestamp: {sf.mag_data.timestamp}")

        if ready_gyro & 0x2:  # Gyro ready
            print(f"\n--- Step 4: run() - GYRO READY, CALLING FUSION ---")

            # Check timestamps before run
            print(f"\nTimestamps before run():")
            print(f"  nom_updt_ts: {sf.nom_updt_ts}")
            print(f"  meas_updt_ts: {sf.meas_updt_ts}")
            print(f"  mag_meas_updt_ts: {sf.mag_meas_updt_ts}")
            print(f"  gyro_data.timestamp: {sf.gyro_data.timestamp}")
            print(f"  acc_data.timestamp: {sf.acc_data.timestamp}")
            print(f"  mag_data.timestamp: {sf.mag_data.timestamp}")

            print(f"\nWill time_update run? {sf.nom_updt_ts < sf.gyro_data.timestamp}")
            print(f"Will measurement_update run? {sf.meas_updt_ts < sf.acc_data.timestamp}")
            print(f"Will measurement_update_mag run? {sf.mag_meas_updt_ts < sf.mag_data.timestamp}")

            output = sf.run()
            fusion_count += 1

            print(f"\n--- AFTER run() - Fusion {fusion_count} complete ---")
            print(f"output.quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
            print(f"output.orientation: [yaw={output.orientation[0]:7.3f}°, pitch={output.orientation[1]:7.3f}°, roll={output.orientation[2]:7.3f}°]")

            print_full_state(sf, f"STATE AFTER FUSION {fusion_count}")

    print(f"\n{'='*80}")
    print(f"DEBUG COMPLETE - Processed {fusion_count} fusions")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
