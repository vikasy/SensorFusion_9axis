#!/usr/bin/env python3
"""
Debug why second fusion doesn't update quaternion
Focus on timestamp values
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("=" * 80)
    print("TIMESTAMP DEBUG - Why doesn't second fusion update?")
    print("=" * 80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nDataset: {dataset_path}\n")

    fusion_count = 0
    max_fusions = 3  # Process 3 fusions

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        print(f"\n{'─'*80}")
        print(f"Sample {idx}: timestamp={timestamp} ns")
        print(f"{'─'*80}")

        # Preprocess
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        print(f"  ready_acc=0x{ready_acc:02X}, ready_gyro=0x{ready_gyro:02X}, ready_mag=0x{ready_mag:02X}")

        # Print timestamps BEFORE run()
        print(f"\nTimestamps BEFORE run():")
        print(f"  nom_updt_ts:      {sf.nom_updt_ts}")
        print(f"  meas_updt_ts:     {sf.meas_updt_ts}")
        print(f"  mag_meas_updt_ts: {sf.mag_meas_updt_ts}")
        print(f"  gyro_data.timestamp:  {sf.gyro_data.timestamp}")
        print(f"  acc_data.timestamp:   {sf.acc_data.timestamp}")
        print(f"  mag_data.timestamp:   {sf.mag_data.timestamp}")

        print(f"\nConditions:")
        print(f"  time_update will run?  {sf.nom_updt_ts < sf.gyro_data.timestamp}")
        print(f"  meas_update will run?  {sf.meas_updt_ts < sf.acc_data.timestamp}")
        print(f"  mag_update will run?   {sf.mag_meas_updt_ts < sf.mag_data.timestamp}")

        if ready_gyro & 0x2:  # Gyro ready
            quat_before = np.array([sf.quat_post.q0, sf.quat_post.q1, sf.quat_post.q2, sf.quat_post.q3])

            output = sf.run()
            fusion_count += 1

            quat_after = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

            print(f"\n=== FUSION {fusion_count} ===")
            print(f"Quat BEFORE: [{quat_before[0]:.8f}, {quat_before[1]:.8f}, {quat_before[2]:.8f}, {quat_before[3]:.8f}]")
            print(f"Quat AFTER:  [{quat_after[0]:.8f}, {quat_after[1]:.8f}, {quat_after[2]:.8f}, {quat_after[3]:.8f}]")

            quat_changed = not np.allclose(quat_before, quat_after, atol=1e-10)
            print(f"Quat changed: {quat_changed}")

            print(f"\nTimestamps AFTER run():")
            print(f"  nom_updt_ts:      {sf.nom_updt_ts}")
            print(f"  meas_updt_ts:     {sf.meas_updt_ts}")
            print(f"  mag_meas_updt_ts: {sf.mag_meas_updt_ts}")

    print(f"\n{'='*80}")
    print(f"Processed {fusion_count} fusions")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
