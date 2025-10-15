#!/usr/bin/env python3
"""
Trace magnetometer timestamp updates to understand why mag update only happens once
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
    print("MAGNETOMETER TIMESTAMP TRACE")
    print("="*80)

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
    max_fusions = 10

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        print(f"\n{'─'*80}")
        print(f"Sample {idx}: CSV timestamp = {timestamp}")
        print(f"{'─'*80}")

        # Preprocess sensors
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        print(f"After preprocessing:")
        print(f"  sf.acc_data.timestamp:       {sf.acc_data.timestamp}")
        print(f"  sf.gyro_data.timestamp:      {sf.gyro_data.timestamp}")
        print(f"  sf.mag_data.timestamp:       {sf.mag_data.timestamp}")
        print(f"  sf.mag_meas_updt_ts:         {sf.mag_meas_updt_ts}")
        print(f"  mag update will happen? {sf.mag_meas_updt_ts < sf.mag_data.timestamp}")

        if ready_gyro & 0x2:  # Gyro ready - fusion will run
            print(f"\n  → FUSION WILL RUN (gyro buffer full)")

            print(f"\nBefore run():")
            print(f"  sf.nom_updt_ts:     {sf.nom_updt_ts}")
            print(f"  sf.meas_updt_ts:    {sf.meas_updt_ts}")
            print(f"  sf.mag_meas_updt_ts: {sf.mag_meas_updt_ts}")

            output = sf.run()
            fusion_count += 1

            print(f"\nAfter run() - Fusion {fusion_count}:")
            print(f"  sf.nom_updt_ts:     {sf.nom_updt_ts} (time update)")
            print(f"  sf.meas_updt_ts:    {sf.meas_updt_ts} (accel update)")
            print(f"  sf.mag_meas_updt_ts: {sf.mag_meas_updt_ts} (mag update)")
            print(f"  output.orientation:  yaw={output.orientation[0]:.3f}°")

            # Check what happened
            if sf.mag_meas_updt_ts == timestamp:
                print(f"  ✓ Mag timestamp updated to {timestamp}")
            else:
                print(f"  ✗ Mag timestamp NOT updated (still {sf.mag_meas_updt_ts})")

    print(f"\n{'='*80}")
    print(f"TRACE COMPLETE - Processed {fusion_count} fusion cycles")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
