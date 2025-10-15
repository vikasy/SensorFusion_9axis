#!/usr/bin/env python3
"""
Trace which update types happen at each fusion cycle
- Time update (gyro integration)
- Measurement update (accel correction)
- Mag update (mag correction)
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
    print("UPDATE TYPE TRACE - Which updates happen at each fusion?")
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
    print("Legend:")
    print("  T = Time update (gyro integration)")
    print("  A = Accel measurement update")
    print("  M = Mag measurement update")
    print()

    fusion_count = 0
    max_samples = 20

    for idx in range(len(df)):
        if idx >= max_samples:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        # Capture state BEFORE preprocessing
        nom_updt_ts_before = sf.nom_updt_ts
        meas_updt_ts_before = sf.meas_updt_ts
        mag_meas_updt_ts_before = sf.mag_meas_updt_ts

        # Preprocess sensors
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        # Capture state AFTER preprocessing (before run)
        gyro_ts_after_preprocess = sf.gyro_data.timestamp
        acc_ts_after_preprocess = sf.acc_data.timestamp
        mag_ts_after_preprocess = sf.mag_data.timestamp

        gyro_ready = (ready_gyro & 0x2) != 0

        if gyro_ready:
            fusion_count += 1

            # Check what will run based on timestamp comparisons
            will_run_time_update = nom_updt_ts_before < gyro_ts_after_preprocess
            will_run_accel_update = meas_updt_ts_before < acc_ts_after_preprocess
            will_run_mag_update = mag_meas_updt_ts_before < mag_ts_after_preprocess

            print(f"Sample {idx:2d} → Fusion #{fusion_count}")
            print(f"  Timestamps after preprocessing:")
            print(f"    gyro_data.timestamp:  {gyro_ts_after_preprocess}")
            print(f"    acc_data.timestamp:   {acc_ts_after_preprocess}")
            print(f"    mag_data.timestamp:   {mag_ts_after_preprocess}")
            print(f"  Previous update timestamps:")
            print(f"    nom_updt_ts:          {nom_updt_ts_before}")
            print(f"    meas_updt_ts:         {meas_updt_ts_before}")
            print(f"    mag_meas_updt_ts:     {mag_meas_updt_ts_before}")
            print(f"  Will execute:")
            print(f"    Time update (gyro):   {will_run_time_update}  (nom_updt_ts < gyro_ts)")
            print(f"    Accel update:         {will_run_accel_update}  (meas_updt_ts < acc_ts)")
            print(f"    Mag update:           {will_run_mag_update}  (mag_meas_updt_ts < mag_ts)")

            # Show update pattern
            pattern = ""
            pattern += "T" if will_run_time_update else "-"
            pattern += "A" if will_run_accel_update else "-"
            pattern += "M" if will_run_mag_update else "-"
            print(f"  Update pattern: [{pattern}]")

            # Actually run
            output = sf.run()
            print()

    print(f"{'='*80}")
    print(f"TRACE COMPLETE")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
