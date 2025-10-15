#!/usr/bin/env python3
"""
Detailed trace of magnetometer update calls and disturbance detection
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# Monkey-patch to trace ALL calls
original_run = SensorFusion9Axis.run
call_count = 0

def traced_run(self):
    global call_count
    call_count += 1

    print(f"\n{'─'*80}")
    print(f"run() call #{call_count}")
    print(f"{'─'*80}")

    # Check mag update condition
    mag_will_update = self.mag_meas_updt_ts < self.mag_data.timestamp
    print(f"Mag update check: mag_meas_updt_ts ({self.mag_meas_updt_ts}) < mag_data.timestamp ({self.mag_data.timestamp}) = {mag_will_update}")

    if mag_will_update:
        mag_measured = self.mag_data.count_avg
        mag_ref_initialized = np.linalg.norm(self.mag_field_ref) > 1e-12

        print(f"  mag_measured: [{mag_measured[0]:.3f}, {mag_measured[1]:.3f}, {mag_measured[2]:.3f}] µT")
        print(f"  mag_field_ref: [{self.mag_field_ref[0]:.3f}, {self.mag_field_ref[1]:.3f}, {self.mag_field_ref[2]:.3f}]")
        print(f"  mag_ref_initialized: {mag_ref_initialized}")

        if mag_ref_initialized:
            # Normalize for disturbance check (like the fix in run())
            mag_norm = np.linalg.norm(mag_measured)
            if mag_norm > 1e-12:
                mag_measured_normalized = mag_measured / mag_norm
                disturbance = self.detect_mag_disturbance(mag_measured_normalized, self.mag_field_ref, 0.3)
            else:
                disturbance = True
            print(f"  mag_measured (normalized): [{mag_measured_normalized[0]:.6f}, {mag_measured_normalized[1]:.6f}, {mag_measured_normalized[2]:.6f}]")
            print(f"  disturbance detected: {disturbance}")
        else:
            print(f"  disturbance check skipped (reference not initialized)")

    # Call original
    result = original_run(self)

    if mag_will_update:
        print(f"After run(): mag_meas_updt_ts = {self.mag_meas_updt_ts}")

    return result

SensorFusion9Axis.run = traced_run

def main():
    print("="*80)
    print("DETAILED MAGNETOMETER UPDATE TRACE")
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
    max_fusions = 5

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:  # Gyro ready
            print(f"\n{'#'*80}")
            print(f"FUSION CYCLE {fusion_count + 1} (Sample {idx})")
            print(f"{'#'*80}")

            output = sf.run()
            fusion_count += 1

            print(f"\nOutput: yaw={output.orientation[0]:.3f}°, pitch={output.orientation[1]:.3f}°, roll={output.orientation[2]:.3f}°")

    print(f"\n{'='*80}")
    print(f"TRACE COMPLETE - {fusion_count} fusion cycles")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
