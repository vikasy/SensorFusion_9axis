#!/usr/bin/env python3
"""
Trace the magnetometer's role in initialization and first fusion
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def print_mag_state(sf, label=""):
    """Print magnetometer-related state"""
    print(f"\n{label}")
    print(f"  mag_data.count_avg: [{sf.mag_data.count_avg[0]:.6f}, {sf.mag_data.count_avg[1]:.6f}, {sf.mag_data.count_avg[2]:.6f}] µT")
    print(f"  mag_field_ref: [{sf.mag_field_ref[0]:.6f}, {sf.mag_field_ref[1]:.6f}, {sf.mag_field_ref[2]:.6f}]")
    mag_ref_norm = np.linalg.norm(sf.mag_field_ref)
    print(f"  mag_field_ref norm: {mag_ref_norm:.6f}")
    print(f"  mag_meas_updt_ts: {sf.mag_meas_updt_ts}")

def main():
    print("="*80)
    print("MAGNETOMETER ROLE IN INITIALIZATION AND FIRST FUSION")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\n{'─'*80}")
    print("INITIAL STATE:")
    print(f"{'─'*80}")
    print_mag_state(sf, "After __init__:")

    # Process samples until first fusion
    fusion_count = 0
    for idx in range(min(10, len(df))):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        print(f"\n{'='*80}")
        print(f"SAMPLE {idx}")
        print(f"{'='*80}")
        print(f"Input mag_counts: [{mag_counts[0]:7.0f}, {mag_counts[1]:7.0f}, {mag_counts[2]:7.0f}]")

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        print_mag_state(sf, f"After preprocessing:")

        if ready_gyro & 0x2:  # Gyro ready - fusion will run
            fusion_count += 1
            print(f"\n{'─'*80}")
            print(f"FUSION {fusion_count} - BEFORE run()")
            print(f"{'─'*80}")
            print(f"  orient_init: {sf.orient_init}")
            print(f"  quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
            print_mag_state(sf, "  Magnetometer state:")

            output = sf.run()

            print(f"\n{'─'*80}")
            print(f"FUSION {fusion_count} - AFTER run()")
            print(f"{'─'*80}")
            print(f"  orient_init: {sf.orient_init}")
            print(f"  quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
            print(f"  Output orientation: [yaw={output.orientation[0]:.3f}°, pitch={output.orientation[1]:.3f}°, roll={output.orientation[2]:.3f}°]")
            print_mag_state(sf, "  Magnetometer state:")

            # Check if mag update ran
            print(f"\n  Analysis:")
            if sf.mag_meas_updt_ts > 0:
                print(f"    ✓ Magnetometer update RAN (mag_meas_updt_ts updated)")
            else:
                print(f"    ✗ Magnetometer update DID NOT run (mag_meas_updt_ts still 0)")

            mag_ref_norm = np.linalg.norm(sf.mag_field_ref)
            if mag_ref_norm > 0:
                print(f"    ✓ Magnetometer reference INITIALIZED (norm={mag_ref_norm:.6f})")
            else:
                print(f"    ✗ Magnetometer reference NOT initialized (still zero)")

            if fusion_count >= 2:
                break

    print(f"\n{'='*80}")
    print("CHECKING INITIALIZATION CODE")
    print(f"{'='*80}")

    print(f"\nLooking at _init_orient_mag() in sensor_fusion_9axis.py:")
    print(f"  Lines 134-153:")
    print(f"  ```python")
    print(f"  def _init_orient_mag(self, accel_avg: np.ndarray, mag_avg: np.ndarray):")
    print(f"      # For now, just use the 6-axis tilt initialization")
    print(f"      # Magnetometer heading initialization needs more work")
    print(f"      # TODO: Implement proper tilt-compensated magnetometer initialization")
    print(f"      rot_mtx = self._tilt_rotation_matrix(accel_avg)")
    print(f"      self.rot_mtx_post = rot_mtx")
    print(f"      ...")
    print(f"  ```")
    print(f"\n  ⚠️  The magnetometer data (mag_avg) is PASSED to _init_orient_mag()")
    print(f"  ⚠️  BUT it is NOT USED in the function!")
    print(f"  ⚠️  Only accelerometer tilt is used for initialization")
    print(f"  ⚠️  Magnetometer heading initialization is marked as TODO")

    print(f"\n{'='*80}")
    print("MAGNETOMETER USAGE SUMMARY")
    print(f"{'='*80}")

    print(f"\n1. INITIALIZATION (first fusion):")
    print(f"   - Magnetometer data: AVAILABLE but NOT USED")
    print(f"   - Only accelerometer used for tilt initialization")
    print(f"   - Initial yaw defaults to 0° (no heading initialization)")

    print(f"\n2. MAGNETOMETER REFERENCE INITIALIZATION:")
    print(f"   - Happens during first measurement_update_mag()")
    print(f"   - Uses current orientation (from accel init) + mag measurement")
    print(f"   - Stores reference field in global frame")

    print(f"\n3. SUBSEQUENT FUSIONS:")
    print(f"   - Magnetometer used for heading correction")
    print(f"   - Compares measured field vs expected field")
    print(f"   - Corrects yaw drift from gyroscope")

if __name__ == '__main__':
    main()
