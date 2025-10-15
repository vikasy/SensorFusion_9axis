#!/usr/bin/env python3
"""
Detailed debug of measurement updates (accel and mag)
Shows complete fusion cycle with all update steps
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# Monkey-patch to add debug output to measurement updates
original_meas_update = SensorFusion9Axis.measurement_update
original_mag_update = SensorFusion9Axis.measurement_update_mag

def debug_measurement_update(self, debug=False):
    """Wrapper to add detailed debug output"""
    print(f"\n{'='*80}")
    print("ACCELEROMETER MEASUREMENT UPDATE")
    print(f"{'='*80}")

    print(f"\nState BEFORE accel update:")
    print(f"  quat_post: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")
    print(f"  bias_post_s: [{self.bias_post_s[0]:.6f}, {self.bias_post_s[1]:.6f}, {self.bias_post_s[2]:.6f}] dps")

    # Call original
    result = original_meas_update(self, debug=debug)

    print(f"\nState AFTER accel update:")
    print(f"  quat_post: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")
    print(f"  bias_post_s: [{self.bias_post_s[0]:.6f}, {self.bias_post_s[1]:.6f}, {self.bias_post_s[2]:.6f}] dps")
    print(f"  ornt_err_post_s: [{self.ornt_err_post_s[0]:.6f}, {self.ornt_err_post_s[1]:.6f}, {self.ornt_err_post_s[2]:.6f}]")
    print(f"  bias_err_post_s: [{self.bias_err_post_s[0]:.6f}, {self.bias_err_post_s[1]:.6f}, {self.bias_err_post_s[2]:.6f}]")

    return result

def debug_mag_update(self):
    """Wrapper to add detailed debug output"""
    print(f"\n{'='*80}")
    print("MAGNETOMETER MEASUREMENT UPDATE")
    print(f"{'='*80}")

    # Get mag data before update
    mag_measured_raw = self.mag_data.count_avg.copy()
    mag_ref_before = self.mag_field_ref.copy()
    quat_before = [self.quat_post.q0, self.quat_post.q1, self.quat_post.q2, self.quat_post.q3]

    print(f"\nMagnetometer data:")
    print(f"  mag_measured (µT): [{mag_measured_raw[0]:.6f}, {mag_measured_raw[1]:.6f}, {mag_measured_raw[2]:.6f}]")
    print(f"  mag_field_ref: [{mag_ref_before[0]:.6f}, {mag_ref_before[1]:.6f}, {mag_ref_before[2]:.6f}]")

    mag_ref_initialized = np.linalg.norm(mag_ref_before) > 1e-12
    print(f"  Reference initialized: {mag_ref_initialized}")

    print(f"\nState BEFORE mag update:")
    print(f"  quat_post: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")
    print(f"  bias_post_s: [{self.bias_post_s[0]:.6f}, {self.bias_post_s[1]:.6f}, {self.bias_post_s[2]:.6f}] dps")

    # Call original
    result = original_mag_update(self)

    quat_after = [self.quat_post.q0, self.quat_post.q1, self.quat_post.q2, self.quat_post.q3]
    quat_changed = any(abs(quat_after[i] - quat_before[i]) > 1e-10 for i in range(4))

    print(f"\nState AFTER mag update:")
    print(f"  quat_post: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")
    print(f"  bias_post_s: [{self.bias_post_s[0]:.6f}, {self.bias_post_s[1]:.6f}, {self.bias_post_s[2]:.6f}] dps")
    print(f"  mag_field_ref: [{self.mag_field_ref[0]:.6f}, {self.mag_field_ref[1]:.6f}, {self.mag_field_ref[2]:.6f}]")

    if not mag_ref_initialized and np.linalg.norm(self.mag_field_ref) > 1e-12:
        print(f"\n  ✓ Magnetometer reference INITIALIZED")
    elif mag_ref_initialized and quat_changed:
        print(f"\n  ✓ Quaternion UPDATED (mag correction applied)")
    elif mag_ref_initialized and not quat_changed:
        print(f"\n  → No quaternion change (mag field matches expected)")

    print(f"  ornt_err_post_s: [{self.ornt_err_post_s[0]:.6f}, {self.ornt_err_post_s[1]:.6f}, {self.ornt_err_post_s[2]:.6f}]")
    print(f"  bias_err_post_s: [{self.bias_err_post_s[0]:.6f}, {self.bias_err_post_s[1]:.6f}, {self.bias_err_post_s[2]:.6f}]")
    print(f"  mag_dist_err_post_s: [{self.mag_dist_err_post_s[0]:.6f}, {self.mag_dist_err_post_s[1]:.6f}, {self.mag_dist_err_post_s[2]:.6f}]")

    return result

# Apply monkey patches
SensorFusion9Axis.measurement_update = debug_measurement_update
SensorFusion9Axis.measurement_update_mag = debug_mag_update

def main():
    print("="*80)
    print("DETAILED MEASUREMENT UPDATE DEBUG")
    print("First 5 fusion cycles with complete update steps")
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
    print(f"Platform: INVENSENSE\n")

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

            # Ground truth
            gt_yaw = row['gt_yaw_deg']
            gt_pitch = row['gt_pitch_deg']
            gt_roll = row['gt_roll_deg']
            print(f"\nGround Truth: yaw={gt_yaw:.3f}°, pitch={gt_pitch:.3f}°, roll={gt_roll:.3f}°")

            # Run fusion (will call our debug wrappers)
            output = sf.run()
            fusion_count += 1

            print(f"\n{'─'*80}")
            print(f"FUSION CYCLE {fusion_count} COMPLETE")
            print(f"{'─'*80}")
            print(f"Final Output:")
            print(f"  quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
            print(f"  orientation: [yaw={output.orientation[0]:.3f}°, pitch={output.orientation[1]:.3f}°, roll={output.orientation[2]:.3f}°]")

            # Compute errors
            yaw_err = output.orientation[0] - gt_yaw
            if yaw_err > 180:
                yaw_err -= 360
            elif yaw_err < -180:
                yaw_err += 360
            pitch_err = output.orientation[1] - gt_pitch
            roll_err = output.orientation[2] - gt_roll

            print(f"\nErrors vs Ground Truth:")
            print(f"  Yaw error:   {yaw_err:+.3f}°")
            print(f"  Pitch error: {pitch_err:+.3f}°")
            print(f"  Roll error:  {roll_err:+.3f}°")

    print(f"\n{'='*80}")
    print(f"DEBUG COMPLETE - Processed {fusion_count} fusion cycles")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
