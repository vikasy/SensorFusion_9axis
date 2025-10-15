#!/usr/bin/env python3
"""
COMPLETE DEBUG: First TWO fusion cycles for 9-axis
Shows:
1. count_buff for all sensors (accel, gyro, mag)
2. count_avg for all sensors
3. Input data verification against CSV
4. Quaternion integration after EACH of the 4 gyro samples
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# Create a wrapper class to enable debug mode
class SensorFusion9AxisDebug(SensorFusion9Axis):
    """Wrapper to enable debug mode in run()"""

    def run_debug(self):
        """Run with debug enabled for time_update"""
        curr_time_msec = self.timestamp_ms()

        # Handle reset
        if self.reset_flag:
            print("9-axis SF algo reset")
            self._reset()
            self._reset_9axis()
            from sensor_fusion_6axis import AlgoOutput
            return AlgoOutput()

        # Initialize orientation on first run
        if not self.orient_init:
            print("9-axis SF algo init orient")
            from sensor_fusion_6axis import GTOMSEC2
            accel_avg = self.acc_data.count_avg * GTOMSEC2
            mag_avg = self.mag_data.count_avg
            self._init_orient_mag(accel_avg, mag_avg)

        # Time update if new gyro data available
        if self.nom_updt_ts < self.gyro_data.timestamp:
            self.time_update(debug=True)  # Enable debug!

        # Measurement update if new accel data available
        if self.meas_updt_ts < self.acc_data.timestamp:
            self.measurement_update(debug=False)

        # Magnetometer update if new mag data available
        from sensor_fusion_6axis import EPSILON
        if self.mag_meas_updt_ts < self.mag_data.timestamp:
            mag_measured = self.mag_data.count_avg

            mag_ref_initialized = np.linalg.norm(self.mag_field_ref) > EPSILON

            if mag_ref_initialized:
                disturbance = self.detect_mag_disturbance(mag_measured, self.mag_field_ref, 0.3)
            else:
                disturbance = False

            if not disturbance:
                self.measurement_update_mag()

            self.mag_meas_updt_ts = self.mag_data.timestamp

        # Update gravity vector
        from sensor_fusion_6axis import GTOMSEC2
        for i in range(3):
            self.grav_post_s[i] = -1.0 * GTOMSEC2 * self.rot_mtx_post[i, 2]

        # Convert rotation matrix to Euler angles
        from sensor_fusion_6axis import rotation_matrix_to_angles
        self.theta_post, self.phi_post, self.psi_post, self.rho_post, self.chi_post = \
            rotation_matrix_to_angles(self.rot_mtx_post, self.theta_post, self.psi_post)

        # Prepare output
        from sensor_fusion_6axis import AlgoOutput, Quaternion, AlgoType, NSEC2MSEC
        output = AlgoOutput()
        output.algo_type = AlgoType.SF_9AGM
        output.quat = Quaternion(
            q0=self.quat_post.q0,
            q1=self.quat_post.q1,
            q2=self.quat_post.q2,
            q3=self.quat_post.q3
        )
        output.orientation = np.array([self.psi_post, self.theta_post, self.phi_post])
        output.gravity = self.grav_post_s.copy()
        output.linear_acc = self.acc_post_g[0:3].copy()
        output.valid_flag = 0xFFFFFFFF
        output.mode = self.op_mode
        output.timestamp_ns = curr_time_msec * NSEC2MSEC

        return output

    @staticmethod
    def timestamp_ms():
        import time
        return int(time.time() * 1000)


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
    print(f"  gyro count_avg:  [{gyro_avg[0]:10.6f}, {gyro_avg[1]:10.6f}, {gyro_avg[2]:10.6f}] dps (unused, always zero)")

    mag_avg = sf.mag_data.count_avg
    print(f"  mag count_avg:   [{mag_avg[0]:10.6f}, {mag_avg[1]:10.6f}, {mag_avg[2]:10.6f}] µT")

    # Verify against input if provided
    if row is not None and platform is not None:
        print("\n" + "─"*80)
        print("VERIFICATION (input CSV vs buffered):")
        print("─"*80)
        accel_csv = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']])
        gyro_csv = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']])
        mag_csv = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']])

        # Latest buffered sample should match CSV input
        accel_buf = sf.acc_data.count_buff[0]
        gyro_buf = sf.gyro_data.count_buff[0]
        mag_buf = sf.mag_data.count_buff[0]

        print(f"\nRaw counts match:")
        accel_match = np.allclose(accel_csv, accel_buf, atol=1)
        print(f"  Accel CSV:    [{accel_csv[0]:7.0f}, {accel_csv[1]:7.0f}, {accel_csv[2]:7.0f}]")
        print(f"  Accel buf[0]: [{accel_buf[0]:7.0f}, {accel_buf[1]:7.0f}, {accel_buf[2]:7.0f}] {'✓' if accel_match else '✗ MISMATCH'}")

        gyro_match = np.allclose(gyro_csv, gyro_buf, atol=1)
        print(f"  Gyro CSV:     [{gyro_csv[0]:7.0f}, {gyro_csv[1]:7.0f}, {gyro_csv[2]:7.0f}]")
        print(f"  Gyro buf[0]:  [{gyro_buf[0]:7.0f}, {gyro_buf[1]:7.0f}, {gyro_buf[2]:7.0f}] {'✓' if gyro_match else '✗ MISMATCH'}")

        mag_match = np.allclose(mag_csv, mag_buf, atol=1)
        print(f"  Mag CSV:      [{mag_csv[0]:7.0f}, {mag_csv[1]:7.0f}, {mag_csv[2]:7.0f}]")
        print(f"  Mag buf[0]:   [{mag_buf[0]:7.0f}, {mag_buf[1]:7.0f}, {mag_buf[2]:7.0f}] {'✓' if mag_match else '✗ MISMATCH'}")

        # Verify scaled values
        accel_g_csv = accel_csv * platform.accel_scale_factor
        gyro_dps_csv = gyro_csv * platform.gyro_scale_factor
        mag_ut_csv = mag_csv * platform.mag_scale_factor

        print(f"\nScaled values (CSV):")
        print(f"  Accel (g):   [{accel_g_csv[0]:10.6f}, {accel_g_csv[1]:10.6f}, {accel_g_csv[2]:10.6f}]")
        print(f"  Gyro (dps):  [{gyro_dps_csv[0]:10.6f}, {gyro_dps_csv[1]:10.6f}, {gyro_dps_csv[2]:10.6f}]")
        print(f"  Mag (µT):    [{mag_ut_csv[0]:10.6f}, {mag_ut_csv[1]:10.6f}, {mag_ut_csv[2]:10.6f}]")


def main():
    print("=" * 80)
    print("COMPLETE BUFFER AND QUATERNION INTEGRATION DEBUG")
    print("First TWO fusion cycles")
    print("=" * 80)

    # Load static dataset for simplicity
    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9AxisDebug(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nDataset: {dataset_path}")
    print(f"Platform: INVENSENSE")
    print(f"  accel_scale: {platform.accel_scale_factor:.10f} g/count")
    print(f"  gyro_scale: {platform.gyro_scale_factor:.10f} dps/count")
    print(f"  mag_scale: {platform.mag_scale_factor:.10f} µT/count")
    print(f"\nNote: Gyro uses count_buff directly (NOT count_avg)")
    print(f"      Accel/Mag use count_avg (computed weighted average)")

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
            print(f"FUSION CYCLE {fusion_count + 1} - GYRO READY, CALLING run_debug()")
            print(f"{'='*80}")

            # Call run_debug which enables debug mode in time_update
            output = sf.run_debug()
            fusion_count += 1

            print(f"\n{'='*80}")
            print(f"FUSION CYCLE {fusion_count} COMPLETE")
            print(f"{'='*80}")
            print(f"Output:")
            print(f"  quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
            print(f"  orientation: [yaw={output.orientation[0]:7.3f}°, pitch={output.orientation[1]:7.3f}°, roll={output.orientation[2]:7.3f}°]")

    print(f"\n{'='*80}")
    print(f"DEBUG COMPLETE - Processed {fusion_count} fusions")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
