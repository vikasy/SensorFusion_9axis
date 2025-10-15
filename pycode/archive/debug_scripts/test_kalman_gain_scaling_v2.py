#!/usr/bin/env python3
"""
Test effect of artificially scaling Kalman gain on filter convergence
Version 2: Properly scale gain BEFORE using it in state update
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in degrees"""
    dot_product = np.dot(q1, q2)
    return 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi

def wrap_angle(angle):
    """Wrap angle to [-180, 180] range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def run_fusion_with_gain_scale(gain_scale: float, max_samples: int = 100):
    """
    Run sensor fusion with scaled Kalman gain

    Args:
        gain_scale: Multiplier for Kalman gain (1.0 = baseline, 2.0 = double, etc.)
        max_samples: Number of samples to process

    Returns:
        List of error dictionaries for each fusion cycle
    """
    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Monkey-patch to scale Kalman gain INSIDE the measurement update
    # Store the original methods
    import types
    from sensor_fusion_9axis import (
        SF_DELTA_T, DEG2RAD, GTOMSEC2, EPSILON, SF_OVERSAMPLE_RATIO,
        cross_product_matrix, quaternion_integrate
    )
    from QuatMath.QuatNormal import quat_normalize
    from QuatMath.Quat2RodMat import quat_to_rotation_matrix

    def scaled_measurement_update(self, debug=False):
        """Measurement update with scaled Kalman gain"""
        if debug:
            print(f"\n=== MEASUREMENT UPDATE (ts={self.acc_data.timestamp}) ===")

        # Compute gravity error
        for i in range(3):
            self.grav_gyr_pri_s[i] = -self.rot_mtx_post[i, 2] * GTOMSEC2
            self.grav_err_pri_s[i] = self.acc_data.count_buff[SF_OVERSAMPLE_RATIO - 1, i]
            self.grav_err_pri_s[i] *= -1.0
            self.grav_err_pri_s[i] *= self.acc_data.scale_factor * GTOMSEC2
            self.grav_err_pri_s[i] += self.lin_acc_tc * self.acc_post_s[i]
            self.grav_err_pri_s[i] -= self.grav_gyr_pri_s[i]

        # Compute measurement matrix C
        C = np.zeros((4, 3, 3))
        cp_mat = cross_product_matrix(self.grav_gyr_pri_s)
        C[0] = -DEG2RAD * cp_mat
        C[1] = (DEG2RAD * SF_DELTA_T) * cp_mat
        C[2] = np.eye(3)
        C[3] = np.zeros((3, 3))

        # Compute Kalman gain
        F = np.zeros((4, 3, 3))
        for i in range(4):
            F[i] = np.zeros((3, 3))
            for j in range(4):
                C_transp = C[j].T
                term = self.proc_noise_var[i, j] @ C_transp
                F[i] += term

        G = np.eye(3) * self.meas_noise_var_acc
        for i in range(4):
            G += C[i] @ F[i]

        try:
            G_inv = np.linalg.inv(G)
            inv_exist = True
        except np.linalg.LinAlgError:
            inv_exist = False

        if inv_exist:
            for i in range(4):
                self.kalman_gain[i] = F[i] @ G_inv
                # SCALE THE GAIN HERE!
                self.kalman_gain[i] *= gain_scale

        # Measurement update: xe+ = xe- + K*ze
        M_updt = np.zeros(12)
        for k in range(4):
            for j in range(3):
                M_updt[3*k + j] = 0.0
                for i in range(3):
                    M_updt[3*k + j] += self.kalman_gain[k][j, i] * self.grav_err_pri_s[i]

        # Update state error estimates
        self.ornt_err_post_s = M_updt[0:3]
        self.bias_err_post_s = M_updt[3:6]
        self.acc_err_post_s = M_updt[6:9]
        self.mag_dist_err_post_s = M_updt[9:12]

        gyro_corr = -self.ornt_err_post_s / SF_DELTA_T

        # Update quaternion with correction
        quat_int = quaternion_integrate(
            self.quat_post.to_array(),
            gyro_corr,
            SF_DELTA_T
        )
        quat_norm = quat_normalize(quat_int)
        self.quat_post.from_array(quat_norm)

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update timestamp
        self.meas_updt_ts = self.acc_data.timestamp

        # Update aposteriori covariance matrix
        self._update_error_covariance_9axis(C)

        # Update gyro bias and linear acceleration
        self.bias_post_s -= self.bias_err_post_s
        self.bias_post_s = np.clip(self.bias_post_s, -200.0, 200.0)
        self.acc_post_s = self.lin_acc_tc * self.acc_post_s - self.acc_err_post_s

        # Transform acceleration to global frame
        for j in range(3):
            self.acc_post_g[j] = 0.0
            for k in range(3):
                self.acc_post_g[j] += self.rot_mtx_post[j, k] * self.acc_post_s[k]
        self.acc_post_g[3] -= GTOMSEC2

    def scaled_measurement_update_mag(self):
        """Magnetometer measurement update with scaled Kalman gain"""
        # Get measured magnetometer data
        mag_measured = np.zeros(3)
        for i in range(3):
            mag_measured[i] = self.mag_data.count_avg[i]
            mag_measured[i] -= self.mag_cal_offset[i]

        # Normalize measured magnetic field
        mag_norm = np.linalg.norm(mag_measured)
        if mag_norm < EPSILON:
            return

        mag_measured = mag_measured / mag_norm

        # Initialize reference field if not set
        if np.linalg.norm(self.mag_field_ref) < EPSILON:
            self.mag_field_ref = self.rot_mtx_post.T @ mag_measured
            ref_norm = np.linalg.norm(self.mag_field_ref)
            if ref_norm > EPSILON:
                self.mag_field_ref = self.mag_field_ref / ref_norm

        # Project reference magnetic field to sensor frame
        mag_expected = self.rot_mtx_post @ self.mag_field_ref

        # Compute measurement error
        mag_error = mag_measured - mag_expected

        # Compute measurement matrix C
        C = np.zeros((4, 3, 3))
        cp_mat = cross_product_matrix(mag_expected)
        C[0] = -DEG2RAD * cp_mat
        C[1] = np.zeros((3, 3))
        C[2] = np.zeros((3, 3))
        C[3] = np.eye(3)

        # Compute Kalman gain
        F = np.zeros((4, 3, 3))
        for i in range(4):
            F[i] = np.zeros((3, 3))
            for j in range(4):
                C_transp = C[j].T
                F[i] += self.proc_noise_var[i, j] @ C_transp

        G = np.eye(3) * self.meas_noise_var_mag
        for i in range(4):
            G += C[i] @ F[i]

        try:
            G_inv = np.linalg.inv(G)
            inv_exist = True
        except np.linalg.LinAlgError:
            inv_exist = False
            return

        if inv_exist:
            for i in range(4):
                self.kalman_gain[i] = F[i] @ G_inv
                # SCALE THE GAIN HERE!
                self.kalman_gain[i] *= gain_scale

        # Measurement update
        M_updt = np.zeros(12)
        for k in range(4):
            for j in range(3):
                M_updt[3*k + j] = 0.0
                for i in range(3):
                    M_updt[3*k + j] += self.kalman_gain[k][j, i] * mag_error[i]

        # Update state error estimates
        self.ornt_err_post_s = M_updt[0:3]
        self.bias_err_post_s = M_updt[3:6]
        self.acc_err_post_s = M_updt[6:9]
        self.mag_dist_err_post_s = M_updt[9:12]

        gyro_corr = -self.ornt_err_post_s / SF_DELTA_T

        # Update quaternion with correction
        quat_int = quaternion_integrate(
            self.quat_post.to_array(),
            gyro_corr,
            SF_DELTA_T
        )
        quat_norm = quat_normalize(quat_int)
        self.quat_post.from_array(quat_norm)

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update aposteriori covariance matrix
        self._update_error_covariance_9axis(C)

        # Update gyro bias
        self.bias_post_s -= self.bias_err_post_s

        # Store calibrated magnetometer data
        self.mag_post_s = mag_measured * mag_norm
        self.mag_post_g = self.rot_mtx_post.T @ self.mag_post_s

    # Apply monkey patches
    sf.measurement_update = types.MethodType(scaled_measurement_update, sf)
    sf.measurement_update_mag = types.MethodType(scaled_measurement_update_mag, sf)

    fusion_data = []
    fusion_count = 0

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

            # Get ground truth
            gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
            gt_yaw = row['gt_yaw_deg']
            gt_pitch = row['gt_pitch_deg']
            gt_roll = row['gt_roll_deg']

            # Get Python output
            python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            python_yaw = output.orientation[0]
            python_pitch = output.orientation[1]
            python_roll = output.orientation[2]

            # Compute errors
            quat_error = quaternion_angular_distance(gt_quat, python_quat)
            yaw_err = wrap_angle(python_yaw - gt_yaw)
            pitch_err = wrap_angle(python_pitch - gt_pitch)
            roll_err = wrap_angle(python_roll - gt_roll)

            fusion_data.append({
                'fusion': fusion_count,
                'sample': idx,
                'quat_error': quat_error,
                'yaw_error': yaw_err,
                'pitch_error': pitch_err,
                'roll_error': roll_err
            })

    return fusion_data

def main():
    print("="*80)
    print("KALMAN GAIN SCALING TEST v2 (Proper Implementation)")
    print("="*80)
    print("\nTesting effect of scaling Kalman gain BEFORE state update")
    print("Static dataset - errors should converge to near-zero\n")

    gain_scales = [1.0, 2.0, 10.0]
    max_samples = 100

    results = {}

    for gain_scale in gain_scales:
        print(f"\n{'='*80}")
        print(f"Running with gain scale = {gain_scale}x")
        print(f"{'='*80}")

        fusion_data = run_fusion_with_gain_scale(gain_scale, max_samples)
        results[gain_scale] = fusion_data

        if len(fusion_data) > 0:
            errors = np.array([d['quat_error'] for d in fusion_data])
            yaw_errors = np.array([d['yaw_error'] for d in fusion_data])
            pitch_errors = np.array([d['pitch_error'] for d in fusion_data])
            roll_errors = np.array([d['roll_error'] for d in fusion_data])

            print(f"\nQuaternion Angular Error:")
            print(f"  Initial (fusion #1):  {errors[0]:.4f}°")
            print(f"  Final (fusion #{len(fusion_data)}): {errors[-1]:.4f}°")
            print(f"  Improvement: {errors[0] - errors[-1]:+.4f}° ({(errors[0] - errors[-1])/errors[0]*100:+.1f}%)")
            print(f"  Mean: {errors.mean():.4f}°, Std: {errors.std():.4f}°, Max: {errors.max():.4f}°")

            print(f"\nEuler Angle Errors (final):")
            print(f"  Yaw:   {yaw_errors[-1]:+.4f}° (drift: {abs(yaw_errors[-1] - yaw_errors[0]):.4f}°)")
            print(f"  Pitch: {pitch_errors[-1]:+.4f}° (drift: {abs(pitch_errors[-1] - pitch_errors[0]):.4f}°)")
            print(f"  Roll:  {roll_errors[-1]:+.4f}° (drift: {abs(roll_errors[-1] - roll_errors[0]):.4f}°)")

            # Convergence check
            first_half_mean = errors[:len(errors)//2].mean()
            second_half_mean = errors[len(errors)//2:].mean()

            if errors[-1] < 0.1:
                verdict = "✓ EXCELLENT"
            elif errors[-1] < 0.5:
                verdict = "✓ GOOD"
            elif errors[-1] < 1.0:
                verdict = "⚠ ACCEPTABLE"
            else:
                verdict = "✗ POOR"

            if second_half_mean < first_half_mean * 0.8:
                conv_status = "✓ Converging"
            elif second_half_mean < first_half_mean:
                conv_status = "⚠ Slow"
            else:
                conv_status = "✗ Diverging"

            print(f"\nVerdict: {verdict}, {conv_status}")

    # Comparison
    print("\n" + "="*80)
    print("COMPARISON SUMMARY")
    print("="*80)

    print(f"\n{'Scale':>8} {'Initial':>12} {'Final':>12} {'Change':>12} {'% Change':>12} {'Status':>15}")
    print("-"*80)

    for gain_scale in gain_scales:
        data = results[gain_scale]
        if len(data) > 0:
            errors = np.array([d['quat_error'] for d in data])
            initial = errors[0]
            final = errors[-1]
            change = final - initial
            pct = (change / initial) * 100

            if final < 0.5:
                status = "✓ Good"
            elif final < 1.0:
                status = "⚠ Acceptable"
            else:
                status = "✗ Poor"

            print(f"{gain_scale:>8.1f}x {initial:>11.4f}° {final:>11.4f}° {change:>+11.4f}° {pct:>+11.1f}% {status:>15}")

    # Analysis
    print("\n" + "="*80)
    print("CONCLUSION")
    print("="*80)

    best_scale = min(gain_scales, key=lambda s: np.array([d['quat_error'] for d in results[s]])[-1])
    best_final = np.array([d['quat_error'] for d in results[best_scale]])[-1]

    print(f"\nBest: {best_scale}x gain (final error: {best_final:.4f}°)")

    if best_scale == 1.0:
        print("\n→ Baseline gain is already optimal")
        print("→ Problem is NOT weak corrections")
        print("→ Root cause likely: gyro bias, mag initialization, or noise tuning")
    else:
        print(f"\n→ Higher gain ({best_scale}x) improves performance")
        print(f"→ Corrections are too weak (gains too conservative)")
        print(f"→ Consider tuning noise parameters to increase measurement trust")

    print("\n" + "="*80)

if __name__ == '__main__':
    main()
