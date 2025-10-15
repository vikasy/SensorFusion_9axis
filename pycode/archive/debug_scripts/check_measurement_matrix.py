#!/usr/bin/env python3
"""
Check if the measurement matrix C is computed correctly.
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import (
    SensorFusion6Axis, cross_product_matrix,
    DEG2RAD, SF_DELTA_T, GTOMSEC2
)
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("CHECKING MEASUREMENT MATRIX C")
    print("="*80)
    print()

    # Constants
    print(f"DEG2RAD = {DEG2RAD}")
    print(f"SF_DELTA_T = {SF_DELTA_T} seconds")
    print(f"DEG2RAD * SF_DELTA_T = {DEG2RAD * SF_DELTA_T}")
    print()

    # Expected gravity for static device (level, Z-down)
    gravity = np.array([0, 0, -GTOMSEC2])
    print(f"Expected gravity (static, level): {gravity}")
    print()

    # Cross product matrix
    cp_mat = cross_product_matrix(gravity)
    print(f"Cross product matrix of gravity:")
    print(cp_mat)
    print()

    # C matrices
    C0 = -DEG2RAD * cp_mat
    C1 = (DEG2RAD * SF_DELTA_T) * cp_mat
    C2 = np.eye(3)

    print(f"C[0] = -DEG2RAD * [grav×]:")
    print(C0)
    print(f"  Max element: {np.max(np.abs(C0))}")
    print()

    print(f"C[1] = (DEG2RAD * SF_DELTA_T) * [grav×]:")
    print(C1)
    print(f"  Max element: {np.max(np.abs(C1))}")
    print()

    print(f"C[2] = I:")
    print(C2)
    print()

    # The key insight: C[1] relates bias error to gravity error
    # For a static device with constant gyro bias:
    #   - Bias causes orientation to drift at rate = bias
    #   - After time SF_DELTA_T, orientation error = bias * SF_DELTA_T
    #   - Orientation error causes gravity error = [orientation_error × gravity]
    #   - So: gravity_error = [bias * SF_DELTA_T × gravity]
    #   - In measurement equation: z = C[0]*orient_err + C[1]*bias_err + C[2]*acc_err
    #   - The C[1] term captures: gravity_error = C[1] * bias_err

    print("="*80)
    print("PHYSICAL INTERPRETATION")
    print("="*80)
    print()
    print("For a gyro bias of 1 dps:")
    bias_dps = np.array([1.0, 0, 0])  # 1 dps in X
    bias_rad_s = bias_dps * DEG2RAD

    print(f"  Bias: {bias_dps} dps = {bias_rad_s} rad/s")
    print(f"  After {SF_DELTA_T}s, orientation error: {bias_rad_s * SF_DELTA_T} rad")
    print(f"  Gravity error from C[1]: {C1 @ bias_rad_s}")
    print(f"  Magnitude: {np.linalg.norm(C1 @ bias_rad_s):.6f} m/s²")
    print()

    # Compare to typical gravity error from accelerometer noise
    acc_noise_std = np.sqrt(1.18e-4)  # From R matrix
    print(f"Typical accelerometer noise: {acc_noise_std:.6f} m/s² (1-sigma)")
    print()

    signal_to_noise = np.linalg.norm(C1 @ bias_rad_s) / acc_noise_std
    print(f"Signal-to-noise ratio for 1 dps bias: {signal_to_noise:.4f}")
    print()

    if signal_to_noise < 1.0:
        print("⚠️  WARNING: Bias signal is SMALLER than measurement noise!")
        print("   This means bias is very weakly observable.")
        print("   Filter will converge very slowly or not at all.")
    else:
        print("✓ Bias signal is larger than noise, should be observable.")

    print()

    # Check actual bias in synthetic data
    print("="*80)
    print("ACTUAL SYNTHETIC DATA BIAS")
    print("="*80)
    print()

    dataset_path = "../test/data/datasets/synthetic/static_60s.csv"
    df = pd.read_csv(dataset_path)

    gyro_scale = 1.0 / 32.768  # dps per count for ±1000 dps range
    gyro_x_mean = df['gyro_x_counts'].mean() * gyro_scale
    gyro_y_mean = df['gyro_y_counts'].mean() * gyro_scale
    gyro_z_mean = df['gyro_z_counts'].mean() * gyro_scale

    actual_bias = np.array([gyro_x_mean, gyro_y_mean, gyro_z_mean])
    print(f"Actual bias in data: {actual_bias} dps")

    actual_bias_rad_s = actual_bias * DEG2RAD
    gravity_error_from_bias = C1 @ actual_bias_rad_s

    print(f"Gravity error from actual bias: {gravity_error_from_bias}")
    print(f"Magnitude: {np.linalg.norm(gravity_error_from_bias):.6f} m/s²")
    print()

    actual_snr = np.linalg.norm(gravity_error_from_bias) / acc_noise_std
    print(f"Actual signal-to-noise ratio: {actual_snr:.4f}")
    print()

    if actual_snr < 3.0:
        print("⚠️  Actual bias signal is only {:.1f}x larger than noise.".format(actual_snr))
        print("   This explains slow convergence!")
    else:
        print("✓ Actual bias signal is clearly observable (SNR > 3).")

if __name__ == '__main__':
    main()
