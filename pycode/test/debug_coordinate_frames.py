#!/usr/bin/env python3
"""
Debug coordinate frame issues between realistic and synthetic datasets

Author: Vikas Yadav
Date: 2025-10-15
"""

import numpy as np
import pandas as pd

def quat_to_rotation_matrix(q):
    """Convert quaternion [w,x,y,z] to rotation matrix"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

def gravity_from_quat(q):
    """Get gravity vector in body frame from quaternion"""
    # World gravity is [0, 0, -1] (pointing down in NED)
    # Or [0, 0, +1] pointing up in ENU
    # Transform to body frame
    R = quat_to_rotation_matrix(q)
    # Assuming NED: gravity in world = [0, 0, 9.81] (down)
    g_world_ned = np.array([0, 0, 9.81])
    g_body = R.T @ g_world_ned  # R.T transforms from world to body
    return g_body

def main():
    # Load realistic dataset
    df_real = pd.read_csv('../../test/data/datasets/realistic/walking.csv')

    # Load synthetic dataset
    df_syn = pd.read_csv('../../test/data/datasets/synthetic/rotation_sequence_15s.csv')

    print("="*80)
    print("COORDINATE FRAME ANALYSIS")
    print("="*80)

    # Analyze first sample (should be at rest, near identity quaternion)
    print("\n--- FIRST SAMPLE (At Rest) ---")

    # Realistic
    row_real = df_real.iloc[0]
    accel_real = np.array([row_real['accel_x_counts'], row_real['accel_y_counts'], row_real['accel_z_counts']])
    quat_real = np.array([row_real['gt_quat_w'], row_real['gt_quat_x'], row_real['gt_quat_y'], row_real['gt_quat_z']])

    # Synthetic
    row_syn = df_syn.iloc[0]
    accel_syn = np.array([row_syn['accel_x_counts'], row_syn['accel_y_counts'], row_syn['accel_z_counts']])
    quat_syn = np.array([row_syn['gt_quat_w'], row_syn['gt_quat_x'], row_syn['gt_quat_y'], row_syn['gt_quat_z']])

    print("\nRealistic:")
    print(f"  Accel (counts):  {accel_real}")
    print(f"  Accel (norm):    {accel_real / np.linalg.norm(accel_real)}")
    print(f"  GT Quat:         {quat_real}")
    print(f"  GT RPY (deg):    [{row_real['gt_roll_deg']:.2f}, {row_real['gt_pitch_deg']:.2f}, {row_real['gt_yaw_deg']:.2f}]")

    print("\nSynthetic:")
    print(f"  Accel (counts):  {accel_syn}")
    print(f"  Accel (norm):    {accel_syn / np.linalg.norm(accel_syn)}")
    print(f"  GT Quat:         {quat_syn}")
    print(f"  GT RPY (deg):    [{row_syn['gt_roll_deg']:.2f}, {row_syn['gt_pitch_deg']:.2f}, {row_syn['gt_yaw_deg']:.2f}]")

    # Expected gravity in body frame from GT quaternion
    print("\n--- GRAVITY DIRECTION CHECK ---")
    print("\nIf GT quaternion is correct, it should transform world gravity to match measured accel")

    # For realistic
    g_body_real_ned = gravity_from_quat(quat_real)
    g_body_real_enu = -g_body_real_ned  # ENU convention

    accel_norm_real = accel_real / np.linalg.norm(accel_real)

    print("\nRealistic:")
    print(f"  Measured accel (norm): {accel_norm_real}")
    print(f"  Expected from GT (NED): {g_body_real_ned / np.linalg.norm(g_body_real_ned)}")
    print(f"  Expected from GT (ENU): {g_body_real_enu / np.linalg.norm(g_body_real_enu)}")
    print(f"  Match NED? {np.allclose(accel_norm_real, g_body_real_ned / np.linalg.norm(g_body_real_ned), atol=0.01)}")
    print(f"  Match ENU? {np.allclose(accel_norm_real, g_body_real_enu / np.linalg.norm(g_body_real_enu), atol=0.01)}")

    # For synthetic
    g_body_syn_ned = gravity_from_quat(quat_syn)
    g_body_syn_enu = -g_body_syn_ned

    accel_norm_syn = accel_syn / np.linalg.norm(accel_syn)

    print("\nSynthetic:")
    print(f"  Measured accel (norm): {accel_norm_syn}")
    print(f"  Expected from GT (NED): {g_body_syn_ned / np.linalg.norm(g_body_syn_ned)}")
    print(f"  Expected from GT (ENU): {g_body_syn_enu / np.linalg.norm(g_body_syn_enu)}")
    print(f"  Match NED? {np.allclose(accel_norm_syn, g_body_syn_ned / np.linalg.norm(g_body_syn_ned), atol=0.01)}")
    print(f"  Match ENU? {np.allclose(accel_norm_syn, g_body_syn_enu / np.linalg.norm(g_body_syn_enu), atol=0.01)}")

    #  Check a rotated sample
    print("\n--- ROTATED SAMPLE (Sample 50) ---")

    row_real_50 = df_real.iloc[50]
    accel_real_50 = np.array([row_real_50['accel_x_counts'], row_real_50['accel_y_counts'], row_real_50['accel_z_counts']])
    quat_real_50 = np.array([row_real_50['gt_quat_w'], row_real_50['gt_quat_x'], row_real_50['gt_quat_y'], row_real_50['gt_quat_z']])

    print("\nRealistic (sample 50):")
    print(f"  Accel (counts):  {accel_real_50}")
    print(f"  GT Quat:         {quat_real_50}")
    print(f"  GT RPY (deg):    [{row_real_50['gt_roll_deg']:.2f}, {row_real_50['gt_pitch_deg']:.2f}, {row_real_50['gt_yaw_deg']:.2f}]")

    g_body_real_50_ned = gravity_from_quat(quat_real_50)
    g_body_real_50_enu = -g_body_real_50_ned
    accel_norm_real_50 = accel_real_50 / np.linalg.norm(accel_real_50)

    print(f"  Measured accel (norm): {accel_norm_real_50}")
    print(f"  Expected from GT (NED): {g_body_real_50_ned / np.linalg.norm(g_body_real_50_ned)}")
    print(f"  Expected from GT (ENU): {g_body_real_50_enu / np.linalg.norm(g_body_real_50_enu)}")
    print(f"  Match NED? {np.allclose(accel_norm_real_50, g_body_real_50_ned / np.linalg.norm(g_body_real_50_ned), atol=0.1)}")
    print(f"  Match ENU? {np.allclose(accel_norm_real_50, g_body_real_50_enu / np.linalg.norm(g_body_real_50_enu), atol=0.1)}")

    print("\n" + "="*80)
    print("CONCLUSION")
    print("="*80)
    print("\nThe analysis above should reveal whether:")
    print("1. Sensor and GT use different coordinate conventions (NED vs ENU)")
    print("2. Sensor axes need remapping")
    print("3. GT quaternion is incorrect or uses different definition")

if __name__ == '__main__':
    main()
