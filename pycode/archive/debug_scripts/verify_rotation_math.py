#!/usr/bin/env python3
"""
Verify rotation math - does the quaternion change match the gyro rates?
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SF_GYRO_SAMP_INTVL, quaternion_integrate, quat_normalize

def quat_to_euler_angles(quat):
    """Convert quaternion to Euler angles (yaw, pitch, roll) in degrees"""
    q0, q1, q2, q3 = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)

print("="*80)
print("ROTATION MATH VERIFICATION")
print("="*80)

# Initial quaternion (after initialization)
quat_init = np.array([0.99998919, 0.00448056, 0.00124542, -0.00000558])
yaw0, pitch0, roll0 = quat_to_euler_angles(quat_init)
print(f"\nInitial quaternion: [{quat_init[0]:.8f}, {quat_init[1]:.8f}, {quat_init[2]:.8f}, {quat_init[3]:.8f}]")
print(f"Initial Euler: yaw={yaw0:.6f}°, pitch={pitch0:.6f}°, roll={roll0:.6f}°")

# Gyro samples (4 samples)
gyro_samples = [
    np.array([0.09155553, -0.33570360, -0.30518509]),  # Sample 0
    np.array([0.09155553, -0.33570360, -0.30518509]),  # Sample 1
    np.array([0.06103702, -0.33570360, -0.33570360]),  # Sample 2
    np.array([0.09155553, -0.33570360, -0.36622211]),  # Sample 3
]

dt = SF_GYRO_SAMP_INTVL
print(f"\nTime per sample: {dt} sec = {dt*1000} ms")
print(f"Total time (4 samples): {4*dt} sec = {4*dt*1000} ms")

# Expected rotation (simple calculation)
total_omega = np.sum(gyro_samples, axis=0)
expected_rotation = total_omega * dt  # Each sample integrates for dt seconds

print(f"\n{'─'*80}")
print("EXPECTED ROTATION (Simple Linear Calculation):")
print(f"{'─'*80}")
print(f"Average omega X: {np.mean([s[0] for s in gyro_samples]):.6f} dps")
print(f"Average omega Y: {np.mean([s[1] for s in gyro_samples]):.6f} dps")
print(f"Average omega Z: {np.mean([s[2] for s in gyro_samples]):.6f} dps")
print(f"\nTotal rotation per sample (omega * dt):")
print(f"  X: {gyro_samples[0][0] * dt:.8f}° per sample")
print(f"  Y: {gyro_samples[0][1] * dt:.8f}° per sample")
print(f"  Z: {gyro_samples[0][2] * dt:.8f}° per sample")
print(f"\nTotal rotation over 4 samples:")
print(f"  X: {total_omega[0] * dt:.8f}°")
print(f"  Y: {total_omega[1] * dt:.8f}°")
print(f"  Z: {total_omega[2] * dt:.8f}°")

# Actual quaternion integration
print(f"\n{'─'*80}")
print("ACTUAL QUATERNION INTEGRATION:")
print(f"{'─'*80}")

quat = quat_init.copy()
for i, omega in enumerate(gyro_samples):
    quat_before = quat.copy()
    yaw_before, pitch_before, roll_before = quat_to_euler_angles(quat_before)

    quat_int = quaternion_integrate(quat, omega, dt)
    quat = quat_int

    yaw_after, pitch_after, roll_after = quat_to_euler_angles(quat)

    print(f"\nSample {i}: omega=[{omega[0]:.6f}, {omega[1]:.6f}, {omega[2]:.6f}] dps")
    print(f"  Quat before: [{quat_before[0]:.8f}, {quat_before[1]:.8f}, {quat_before[2]:.8f}, {quat_before[3]:.8f}]")
    print(f"  Quat after:  [{quat[0]:.8f}, {quat[1]:.8f}, {quat[2]:.8f}, {quat[3]:.8f}]")
    print(f"  Euler before: yaw={yaw_before:.6f}°, pitch={pitch_before:.6f}°, roll={roll_before:.6f}°")
    print(f"  Euler after:  yaw={yaw_after:.6f}°, pitch={pitch_after:.6f}°, roll={roll_after:.6f}°")
    print(f"  Change:       Δyaw={yaw_after-yaw_before:.6f}°, Δpitch={pitch_after-pitch_before:.6f}°, Δroll={roll_after-roll_before:.6f}°")

# Normalize
quat_norm = quat_normalize(quat)
yaw_final, pitch_final, roll_final = quat_to_euler_angles(quat_norm)

print(f"\n{'─'*80}")
print("FINAL RESULT:")
print(f"{'─'*80}")
print(f"Quat (normalized): [{quat_norm[0]:.8f}, {quat_norm[1]:.8f}, {quat_norm[2]:.8f}, {quat_norm[3]:.8f}]")
print(f"Final Euler: yaw={yaw_final:.6f}°, pitch={pitch_final:.6f}°, roll={roll_final:.6f}°")

print(f"\n{'─'*80}")
print("TOTAL CHANGE:")
print(f"{'─'*80}")
print(f"Δyaw:   {yaw_final - yaw0:.6f}° (expected: {total_omega[2] * dt:.6f}°)")
print(f"Δpitch: {pitch_final - pitch0:.6f}° (expected: {total_omega[1] * dt:.6f}°)")
print(f"Δroll:  {roll_final - roll0:.6f}° (expected: {total_omega[0] * dt:.6f}°)")

print(f"\n{'─'*80}")
print("VERIFICATION:")
print(f"{'─'*80}")
print(f"User's calculation: 0.33 dps × 40 ms = 0.33 × 0.040 = 0.0132°")
print(f"Actual Y (pitch):   {abs(pitch_final - pitch0):.6f}°")
print(f"Actual Z (yaw):     {abs(yaw_final - yaw0):.6f}°")
print(f"\nRatio (Actual/Expected): {abs(pitch_final - pitch0) / 0.0132:.2f}x")
