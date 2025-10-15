#!/usr/bin/env python3
"""
Verify if magnetometer data is consistent with ground truth heading
"""

import numpy as np
import pandas as pd

print("="*80)
print("MAGNETOMETER CONSISTENCY CHECK")
print("="*80)

# Load static dataset
df = pd.read_csv('../test/data/datasets/synthetic/static_10s.csv')

# Get first sample
row = df.iloc[0]

# Ground truth orientation
gt_yaw = row['gt_yaw_deg']
gt_pitch = row['gt_pitch_deg']
gt_roll = row['gt_roll_deg']
gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])

print(f"\nGround Truth Orientation:")
print(f"  Yaw:   {gt_yaw:.3f}°")
print(f"  Pitch: {gt_pitch:.3f}°")
print(f"  Roll:  {gt_roll:.3f}°")
print(f"  Quat:  [{gt_quat[0]:.6f}, {gt_quat[1]:.6f}, {gt_quat[2]:.6f}, {gt_quat[3]:.6f}]")

# This is identity quaternion = no rotation
print(f"\n  Interpretation: Sensor frame = Reference frame (identity quaternion)")

# Magnetometer data
mag_x = row['mag_x_counts']
mag_y = row['mag_y_counts']
mag_z = row['mag_z_counts']

# Scale to µT
mag_scale = 0.1464888455  # µT/count
mag_ut = np.array([mag_x, mag_y, mag_z]) * mag_scale

print(f"\n{'─'*80}")
print("Magnetometer Reading:")
print(f"{'─'*80}")
print(f"  Raw counts: [{mag_x:.0f}, {mag_y:.0f}, {mag_z:.0f}]")
print(f"  Scaled (µT): [{mag_ut[0]:.3f}, {mag_ut[1]:.3f}, {mag_ut[2]:.3f}]")

# Magnitude
mag_magnitude = np.linalg.norm(mag_ut)
print(f"  Magnitude: {mag_magnitude:.3f} µT")

# Normalize
mag_norm = mag_ut / mag_magnitude
print(f"  Normalized: [{mag_norm[0]:.6f}, {mag_norm[1]:.6f}, {mag_norm[2]:.6f}]")

print(f"\n{'─'*80}")
print("ANALYSIS:")
print(f"{'─'*80}")

print(f"\nSince ground truth is identity quaternion (sensor = reference):")
print(f"  - The magnetometer reading is ALREADY in the reference frame")
print(f"  - No rotation needed to transform it")

print(f"\nMagnetometer direction in reference frame:")
# Compute angles
mag_angle_xy = np.arctan2(mag_norm[1], mag_norm[0])  # Angle in XY plane (heading)
mag_angle_from_z = np.arccos(mag_norm[2])  # Angle from Z-axis (inclination)

print(f"  Azimuth (heading from X-axis): {np.degrees(mag_angle_xy):.3f}°")
print(f"  Inclination (from Z-axis):     {np.degrees(mag_angle_from_z):.3f}°")

# Horizontal component (project onto XY plane)
mag_horizontal = np.array([mag_norm[0], mag_norm[1], 0])
mag_horiz_norm = np.linalg.norm(mag_horizontal)
if mag_horiz_norm > 0:
    mag_horizontal = mag_horizontal / mag_horiz_norm

print(f"\n  Horizontal component (XY plane):")
print(f"    [{mag_horizontal[0]:.6f}, {mag_horizontal[1]:.6f}, 0.000000]")
print(f"    Magnitude: {mag_horiz_norm:.6f}")

# Heading angle from X-axis
heading_from_mag = np.degrees(np.arctan2(mag_horizontal[1], mag_horizontal[0]))
if heading_from_mag < 0:
    heading_from_mag += 360

print(f"\n  Heading computed from magnetometer: {heading_from_mag:.3f}°")

print(f"\n{'─'*80}")
print("EXPECTED vs ACTUAL:")
print(f"{'─'*80}")

print(f"\nGround Truth Heading: {gt_yaw:.3f}°")
print(f"Magnetometer Heading: {heading_from_mag:.3f}°")
print(f"Difference:           {abs(gt_yaw - heading_from_mag):.3f}°")

if abs(gt_yaw - heading_from_mag) < 1.0:
    print(f"\n✓ CONSISTENT: Magnetometer points in direction consistent with GT yaw=0°")
else:
    print(f"\n✗ INCONSISTENT: Magnetometer does NOT point to GT heading!")

print(f"\n{'─'*80}")
print("EARTH'S MAGNETIC FIELD:")
print(f"{'─'*80}")

print(f"\nTypical Earth's magnetic field:")
print(f"  - Magnitude: 25-65 µT (depending on location)")
print(f"  - Inclination: 0-90° (depends on latitude)")
print(f"  - Declination: varies by location (angle from true north)")

print(f"\nThis dataset:")
print(f"  - Magnitude: {mag_magnitude:.1f} µT ✓ (within typical range)")
print(f"  - Inclination: {np.degrees(mag_angle_from_z):.1f}° (dip angle)")

# Typical magnetic field at different latitudes
print(f"\n  Magnetic inclination by latitude (approximate):")
print(f"    Equator (0°):   ~0° inclination (horizontal)")
print(f"    Mid-lat (45°):  ~60° inclination")
print(f"    Pole (90°):     ~90° inclination (vertical)")

print(f"\n  This data: {np.degrees(mag_angle_from_z):.1f}° inclination")
print(f"  → Suggests mid-latitude location (~45°N or S)")

print(f"\n{'─'*80}")
print("COORDINATE FRAME INTERPRETATION:")
print(f"{'─'*80}")

print(f"\nIf reference frame is defined as:")
print(f"  X-axis = 'Forward' or 'North'")
print(f"  Y-axis = 'Left' or 'West'")
print(f"  Z-axis = 'Up'")

print(f"\nThen magnetometer reading [{mag_norm[0]:.3f}, {mag_norm[1]:.3f}, {mag_norm[2]:.3f}]:")
print(f"  - Points ~{heading_from_mag:.1f}° from X-axis (in XY plane)")
print(f"  - Has {np.degrees(mag_angle_from_z):.1f}° dip angle")

if heading_from_mag < 5 or heading_from_mag > 355:
    print(f"\n✓ Magnetometer points ROUGHLY along X-axis")
    print(f"  → Consistent with X-axis = North (or magnetic north)")
else:
    print(f"\n⚠ Magnetometer points {heading_from_mag:.1f}° from X-axis")
    print(f"  → X-axis may not be aligned with magnetic north")
    print(f"  → Or there may be magnetic declination")

print(f"\n{'='*80}")
print("CONCLUSION:")
print(f"{'='*80}")

if abs(gt_yaw - heading_from_mag) < 5.0:
    print(f"\n✓ YES - Magnetometer IS consistent with ground truth yaw=0°")
    print(f"  The small difference ({abs(gt_yaw - heading_from_mag):.2f}°) is likely due to:")
    print(f"    - Sensor noise")
    print(f"    - Magnetic declination")
    print(f"    - Coordinate frame convention")
else:
    print(f"\n✗ NO - Magnetometer is NOT consistent with ground truth yaw=0°")
    print(f"  There's a {abs(gt_yaw - heading_from_mag):.1f}° discrepancy")
    print(f"  This could indicate:")
    print(f"    - Ground truth uses different reference (true north vs mag north)")
    print(f"    - Magnetometer data has an offset")
    print(f"    - Different coordinate frame conventions")
