#!/usr/bin/env python3
"""
Trace where the initial pitch=0.143° and roll=0.513° come from
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import GTOMSEC2

print("="*80)
print("TRACING INITIAL ORIENTATION SOURCE")
print("="*80)

# From debug output - the accelerometer data at initialization
accel_counts_avg = np.array([-0.00250252, 0.00900296, 1.00463881])  # g
print(f"\nAccelerometer count_avg (at sample 3):")
print(f"  [{accel_counts_avg[0]:.8f}, {accel_counts_avg[1]:.8f}, {accel_counts_avg[2]:.8f}] g")

# Convert to m/s²
accel_mps2 = accel_counts_avg * GTOMSEC2
print(f"\nAccelerometer in m/s²:")
print(f"  [{accel_mps2[0]:.6f}, {accel_mps2[1]:.6f}, {accel_mps2[2]:.6f}] m/s²")

# Magnitude
accel_mag = np.linalg.norm(accel_mps2)
print(f"\nMagnitude: {accel_mag:.6f} m/s² (expect ~9.81)")

# Normalize to get gravity direction in sensor frame
gravity_sensor = accel_mps2 / accel_mag
print(f"\nNormalized gravity direction (down vector in sensor frame):")
print(f"  [{gravity_sensor[0]:.8f}, {gravity_sensor[1]:.8f}, {gravity_sensor[2]:.8f}]")

print(f"\n{'─'*80}")
print("WHAT DOES THIS MEAN?")
print(f"{'─'*80}")

print(f"\nIf the sensor were perfectly level:")
print(f"  gravity_sensor would be [0, 0, 1]")
print(f"  (pointing straight down in +Z direction)")

print(f"\nActual gravity_sensor:")
print(f"  X component: {gravity_sensor[0]:.8f} (small negative)")
print(f"  Y component: {gravity_sensor[1]:.8f} (small positive)")
print(f"  Z component: {gravity_sensor[2]:.8f} (close to 1)")

print(f"\nThis means the sensor is TILTED:")

print(f"\n{'─'*80}")
print("COMPUTING TILT ANGLES FROM GRAVITY")
print(f"{'─'*80}")

# Pitch: rotation around Y-axis (nose up/down)
# When sensor pitches up (nose up), gravity vector's X component becomes negative
# pitch = arcsin(-gx)
pitch_rad = np.arcsin(-gravity_sensor[0])
pitch_deg = np.degrees(pitch_rad)

print(f"\nPitch (rotation around Y-axis):")
print(f"  Formula: pitch = arcsin(-gravity_x)")
print(f"  pitch = arcsin(-({gravity_sensor[0]:.8f}))")
print(f"  pitch = arcsin({-gravity_sensor[0]:.8f})")
print(f"  pitch = {pitch_deg:.6f}°")
print(f"\n  Interpretation: Sensor nose is tilted UP by {pitch_deg:.3f}°")

# Roll: rotation around X-axis (left/right tilt)
# When sensor rolls left (left side down), gravity's Y component becomes positive
# roll = atan2(gy, gz)
roll_rad = np.arctan2(gravity_sensor[1], gravity_sensor[2])
roll_deg = np.degrees(roll_rad)

print(f"\nRoll (rotation around X-axis):")
print(f"  Formula: roll = atan2(gravity_y, gravity_z)")
print(f"  roll = atan2({gravity_sensor[1]:.8f}, {gravity_sensor[2]:.8f})")
print(f"  roll = {roll_deg:.6f}°")
print(f"\n  Interpretation: Sensor is tilted LEFT by {roll_deg:.3f}°")

print(f"\n{'─'*80}")
print("VERIFICATION WITH DEBUG OUTPUT")
print(f"{'─'*80}")

print(f"\nFrom debug_initialization.py:")
print(f"  Expected from accel: pitch=0.143°, roll=0.513°")
print(f"\nOur calculation:")
print(f"  Computed pitch: {pitch_deg:.3f}°  ✓")
print(f"  Computed roll:  {roll_deg:.3f}°  ✓")

print(f"\nFrom debug_9axis_complete.py (Quat BEFORE integration):")
print(f"  Initial Euler: pitch=0.129°, roll=0.517°")
print(f"  (Slight difference due to quaternion->Euler conversion and subsequent updates)")

print(f"\n{'─'*80}")
print("SUMMARY")
print(f"{'─'*80}")

print(f"\nThe initial pitch={pitch_deg:.3f}° and roll={roll_deg:.3f}° come from:")
print(f"  1. Accelerometer measures gravity: {accel_mps2}")
print(f"  2. Gravity direction shows sensor is TILTED (not level)")
print(f"  3. Tilt is computed during initialization in _init_orient_mag()")
print(f"  4. Initial quaternion encodes this tilt")
print(f"  5. THEN gyro integration starts from this tilted orientation")

print(f"\nThis is CORRECT behavior:")
print(f"  - Real sensors are rarely perfectly level")
print(f"  - Accelerometer provides absolute tilt reference")
print(f"  - Gyro integration starts from this real-world orientation")

print(f"\n{'─'*80}")
print("PHYSICAL INTERPRETATION")
print(f"{'─'*80}")

print(f"\nImagine the sensor on a table:")
print(f"  - If table is perfectly level: pitch=0°, roll=0°")
print(f"  - If table tilts forward: pitch > 0° (nose up)")
print(f"  - If table tilts left: roll > 0° (left side up)")

print(f"\nYour sensor:")
print(f"  - Nose tilted up: {pitch_deg:.3f}°")
print(f"  - Left side tilted up: {roll_deg:.3f}°")
print(f"  - Almost level, but not perfectly!")

print(f"\n{'─'*80}")
print("GYRO INTEGRATION EFFECT")
print(f"{'─'*80}")

print(f"\nStarting orientation: pitch={pitch_deg:.3f}°")
print(f"Gyro Y-axis: {-0.336:.3f} dps (negative = pitch down)")
print(f"After 40ms: pitch change = -0.013°")
print(f"Final orientation: pitch = {pitch_deg:.3f} - 0.013 = {pitch_deg - 0.013:.3f}°")
print(f"\nActual from debug: pitch = 0.129° ✓")
