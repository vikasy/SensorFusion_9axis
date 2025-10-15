#!/usr/bin/env python3
"""
Test the magnetometer disturbance detection with actual values
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis

# Sample data from trace
mag_measured_samples = [
    [24.903, 0.439, 43.361],   # Sample 2
    [25.196, 1.465, 43.068],   # Sample 3
    [24.757, 0.000, 43.800],   # Sample 4
    [24.024, -1.172, 42.921],  # Sample 5
]

mag_field_ref = np.array([0.493, 0.016, 0.870])  # Normalized reference from first sample

print("="*80)
print("MAGNETOMETER DISTURBANCE DETECTION TEST")
print("="*80)
print(f"\nReference field (normalized): [{mag_field_ref[0]:.6f}, {mag_field_ref[1]:.6f}, {mag_field_ref[2]:.6f}]")
print(f"Reference magnitude: {np.linalg.norm(mag_field_ref):.6f}\n")

for i, mag_raw in enumerate(mag_measured_samples):
    mag_raw = np.array(mag_raw)
    print(f"\n{'─'*80}")
    print(f"Sample {i+2}:")
    print(f"{'─'*80}")
    print(f"Raw magnetometer: [{mag_raw[0]:.3f}, {mag_raw[1]:.3f}, {mag_raw[2]:.3f}] µT")

    # Normalize
    mag_norm = np.linalg.norm(mag_raw)
    mag_measured_normalized = mag_raw / mag_norm

    print(f"Normalized: [{mag_measured_normalized[0]:.6f}, {mag_measured_normalized[1]:.6f}, {mag_measured_normalized[2]:.6f}]")
    print(f"Magnitude: {np.linalg.norm(mag_measured_normalized):.6f}")

    # Test disturbance detection
    disturbance = SensorFusion9Axis.detect_mag_disturbance(mag_measured_normalized, mag_field_ref, 0.3)

    # Detailed check
    ref_norm = np.linalg.norm(mag_field_ref)
    meas_norm = np.linalg.norm(mag_measured_normalized)

    mag_diff = abs(meas_norm - ref_norm) / ref_norm
    dot_product = np.dot(mag_measured_normalized, mag_field_ref)

    print(f"\nDisturbance checks:")
    print(f"  Magnitude difference: {mag_diff:.6f} (threshold: 0.3)")
    print(f"    → Magnitude OK: {mag_diff <= 0.3}")
    print(f"  Dot product: {dot_product:.6f} (threshold: 0.866 = cos(30°))")
    print(f"    → Direction OK: {dot_product >= 0.866}")
    print(f"\nDisturbance detected: {disturbance}")

print(f"\n{'='*80}")
print("TEST COMPLETE")
print(f"{'='*80}")
