#!/usr/bin/env python3
"""
Comprehensive accuracy and performance analysis for sensor fusion.
Compares C and Python implementations by loading quaternion CSV files.
"""

import numpy as np
import sys
import os

def load_quaternions_csv(filepath):
    """Load quaternion outputs from CSV file."""
    data = []
    with open(filepath, 'r') as f:
        header = next(f).strip()  # Read header
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 7:
                sample_idx = int(parts[0])
                timestamp = int(parts[1])
                sensor_id = int(parts[2])
                # Handle both formats: Python (7 cols) and C (8 cols)
                if len(parts) == 7:
                    q0, q1, q2, q3 = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                else:  # C format with sensor_data column
                    q0, q1, q2, q3 = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                data.append({
                    'sample_idx': sample_idx,
                    'timestamp': timestamp,
                    'sensor_id': sensor_id,
                    'quat': np.array([q0, q1, q2, q3])
                })
    return data

def quaternion_distance(q1, q2):
    """
    Compute distance between two quaternions.
    Uses the minimum of d(q1, q2) and d(q1, -q2) to handle double-cover.
    """
    d1 = np.linalg.norm(q1 - q2)
    d2 = np.linalg.norm(q1 + q2)
    return min(d1, d2)

def quaternion_angle_difference(q1, q2):
    """
    Compute angular difference between two quaternions in degrees.
    """
    # Ensure quaternions are normalized
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute dot product
    dot = np.abs(np.dot(q1, q2))
    dot = np.clip(dot, 0.0, 1.0)

    # Compute angle
    angle_rad = 2 * np.arccos(dot)
    angle_deg = np.degrees(angle_rad)

    return angle_deg

def main():
    print("=" * 80)
    print("COMPREHENSIVE ACCURACY AND PERFORMANCE ANALYSIS")
    print("=" * 80)
    print()

    # Load C and Python reference files
    c_ref_file = '../build/c_reference_quaternions_0922.csv'
    py_ref_file = '../build/python_reference_quaternions_0922.csv'

    if not os.path.exists(c_ref_file):
        print(f"Error: C reference file not found at {c_ref_file}")
        print("Please run: ./bin/generate_c_reference")
        return

    if not os.path.exists(py_ref_file):
        print(f"Error: Python reference file not found at {py_ref_file}")
        print("Please run: python3 generate_python_reference.py")
        return

    print("Loading C reference quaternions...")
    c_data = load_quaternions_csv(c_ref_file)
    print(f"  Loaded {len(c_data)} C quaternion outputs")
    print()

    print("Loading Python reference quaternions...")
    python_quats = load_quaternions_csv(py_ref_file)
    print(f"  Loaded {len(python_quats)} Python quaternion outputs")
    print()

    # Compare C and Python
    print("=" * 80)
    print("ACCURACY COMPARISON: C vs Python")
    print("=" * 80)
    print()

    min_len = min(len(c_data), len(python_quats))
    print(f"Comparing first {min_len} quaternion outputs...")
    print()

    distances = []
    angles = []
    mismatches = []

    for i in range(min_len):
        c_quat = c_data[i]['quat']
        py_quat = python_quats[i]['quat']

        dist = quaternion_distance(c_quat, py_quat)
        angle = quaternion_angle_difference(c_quat, py_quat)

        distances.append(dist)
        angles.append(angle)

        # Track significant mismatches (>1e-4)
        if dist > 1e-4:
            mismatches.append({
                'index': i,
                'distance': dist,
                'angle': angle,
                'c_quat': c_quat,
                'py_quat': py_quat
            })

    distances = np.array(distances)
    angles = np.array(angles)

    print(f"Quaternion Distance Statistics:")
    print(f"  Mean:        {np.mean(distances):.6e}")
    print(f"  Median:      {np.median(distances):.6e}")
    print(f"  Std Dev:     {np.std(distances):.6e}")
    print(f"  Min:         {np.min(distances):.6e}")
    print(f"  Max:         {np.max(distances):.6e}")
    print(f"  95th %ile:   {np.percentile(distances, 95):.6e}")
    print(f"  99th %ile:   {np.percentile(distances, 99):.6e}")
    print()

    print(f"Angular Difference Statistics (degrees):")
    print(f"  Mean:        {np.mean(angles):.6f}°")
    print(f"  Median:      {np.median(angles):.6f}°")
    print(f"  Std Dev:     {np.std(angles):.6f}°")
    print(f"  Min:         {np.min(angles):.6f}°")
    print(f"  Max:         {np.max(angles):.6f}°")
    print(f"  95th %ile:   {np.percentile(angles, 95):.6f}°")
    print(f"  99th %ile:   {np.percentile(angles, 99):.6f}°")
    print()

    # Accuracy thresholds
    thresholds = [1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3]
    print("Accuracy Distribution:")
    for thresh in thresholds:
        count = np.sum(distances <= thresh)
        percent = 100.0 * count / len(distances)
        print(f"  Distance <= {thresh:.0e}: {count:4d} / {len(distances)} ({percent:.1f}%)")
    print()

    # Report significant mismatches
    if mismatches:
        print(f"Found {len(mismatches)} significant mismatches (distance > 1e-4):")
        for mismatch in mismatches[:10]:  # Show first 10
            print(f"  Sample {mismatch['index']:4d}: dist={mismatch['distance']:.6e}, angle={mismatch['angle']:.6f}°")
            print(f"    C:      [{mismatch['c_quat'][0]:.10f}, {mismatch['c_quat'][1]:.10f}, "
                  f"{mismatch['c_quat'][2]:.10f}, {mismatch['c_quat'][3]:.10f}]")
            print(f"    Python: [{mismatch['py_quat'][0]:.10f}, {mismatch['py_quat'][1]:.10f}, "
                  f"{mismatch['py_quat'][2]:.10f}, {mismatch['py_quat'][3]:.10f}]")
        if len(mismatches) > 10:
            print(f"  ... and {len(mismatches) - 10} more")
    else:
        print("No significant mismatches found! C and Python outputs are highly consistent.")
    print()

    # Performance metrics
    print("=" * 80)
    print("PERFORMANCE METRICS")
    print("=" * 80)
    print()
    print(f"Total C outputs: {len(c_data)}")
    print(f"Total Python outputs: {len(python_quats)}")
    print(f"Outputs compared: {min_len}")
    print()

    # Summary
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print()

    if np.max(distances) < 1e-4:
        print("✓ EXCELLENT: Maximum distance < 1e-4")
        print("  C and Python implementations are numerically equivalent.")
    elif np.max(distances) < 1e-3:
        print("✓ GOOD: Maximum distance < 1e-3")
        print("  Minor numerical differences exist but are acceptable.")
    else:
        print("✗ POOR: Maximum distance >= 1e-3")
        print("  Significant numerical differences detected. Investigation needed.")

    print()
    print(f"Mean angular error: {np.mean(angles):.6f}° (target: < 0.1°)")
    print(f"Max angular error:  {np.max(angles):.6f}° (target: < 1.0°)")
    print()

if __name__ == '__main__':
    main()
