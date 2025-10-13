#!/usr/bin/env python3
"""
Compare C and Python sensor fusion outputs from CSV files
"""

import csv
import math
import numpy as np

def quaternion_distance(q1, q2):
    """Compute quaternion distance: 1 - |q1 · q2|"""
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    if dot > 1.0:
        dot = 1.0
    return 1.0 - dot

def angular_error(q1, q2):
    """Compute angular error from quaternion distance in degrees"""
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    if dot > 1.0:
        dot = 1.0
    angle_rad = 2.0 * math.acos(dot)
    return math.degrees(angle_rad)

def angle_diff(a1, a2):
    """Compute smallest angle difference handling wraparound"""
    diff = abs(a1 - a2)
    if diff > 180:
        diff = 360 - diff
    return diff

def main():
    print("\n" + "="*80)
    print("C vs PYTHON SYNCHRONIZED COMPARISON")
    print("="*80)
    print()

    # Read C outputs
    c_data = []
    with open('c_outputs_0922.csv', 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            c_data.append({
                'ts': int(row['timestamp']),
                'q0': float(row['q0']),
                'q1': float(row['q1']),
                'q2': float(row['q2']),
                'q3': float(row['q3']),
                'roll': float(row['roll']),
                'pitch': float(row['pitch']),
                'yaw': float(row['yaw'])
            })

    # Read Python outputs
    py_data = []
    with open('python_outputs_0922.csv', 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            py_data.append({
                'ts': int(row['timestamp']),
                'q0': float(row['q0']),
                'q1': float(row['q1']),
                'q2': float(row['q2']),
                'q3': float(row['q3']),
                'roll': float(row['roll']),
                'pitch': float(row['pitch']),
                'yaw': float(row['yaw'])
            })

    print(f"Loaded {len(c_data)} C samples")
    print(f"Loaded {len(py_data)} Python samples")

    if len(c_data) != len(py_data):
        print(f"\n⚠️  WARNING: Different number of samples!")
        return

    # Compare sample by sample
    quat_errors = []
    angle_errors = []
    roll_errors = []
    pitch_errors = []
    yaw_errors = []

    large_error_samples = []

    for i, (c, py) in enumerate(zip(c_data, py_data)):
        if c['ts'] != py['ts']:
            print(f"\n⚠️  ERROR: Timestamp mismatch at sample {i}: C={c['ts']}, Python={py['ts']}")
            continue

        c_quat = [c['q0'], c['q1'], c['q2'], c['q3']]
        py_quat = [py['q0'], py['q1'], py['q2'], py['q3']]

        quat_dist = quaternion_distance(c_quat, py_quat)
        ang_err = angular_error(c_quat, py_quat)

        roll_err = angle_diff(c['roll'], py['roll'])
        pitch_err = angle_diff(c['pitch'], py['pitch'])
        yaw_err = angle_diff(c['yaw'], py['yaw'])

        quat_errors.append(quat_dist)
        angle_errors.append(ang_err)
        roll_errors.append(roll_err)
        pitch_errors.append(pitch_err)
        yaw_errors.append(yaw_err)

        # Flag samples with significant errors
        if quat_dist > 0.01 or ang_err > 5.0:
            large_error_samples.append({
                'idx': i,
                'ts': c['ts'],
                'quat_dist': quat_dist,
                'ang_err': ang_err,
                'c_quat': c_quat,
                'py_quat': py_quat,
                'c_angles': [c['roll'], c['pitch'], c['yaw']],
                'py_angles': [py['roll'], py['pitch'], py['yaw']]
            })

    # Compute statistics
    quat_errors = np.array(quat_errors)
    angle_errors = np.array(angle_errors)
    roll_errors = np.array(roll_errors)
    pitch_errors = np.array(pitch_errors)
    yaw_errors = np.array(yaw_errors)

    print("\n" + "="*80)
    print("COMPARISON STATISTICS")
    print("="*80)
    print()
    print("Quaternion Distance (1 - |q1·q2|):")
    print(f"  Mean:   {np.mean(quat_errors):.6f}")
    print(f"  Median: {np.median(quat_errors):.6f}")
    print(f"  Max:    {np.max(quat_errors):.6f}")
    print(f"  Min:    {np.min(quat_errors):.6f}")
    print()
    print("Angular Error (degrees):")
    print(f"  Mean:   {np.mean(angle_errors):.3f}°")
    print(f"  Median: {np.median(angle_errors):.3f}°")
    print(f"  Max:    {np.max(angle_errors):.3f}°")
    print(f"  Min:    {np.min(angle_errors):.3f}°")
    print()
    print("Euler Angle Errors (degrees):")
    print(f"  Roll:   mean={np.mean(roll_errors):.3f}°, max={np.max(roll_errors):.3f}°")
    print(f"  Pitch:  mean={np.mean(pitch_errors):.3f}°, max={np.max(pitch_errors):.3f}°")
    print(f"  Yaw:    mean={np.mean(yaw_errors):.3f}°, max={np.max(yaw_errors):.3f}°")
    print()

    # Count samples within tolerance
    quat_within_tol = np.sum(quat_errors < 0.001)
    angle_within_tol = np.sum(angle_errors < 1.0)

    print(f"Samples within tolerance:")
    print(f"  Quat distance < 0.001: {quat_within_tol}/{len(quat_errors)} ({100*quat_within_tol/len(quat_errors):.1f}%)")
    print(f"  Angle error < 1.0°:    {angle_within_tol}/{len(angle_errors)} ({100*angle_within_tol/len(angle_errors):.1f}%)")
    print()

    # Show first few samples with large errors
    if large_error_samples:
        print(f"\n⚠️  Found {len(large_error_samples)} samples with large errors")
        print(f"\nFirst 10 samples with errors:")
        for sample in large_error_samples[:10]:
            print(f"\nSample {sample['idx']} (ts={sample['ts']}):")
            print(f"  Quat distance: {sample['quat_dist']:.6f}, Angle error: {sample['ang_err']:.3f}°")
            print(f"  C quat:      [{sample['c_quat'][0]:.6f}, {sample['c_quat'][1]:.6f}, {sample['c_quat'][2]:.6f}, {sample['c_quat'][3]:.6f}]")
            print(f"  Python quat: [{sample['py_quat'][0]:.6f}, {sample['py_quat'][1]:.6f}, {sample['py_quat'][2]:.6f}, {sample['py_quat'][3]:.6f}]")
            print(f"  C angles:      Roll={sample['c_angles'][0]:.2f}°, Pitch={sample['c_angles'][1]:.2f}°, Yaw={sample['c_angles'][2]:.2f}°")
            print(f"  Python angles: Roll={sample['py_angles'][0]:.2f}°, Pitch={sample['py_angles'][1]:.2f}°, Yaw={sample['py_angles'][2]:.2f}°")

    # Overall assessment
    print("\n" + "="*80)
    mean_quat_err = np.mean(quat_errors)
    mean_angle_err = np.mean(angle_errors)

    if mean_quat_err < 0.001 and mean_angle_err < 1.0:
        print("✅ RESULT: C and Python implementations MATCH within tolerance")
    elif mean_quat_err < 0.01 and mean_angle_err < 5.0:
        print("⚠️  RESULT: C and Python implementations have MINOR differences")
    else:
        print("❌ RESULT: C and Python implementations have SIGNIFICANT differences")

    print(f"   Mean quaternion distance: {mean_quat_err:.6f} (threshold: 0.001)")
    print(f"   Mean angular error: {mean_angle_err:.3f}° (threshold: 1.0°)")
    print("="*80)
    print()

if __name__ == '__main__':
    main()
