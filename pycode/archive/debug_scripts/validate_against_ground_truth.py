#!/usr/bin/env python3
"""
Validate Python sensor fusion implementation against ground truth data.
Uses test_input_output_0922.h which contains both inputs and expected outputs.
"""

import numpy as np
import sys
import os
import re

# Add pycode to path
sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def parse_test_data_header(filepath):
    """
    Parse test_input_output_0922.h to extract sensor inputs and expected outputs.
    """
    with open(filepath, 'r') as f:
        content = f.read()

    # Extract sensor input data
    input_pattern = r'static const test_sensor_sample_t sensor_input_data\[\] = \{(.*?)\};'
    input_match = re.search(input_pattern, content, re.DOTALL)

    if not input_match:
        raise ValueError("Could not find sensor_input_data in header file")

    input_data = []
    # Parse each line like: {0, -51, -529, 8018, 72146875ULL},
    for line in input_match.group(1).split('\n'):
        line = line.strip()
        if not line or line.startswith('//') or line.startswith('/*'):
            continue

        # Extract values between { and }
        match = re.search(r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}', line)
        if match:
            sensor_id = int(match.group(1))
            x = int(match.group(2))
            y = int(match.group(3))
            z = int(match.group(4))
            ts = int(match.group(5))
            input_data.append({
                'id': sensor_id,
                'x': x,
                'y': y,
                'z': z,
                'ts': ts
            })

    # Extract expected output data
    output_pattern = r'static const test_expected_output_t expected_output_data\[\] = \{(.*?)\};'
    output_match = re.search(output_pattern, content, re.DOTALL)

    if not output_match:
        raise ValueError("Could not find expected_output_data in header file")

    expected_outputs = []
    # Parse each line like: {0.999999f, -0.001572f, -0.000515f, 0.000035f, 0.000f, 0.000f, 0.000f, 91883500ULL},
    for line in output_match.group(1).split('\n'):
        line = line.strip()
        if not line or line.startswith('//') or line.startswith('/*'):
            continue

        # Extract quaternion values
        match = re.search(r'\{([0-9.-]+)f,\s*([0-9.-]+)f,\s*([0-9.-]+)f,\s*([0-9.-]+)f,\s*([0-9.-]+)f,\s*([0-9.-]+)f,\s*([0-9.-]+)f,\s*(\d+)ULL\}', line)
        if match:
            q0 = float(match.group(1))
            q1 = float(match.group(2))
            q2 = float(match.group(3))
            q3 = float(match.group(4))
            roll = float(match.group(5))
            pitch = float(match.group(6))
            yaw = float(match.group(7))
            ts = int(match.group(8))
            expected_outputs.append({
                'quat': np.array([q0, q1, q2, q3]),
                'euler': np.array([roll, pitch, yaw]),
                'ts': ts
            })

    return input_data, expected_outputs

def quaternion_distance(q1, q2):
    """Compute minimum distance between quaternions (handles q/-q ambiguity)."""
    d1 = np.linalg.norm(q1 - q2)
    d2 = np.linalg.norm(q1 + q2)
    return min(d1, d2)

def quaternion_angle_difference(q1, q2):
    """Compute angular difference in degrees."""
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = np.abs(np.dot(q1, q2))
    dot = np.clip(dot, 0.0, 1.0)
    angle_rad = 2 * np.arccos(dot)
    return np.degrees(angle_rad)

def main():
    print("=" * 80)
    print("PYTHON FUSION VALIDATION AGAINST GROUND TRUTH")
    print("=" * 80)
    print()

    # Parse test data
    test_data_file = '../test/data/testdata/fusion/test_input_output_0922.h'
    print(f"Parsing test data: {test_data_file}")

    try:
        input_data, expected_outputs = parse_test_data_header(test_data_file)
    except Exception as e:
        print(f"Error parsing test data: {e}")
        return

    print(f"  Found {len(input_data)} input samples")
    print(f"  Found {len(expected_outputs)} expected outputs")
    print()

    # Filter out identity quaternions (ground truth not available)
    valid_expected = []
    for exp in expected_outputs:
        # Skip identity quaternions {1, 0, 0, 0} - no real ground truth
        if not (exp['quat'][0] == 1.0 and np.all(exp['quat'][1:] == 0.0)):
            valid_expected.append(exp)

    print(f"  Valid ground truth outputs (non-identity): {len(valid_expected)}")
    print()

    # Initialize Python sensor fusion
    print("Initializing Python sensor fusion...")
    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )
    print(f"  Initialized with INVENSENSE sensor specs")
    print(f"    Accel scale: {platform.accel_scale_factor:.10f} g/count")
    print(f"    Gyro scale:  {platform.gyro_scale_factor:.10f} dps/count")
    print()

    # Run fusion and collect outputs
    print("Running sensor fusion...")
    python_outputs = []
    fusion_count = 0

    for i, sample in enumerate(input_data):
        sensor_id = sample['id']

        # Skip magnetometer for 6-axis fusion
        if sensor_id == 2:
            continue

        sensor_xyz = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)

        ready = sf.preprocess_sensor_data(sensor_id, sensor_xyz, sample['ts'])

        if ready:
            output = sf.run()
            quat_array = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            python_outputs.append({
                'quat': quat_array.copy(),
                'ts': sample['ts'],
                'sample_idx': i
            })
            fusion_count += 1

            if fusion_count % 100 == 0:
                print(f"  Processed {fusion_count} fusion runs...")

    print(f"  Total Python fusion outputs: {fusion_count}")
    print()

    # Match outputs by timestamp
    print("Matching outputs by timestamp...")
    matched_pairs = []

    for py_out in python_outputs:
        # Find closest ground truth by timestamp
        closest_gt = None
        min_time_diff = float('inf')

        for gt_out in valid_expected:
            time_diff = abs(py_out['ts'] - gt_out['ts'])
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_gt = gt_out

        # Only match if timestamps are close (within 1ms = 1,000,000 ns)
        if min_time_diff < 1_000_000:
            matched_pairs.append({
                'python': py_out,
                'ground_truth': closest_gt,
                'time_diff_ns': min_time_diff
            })

    print(f"  Matched {len(matched_pairs)} Python outputs to ground truth")
    print()

    if len(matched_pairs) == 0:
        print("ERROR: No outputs could be matched! Check timestamp alignment.")
        return

    # Compute accuracy metrics
    print("=" * 80)
    print("ACCURACY ANALYSIS")
    print("=" * 80)
    print()

    distances = []
    angles = []

    for pair in matched_pairs:
        py_quat = pair['python']['quat']
        gt_quat = pair['ground_truth']['quat']

        dist = quaternion_distance(py_quat, gt_quat)
        angle = quaternion_angle_difference(py_quat, gt_quat)

        distances.append(dist)
        angles.append(angle)

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

    # Accuracy distribution
    thresholds = [1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1]
    print("Accuracy Distribution:")
    for thresh in thresholds:
        count = np.sum(distances <= thresh)
        percent = 100.0 * count / len(distances)
        print(f"  Distance <= {thresh:.0e}: {count:4d} / {len(distances)} ({percent:.1f}%)")
    print()

    # Show worst mismatches
    worst_indices = np.argsort(distances)[-10:][::-1]

    print("Top 10 Worst Mismatches:")
    for idx in worst_indices:
        pair = matched_pairs[idx]
        print(f"  Sample {pair['python']['sample_idx']:4d}: dist={distances[idx]:.6e}, angle={angles[idx]:.6f}°")
        print(f"    Ground Truth: [{pair['ground_truth']['quat'][0]:.10f}, {pair['ground_truth']['quat'][1]:.10f}, "
              f"{pair['ground_truth']['quat'][2]:.10f}, {pair['ground_truth']['quat'][3]:.10f}]")
        print(f"    Python:       [{pair['python']['quat'][0]:.10f}, {pair['python']['quat'][1]:.10f}, "
              f"{pair['python']['quat'][2]:.10f}, {pair['python']['quat'][3]:.10f}]")
        print(f"    Timestamp: {pair['ground_truth']['ts']}, Time diff: {pair['time_diff_ns']} ns")
    print()

    # Summary
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print()

    if np.max(angles) < 1.0:
        print("✓ EXCELLENT: Max angular error < 1.0°")
        print("  Python implementation matches ground truth within acceptable tolerance.")
    elif np.max(angles) < 5.0:
        print("✓ GOOD: Max angular error < 5.0°")
        print("  Python implementation is reasonably accurate but could be improved.")
    elif np.max(angles) < 20.0:
        print("⚠ FAIR: Max angular error < 20.0°")
        print("  Python implementation has noticeable errors. Investigation recommended.")
    else:
        print("✗ POOR: Max angular error >= 20.0°")
        print("  Python implementation has significant errors. Debugging required.")

    print()
    print(f"Mean angular error: {np.mean(angles):.6f}° (target: < 0.5°)")
    print(f"Max angular error:  {np.max(angles):.6f}° (target: < 1.0°)")
    print()

if __name__ == '__main__':
    main()
