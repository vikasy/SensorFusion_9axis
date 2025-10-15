#!/usr/bin/env python3
"""
Track magnetometer measurement updates and their effect on orientation
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

def main():
    print("="*80)
    print("MAGNETOMETER CORRECTION TRACKING")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Monkey-patch to track mag update details
    original_mag_update = sf.measurement_update_mag
    mag_update_count = 0
    mag_corrections = []

    def tracked_mag_update():
        nonlocal mag_update_count

        # Store state before
        quat_before = sf.quat_post.to_array().copy()
        bias_before = sf.bias_post_s.copy()

        # Get mag data
        mag_measured_raw = sf.mag_data.count_avg.copy()
        mag_norm = np.linalg.norm(mag_measured_raw)
        mag_measured = mag_measured_raw / mag_norm if mag_norm > 1e-12 else mag_measured_raw

        # Get expected (reference projected to sensor frame)
        mag_expected = sf.rot_mtx_post @ sf.mag_field_ref if np.linalg.norm(sf.mag_field_ref) > 1e-12 else np.zeros(3)

        # Innovation
        mag_error = mag_measured - mag_expected
        mag_error_mag = np.linalg.norm(mag_error)

        # Call original
        original_mag_update()

        # Store state after
        quat_after = sf.quat_post.to_array().copy()
        bias_after = sf.bias_post_s.copy()

        # Compute changes
        quat_change = quaternion_angular_distance(quat_before, quat_after)
        bias_change = np.linalg.norm(bias_after - bias_before)

        mag_update_count += 1
        mag_corrections.append({
            'count': mag_update_count,
            'mag_measured': mag_measured.copy(),
            'mag_expected': mag_expected.copy(),
            'mag_error': mag_error.copy(),
            'error_mag': mag_error_mag,
            'quat_change': quat_change,
            'bias_change': bias_change,
            'quat_before': quat_before,
            'quat_after': quat_after,
            'bias_before': bias_before,
            'bias_after': bias_after
        })

    sf.measurement_update_mag = tracked_mag_update

    print(f"\nDataset: {dataset_path}")
    print(f"Tracking magnetometer corrections\n")

    fusion_count = 0
    max_samples = 50

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

    # Analysis
    print("="*80)
    print("MAGNETOMETER CORRECTION ANALYSIS")
    print("="*80)

    if len(mag_corrections) > 0:
        print(f"\nTotal mag updates: {len(mag_corrections)}")

        errors = np.array([c['error_mag'] for c in mag_corrections])
        quat_changes = np.array([c['quat_change'] for c in mag_corrections])
        bias_changes = np.array([c['bias_change'] for c in mag_corrections])

        print(f"\nMag Innovation (error magnitude):")
        print(f"  Mean:  {errors.mean():.6f}")
        print(f"  Std:   {errors.std():.6f}")
        print(f"  Min:   {errors.min():.6f}")
        print(f"  Max:   {errors.max():.6f}")

        print(f"\nQuaternion changes from mag updates:")
        print(f"  Mean:  {quat_changes.mean():.6f}°")
        print(f"  Std:   {quat_changes.std():.6f}°")
        print(f"  Min:   {quat_changes.min():.6f}°")
        print(f"  Max:   {quat_changes.max():.6f}°")

        print(f"\nBias changes from mag updates:")
        print(f"  Mean:  {bias_changes.mean():.6f} dps")
        print(f"  Std:   {bias_changes.std():.6f} dps")
        print(f"  Min:   {bias_changes.min():.6f} dps")
        print(f"  Max:   {bias_changes.max():.6f} dps")

        # Show first few and last few updates
        print("\n" + "="*80)
        print("FIRST 5 MAG UPDATES")
        print("="*80)

        for i in range(min(5, len(mag_corrections))):
            c = mag_corrections[i]
            print(f"\nMag Update #{c['count']}:")
            print(f"  Measured:  [{c['mag_measured'][0]:+.6f}, {c['mag_measured'][1]:+.6f}, {c['mag_measured'][2]:+.6f}]")
            print(f"  Expected:  [{c['mag_expected'][0]:+.6f}, {c['mag_expected'][1]:+.6f}, {c['mag_expected'][2]:+.6f}]")
            print(f"  Error:     [{c['mag_error'][0]:+.6f}, {c['mag_error'][1]:+.6f}, {c['mag_error'][2]:+.6f}] (mag: {c['error_mag']:.6f})")
            print(f"  Quat Δ:    {c['quat_change']:.6f}°")
            print(f"  Bias Δ:    {c['bias_change']:.6f} dps")

        if len(mag_corrections) > 5:
            print("\n" + "="*80)
            print("LAST 5 MAG UPDATES")
            print("="*80)

            for i in range(max(0, len(mag_corrections)-5), len(mag_corrections)):
                c = mag_corrections[i]
                print(f"\nMag Update #{c['count']}:")
                print(f"  Measured:  [{c['mag_measured'][0]:+.6f}, {c['mag_measured'][1]:+.6f}, {c['mag_measured'][2]:+.6f}]")
                print(f"  Expected:  [{c['mag_expected'][0]:+.6f}, {c['mag_expected'][1]:+.6f}, {c['mag_expected'][2]:+.6f}]")
                print(f"  Error:     [{c['mag_error'][0]:+.6f}, {c['mag_error'][1]:+.6f}, {c['mag_error'][2]:+.6f}] (mag: {c['error_mag']:.6f})")
                print(f"  Quat Δ:    {c['quat_change']:.6f}°")
                print(f"  Bias Δ:    {c['bias_change']:.6f} dps")

        # Check reference
        print("\n" + "="*80)
        print("MAGNETOMETER REFERENCE")
        print("="*80)
        print(f"\nMag field reference (global frame):")
        print(f"  [{sf.mag_field_ref[0]:+.6f}, {sf.mag_field_ref[1]:+.6f}, {sf.mag_field_ref[2]:+.6f}]")
        print(f"  Magnitude: {np.linalg.norm(sf.mag_field_ref):.6f}")

        # Check if reference is reasonable
        # For static data, all mag measurements should be similar
        mag_meas_list = [c['mag_measured'] for c in mag_corrections]
        mag_meas_array = np.array(mag_meas_list)
        mag_meas_std = mag_meas_array.std(axis=0)

        print(f"\nMag measurement consistency (std dev across all updates):")
        print(f"  [{mag_meas_std[0]:.6f}, {mag_meas_std[1]:.6f}, {mag_meas_std[2]:.6f}]")
        print(f"  Total std: {np.linalg.norm(mag_meas_std):.6f}")

        if np.linalg.norm(mag_meas_std) < 0.01:
            print(f"  ✓ Measurements are CONSISTENT (static data)")
        else:
            print(f"  ⚠ Measurements show VARIATION (unexpected for static)")

        # Verdict
        print("\n" + "="*80)
        print("VERDICT")
        print("="*80)

        if errors.mean() > 0.1:
            print(f"\n✗ Large mag innovations (mean={errors.mean():.6f})")
            print(f"  This suggests mag reference or expected calculation is wrong")
        else:
            print(f"\n✓ Mag innovations are small (mean={errors.mean():.6f})")

        if quat_changes.mean() > 0.1:
            print(f"\n⚠ Mag corrections causing large quat changes (mean={quat_changes.mean():.6f}°)")
            print(f"  This could destabilize the filter")
        else:
            print(f"\n✓ Mag corrections are reasonable (mean={quat_changes.mean():.6f}°)")

        if bias_changes.mean() > 0.1:
            print(f"\n✗ Mag updates changing bias significantly (mean={bias_changes.mean():.6f} dps)")
            print(f"  MAG SHOULD NOT AFFECT GYRO BIAS!")
            print(f"  This is a BUG in the measurement matrix C!")
        else:
            print(f"\n✓ Mag not significantly affecting bias")

    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
