#!/usr/bin/env python3
"""
Debug the initialization step - where does the initial quaternion come from?
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("=" * 80)
    print("INITIALIZATION DEBUG - Where does initial quaternion come from?")
    print("=" * 80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nInitial state after __init__:")
    print(f"  orient_init: {sf.orient_init}")
    print(f"  quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
    print(f"  acc_data.count_avg: [{sf.acc_data.count_avg[0]:.6f}, {sf.acc_data.count_avg[1]:.6f}, {sf.acc_data.count_avg[2]:.6f}] g")
    print(f"  mag_data.count_avg: [{sf.mag_data.count_avg[0]:.6f}, {sf.mag_data.count_avg[1]:.6f}, {sf.mag_data.count_avg[2]:.6f}] µT")

    # Process samples until first fusion
    for idx in range(len(df)):
        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        print(f"\n{'─'*80}")
        print(f"Sample {idx}:")
        print(f"  accel_counts: [{accel_counts[0]:7.0f}, {accel_counts[1]:7.0f}, {accel_counts[2]:7.0f}]")
        print(f"  gyro_counts:  [{gyro_counts[0]:7.0f}, {gyro_counts[1]:7.0f}, {gyro_counts[2]:7.0f}]")
        print(f"  mag_counts:   [{mag_counts[0]:7.0f}, {mag_counts[1]:7.0f}, {mag_counts[2]:7.0f}]")

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        print(f"  After preprocessing:")
        print(f"    acc_data.count_avg: [{sf.acc_data.count_avg[0]:.8f}, {sf.acc_data.count_avg[1]:.8f}, {sf.acc_data.count_avg[2]:.8f}] g")
        print(f"    mag_data.count_avg: [{sf.mag_data.count_avg[0]:.8f}, {sf.mag_data.count_avg[1]:.8f}, {sf.mag_data.count_avg[2]:.8f}] µT")
        print(f"    orient_init: {sf.orient_init}")

        if ready_gyro & 0x2:  # Gyro ready
            print(f"\n{'='*80}")
            print(f"GYRO READY - About to call run()")
            print(f"{'='*80}")
            print(f"State BEFORE run():")
            print(f"  orient_init: {sf.orient_init}")
            print(f"  quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
            print(f"  acc_data.count_avg: [{sf.acc_data.count_avg[0]:.8f}, {sf.acc_data.count_avg[1]:.8f}, {sf.acc_data.count_avg[2]:.8f}] g")
            print(f"  mag_data.count_avg: [{sf.mag_data.count_avg[0]:.8f}, {sf.mag_data.count_avg[1]:.8f}, {sf.mag_data.count_avg[2]:.8f}] µT")

            # Manually compute what initialization should produce
            from sensor_fusion_6axis import GTOMSEC2
            accel_avg_mps2 = sf.acc_data.count_avg * GTOMSEC2
            mag_avg_ut = sf.mag_data.count_avg

            print(f"\nInitialization inputs:")
            print(f"  accel_avg (m/s²): [{accel_avg_mps2[0]:.6f}, {accel_avg_mps2[1]:.6f}, {accel_avg_mps2[2]:.6f}]")
            print(f"  mag_avg (µT):     [{mag_avg_ut[0]:.6f}, {mag_avg_ut[1]:.6f}, {mag_avg_ut[2]:.6f}]")

            # Compute expected tilt from accelerometer
            accel_norm = np.linalg.norm(accel_avg_mps2)
            print(f"  accel magnitude: {accel_norm:.6f} m/s² (expect ~9.81)")

            if accel_norm > 0:
                down = accel_avg_mps2 / accel_norm
                print(f"  down vector (normalized): [{down[0]:.6f}, {down[1]:.6f}, {down[2]:.6f}]")

                # Compute expected pitch and roll from down vector
                pitch_rad = np.arcsin(-down[0])
                roll_rad = np.arctan2(down[1], down[2])
                pitch_deg = np.degrees(pitch_rad)
                roll_deg = np.degrees(roll_rad)

                print(f"  Expected from accel: pitch={pitch_deg:.3f}°, roll={roll_deg:.3f}°")

            output = sf.run()

            print(f"\nState AFTER run():")
            print(f"  orient_init: {sf.orient_init}")
            print(f"  quat_post: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
            print(f"  Output orientation: [yaw={output.orientation[0]:.3f}°, pitch={output.orientation[1]:.3f}°, roll={output.orientation[2]:.3f}°]")

            break

if __name__ == '__main__':
    main()
