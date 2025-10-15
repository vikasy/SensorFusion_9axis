#!/usr/bin/env python3
"""
Trace when fusion cycles happen and why
"""

import numpy as np
import pandas as pd
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE

def main():
    print("="*80)
    print("FUSION TIMING TRACE")
    print("="*80)

    dataset_path = "../test/data/datasets/synthetic/static_10s.csv"
    df = pd.read_csv(dataset_path)

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    print(f"\nKey: SF_OVERSAMPLE_RATIO = 4 (need 4 gyro samples to trigger fusion)\n")

    fusion_count = 0
    max_fusions = 5

    for idx in range(len(df)):
        if fusion_count >= max_fusions:
            break

        row = df.iloc[idx]
        timestamp = int(row['timestamp_ns'])

        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

        print(f"\n{'─'*80}")
        print(f"Sample {idx}: timestamp={timestamp}")
        print(f"{'─'*80}")

        # Preprocess and check ready flags
        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        # Check buffer fill status
        print(f"Sensor ready flags: acc={ready_acc:#x}, gyro={ready_gyro:#x}, mag={ready_mag:#x}")

        # Decode gyro ready flag
        gyro_ready = (ready_gyro & 0x2) != 0
        print(f"  Gyro buffer full (0x2 flag): {gyro_ready}")

        if gyro_ready:
            fusion_count += 1
            print(f"\n  → FUSION CYCLE {fusion_count} TRIGGERED")

            # Show which samples are in the buffers
            print(f"\n  Buffers contain samples from CSV indices:")
            print(f"    Accel: samples {idx-3} to {idx}")
            print(f"    Gyro:  samples {idx-3} to {idx}")
            print(f"    Mag:   sample {idx}")

            output = sf.run()
            print(f"\n  Output: yaw={output.orientation[0]:.3f}°, pitch={output.orientation[1]:.3f}°, roll={output.orientation[2]:.3f}°")

    print(f"\n{'='*80}")
    print(f"TRACE COMPLETE")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
