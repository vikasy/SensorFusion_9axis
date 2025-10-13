#!/usr/bin/env python3
"""
Generate Python 6-axis sensor fusion output using FREESCALE sensor specifications
Uses same test data as INVENSENSE but with FREESCALE scale factors

Author: Vikas Yadav
Date: 2025-10-12
"""

import sys
import re
import csv
import numpy as np

# Import sensor fusion
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

# FREESCALE sensor specifications
# FXOS8700CQ: 14-bit accelerometer, ±4g range
FSL_ACCEL_RANGE_G = 4.0
FSL_ACCEL_MAX_COUNT = 8191  # 14-bit: 2^13 - 1
FSL_ACC_SCALE = FSL_ACCEL_RANGE_G / FSL_ACCEL_MAX_COUNT

# FXAS21000: 16-bit gyroscope, ±1000 dps range
FSL_GYRO_RANGE_DPS = 1000.0
FSL_GYRO_MAX_COUNT = 32767  # 16-bit
FSL_GYRO_SCALE = FSL_GYRO_RANGE_DPS / FSL_GYRO_MAX_COUNT

print("=" * 80)
print("FREESCALE Sensor Specifications:")
print(f"  Accelerometer (FXOS8700CQ): 14-bit, ±{FSL_ACCEL_RANGE_G}g")
print(f"    Scale: {FSL_ACC_SCALE:.10f} g/count")
print(f"  Gyroscope (FXAS21000): 16-bit, ±{FSL_GYRO_RANGE_DPS}dps")
print(f"    Scale: {FSL_GYRO_SCALE:.10f} dps/count")
print("=" * 80)
print()

# Initialize fusion with FREESCALE specs
fusion = SensorFusion6Axis(acc_scale=FSL_ACC_SCALE, gyro_scale=FSL_GYRO_SCALE)

# Parse test data
print("Loading test data...")
with open('../test/data/testdata/fusion/test_input_output_0922.h', 'r') as f:
    content = f.read()

pattern = r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}'
matches = re.findall(pattern, content)

sensor_data = []
for match in matches:
    sensor_data.append({
        'id': int(match[0]),
        'x': int(match[1]),
        'y': int(match[2]),
        'z': int(match[3]),
        'ts': int(match[4])
    })

print(f"Loaded {len(sensor_data)} sensor samples")

# Output file
output_file = 'python_freescale_quaternions_0922.csv'
fusion_count = 0

print(f"Processing samples with FREESCALE specs...")

with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['sample', 'timestamp', 'sensor_id', 'q0', 'q1', 'q2', 'q3'])

    for sample in sensor_data:
        sensor_id = sample['id']

        # Skip magnetometer (id=2)
        if sensor_id == 2:
            continue

        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess sensor data
        ready = fusion.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Run fusion only when both sensors are ready (ready == 3)
        if ready == 3:
            output = fusion.run()

            # Write quaternion
            writer.writerow([
                fusion_count,
                timestamp,
                sensor_id,
                f"{output.quat.q0:.15f}",
                f"{output.quat.q1:.15f}",
                f"{output.quat.q2:.15f}",
                f"{output.quat.q3:.15f}"
            ])

            fusion_count += 1

            if fusion_count % 100 == 0:
                print(f"  Fusion runs: {fusion_count}...")

print(f"\nComplete! Generated {fusion_count} fusion outputs")
print(f"Output saved to: {output_file}")
