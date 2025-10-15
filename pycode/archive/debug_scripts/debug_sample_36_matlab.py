#!/usr/bin/env python3
"""
Debug Python sensor fusion at sample 38 (which matches MATLAB sample 36)
to compare intermediate values
"""

import sys
sys.path.insert(0, '/Users/vikasyadav/Library/CloudStorage/OneDrive-Personal/Documents/GitHub/SensorFusion_9axis/pycode')

from sensor_fusion_6axis import SensorFusion6Axis, PhysSensor, SensorID
import numpy as np

# Initialize fusion
fusion = SensorFusion6Axis(acc_scale=4.0/32767.0, gyro_scale=1000.0/32767.0)

# Parse test data
with open('../test/data/testdata/fusion/test_input_output_0922.h', 'r') as f:
    content = f.read()

import re
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

# Constants matching Python
SF_OVERSAMPLE_RATIO = 4
ACC_SCALE = 4.0 / 32767.0  # ±4g
GYRO_SCALE = 1000.0 / 32767.0  # ±1000 dps
G2MPSECSQ = 9.80665

# Buffers
acc_buffer = np.zeros((3, SF_OVERSAMPLE_RATIO))
gyro_buffer = np.zeros((3, SF_OVERSAMPLE_RATIO))
acc_count = 0
gyro_count = 0

fusion_count = 0

for sample in sensor_data:
    sensor_id = sample['id']

    # Skip magnetometer
    if sensor_id == 2:
        continue

    sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=float)
    timestamp = sample['ts']

    if sensor_id == 0:  # Accelerometer
        acc_buffer[:, acc_count] = sensor_counts
        acc_count += 1
        if acc_count == 1:
            acc_ts = timestamp
        acc_ready = (acc_count == SF_OVERSAMPLE_RATIO)
        if acc_ready:
            acc_count = 0

    elif sensor_id == 1:  # Gyroscope
        gyro_buffer[:, gyro_count] = sensor_counts
        gyro_count += 1
        if gyro_count == 1:
            gyro_ts = timestamp
        gyro_ready = (gyro_count == SF_OVERSAMPLE_RATIO)
        if gyro_ready:
            gyro_count = 0

    # Run fusion when both ready
    if 'acc_ready' in locals() and 'gyro_ready' in locals() and acc_ready and gyro_ready:
        acc_ready = False
        gyro_ready = False

        # Prepare sensor data
        acc_data = np.mean(acc_buffer, axis=1) * ACC_SCALE * G2MPSECSQ
        gyro_data = np.mean(gyro_buffer, axis=1) * GYRO_SCALE
        gyro_hist = gyro_buffer * GYRO_SCALE

        # Debug sample 38 (matches MATLAB sample 36)
        if fusion_count == 38:
            print(f"\n=== Python Sample 38 (matches MATLAB 36) DEBUG ===")
            print(f"Acc input: [{acc_data[0]:.15f}, {acc_data[1]:.15f}, {acc_data[2]:.15f}]")
            print(f"Gyro input (mean): [{gyro_data[0]:.15f}, {gyro_data[1]:.15f}, {gyro_data[2]:.15f}]")
            print(f"QuatPost BEFORE: [{fusion.quat_post.q0:.15f}, {fusion.quat_post.q1:.15f}, {fusion.quat_post.q2:.15f}, {fusion.quat_post.q3:.15f}]")

        # Create sensor data structures
        acc_sensor = PhysSensor()
        acc_sensor.id = SensorID.ACC
        acc_sensor.ts = acc_ts
        acc_sensor.data = acc_data.copy()

        gyro_sensor = PhysSensor()
        gyro_sensor.id = SensorID.GYRO
        gyro_sensor.ts = gyro_ts
        gyro_sensor.data = gyro_data.copy()
        gyro_sensor.count_buff = gyro_hist.T.copy()  # Transpose for correct shape

        # Run fusion
        output = fusion.measurement_update(acc_sensor, gyro_sensor)

        # Debug sample 38
        if fusion_count == 38:
            print(f"QuatPost AFTER: [{fusion.quat_post.q0:.15f}, {fusion.quat_post.q1:.15f}, {fusion.quat_post.q2:.15f}, {fusion.quat_post.q3:.15f}]")
            print(f"\nCompare with MATLAB sample 36:")
            print(f"MATLAB BEFORE: [0.998707152388492, -0.049142921836704, -0.012654823587912, 0.002975305258364]")
            print(f"MATLAB AFTER:  [0.998682988038471, -0.049526819839122, -0.013041776258522, 0.003048867199122]")
            break

        fusion_count += 1

print(f"\nProcessed {fusion_count} fusion iterations")
