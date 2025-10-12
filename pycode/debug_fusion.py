"""
Debug script to understand the gyroscope integration issue
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sensor_fusion_6axis import SensorFusion6Axis, SensorID

# MPU9250 sensor specifications
ACC_SCALE = 1.0 / 16384.0
GYRO_SCALE = 1.0 / 131.0

sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

# Test with a few samples of rotation
sample_rate = 100
rotation_rate = 45.0  # deg/s

print("="*70)
print("DEBUGGING GYROSCOPE INTEGRATION")
print("="*70)

for i in range(5):
    timestamp = i * (1000000000 // sample_rate)

    # Level accelerometer
    acc_data = np.array([0, 0, -16384], dtype=np.int16)

    # Constant rotation about Z
    gyro_data = np.array([0, 0, int(rotation_rate * 131)], dtype=np.int16)

    print(f"\n--- Sample {i} ---")
    print(f"Gyro input (counts): {gyro_data}")
    print(f"Gyro input (deg/s): {gyro_data[2] * GYRO_SCALE:.2f}")

    # Before preprocessing
    quat_before_preproc = sf6.quat_post.to_array()
    print(f"Quat before preprocess: [{quat_before_preproc[0]:.6f}, {quat_before_preproc[1]:.6f}, {quat_before_preproc[2]:.6f}, {quat_before_preproc[3]:.6f}]")

    # Preprocess
    sf6.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
    sf6.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)

    # Before run
    quat_before_run = sf6.quat_post.to_array()

    # Run fusion
    output = sf6.run()

    # After run
    quat_after_run = output.quat.to_array()

    print(f"Quat after run: [{quat_after_run[0]:.6f}, {quat_after_run[1]:.6f}, {quat_after_run[2]:.6f}, {quat_after_run[3]:.6f}]")
    print(f"Yaw angle: {output.orientation[0]:.6f}°")
    print(f"Omega: {sf6.omega}")
    print(f"Bias: {sf6.bias_post_s}")
    print(f"Bias error: {sf6.bias_err_post_s}")
    print(f"Orient error: {sf6.ornt_err_post_s}")

    # Check if quaternion changed
    quat_change = np.linalg.norm(quat_after_run - quat_before_preproc)
    print(f"Quaternion change magnitude: {quat_change:.10f}")

print("\n" + "="*70)
print("Expected: Yaw should increase by ~0.45° per sample (45 deg/s * 0.01s)")
print("="*70)
