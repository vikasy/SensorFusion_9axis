"""
Debug script to check gravity error during rotation
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sensor_fusion_6axis import SensorFusion6Axis, SensorID, GTOMSEC2, SF_DELTA_T

# MPU9250 sensor specifications
ACC_SCALE = 1.0 / 16384.0
GYRO_SCALE = 1.0 / 131.0

sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

# Test with a few samples of rotation
sample_rate = 100
rotation_rate = 45.0  # deg/s

print("="*70)
print("DEBUGGING GRAVITY ERROR DURING ROTATION")
print("="*70)

for i in range(3):
    timestamp = i * (1000000000 // sample_rate)

    # Level accelerometer (1g down in -Z)
    acc_data = np.array([0, 0, -16384], dtype=np.int16)

    # Constant rotation about Z
    gyro_data = np.array([0, 0, int(rotation_rate * 131)], dtype=np.int16)

    print(f"\n--- Sample {i} ---")

    # Preprocess
    sf6.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
    sf6.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)

    # Before measurement update
    quat_before = sf6.quat_post.to_array()

    # Manually compute what measurement_update sees
    if i > 0:  # After first initialization
        # Compute gravity from gyro (expected)
        grav_gyr_pri_s = np.zeros(3)
        for idx in range(3):
            grav_gyr_pri_s[idx] = -sf6.rot_mtx_post[idx, 2] * GTOMSEC2

        # Compute gravity from accel (measured)
        grav_acc = np.zeros(3)
        for idx in range(3):
            grav_acc[idx] = acc_data[idx] * ACC_SCALE * GTOMSEC2 * -1.0
            grav_acc[idx] += sf6.lin_acc_tc * sf6.acc_post_s[idx]

        # Gravity error
        grav_err = grav_acc - grav_gyr_pri_s

        print(f"Grav from gyro (expected): {grav_gyr_pri_s}")
        print(f"Grav from accel (measured): {grav_acc}")
        print(f"Gravity ERROR: {grav_err}")
        print(f"Error magnitude: {np.linalg.norm(grav_err):.6f}")

    # Run fusion
    output = sf6.run()

    quat_after = output.quat.to_array()

    print(f"Quat before: [{quat_before[0]:.6f}, {quat_before[1]:.6f}, {quat_before[2]:.6f}, {quat_before[3]:.6f}]")
    print(f"Quat after:  [{quat_after[0]:.6f}, {quat_after[1]:.6f}, {quat_after[2]:.6f}, {quat_after[3]:.6f}]")
    print(f"Yaw: {output.orientation[0]:.6f}°")
    print(f"Orient error applied: {sf6.ornt_err_post_s}")

print("\n" + "="*70)
print("Analysis:")
print("If gravity error is near zero, measurement update shouldn't correct.")
print("If gravity error is large, measurement update is seeing false error.")
print("="*70)
