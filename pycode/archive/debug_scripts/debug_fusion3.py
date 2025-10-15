"""
Debug which updates are being called
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sensor_fusion_6axis import SensorFusion6Axis, SensorID

# Patch the run method to add debug output
original_run = SensorFusion6Axis.run

def debug_run(self):
    print(f"  nom_updt_ts: {self.nom_updt_ts}, gyro_ts: {self.gyro_data.timestamp}")
    print(f"  meas_updt_ts: {self.meas_updt_ts}, acc_ts: {self.acc_data.timestamp}")

    # Check which updates will be called
    will_call_time_update = self.nom_updt_ts < self.gyro_data.timestamp
    will_call_meas_update = self.meas_updt_ts < self.acc_data.timestamp

    print(f"  Will call time_update: {will_call_time_update}")
    print(f"  Will call meas_update: {will_call_meas_update}")

    result = original_run(self)

    print(f"  After: nom_updt_ts: {self.nom_updt_ts}, meas_updt_ts: {self.meas_updt_ts}")

    return result

SensorFusion6Axis.run = debug_run

# MPU9250 sensor specifications
ACC_SCALE = 1.0 / 16384.0
GYRO_SCALE = 1.0 / 131.0

sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

# Test with a few samples
sample_rate = 100
rotation_rate = 45.0

print("="*70)
print("DEBUGGING UPDATE CALLS")
print("="*70)

for i in range(4):
    timestamp = i * (1000000000 // sample_rate)

    acc_data = np.array([0, 0, -16384], dtype=np.int16)
    gyro_data = np.array([0, 0, int(rotation_rate * 131)], dtype=np.int16)

    print(f"\n--- Sample {i} (timestamp={timestamp}) ---")

    sf6.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
    sf6.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)

    output = sf6.run()

    print(f"  Final yaw: {output.orientation[0]:.6f}°")
