"""
Debug test to check if quaternion is changing during integration
"""

import numpy as np
import re
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

def parse_testdata_header(header_file):
    """Parse the C header file to extract sensor inputs"""
    with open(header_file, 'r') as f:
        content = f.read()

    # Parse sensor input data
    sensor_pattern = r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}'
    sensor_matches = re.findall(sensor_pattern, content)

    sensor_data = []
    for match in sensor_matches:
        sensor_data.append({
            'id': int(match[0]),
            'x': int(match[1]),
            'y': int(match[2]),
            'z': int(match[3]),
            'ts': int(match[4])
        })

    return sensor_data

def main():
    header_file = '../test/data/testdata/fusion/test_input_output_0922.h'

    print("Parsing testdata header file...")
    sensor_data = parse_testdata_header(header_file)
    print(f"Loaded {len(sensor_data)} sensor samples\n")

    # Initialize sensor fusion (must match C)
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1000.0 / 32767.0  # MPU9250 ±1000dps range

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    print("="*80)
    print("DEBUG: First 20 samples with detailed quaternion integration trace")
    print("="*80)

    sample_count = 0

    for i, sample in enumerate(sensor_data):
        if i >= 20:  # Only debug first 20 samples
            break

        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        # Skip magnetometer data (id=2)
        if sensor_id == 2:
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        print(f"\n--- Sample {i} (sensor_id={sensor_id}, ts={timestamp}) ---")
        print(f"Sensor counts: [{sample['x']}, {sample['y']}, {sample['z']}]")

        # Preprocess sensor data
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)
        print(f"Ready flags: 0x{ready:02X} (ACC_READY=0x01, GYRO_READY=0x02)")
        print(f"orient_init={sf.orient_init}, nom_updt_ts={sf.nom_updt_ts}, gyro.ts={sf.gyro_data.timestamp}")
        print(f"meas_updt_ts={sf.meas_updt_ts}, acc.ts={sf.acc_data.timestamp}")

        # Check if time_update will run
        will_run_time_update = sf.nom_updt_ts < sf.gyro_data.timestamp
        will_run_meas_update = sf.meas_updt_ts < sf.acc_data.timestamp

        print(f"Will run time_update: {will_run_time_update}")
        print(f"Will run meas_update: {will_run_meas_update}")

        # Modify run() temporarily to add debug flag
        if will_run_time_update:
            print("\n*** CALLING TIME_UPDATE WITH DEBUG ***")
            sf.time_update(debug=True)

        # Run normally (but skip time_update since we already ran it)
        output = sf.run()
        sample_count += 1

        print(f"\nFinal quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
        angles = output.orientation
        print(f"Angles: Roll={angles[2]:.2f}°, Pitch={angles[1]:.2f}°, Yaw={angles[0]:.2f}°")

if __name__ == '__main__':
    main()
