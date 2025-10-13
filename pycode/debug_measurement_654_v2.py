"""Debug measurement_update at specific sample around divergence point"""
import numpy as np
import re
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

def parse_testdata_header(header_file):
    with open(header_file, 'r') as f:
        content = f.read()

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
    sensor_data = parse_testdata_header(header_file)

    # Initialize with correct scales
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1000.0 / 32767.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    sample_count = 0
    target_sample = 654  # C sample where we want to debug measurement_update

    print(f"Running until sample {target_sample}...")

    for i, sample in enumerate(sensor_data):
        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        if sensor_id == 2:  # Skip magnetometer
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Run with debug at target sample
        if sample_count == target_sample:
            print(f"\n{'='*80}")
            print(f"PYTHON - Sample {sample_count} (input index {i})")
            print(f"{'='*80}")
            print(f"Timestamp: {timestamp}")
            print(f"Sensor ID: {sensor_id} (0=ACC, 1=GYRO)")

            # Manually check if measurement_update will run
            will_run_meas = sf.meas_updt_ts < sf.acc_data.timestamp
            print(f"Will run meas_update: {will_run_meas}")

            if will_run_meas:
                # Call measurement_update with debug
                sf.measurement_update(debug=True)
                print(f"\nFinal BiasPostS: [{sf.bias_post_s[0]:.8f}, {sf.bias_post_s[1]:.8f}, {sf.bias_post_s[2]:.8f}]")
            break

        # Run fusion normally
        output = sf.run()
        sample_count += 1

    print(f"\nProcessed {sample_count} samples")

if __name__ == '__main__':
    main()
