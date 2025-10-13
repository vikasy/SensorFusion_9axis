"""Track BiasPostS evolution for first 21 fusion runs"""
import numpy as np
import re
import csv
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

    fusion_run_count = 0
    max_runs = 21

    print(f"Tracking BiasPostS for first {max_runs} fusion runs...")
    print(f"\n{'Run':>4} {'Timestamp':>15} {'BiasPostS[0]':>18} {'BiasPostS[1]':>18} {'BiasPostS[2]':>18}")
    print(f"{'-'*4} {'-'*15} {'-'*18} {'-'*18} {'-'*18}")

    for i, sample in enumerate(sensor_data):
        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        if sensor_id == 2:  # Skip magnetometer
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Check if fusion will run
        if ready == 3:
            # Run fusion
            output = sf.run()

            # Print BiasPostS after this run
            print(f"{fusion_run_count:>4} {timestamp:>15} {sf.bias_post_s[0]:>18.12f} {sf.bias_post_s[1]:>18.12f} {sf.bias_post_s[2]:>18.12f}")

            fusion_run_count += 1

            if fusion_run_count > max_runs:
                break

    print(f"\nProcessed {fusion_run_count} fusion runs")

if __name__ == '__main__':
    main()
