"""Generate Python reference quaternions for all samples"""
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
    ACC_SCALE = 4.0 / 32767.0  # ±4g range (MPU9250 configuration - matches C code sensor_spec_agm.h)
    GYRO_SCALE = 1000.0 / 32767.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Output file for Python reference
    output_file = 'python_reference_quaternions_0922.csv'

    print(f"Generating Python reference quaternions...")
    print(f"Output file: {output_file}")

    sample_count = 0

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        # Header: sample_count, timestamp, sensor_id, q0, q1, q2, q3
        writer.writerow(['sample', 'timestamp', 'sensor_id', 'q0', 'q1', 'q2', 'q3'])

        fusion_run_count = 0

        for i, sample in enumerate(sensor_data):
            sensor_id = sample['id']
            sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
            timestamp = sample['ts']

            if sensor_id == 2:  # Skip magnetometer
                continue

            py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

            # Preprocess
            ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

            # Run fusion only when both sensors are ready (ready == 3)
            if ready == 3:
                output = sf.run()

                # Write quaternion only when fusion actually ran
                writer.writerow([
                    fusion_run_count,
                    timestamp,
                    sensor_id,
                    f"{output.quat.q0:.15f}",
                    f"{output.quat.q1:.15f}",
                    f"{output.quat.q2:.15f}",
                    f"{output.quat.q3:.15f}"
                ])

                fusion_run_count += 1

                if fusion_run_count % 100 == 0:
                    print(f"  Fusion runs: {fusion_run_count}...")

            sample_count += 1

    print(f"\nComplete! Processed {sample_count} input samples")
    print(f"Generated {fusion_run_count} fusion outputs (reference quaternions)")
    print(f"Output saved to: {output_file}")

if __name__ == '__main__':
    main()
