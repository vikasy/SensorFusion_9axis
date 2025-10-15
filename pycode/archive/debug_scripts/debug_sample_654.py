"""Debug specific sample around divergence point"""
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
    target_ts = 4741909075  # Timestamp where C shows sample 654

    print(f"Running until timestamp {target_ts}...")

    for i, sample in enumerate(sensor_data):
        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        if sensor_id == 2:  # Skip magnetometer
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Run fusion
        output = sf.run()
        sample_count += 1

        # Check if we're at target timestamp
        if timestamp == target_ts:
            will_run_time_update = sf.nom_updt_ts < sf.gyro_data.timestamp
            print(f"\n{'='*80}")
            print(f"PYTHON - Sample {sample_count} (input index {i})")
            print(f"{'='*80}")
            print(f"Timestamp: {timestamp}")
            print(f"Sensor ID: {sensor_id} (0=ACC, 1=GYRO)")
            print(f"Sensor counts: [{sample['x']}, {sample['y']}, {sample['z']}]")
            print(f"Will run time_update: {will_run_time_update}")
            print(f"  nom_updt_ts: {sf.nom_updt_ts}")
            print(f"  gyro.timestamp: {sf.gyro_data.timestamp}")
            print(f"\nState:")
            print(f"  Quat: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
            print(f"  BiasErrPostS: [{sf.bias_err_post_s[0]:.8f}, {sf.bias_err_post_s[1]:.8f}, {sf.bias_err_post_s[2]:.8f}]")
            print(f"  BiasPostS: [{sf.bias_post_s[0]:.8f}, {sf.bias_post_s[1]:.8f}, {sf.bias_post_s[2]:.8f}]")

            if will_run_time_update and sensor_id == 1:
                print(f"\nCalling time_update with debug...")
                sf.time_update(debug=True)
                print(f"\nAfter time_update:")
                print(f"  Quat: [{sf.quat_post.q0:.8f}, {sf.quat_post.q1:.8f}, {sf.quat_post.q2:.8f}, {sf.quat_post.q3:.8f}]")
                break

        if timestamp > target_ts:
            print(f"Passed target timestamp. Last sample was at {sample_count}")
            break

    print(f"\\nProcessed {sample_count} samples")

if __name__ == '__main__':
    main()
