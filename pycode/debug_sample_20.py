"""Debug measurement_update at fusion run 20 where divergence occurs"""
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

    fusion_run_count = 0
    target_run = 20  # Fusion run 20

    print(f"Running until fusion run {target_run}...")

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
            # Check if this is our target fusion run
            if fusion_run_count == target_run:
                print(f"\n{'='*80}")
                print(f"PYTHON - Fusion run {fusion_run_count} (input index {i})")
                print(f"{'='*80}")
                print(f"Timestamp: {timestamp}")
                print(f"Sensor ID: {sensor_id} (0=ACC, 1=GYRO)")
                print(f"Sensor counts: [{sample['x']}, {sample['y']}, {sample['z']}]")

                print(f"\n=== Before run() ===")
                print(f"BiasPostS: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")

                # Check if measurement_update will run
                will_run_meas = sf.meas_updt_ts < sf.acc_data.timestamp
                print(f"Will run measurement_update: {will_run_meas}")

                # Run fusion
                output = sf.run()

                print(f"\n=== After run() ===")
                print(f"Quat: [{output.quat.q0:.15f}, {output.quat.q1:.15f}, {output.quat.q2:.15f}, {output.quat.q3:.15f}]")
                print(f"BiasPostS: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")

                # Compare with C quaternion
                c_quat = [0.998929262161255, -0.045318800956011, -0.009165452793241, 0.001594663248397]
                print(f"\n=== Comparison with C ===")
                print(f"C Quat:      [{c_quat[0]:.15f}, {c_quat[1]:.15f}, {c_quat[2]:.15f}, {c_quat[3]:.15f}]")
                print(f"Python Quat: [{output.quat.q0:.15f}, {output.quat.q1:.15f}, {output.quat.q2:.15f}, {output.quat.q3:.15f}]")
                print(f"\nDifferences:")
                print(f"  q0: {output.quat.q0 - c_quat[0]:+.15e}")
                print(f"  q1: {output.quat.q1 - c_quat[1]:+.15e}")
                print(f"  q2: {output.quat.q2 - c_quat[2]:+.15e}")
                print(f"  q3: {output.quat.q3 - c_quat[3]:+.15e}")

                break

            output = sf.run()
            fusion_run_count += 1

        if fusion_run_count > target_run:
            print(f"Passed target fusion run without finding it")
            break

    print(f"\nProcessed {fusion_run_count} fusion runs")

if __name__ == '__main__':
    main()
