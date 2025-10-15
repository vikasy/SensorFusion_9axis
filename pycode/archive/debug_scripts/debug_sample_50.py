"""
Debug sample 50 where divergence is noticeable
Compare C and Python intermediate values
"""
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
    ACC_SCALE = 4.0 / 32767.0  # ±4g range
    GYRO_SCALE = 1000.0 / 32767.0  # ±1000 dps range

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    fusion_run_count = 0
    target_run = 50

    print(f"Running to fusion run #{target_run} with detailed debug")

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
            fusion_run_count += 1

            if fusion_run_count == target_run:
                print(f"\n{'='*80}")
                print(f"PYTHON Fusion run #{fusion_run_count}")
                print(f"{'='*80}")
                print(f"Input sample index: {i}")
                print(f"Timestamp: {timestamp}")

            # Run fusion with debug for target run
            if fusion_run_count == target_run:
                # Initialize orientation if needed
                if not sf.orient_init:
                    mag_check = 0.0
                    for j in range(3):
                        val = sf.acc_data.count_avg[j] * sf.acc_data.scale_factor * 9.80665
                        mag_check += val * val

                    if mag_check < 1e-6:
                        accel_avg = sf.acc_data.count_buff[0] * sf.acc_data.scale_factor * 9.80665
                    else:
                        accel_avg = sf.acc_data.count_avg * sf.acc_data.scale_factor * 9.80665

                    sf._init_orient(accel_avg)

                # Run time update
                if sf.nom_updt_ts < sf.gyro_data.timestamp:
                    print(f"\nRunning time_update...")
                    print(f"Quat BEFORE time_update: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")
                    print(f"BiasPostS BEFORE time_update: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")
                    sf.time_update(debug=False)
                    print(f"Quat AFTER time_update: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

                # Run measurement update
                if sf.meas_updt_ts < sf.acc_data.timestamp:
                    print(f"\nRunning measurement_update WITH DEBUG...")
                    sf.measurement_update(debug=True)
                    print(f"\nFinal Quat: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")
                    print(f"Final BiasPostS: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")

                # Update gravity
                for i_ch in range(3):
                    sf.grav_post_s[i_ch] = -1.0 * 9.80665 * sf.rot_mtx_post[i_ch, 2]

                # Clear signal
                sf.signal_sf_run = 0
                break
            else:
                # Run normally without debug
                output = sf.run()
                if fusion_run_count % 10 == 0:
                    print(f"Run {fusion_run_count}: Quat=[{sf.quat_post.q0:.6f}, {sf.quat_post.q1:.6f}, {sf.quat_post.q2:.6f}, {sf.quat_post.q3:.6f}], BiasPostS=[{sf.bias_post_s[0]:.6f}, {sf.bias_post_s[1]:.6f}, {sf.bias_post_s[2]:.6f}]")

    print(f"\n{'='*80}")
    print(f"DONE - Compare with C debug output")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
