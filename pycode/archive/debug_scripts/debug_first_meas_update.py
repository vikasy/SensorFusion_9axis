"""Debug first measurement_update to compare with C"""
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

    print(f"Looking for first measurement_update...\n")

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
            # Check if measurement_update will run
            will_run_meas = sf.meas_updt_ts < sf.acc_data.timestamp

            if fusion_run_count == 0 and will_run_meas:
                print(f"={'='*80}")
                print(f"PYTHON - First measurement_update (fusion run {fusion_run_count})")
                print(f"={'='*80}")
                print(f"Timestamp: {timestamp}")
                print(f"Input sample index: {i}")

                # Check if orientation needs initialization (like C code does)
                if not sf.orient_init:
                    print("\n6-axis SF algo initial orientation lock")

                    # Check if count_avg has been populated
                    mag_check = 0.0
                    for ii in range(3):
                        val = sf.acc_data.count_avg[ii] * sf.acc_data.scale_factor * 9.80665
                        mag_check += val * val

                    if mag_check < 1e-6:
                        accel_avg = sf.acc_data.count_buff[0] * sf.acc_data.scale_factor * 9.80665
                    else:
                        accel_avg = sf.acc_data.count_avg * sf.acc_data.scale_factor * 9.80665

                    sf._init_orient(accel_avg)
                    print(f"Quat after init_orient: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

                print(f"\nBefore time_update:")
                print(f"  Quat: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

                # Manually call time_update first
                sf.time_update()

                # Now debug measurement_update
                print(f"\nAfter time_update (before measurement_update):")
                print(f"  BiasPostS: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")
                print(f"  Quat: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

                # Call measurement_update with debug
                sf.measurement_update(debug=True)

                print(f"\nAfter measurement_update:")
                print(f"  BiasPostS: [{sf.bias_post_s[0]:.15f}, {sf.bias_post_s[1]:.15f}, {sf.bias_post_s[2]:.15f}]")
                print(f"  Quat: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

                # Clear signal flags
                sf.signal_sf_run = 0
                break

            fusion_run_count += 1

if __name__ == '__main__':
    main()
