"""
Compare C and Python measurement_update outputs for the first fusion run
to identify exactly where the BiasErrPostS sign error occurs
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

    # Initialize with correct scales (must match C code configuration)
    ACC_SCALE = 4.0 / 32767.0  # ±4g range (MPU9250 configuration - matches C code sensor_spec_agm.h)
    GYRO_SCALE = 1000.0 / 32767.0  # ±1000 dps range

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    fusion_run_count = 0

    print(f"Python measurement_update debug for first fusion run")
    print(f"C output will be shown by test_compare_python with debug_meas=True")

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
            print(f"\n{'='*80}")
            print(f"PYTHON Fusion run #{fusion_run_count}")
            print(f"{'='*80}")

            # Initialize orientation if needed
            if not sf.orient_init:
                mag_check = 0.0
                for i in range(3):
                    val = sf.acc_data.count_avg[i] * sf.acc_data.scale_factor * 9.80665
                    mag_check += val * val

                if mag_check < 1e-6:
                    accel_avg = sf.acc_data.count_buff[0] * sf.acc_data.scale_factor * 9.80665
                else:
                    accel_avg = sf.acc_data.count_avg * sf.acc_data.scale_factor * 9.80665

                sf._init_orient(accel_avg)
                print(f"Quat after init_orient: [{sf.quat_post.q0:.15f}, {sf.quat_post.q1:.15f}, {sf.quat_post.q2:.15f}, {sf.quat_post.q3:.15f}]")

            # Run time update
            if sf.nom_updt_ts < sf.gyro_data.timestamp:
                print(f"Running time_update...")
                sf.time_update()

            # Run measurement update with debug for first run only
            if sf.meas_updt_ts < sf.acc_data.timestamp:
                if fusion_run_count == 1:
                    print(f"Running measurement_update WITH DEBUG...")
                    sf.measurement_update(debug=True)
                else:
                    sf.measurement_update(debug=False)

            # Update gravity
            for i_ch in range(3):
                sf.grav_post_s[i_ch] = -1.0 * 9.80665 * sf.rot_mtx_post[i_ch, 2]

            # Clear signal
            sf.signal_sf_run = 0

            if fusion_run_count >= 1:
                break

    print(f"\n{'='*80}")
    print(f"DONE - Now run test_compare_python to see C debug output")
    print(f"Compare the intermediate values to find where sign error occurs")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
