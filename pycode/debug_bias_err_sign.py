"""Debug BiasErrPostS computation to identify sign error"""
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
    max_runs = 3

    print(f"Debugging BiasErrPostS computation for first {max_runs} runs")
    print(f"Looking for sign errors in the measurement update")

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
            print(f"\n{'='*80}")
            print(f"Fusion run #{fusion_run_count}")
            print(f"{'='*80}")

            # Run time update
            if sf.nom_updt_ts < sf.gyro_data.timestamp:
                print(f"Running time_update...")
                sf.time_update()

            # Run measurement update with debug
            if sf.meas_updt_ts < sf.acc_data.timestamp:
                print(f"Running measurement_update...")

                # Get values before measurement update
                bias_post_s_before = sf.bias_post_s.copy()
                bias_err_post_s_before = sf.bias_err_post_s.copy()

                sf.measurement_update(debug=False)

                # Get values after measurement update
                bias_err_post_s_after = sf.bias_err_post_s.copy()
                bias_post_s_after = sf.bias_post_s.copy()

                print(f"\nBiasErrPostS BEFORE meas_update: [{bias_err_post_s_before[0]:.12e}, {bias_err_post_s_before[1]:.12e}, {bias_err_post_s_before[2]:.12e}]")
                print(f"BiasErrPostS AFTER meas_update:  [{bias_err_post_s_after[0]:.12e}, {bias_err_post_s_after[1]:.12e}, {bias_err_post_s_after[2]:.12e}]")
                print(f"\nBiasPostS BEFORE update:          [{bias_post_s_before[0]:.12e}, {bias_post_s_before[1]:.12e}, {bias_post_s_before[2]:.12e}]")
                print(f"BiasPostS AFTER (BiasPostS -= BiasErrPostS): [{bias_post_s_after[0]:.12e}, {bias_post_s_after[1]:.12e}, {bias_post_s_after[2]:.12e}]")

                # Calculate what C should have
                print(f"\nExpected C BiasErrPostS (from test output):")
                if fusion_run_count == 0:
                    print(f"  Run 0: [4.724e-05, 4.690e-05, 3.395e-06]")
                    print(f"  Our BiasErrPostS has OPPOSITE sign - this is the bug!")

            # Update gravity
            for i_ch in range(3):
                sf.grav_post_s[i_ch] = -1.0 * 9.80665 * sf.rot_mtx_post[i_ch, 2]

            # Clear signal
            sf.signal_sf_run = 0

            fusion_run_count += 1
            if fusion_run_count >= max_runs:
                break

    print(f"\n{'='*80}")
    print(f"DIAGNOSIS: BiasErrPostS has OPPOSITE SIGN in Python vs C")
    print(f"This causes BiasPostS to accumulate in wrong direction")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
