"""Compare BiasPostS evolution between C (from test output) and Python"""
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
    max_runs = 30

    print(f"Comparing BiasPostS between C and Python for first {max_runs} fusion runs")
    print(f"C output from test_compare_python shows: BiasPostS=[x, y, z]")
    print(f"\n{'Run':>4} {'Python BiasPostS[0]':>20} {'Python BiasPostS[1]':>20} {'Python BiasPostS[2]':>20}")
    print(f"{'-'*4} {'-'*20} {'-'*20} {'-'*20}")

    # C BiasPostS values from test_compare_python output
    c_bias = {
        1: [-0.000047244363, -0.000046895707, -0.000003394524],
        2: [-0.000132009529, -0.000262990910, -0.000018485864],
        3: [-0.000465917339, -0.000697599350, -0.000049643491],
        4: [-0.001072692465, -0.001440028461, -0.000103150914],
        5: [-0.002693931386, -0.002917782858, -0.000210195181],
        6: [-0.004486627028, -0.004723890391, -0.000340975555],
        7: [-0.005971592304, -0.006838715685, -0.000495378842],
        8: [-0.008506592324, -0.009737854649, -0.000706552591],
        9: [-0.011721087032, -0.013358158269, -0.000970658583],
       10: [-0.016091484717, -0.017788922612, -0.001292088693],
       11: [-0.021234494249, -0.023618247515, -0.001719362446],
       12: [-0.027271383922, -0.030172296646, -0.002198754303],
       13: [-0.034397866242, -0.037009506015, -0.002691904018],
       14: [-0.042508629110, -0.045250374822, -0.003291078287],
       15: [-0.051740768245, -0.054610933556, -0.003971817296],
       16: [-0.061435546215, -0.065885552263, -0.004810226733],
       17: [-0.072695984604, -0.078034211493, -0.005704658235],
       18: [-0.085063941498, -0.090654024556, -0.006625228148],
       19: [-0.098817357218, -0.105007504776, -0.007677922097],
       20: [-0.114471010174, -0.122828644043, -0.009010104319],
       21: [-0.131315903846, -0.141109218834, -0.010365511651],
       22: [-0.149653857875, -0.160924694447, -0.011835466602],
    }

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
            print(f"{fusion_run_count:>4} {sf.bias_post_s[0]:>20.12f} {sf.bias_post_s[1]:>20.12f} {sf.bias_post_s[2]:>20.12f}")

            # Compare with C if available
            if fusion_run_count in c_bias:
                c = c_bias[fusion_run_count]
                diff = [sf.bias_post_s[i] - c[i] for i in range(3)]
                print(f"  C: [{c[0]:>18.12f}, {c[1]:>18.12f}, {c[2]:>18.12f}]")
                print(f"  Δ: [{diff[0]:>+18.12e}, {diff[1]:>+18.12e}, {diff[2]:>+18.12e}]")

            fusion_run_count += 1

            if fusion_run_count > max_runs:
                break

    print(f"\nProcessed {fusion_run_count} fusion runs")

if __name__ == '__main__':
    main()
