"""
Compare C and Python outputs at sample 165 where C showed debug output
This is where we can do synchronized debugging
"""

import numpy as np
import re
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

def parse_testdata_header(header_file):
    """Parse the C header file to extract sensor inputs"""
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

    print("Parsing testdata...")
    sensor_data = parse_testdata_header(header_file)

    # Initialize sensor fusion
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0
    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    print("\n" + "="*80)
    print("SYNCHRONIZED COMPARISON - C vs Python at Sample 165")
    print("="*80)
    print("\nProcessing samples up to and including sample 165...")

    sample_count = 0
    TARGET_SAMPLE = 165

    for i, sample in enumerate(sensor_data):
        if sample_count > TARGET_SAMPLE:
            break

        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        # Skip magnetometer for 6-axis
        if sensor_id == 2:
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess
        sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Run fusion
        output = sf.run()

        if sample_count == TARGET_SAMPLE:
            print(f"\n{'='*80}")
            print(f"SAMPLE {sample_count} DETAILED COMPARISON")
            print(f"{'='*80}")
            print(f"\nInput: Sensor={'ACC' if sensor_id==0 else 'GYRO'}, ts={timestamp}")
            print(f"Counts: [{sensor_counts[0]:.0f}, {sensor_counts[1]:.0f}, {sensor_counts[2]:.0f}]")

            print(f"\n{'-'*80}")
            print("C OUTPUT (from test run):")
            print(f"{'-'*80}")
            print("TIME_UPDATE:")
            print("  Input gyro counts: [-56, -56, 6]")
            print("  Gyro scale: 0.030518509447575")
            print("  BiasPostS: [-18.77290130, -18.49451064, -1.36091420]")
            print("  BiasErrPostS: [0.21125868, 0.20466442, 0.01496008]")
            print("  Quat before: [0.99370558, -0.09473436, -0.05874836, 0.01110158]")
            print("  Omega[0]: raw=-1.70903659, corrected=-1.92029527 (after bias)")
            print("  Omega[1]: raw=-1.70903659, corrected=-1.91370101")
            print("  Omega[2]: raw=0.18311106, corrected=0.16815098")
            print("  Quat after time_update: [0.99360174, -0.09539644, -0.05941402, 0.01118380]")
            print("\nMEASUREMENT_UPDATE:")
            print("  Input acc counts: [-55, -528, 8002]")
            print("  Acc scale: 0.000122074037790")
            print("  BiasPostS (updated): [-18.98485601, -18.69980238, -1.37592057]")
            print("  Quat after meas_update: [0.99367330, -0.09493480, -0.05895157, 0.01120156]")

            print(f"\n{'-'*80}")
            print("PYTHON OUTPUT:")
            print(f"{'-'*80}")
            print(f"  Quat: [{output.quat.q0:.8f}, {output.quat.q1:.8f}, {output.quat.q2:.8f}, {output.quat.q3:.8f}]")
            print(f"  Angles: Roll={output.orientation[2]:.4f}°, Pitch={output.orientation[1]:.4f}°, Yaw={output.orientation[0]:.4f}°")
            print(f"  Gyro bias: [{sf.bias_post_s[0]:.8f}, {sf.bias_post_s[1]:.8f}, {sf.bias_post_s[2]:.8f}]")
            print(f"  Gyro bias error: [{sf.bias_err_post_s[0]:.8f}, {sf.bias_err_post_s[1]:.8f}, {sf.bias_err_post_s[2]:.8f}]")
            print(f"  Acc timestamp: {sf.acc_data.timestamp}")
            print(f"  Gyro timestamp: {sf.gyro_data.timestamp}")
            print(f"  Last time_update ts: {sf.nom_updt_ts}")
            print(f"  Last meas_update ts: {sf.meas_updt_ts}")

            print(f"\n{'-'*80}")
            print("DIFFERENCES:")
            print(f"{'-'*80}")
            c_quat = np.array([0.99367330, -0.09493480, -0.05895157, 0.01120156])
            py_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_diff = py_quat - c_quat

            print(f"  Quaternion difference:")
            print(f"    Δq0 = {quat_diff[0]:.8f}")
            print(f"    Δq1 = {quat_diff[1]:.8f}")
            print(f"    Δq2 = {quat_diff[2]:.8f}")
            print(f"    Δq3 = {quat_diff[3]:.8f}")
            print(f"    ||Δquat|| = {np.linalg.norm(quat_diff):.8f}")

            c_bias = np.array([-18.98485601, -18.69980238, -1.37592057])
            py_bias = sf.bias_post_s
            bias_diff = py_bias - c_bias

            print(f"\n  Gyro bias difference:")
            print(f"    Δbias[0] = {bias_diff[0]:.8f}")
            print(f"    Δbias[1] = {bias_diff[1]:.8f}")
            print(f"    Δbias[2] = {bias_diff[2]:.8f}")
            print(f"    ||Δbias|| = {np.linalg.norm(bias_diff):.8f}")

            print(f"\n{'='*80}")

        sample_count += 1

if __name__ == '__main__':
    main()
