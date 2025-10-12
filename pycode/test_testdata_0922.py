"""
Test Python sensor fusion using real testdata_0922 dataset
Compare outputs with expected outputs from C implementation
"""

import numpy as np
import re
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

def parse_testdata_header(header_file):
    """Parse the C header file to extract sensor inputs and expected outputs"""
    with open(header_file, 'r') as f:
        content = f.read()

    # Parse sensor input data
    # Format: {id, x, y, z, timestamp}
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

    # Parse expected output data
    # Format: {q0, q1, q2, q3, roll, pitch, yaw, timestamp}
    output_pattern = r'\{(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(-?\d+\.\d+)f,\s*(\d+)ULL\}'
    output_matches = re.findall(output_pattern, content)

    expected_data = []
    for match in output_matches:
        expected_data.append({
            'q0': float(match[0]),
            'q1': float(match[1]),
            'q2': float(match[2]),
            'q3': float(match[3]),
            'roll': float(match[4]),
            'pitch': float(match[5]),
            'yaw': float(match[6]),
            'ts': int(match[7])
        })

    return sensor_data, expected_data

def main():
    header_file = '../test/data/testdata/fusion/test_input_output_0922.h'

    print("Parsing testdata header file...")
    sensor_data, expected_data = parse_testdata_header(header_file)

    print(f"Loaded {len(sensor_data)} sensor samples")
    print(f"Loaded {len(expected_data)} expected outputs")

    # Initialize sensor fusion with same parameters as C code
    # MPU9250: ±2g accel (1/16384 counts/g), ±250dps gyro (1/131 counts/dps)
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    print("\n" + "="*80)
    print("Running Python sensor fusion on testdata_0922...")
    print("="*80)

    output_idx = 0
    sample_count = 0

    max_quat_err = 0.0
    max_angle_err = 0.0
    quat_errors = []
    angle_errors = []

    for i, sample in enumerate(sensor_data):
        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        # Skip magnetometer data (id=2) for 6-axis fusion
        if sensor_id == 2:
            continue

        # Map sensor ID: 0=ACC, 1=GYRO
        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        # Preprocess sensor data (buffers data, matches C sf_6xag_data_preproc)
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Run sensor fusion on EVERY sample (matches C behavior)
        # C calls sf_6xag_algo_run() on every sample, which internally checks
        # timestamps to decide whether to run time_update or measurement_update
        output = sf.run()
        sample_count += 1

        # Check if there's an expected output for this timestamp (C test approach)
        if output_idx < len(expected_data) and expected_data[output_idx]['ts'] == timestamp:
            expected = expected_data[output_idx]

            # Only compare if we've run fusion at least once
            if sample_count > 0:
                # Calculate quaternion error (normalized Euclidean distance)
                quat_diff = np.array([
                    output.quat.q0 - expected['q0'],
                    output.quat.q1 - expected['q1'],
                    output.quat.q2 - expected['q2'],
                    output.quat.q3 - expected['q3']
                ])
                quat_err = np.linalg.norm(quat_diff)
                quat_errors.append(quat_err)

                # Calculate angle error (only roll and pitch, yaw=0 for 6-axis)
                pred_angles = output.orientation
                angle_err = max(
                    abs(pred_angles[2] - expected['roll']),   # roll error
                    abs(pred_angles[1] - expected['pitch'])   # pitch error
                )
                angle_errors.append(angle_err)

                max_quat_err = max(max_quat_err, quat_err)
                max_angle_err = max(max_angle_err, angle_err)

                # Print detailed comparison for first few samples and any large errors
                if output_idx < 10 or quat_err > 0.01 or angle_err > 5.0:
                    print(f"\nSample {output_idx} (input sample {i}, fusion run {sample_count}, ts={timestamp}):")
                    print(f"  Expected quat: [{expected['q0']:.6f}, {expected['q1']:.6f}, {expected['q2']:.6f}, {expected['q3']:.6f}]")
                    print(f"  Python quat:   [{output.quat.q0:.6f}, {output.quat.q1:.6f}, {output.quat.q2:.6f}, {output.quat.q3:.6f}]")
                    print(f"  Quat error:    {quat_err:.8f}")
                    print(f"  Expected angles: Roll={expected['roll']:.2f}°, Pitch={expected['pitch']:.2f}°")
                    print(f"  Python angles:   Roll={pred_angles[2]:.2f}°, Pitch={pred_angles[1]:.2f}°")
                    print(f"  Angle error:     {angle_err:.2f}°")

                    if quat_err > 0.01:
                        print(f"  ⚠️  Large quaternion error!")
                    if angle_err > 5.0:
                        print(f"  ⚠️  Large angle error!")

            output_idx += 1

    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Total fusion runs: {sample_count}")
    print(f"Expected outputs: {len(expected_data)}")
    print(f"Outputs compared: {output_idx}")
    print(f"\nQuaternion Error Statistics:")
    print(f"  Max error: {max_quat_err:.8f}")
    print(f"  Mean error: {np.mean(quat_errors):.8f}")
    print(f"  Median error: {np.median(quat_errors):.8f}")
    print(f"\nAngle Error Statistics:")
    print(f"  Max error: {max_angle_err:.2f}°")
    print(f"  Mean error: {np.mean(angle_errors):.2f}°")
    print(f"  Median error: {np.median(angle_errors):.2f}°")

    # Count samples with significant errors
    large_quat_errors = sum(1 for e in quat_errors if e > 0.01)
    large_angle_errors = sum(1 for e in angle_errors if e > 5.0)

    print(f"\nSamples with large errors:")
    print(f"  Quat error > 0.01: {large_quat_errors}/{len(quat_errors)} ({100*large_quat_errors/len(quat_errors):.1f}%)")
    print(f"  Angle error > 5°: {large_angle_errors}/{len(angle_errors)} ({100*large_angle_errors/len(angle_errors):.1f}%)")

    # Determine pass/fail
    QUAT_THRESHOLD = 0.001  # Max acceptable quaternion error
    ANGLE_THRESHOLD = 1.0   # Max acceptable angle error (degrees)

    if max_quat_err < QUAT_THRESHOLD and max_angle_err < ANGLE_THRESHOLD:
        print(f"\n✅ TEST PASSED - Python matches expected outputs within tolerance")
    else:
        print(f"\n❌ TEST FAILED - Python outputs exceed error thresholds")
        print(f"   Quat threshold: {QUAT_THRESHOLD}, actual: {max_quat_err:.8f}")
        print(f"   Angle threshold: {ANGLE_THRESHOLD}°, actual: {max_angle_err:.2f}°")

if __name__ == '__main__':
    main()
