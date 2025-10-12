"""
Synchronized debugging: Compare Python and C outputs sample-by-sample
Run Python fusion with detailed logging to compare against C reference
"""

import numpy as np
import re
from sensor_fusion_6axis import SensorFusion6Axis, SensorID, SF_OVERSAMPLE_RATIO

def parse_testdata_header(header_file):
    """Parse the C header file to extract sensor inputs"""
    with open(header_file, 'r') as f:
        content = f.read()

    # Parse sensor input data: {id, x, y, z, timestamp}
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
    print(f"Loaded {len(sensor_data)} sensor samples\n")

    # Initialize sensor fusion
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0
    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    print("="*80)
    print("SYNCHRONIZED DEBUGGING - Python 6-axis Sensor Fusion")
    print("="*80)
    print("Running first 50 samples with detailed state tracking...\n")

    sample_count = 0
    MAX_SAMPLES = 50

    for i, sample in enumerate(sensor_data):
        if sample_count >= MAX_SAMPLES:
            break

        sensor_id = sample['id']
        sensor_counts = np.array([sample['x'], sample['y'], sample['z']], dtype=np.float64)
        timestamp = sample['ts']

        # Skip magnetometer for 6-axis
        if sensor_id == 2:
            continue

        py_sensor_id = SensorID.ACC if sensor_id == 0 else SensorID.GYRO

        print(f"\n{'='*80}")
        print(f"Sample {sample_count} (input {i}): Sensor={'ACC' if sensor_id==0 else 'GYRO'}, ts={timestamp}")
        print(f"Input counts: [{sensor_counts[0]:.0f}, {sensor_counts[1]:.0f}, {sensor_counts[2]:.0f}]")

        # Show buffer state BEFORE preprocess
        if sensor_id == 0:
            print(f"  Acc buffer count BEFORE: {sf.acc_count}/{SF_OVERSAMPLE_RATIO}")
            print(f"  Acc timestamp: {sf.acc_data.timestamp}")
        else:
            print(f"  Gyro buffer count BEFORE: {sf.gyro_count}/{SF_OVERSAMPLE_RATIO}")
            print(f"  Gyro timestamp: {sf.gyro_data.timestamp}")

        # Preprocess
        ready = sf.preprocess_sensor_data(py_sensor_id, sensor_counts, timestamp)

        # Show buffer state AFTER preprocess
        if sensor_id == 0:
            print(f"  Acc buffer count AFTER: {sf.acc_count}/{SF_OVERSAMPLE_RATIO}")
            if sf.acc_count == 0:
                print(f"  ✓ Acc buffer FULL - average computed")
                print(f"    Acc avg: [{sf.acc_data.count_avg[0]:.4f}, {sf.acc_data.count_avg[1]:.4f}, {sf.acc_data.count_avg[2]:.4f}] m/s²")
        else:
            print(f"  Gyro buffer count AFTER: {sf.gyro_count}/{SF_OVERSAMPLE_RATIO}")
            if sf.gyro_count == 0:
                print(f"  ✓ Gyro buffer FULL")

        print(f"  Ready flags: {ready} (0x{ready:02x})")
        print(f"  Last time_update ts: {sf.nom_updt_ts}")
        print(f"  Last meas_update ts: {sf.meas_updt_ts}")

        # Run fusion (always, matching C)
        output = sf.run()

        # Show what updates were performed
        time_update_ran = (sf.nom_updt_ts == sf.gyro_data.timestamp)
        meas_update_ran = (sf.meas_updt_ts == sf.acc_data.timestamp)

        print(f"\n  After run():")
        print(f"    Orient initialized: {sf.orient_init}")
        if time_update_ran:
            print(f"    ✓ TIME_UPDATE executed (gyro ts={sf.gyro_data.timestamp})")
        if meas_update_ran:
            print(f"    ✓ MEAS_UPDATE executed (acc ts={sf.acc_data.timestamp})")
        if not time_update_ran and not meas_update_ran:
            print(f"    (No updates - outputting current state)")

        print(f"\n  State:")
        print(f"    Quat: [{output.quat.q0:.6f}, {output.quat.q1:.6f}, {output.quat.q2:.6f}, {output.quat.q3:.6f}]")
        print(f"    Angles: Roll={output.orientation[2]:.2f}°, Pitch={output.orientation[1]:.2f}°, Yaw={output.orientation[0]:.2f}°")
        print(f"    Gyro bias: [{sf.bias_post_s[0]:.6f}, {sf.bias_post_s[1]:.6f}, {sf.bias_post_s[2]:.6f}]")

        sample_count += 1

    print("\n" + "="*80)
    print(f"Processed {sample_count} samples")
    print("="*80)

if __name__ == '__main__':
    main()
