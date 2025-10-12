"""
Export C test data to format usable by Python debug framework

This script processes C test output and extracts sensor data
to run through Python implementation.

Author: Vikas Yadav
Date: 2025-10-12
"""

import re
import json
import numpy as np
import argparse


def parse_c_test_output(output_file):
    """
    Parse C test output to extract sensor data

    Expected format from C test with printf statements:
    SENSOR_DATA,sample,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,timestamp
    """

    test_data = {
        'acc': [],
        'gyro': [],
        'mag': [],
        'acc_ts': [],
        'gyro_ts': [],
        'mag_ts': []
    }

    with open(output_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('SENSOR_DATA,'):
                parts = line.split(',')
                # Format: SENSOR_DATA,sample,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,ts
                if len(parts) >= 11:
                    sample = int(parts[1])
                    acc = [float(parts[2]), float(parts[3]), float(parts[4])]
                    gyro = [float(parts[5]), float(parts[6]), float(parts[7])]
                    mag = [float(parts[8]), float(parts[9]), float(parts[10])]
                    ts = int(parts[11]) if len(parts) > 11 else sample * 10_000_000

                    test_data['acc'].append(acc)
                    test_data['gyro'].append(gyro)
                    test_data['mag'].append(mag)
                    test_data['acc_ts'].append(ts)
                    test_data['gyro_ts'].append(ts)
                    test_data['mag_ts'].append(ts)

    # Convert to numpy arrays
    for key in ['acc', 'gyro', 'mag']:
        test_data[key] = np.array(test_data[key])
    for key in ['acc_ts', 'gyro_ts', 'mag_ts']:
        test_data[key] = np.array(test_data[key])

    return test_data


def save_test_data(test_data, output_file):
    """Save test data to JSON format for Python"""

    # Convert numpy arrays to lists for JSON serialization
    json_data = {}
    for key, value in test_data.items():
        if isinstance(value, np.ndarray):
            json_data[key] = value.tolist()
        else:
            json_data[key] = value

    with open(output_file, 'w') as f:
        json.dump(json_data, f, indent=2)

    print(f"Test data saved to: {output_file}")
    print(f"  Samples: {len(json_data['acc'])}")


def main():
    parser = argparse.ArgumentParser(description='Export C test data for Python debugging')
    parser.add_argument('input_file', help='C test output file')
    parser.add_argument('output_file', help='Output JSON file')

    args = parser.parse_args()

    print(f"Parsing C test output: {args.input_file}")
    test_data = parse_c_test_output(args.input_file)

    print(f"Saving to: {args.output_file}")
    save_test_data(test_data, args.output_file)


if __name__ == '__main__':
    main()
