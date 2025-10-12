"""
Synchronized C/Python Debugging Framework

This tool runs the same test data through both C and Python implementations,
stopping at the first error and providing detailed state comparison.

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
import json
import subprocess
import sys
from pathlib import Path
from sensor_fusion_6axis import SensorFusion6Axis, SensorID
from sensor_fusion_9axis import SensorFusion9Axis

class DebugLogger:
    """Captures detailed internal state at each step"""

    def __init__(self, filename):
        self.filename = filename
        self.log_entries = []
        self.sample_count = 0

    def log_state(self, label, sf, sensor_data=None):
        """Log complete internal state"""

        # Get algo_output safely
        output = sf.algo_output if hasattr(sf, 'algo_output') else None

        entry = {
            'sample': self.sample_count,
            'label': label,
            'timestamp': {
                'nom_updt_ts': int(sf.nom_updt_ts) if hasattr(sf, 'nom_updt_ts') else 0,
                'meas_updt_ts': int(sf.meas_updt_ts) if hasattr(sf, 'meas_updt_ts') else 0,
            },
            'quaternion': {
                'q0': float(sf.quat_post.q0) if sf.quat_post else 0.0,
                'q1': float(sf.quat_post.q1) if sf.quat_post else 0.0,
                'q2': float(sf.quat_post.q2) if sf.quat_post else 0.0,
                'q3': float(sf.quat_post.q3) if sf.quat_post else 0.0,
            },
            'rotation_matrix': sf.rot_mtx_post.flatten().tolist() if sf.rot_mtx_post is not None else None,
            'orientation': {
                'yaw': float(output.orientation[0]) if output else 0.0,
                'pitch': float(output.orientation[1]) if output else 0.0,
                'roll': float(output.orientation[2]) if output else 0.0,
            },
            'gyro_bias': {
                'x': float(sf.bias_post_s[0]) if sf.bias_post_s is not None else 0.0,
                'y': float(sf.bias_post_s[1]) if sf.bias_post_s is not None else 0.0,
                'z': float(sf.bias_post_s[2]) if sf.bias_post_s is not None else 0.0,
            },
            'linear_accel': {
                'x': float(sf.acc_post_s[0]) if hasattr(sf, 'acc_post_s') and sf.acc_post_s is not None else 0.0,
                'y': float(sf.acc_post_s[1]) if hasattr(sf, 'acc_post_s') and sf.acc_post_s is not None else 0.0,
                'z': float(sf.acc_post_s[2]) if hasattr(sf, 'acc_post_s') and sf.acc_post_s is not None else 0.0,
            },
            'gravity': {
                'x': float(output.gravity[0]) if output else 0.0,
                'y': float(output.gravity[1]) if output else 0.0,
                'z': float(output.gravity[2]) if output else 0.0,
            },
            'orientation_init': bool(sf.orient_init) if hasattr(sf, 'orient_init') else False,
            'operation_mode': int(sf.operation_mode) if hasattr(sf, 'operation_mode') else 0,
        }

        # Add sensor data if provided
        if sensor_data:
            entry['sensor_data'] = sensor_data

        # Add 9-axis specific state
        if isinstance(sf, SensorFusion9Axis):
            entry['mag_field_ref'] = {
                'x': float(sf.mag_field_ref_s[0]),
                'y': float(sf.mag_field_ref_s[1]),
                'z': float(sf.mag_field_ref_s[2]),
            }
            entry['mag_dist'] = {
                'x': float(sf.mag_dist_err_post_s[0]),
                'y': float(sf.mag_dist_err_post_s[1]),
                'z': float(sf.mag_dist_err_post_s[2]),
            }

        self.log_entries.append(entry)

    def increment_sample(self):
        """Increment sample counter"""
        self.sample_count += 1

    def save(self):
        """Save log to JSON file"""
        with open(self.filename, 'w') as f:
            json.dump(self.log_entries, f, indent=2)
        print(f"Debug log saved to: {self.filename}")

    def print_state(self, entry):
        """Print state in human-readable format"""
        print(f"\n{'='*70}")
        print(f"Sample #{entry['sample']} - {entry['label']}")
        print(f"{'='*70}")

        if entry.get('sensor_data'):
            print(f"Sensor Data:")
            sd = entry['sensor_data']
            if 'acc' in sd:
                print(f"  ACC: [{sd['acc']['x']:8.3f}, {sd['acc']['y']:8.3f}, {sd['acc']['z']:8.3f}] @ ts={sd['acc']['timestamp']}")
            if 'gyro' in sd:
                print(f"  GYRO: [{sd['gyro']['x']:8.3f}, {sd['gyro']['y']:8.3f}, {sd['gyro']['z']:8.3f}] @ ts={sd['gyro']['timestamp']}")
            if 'mag' in sd:
                print(f"  MAG: [{sd['mag']['x']:8.3f}, {sd['mag']['y']:8.3f}, {sd['mag']['z']:8.3f}] @ ts={sd['mag']['timestamp']}")

        print(f"\nQuaternion:")
        q = entry['quaternion']
        print(f"  [q0={q['q0']:8.6f}, q1={q['q1']:8.6f}, q2={q['q2']:8.6f}, q3={q['q3']:8.6f}]")

        print(f"\nOrientation (deg):")
        o = entry['orientation']
        print(f"  Yaw={o['yaw']:8.3f}° Pitch={o['pitch']:8.3f}° Roll={o['roll']:8.3f}°")

        print(f"\nGyro Bias (rad/s):")
        b = entry['gyro_bias']
        print(f"  [{b['x']:10.6f}, {b['y']:10.6f}, {b['z']:10.6f}]")

        print(f"\nLinear Accel (m/s²):")
        a = entry['linear_accel']
        print(f"  [{a['x']:8.3f}, {a['y']:8.3f}, {a['z']:8.3f}]")

        print(f"\nGravity (m/s²):")
        g = entry['gravity']
        print(f"  [{g['x']:8.3f}, {g['y']:8.3f}, {g['z']:8.3f}]")

        print(f"\nStatus:")
        print(f"  Orient Init: {entry['orientation_init']}")
        print(f"  Op Mode: 0x{entry['operation_mode']:04X}")


def load_c_test_data(test_name):
    """Load test data from C test output"""
    # For now, return None - we'll export from C test
    # In production, this would load from exported CSV/JSON
    return None


def run_python_with_debug(test_data, fusion_type='6axis', stop_at_sample=None, verbose=True):
    """
    Run Python implementation with detailed debug logging

    Args:
        test_data: Dictionary with 'acc', 'gyro', 'mag' arrays and timestamps
        fusion_type: '6axis' or '9axis'
        stop_at_sample: Stop at this sample number (None = run all)
        verbose: Print detailed state at each step
    """

    # Initialize sensor fusion
    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0
    MAG_SCALE = 0.15

    if fusion_type == '6axis':
        sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)
        log_file = 'debug_python_6axis.json'
    else:
        sf = SensorFusion9Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE, mag_scale=MAG_SCALE)
        log_file = 'debug_python_9axis.json'

    logger = DebugLogger(log_file)

    print(f"\n{'='*70}")
    print(f"PYTHON {fusion_type.upper()} SENSOR FUSION - DEBUG MODE")
    print(f"{'='*70}")
    print(f"Samples to process: {len(test_data['acc'])}")
    if stop_at_sample:
        print(f"Will stop at sample: {stop_at_sample}")
    print(f"Debug log: {log_file}")
    print(f"{'='*70}\n")

    # Process each sample
    num_samples = len(test_data['acc'])
    for i in range(num_samples):

        # Prepare sensor data
        sensor_data = {
            'acc': {
                'x': float(test_data['acc'][i, 0]),
                'y': float(test_data['acc'][i, 1]),
                'z': float(test_data['acc'][i, 2]),
                'timestamp': int(test_data['acc_ts'][i])
            },
            'gyro': {
                'x': float(test_data['gyro'][i, 0]),
                'y': float(test_data['gyro'][i, 1]),
                'z': float(test_data['gyro'][i, 2]),
                'timestamp': int(test_data['gyro_ts'][i])
            }
        }

        if fusion_type == '9axis':
            sensor_data['mag'] = {
                'x': float(test_data['mag'][i, 0]),
                'y': float(test_data['mag'][i, 1]),
                'z': float(test_data['mag'][i, 2]),
                'timestamp': int(test_data['mag_ts'][i])
            }

        # Log state before update
        logger.log_state('BEFORE_UPDATE', sf, sensor_data)

        # Feed sensor data
        sf.preprocess_sensor_data(SensorID.ACC,
                                   np.array([test_data['acc'][i, 0],
                                            test_data['acc'][i, 1],
                                            test_data['acc'][i, 2]]),
                                   test_data['acc_ts'][i])

        sf.preprocess_sensor_data(SensorID.GYRO,
                                   np.array([test_data['gyro'][i, 0],
                                            test_data['gyro'][i, 1],
                                            test_data['gyro'][i, 2]]),
                                   test_data['gyro_ts'][i])

        if fusion_type == '9axis':
            sf.preprocess_sensor_data(SensorID.MAG,
                                       np.array([test_data['mag'][i, 0],
                                                test_data['mag'][i, 1],
                                                test_data['mag'][i, 2]]),
                                       test_data['mag_ts'][i])

        # Run fusion
        output = sf.run()

        # Log state after update
        logger.log_state('AFTER_UPDATE', sf)
        logger.increment_sample()

        # Print verbose output if requested
        if verbose:
            logger.print_state(logger.log_entries[-1])

        # Stop at specified sample if requested
        if stop_at_sample is not None and i >= stop_at_sample:
            print(f"\n{'='*70}")
            print(f"STOPPED at sample {i} as requested")
            print(f"{'='*70}")
            break

    # Save complete log
    logger.save()

    return sf, logger


def compare_c_python_logs(c_log_file, python_log_file, tolerance=1e-3):
    """
    Compare C and Python debug logs to find divergence point

    Args:
        c_log_file: JSON file with C debug output
        python_log_file: JSON file with Python debug output
        tolerance: Numerical tolerance for comparisons
    """

    print(f"\n{'='*70}")
    print(f"COMPARING C vs PYTHON LOGS")
    print(f"{'='*70}")

    # Load logs
    with open(c_log_file) as f:
        c_log = json.load(f)
    with open(python_log_file) as f:
        py_log = json.load(f)

    print(f"C samples: {len(c_log)}")
    print(f"Python samples: {len(py_log)}")

    # Compare sample by sample
    divergence_found = False
    for i in range(min(len(c_log), len(py_log))):
        c_entry = c_log[i]
        py_entry = py_log[i]

        # Compare quaternion
        q_diff = max(
            abs(c_entry['quaternion']['q0'] - py_entry['quaternion']['q0']),
            abs(c_entry['quaternion']['q1'] - py_entry['quaternion']['q1']),
            abs(c_entry['quaternion']['q2'] - py_entry['quaternion']['q2']),
            abs(c_entry['quaternion']['q3'] - py_entry['quaternion']['q3'])
        )

        if q_diff > tolerance:
            print(f"\n🚨 DIVERGENCE DETECTED at sample {i}")
            print(f"{'='*70}")
            print(f"\nC Quaternion:")
            print(f"  [q0={c_entry['quaternion']['q0']:8.6f}, q1={c_entry['quaternion']['q1']:8.6f}, "
                  f"q2={c_entry['quaternion']['q2']:8.6f}, q3={c_entry['quaternion']['q3']:8.6f}]")
            print(f"\nPython Quaternion:")
            print(f"  [q0={py_entry['quaternion']['q0']:8.6f}, q1={py_entry['quaternion']['q1']:8.6f}, "
                  f"q2={py_entry['quaternion']['q2']:8.6f}, q3={py_entry['quaternion']['q3']:8.6f}]")
            print(f"\nMax difference: {q_diff}")
            divergence_found = True
            break

    if not divergence_found:
        print(f"\n✅ No divergence detected (tolerance={tolerance})")

    return divergence_found


def generate_simple_test_data(num_samples=100, fusion_type='6axis'):
    """Generate simple test data for debugging"""

    # Level device, no rotation
    acc_data = np.zeros((num_samples, 3))
    acc_data[:, 2] = -16384  # 1g down in Z

    gyro_data = np.zeros((num_samples, 3))

    mag_data = np.zeros((num_samples, 3))
    mag_data[:, 0] = 30  # North component
    mag_data[:, 2] = 40  # Down component

    # Timestamps in nanoseconds
    dt_ns = 10_000_000  # 10ms = 100Hz
    timestamps = np.arange(num_samples) * dt_ns

    test_data = {
        'acc': acc_data,
        'gyro': gyro_data,
        'acc_ts': timestamps,
        'gyro_ts': timestamps,
    }

    if fusion_type == '9axis':
        test_data['mag'] = mag_data
        test_data['mag_ts'] = timestamps

    return test_data


if __name__ == '__main__':

    print("""
    ╔════════════════════════════════════════════════════════════════════╗
    ║         C/Python Synchronized Debugging Framework                  ║
    ║                                                                    ║
    ║  This tool helps debug C implementation by comparing against       ║
    ║  known-good Python implementation with identical test data.        ║
    ╚════════════════════════════════════════════════════════════════════╝
    """)

    # Example: Run Python with simple test data
    print("\nExample: Running Python 6-axis with simple test data...")
    test_data = generate_simple_test_data(num_samples=10, fusion_type='6axis')
    sf, logger = run_python_with_debug(test_data, fusion_type='6axis', verbose=True)

    print(f"\n\nDebug log saved. To use this framework:")
    print(f"  1. Export same test data from C test to CSV/JSON")
    print(f"  2. Run C test with verbose debug logging")
    print(f"  3. Run this script with that test data")
    print(f"  4. Compare logs to find first divergence point")
    print(f"  5. Examine internal states at divergence to root cause")
