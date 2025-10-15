"""
Test to determine correct coordinate frame convention for tilt initialization
"""

import numpy as np
import sys
from sensor_fusion_6axis import SensorFusion6Axis, SensorID

MPU9250_COUNTSPERG = 8192
MPU9250_SAMPLE_PERIOD_NS = 10_000_000

def test_level_device():
    """
    Test level device (0° roll, 0° pitch)
    According to C test formula:
    accel_z = g * cos(0) * cos(0) = g (POSITIVE)
    So counts should be [0, 0, 8192] (positive Z)

    Expected output:
    - Quaternion: [1, 0, 0, 0] (identity)
    - Orientation: [0°, 0°, 0°] (yaw, pitch, roll)
    """

    ACC_SCALE = 1.0 / MPU9250_COUNTSPERG
    GYRO_SCALE = 1.0 / 131.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Level device: positive Z acceleration (gravity pointing up in sensor frame)
    acc_counts = np.array([0, 0, 8192], dtype=np.float64)
    gyro_counts = np.array([0, 0, 0], dtype=np.float64)

    # Run for 200 samples
    for i in range(200):
        timestamp = i * MPU9250_SAMPLE_PERIOD_NS
        sf.preprocess_sensor_data(SensorID.ACC, acc_counts, timestamp)
        sf.preprocess_sensor_data(SensorID.GYRO, gyro_counts, timestamp)
        output = sf.run()

    # Get final output
    quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
    angles = output.orientation

    print("\n" + "="*80)
    print("TEST: Level Device (0° roll, 0° pitch)")
    print("="*80)
    print(f"Input counts: [0, 0, 8192]")
    print(f"Input accel (m/s²): [0.000, 0.000, {8192/MPU9250_COUNTSPERG * 9.80665:.3f}]")
    print(f"\nQuaternion: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
    print(f"Orientation [y,p,r]: [{angles[0]:.2f}°, {angles[1]:.2f}°, {angles[2]:.2f}°]")

    # Check if correct
    expected_quat = np.array([1.0, 0.0, 0.0, 0.0])
    quat_error = np.linalg.norm(quat - expected_quat)

    # Also check the negative (double cover)
    quat_error_neg = np.linalg.norm(quat + expected_quat)
    quat_error = min(quat_error, quat_error_neg)

    angle_errors = np.abs(angles)

    print(f"\nQuaternion error: {quat_error:.6f}")
    print(f"Angle errors: Roll={angle_errors[2]:.2f}°, Pitch={angle_errors[1]:.2f}°, Yaw={angle_errors[0]:.2f}°")

    if quat_error < 0.01 and all(angle_errors < 5.0):
        print("\n✓ PASS - Correct coordinate frame convention")
        return True
    else:
        print("\n✗ FAIL - Wrong coordinate frame convention")
        return False


def test_30deg_roll():
    """
    Test 30° roll
    According to C test formula:
    accel_x = g * sin(30°) * cos(0°) = 0.5g
    accel_y = 0
    accel_z = g * cos(30°) * cos(0°) = 0.866g

    Counts: [4096, 0, 7094]
    """

    ACC_SCALE = 1.0 / MPU9250_COUNTSPERG
    GYRO_SCALE = 1.0 / 131.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    roll_rad = np.deg2rad(30.0)
    pitch_rad = 0.0
    g = 9.80665

    accel_x = g * np.sin(roll_rad) * np.cos(pitch_rad)
    accel_y = 0.0
    accel_z = g * np.cos(roll_rad) * np.cos(pitch_rad)

    ax = int((accel_x / g) * MPU9250_COUNTSPERG)
    ay = int((accel_y / g) * MPU9250_COUNTSPERG)
    az = int((accel_z / g) * MPU9250_COUNTSPERG)

    acc_counts = np.array([ax, ay, az], dtype=np.float64)
    gyro_counts = np.array([0, 0, 0], dtype=np.float64)

    # Run for 200 samples
    for i in range(200):
        timestamp = i * MPU9250_SAMPLE_PERIOD_NS
        sf.preprocess_sensor_data(SensorID.ACC, acc_counts, timestamp)
        sf.preprocess_sensor_data(SensorID.GYRO, gyro_counts, timestamp)
        output = sf.run()

    # Get final output
    quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
    angles = output.orientation

    print("\n" + "="*80)
    print("TEST: 30° Roll")
    print("="*80)
    print(f"Input counts: [{ax}, {ay}, {az}]")
    print(f"Input accel (m/s²): [{accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}]")
    print(f"\nQuaternion: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
    print(f"Orientation [y,p,r]: [{angles[0]:.2f}°, {angles[1]:.2f}°, {angles[2]:.2f}°]")

    expected_roll = 30.0
    expected_pitch = 0.0

    roll_error = abs(angles[2] - expected_roll)
    pitch_error = abs(angles[1] - expected_pitch)

    print(f"\nExpected: Roll=30.00°, Pitch=0.00°")
    print(f"Error: Roll={roll_error:.2f}°, Pitch={pitch_error:.2f}°")

    if roll_error < 5.0 and pitch_error < 5.0:
        print("\n✓ PASS")
        return True
    else:
        print("\n✗ FAIL")
        return False


if __name__ == '__main__':
    print("\n" + "="*80)
    print("COORDINATE FRAME CONVENTION TEST")
    print("Testing current implementation")
    print("="*80)

    test1 = test_level_device()
    test2 = test_30deg_roll()

    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)

    if test1 and test2:
        print("✓ ALL TESTS PASS - Coordinate frame is CORRECT")
        sys.exit(0)
    else:
        print("✗ TESTS FAIL - Need to fix coordinate frame")
        sys.exit(1)
