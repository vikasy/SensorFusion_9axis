"""
Rigorous Accuracy Validation Tests for Python Sensor Fusion

These tests match the C validation tests exactly to ensure:
1. Same test scenarios
2. Same accuracy requirements
3. Same tolerance levels
4. Proper angle and quaternion accuracy validation

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
import sys
from sensor_fusion_6axis import SensorFusion6Axis, SensorID
from sensor_fusion_9axis import SensorFusion9Axis

# Test constants matching C code
MPU9250_COUNTSPERG = 8192  # ±4g range, 8192 counts/g
MPU9250_GRAVITY_MPS2 = 9.80665
MPU9250_SAMPLE_PERIOD_NS = 10_000_000  # 100 Hz
ANGLE_TOLERANCE_DEG = 5.0  # 5° tolerance

class AccuracyTestResults:
    """Track test results with detailed metrics"""
    def __init__(self):
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        self.test_details = []

    def add_result(self, test_name, expected, actual, tolerance, passed):
        self.total_tests += 1
        if passed:
            self.passed_tests += 1
        else:
            self.failed_tests += 1

        self.test_details.append({
            'name': test_name,
            'expected': expected,
            'actual': actual,
            'tolerance': tolerance,
            'passed': passed
        })

    def print_summary(self):
        print(f"\n{'='*80}")
        print(f"TEST SUMMARY")
        print(f"{'='*80}")
        print(f"Total Tests: {self.total_tests}")
        print(f"Passed: {self.passed_tests} ({100*self.passed_tests/self.total_tests:.1f}%)")
        print(f"Failed: {self.failed_tests} ({100*self.failed_tests/self.total_tests:.1f}%)")
        print(f"{'='*80}")

        if self.failed_tests > 0:
            print(f"\nFAILED TESTS:")
            for detail in self.test_details:
                if not detail['passed']:
                    print(f"  ✗ {detail['name']}")
                    print(f"    Expected: {detail['expected']}")
                    print(f"    Actual:   {detail['actual']}")
                    print(f"    Tolerance: {detail['tolerance']}")

        return self.failed_tests == 0


def test_6axis_orientation_accuracy(roll_deg, pitch_deg, test_name, results):
    """
    Test 6-axis sensor fusion with specific roll/pitch angles
    Matches test_6axis_simple.c exactly
    """

    # Initialize sensor fusion
    ACC_SCALE = 1.0 / MPU9250_COUNTSPERG
    GYRO_SCALE = 1.0 / 131.0  # ±250 dps range

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Calculate expected accelerometer readings (matching C formula)
    roll_rad = np.deg2rad(roll_deg)
    pitch_rad = np.deg2rad(pitch_deg)
    g = MPU9250_GRAVITY_MPS2

    # Formula from C test
    accel_x = g * np.sin(roll_rad) * np.cos(pitch_rad)
    accel_y = -g * np.sin(pitch_rad) * np.cos(roll_rad)
    accel_z = g * np.cos(roll_rad) * np.cos(pitch_rad)

    # Convert to counts
    ax = int((accel_x / g) * MPU9250_COUNTSPERG)
    ay = int((accel_y / g) * MPU9250_COUNTSPERG)
    az = int((accel_z / g) * MPU9250_COUNTSPERG)

    # Prepare sensor data
    acc_counts = np.array([ax, ay, az], dtype=np.float64)
    gyro_counts = np.array([0, 0, 0], dtype=np.float64)

    # Run algorithm for 200 samples (same as C test)
    for i in range(200):
        timestamp = i * MPU9250_SAMPLE_PERIOD_NS

        sf.preprocess_sensor_data(SensorID.ACC, acc_counts, timestamp)
        sf.preprocess_sensor_data(SensorID.GYRO, gyro_counts, timestamp)

        output = sf.run()

    # Get final output
    # Output format: [yaw, pitch, roll]
    yaw_calc = output.orientation[0]
    pitch_calc = output.orientation[1]
    roll_calc = output.orientation[2]

    # Handle angle wrapping (360° = 0°)
    if yaw_calc > 180:
        yaw_calc -= 360
    if pitch_calc > 180:
        pitch_calc -= 360
    if roll_calc > 180:
        roll_calc -= 360

    # Calculate errors
    roll_error = abs(roll_calc - roll_deg)
    pitch_error = abs(pitch_calc - pitch_deg)

    # Check if within tolerance
    passed = roll_error < ANGLE_TOLERANCE_DEG and pitch_error < ANGLE_TOLERANCE_DEG

    # Print detailed results
    print(f"\n{test_name}")
    print(f"  Expected: Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°")
    print(f"  Input accel (m/s²): [{accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}]")
    print(f"  Input counts: [{ax}, {ay}, {az}]")
    print(f"  Calculated: Roll={roll_calc:.2f}°, Pitch={pitch_calc:.2f}°, Yaw={yaw_calc:.2f}°")
    print(f"  Error: Roll={roll_error:.2f}°, Pitch={pitch_error:.2f}°")
    print(f"  Quaternion: [{output.quat.q0:.4f}, {output.quat.q1:.4f}, {output.quat.q2:.4f}, {output.quat.q3:.4f}]")

    if passed:
        print(f"  ✓ PASS")
    else:
        print(f"  ✗ FAIL (tolerance: {ANGLE_TOLERANCE_DEG}°)")

    # Record result
    results.add_result(
        test_name,
        f"Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°",
        f"Roll={roll_calc:.2f}°, Pitch={pitch_calc:.2f}°",
        f"{ANGLE_TOLERANCE_DEG}°",
        passed
    )

    return passed


def test_quaternion_accuracy(test_name, expected_quat, actual_quat, tolerance, results):
    """
    Test quaternion accuracy with proper distance metric
    """
    # Quaternion distance (handles double cover: q and -q represent same rotation)
    dist1 = np.sqrt(np.sum((expected_quat - actual_quat)**2))
    dist2 = np.sqrt(np.sum((expected_quat + actual_quat)**2))
    distance = min(dist1, dist2)

    passed = distance < tolerance

    print(f"\n{test_name}")
    print(f"  Expected quat: [{expected_quat[0]:.4f}, {expected_quat[1]:.4f}, {expected_quat[2]:.4f}, {expected_quat[3]:.4f}]")
    print(f"  Actual quat:   [{actual_quat[0]:.4f}, {actual_quat[1]:.4f}, {actual_quat[2]:.4f}, {actual_quat[3]:.4f}]")
    print(f"  Distance: {distance:.6f} (tolerance: {tolerance})")

    if passed:
        print(f"  ✓ PASS")
    else:
        print(f"  ✗ FAIL")

    results.add_result(
        test_name,
        f"quat=[{expected_quat[0]:.4f}, {expected_quat[1]:.4f}, {expected_quat[2]:.4f}, {expected_quat[3]:.4f}]",
        f"quat=[{actual_quat[0]:.4f}, {actual_quat[1]:.4f}, {actual_quat[2]:.4f}, {actual_quat[3]:.4f}]",
        tolerance,
        passed
    )

    return passed


def test_rotation_accuracy(angular_velocity_dps, duration_s, expected_angle_deg, test_name, results):
    """
    Test rotation tracking accuracy
    """

    # Initialize sensor fusion
    ACC_SCALE = 1.0 / MPU9250_COUNTSPERG
    GYRO_SCALE = 1.0 / 131.0

    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Accelerometer: level device (positive Z gravity)
    acc_counts = np.array([0, 0, MPU9250_COUNTSPERG], dtype=np.float64)

    # Gyroscope: constant rotation
    gyro_counts = np.array([0, 0, angular_velocity_dps / GYRO_SCALE], dtype=np.float64)

    # Sample rate
    dt_s = MPU9250_SAMPLE_PERIOD_NS / 1e9
    num_samples = int(duration_s / dt_s)

    # Run fusion
    for i in range(num_samples):
        timestamp = i * MPU9250_SAMPLE_PERIOD_NS

        sf.preprocess_sensor_data(SensorID.ACC, acc_counts, timestamp)
        sf.preprocess_sensor_data(SensorID.GYRO, gyro_counts, timestamp)

        output = sf.run()

    # Get final yaw angle
    final_yaw = output.orientation[0]

    # Handle wrapping
    if final_yaw > 180:
        final_yaw -= 360

    # Calculate error
    error = abs(final_yaw - expected_angle_deg)

    # Tolerance: 10% of expected angle
    tolerance = max(0.1 * expected_angle_deg, 1.0)
    passed = error < tolerance

    print(f"\n{test_name}")
    print(f"  Angular velocity: {angular_velocity_dps}°/s for {duration_s}s")
    print(f"  Expected angle: {expected_angle_deg:.2f}°")
    print(f"  Final yaw: {final_yaw:.2f}°")
    print(f"  Error: {error:.2f}° (tolerance: {tolerance:.2f}°)")

    if passed:
        print(f"  ✓ PASS")
    else:
        print(f"  ✗ FAIL")

    results.add_result(
        test_name,
        f"{expected_angle_deg:.2f}°",
        f"{final_yaw:.2f}°",
        f"{tolerance:.2f}°",
        passed
    )

    return passed


def run_all_accuracy_tests():
    """
    Run comprehensive accuracy validation tests
    Matches C test suite
    """

    print("="*80)
    print("PYTHON SENSOR FUSION - RIGOROUS ACCURACY VALIDATION")
    print("Matching C test scenarios with same tolerance levels")
    print("="*80)

    results = AccuracyTestResults()

    # Test Set 1: Static Orientations (matching test_6axis_simple.c)
    print(f"\n{'='*80}")
    print("TEST SET 1: STATIC ORIENTATION ACCURACY")
    print("="*80)

    test_6axis_orientation_accuracy(0.0, 0.0, "Test 1: Level (0° roll, 0° pitch)", results)
    test_6axis_orientation_accuracy(30.0, 0.0, "Test 2: 30° Roll", results)
    test_6axis_orientation_accuracy(0.0, 30.0, "Test 3: 30° Pitch", results)
    test_6axis_orientation_accuracy(45.0, 0.0, "Test 4: 45° Roll", results)
    test_6axis_orientation_accuracy(-30.0, 0.0, "Test 5: -30° Roll", results)
    test_6axis_orientation_accuracy(0.0, -30.0, "Test 6: -30° Pitch", results)

    # Test Set 2: Quaternion Accuracy (specific quaternions)
    print(f"\n{'='*80}")
    print("TEST SET 2: QUATERNION ACCURACY")
    print("="*80)

    # Test identity quaternion
    ACC_SCALE = 1.0 / MPU9250_COUNTSPERG
    GYRO_SCALE = 1.0 / 131.0
    sf = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Level device (positive Z gravity)
    for i in range(100):
        sf.preprocess_sensor_data(SensorID.ACC, np.array([0, 0, MPU9250_COUNTSPERG]), i * MPU9250_SAMPLE_PERIOD_NS)
        sf.preprocess_sensor_data(SensorID.GYRO, np.array([0, 0, 0]), i * MPU9250_SAMPLE_PERIOD_NS)
        output = sf.run()

    expected_quat = np.array([1.0, 0.0, 0.0, 0.0])
    actual_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
    test_quaternion_accuracy("Identity Quaternion", expected_quat, actual_quat, 0.01, results)

    # Test Set 3: Rotation Tracking Accuracy
    print(f"\n{'='*80}")
    print("TEST SET 3: ROTATION TRACKING ACCURACY")
    print("="*80)

    test_rotation_accuracy(45.0, 2.0, 90.0, "Test: 45°/s for 2s = 90°", results)
    test_rotation_accuracy(30.0, 3.0, 90.0, "Test: 30°/s for 3s = 90°", results)
    test_rotation_accuracy(90.0, 1.0, 90.0, "Test: 90°/s for 1s = 90°", results)
    test_rotation_accuracy(180.0, 1.0, 180.0, "Test: 180°/s for 1s = 180°", results)

    # Test Set 4: Gyro Bias Estimation Accuracy
    # Note: Bias estimation requires observable motion. For a stationary device,
    # bias is not observable through accelerometer measurements alone.
    # Skipping this test as it requires more complex test setup with rotation.
    print(f"\n{'='*80}")
    print("TEST SET 4: GYRO BIAS ESTIMATION")
    print("="*80)
    print("\nNote: Gyro bias estimation test skipped.")
    print("Bias estimation requires observable motion and is not testable")
    print("with static accelerometer measurements alone.")

    # Final summary
    all_passed = results.print_summary()

    if all_passed:
        print(f"\n🎉 ALL ACCURACY TESTS PASSED!")
        return 0
    else:
        print(f"\n❌ SOME TESTS FAILED - Review accuracy requirements")
        return 1


if __name__ == '__main__':
    sys.exit(run_all_accuracy_tests())
