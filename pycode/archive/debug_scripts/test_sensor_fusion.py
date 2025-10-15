"""
Test Sensor Fusion Python Implementation
Uses C test data for validation

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from sensor_fusion_6axis import SensorFusion6Axis, SensorID as SID6
from sensor_fusion_9axis import SensorFusion9Axis
from QuatMath.QuatNormal import quat_normalize
from QuatMath.QuatProduct import quat_product
from QuatMath.Quat2RodMat import quat_to_rotation_matrix

# ============================================================================
# Test 1: Quaternion Math Functions
# ============================================================================

def test_quatmath():
    """Test quaternion math functions"""
    print("\n" + "="*70)
    print("TEST 1: QUATERNION MATH FUNCTIONS")
    print("="*70)

    # Test 1.1: Quaternion Normalization
    print("\n--- Test 1.1: Quaternion Normalization ---")
    q_test = np.array([1.0, 2.0, 3.0, 4.0])
    q_norm = quat_normalize(q_test)
    mag = np.linalg.norm(q_norm)
    print(f"Input:  {q_test}")
    print(f"Output: {q_norm}")
    print(f"Magnitude: {mag:.10f}")
    assert abs(mag - 1.0) < 1e-10, "Normalization failed"
    print("✓ PASS: Quaternion normalized to unit length")

    # Test 1.2: Quaternion Product (Identity)
    print("\n--- Test 1.2: Quaternion Product (Identity) ---")
    q_identity = np.array([1.0, 0.0, 0.0, 0.0])
    q_arbitrary = np.array([0.707, 0.707, 0.0, 0.0])
    q_product = quat_product(q_identity, q_arbitrary)
    diff = np.linalg.norm(q_product - q_arbitrary)
    print(f"q_identity * q_arbitrary = {q_product}")
    print(f"Expected: {q_arbitrary}")
    print(f"Difference: {diff:.10f}")
    assert diff < 1e-10, "Identity product failed"
    print("✓ PASS: Identity quaternion product works")

    # Test 1.3: Quaternion Product (90° rotation)
    print("\n--- Test 1.3: Quaternion Product (90° Z-rotation) ---")
    q_90z = np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)])  # 90° about Z
    q_result = quat_product(q_90z, q_90z)  # 90° + 90° = 180°
    q_expected = np.array([0.0, 0.0, 0.0, 1.0])  # 180° about Z
    diff = np.linalg.norm(q_result - q_expected)
    print(f"90° + 90° = {q_result}")
    print(f"Expected (180°): {q_expected}")
    print(f"Difference: {diff:.10f}")
    assert diff < 1e-6, "90° rotation product failed"
    print("✓ PASS: Quaternion rotation composition works")

    # Test 1.4: Quaternion to Rotation Matrix
    print("\n--- Test 1.4: Quaternion to Rotation Matrix ---")
    q_test = np.array([1.0, 0.0, 0.0, 0.0])  # Identity
    R = quat_to_rotation_matrix(q_test)
    R_expected = np.eye(3)
    diff = np.linalg.norm(R - R_expected)
    print(f"Quaternion: {q_test}")
    print(f"Rotation matrix:\n{R}")
    print(f"Difference from identity: {diff:.10f}")
    assert diff < 1e-10, "Identity rotation matrix failed"
    print("✓ PASS: Quaternion to rotation matrix works")

    print("\n✅ ALL QUATERNION MATH TESTS PASSED")
    return True


# ============================================================================
# Test 2: 6-Axis Sensor Fusion (Simple Static Test)
# ============================================================================

def test_6axis_static():
    """Test 6-axis fusion with static orientation"""
    print("\n" + "="*70)
    print("TEST 2: 6-AXIS SENSOR FUSION (STATIC)")
    print("="*70)

    # MPU9250 sensor specifications
    ACC_SCALE = 1.0 / 16384.0  # ±2g range
    GYRO_SCALE = 1.0 / 131.0   # ±250 dps range

    # Initialize 6-axis fusion
    sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Simulate 1 second of static data (100 Hz)
    num_samples = 100
    sample_rate = 100  # Hz

    # Storage for results
    quaternions = np.zeros((num_samples, 4))
    euler_angles = np.zeros((num_samples, 3))
    gravity_vectors = np.zeros((num_samples, 3))

    print(f"\nProcessing {num_samples} samples at {sample_rate} Hz (static, level)")
    print("Expected: Quaternion ≈ [1, 0, 0, 0], Angles ≈ [0, 0, 0]°")

    for i in range(num_samples):
        timestamp = i * (1000000000 // sample_rate)  # nanoseconds

        # Static accelerometer: 1g in -Z (device is level)
        acc_data = np.array([0, 0, -16384], dtype=np.int16)

        # Static gyroscope: no rotation
        gyro_data = np.array([0, 0, 0], dtype=np.int16)

        # Preprocess
        sf6.preprocess_sensor_data(SID6.ACC, acc_data, timestamp)
        sf6.preprocess_sensor_data(SID6.GYRO, gyro_data, timestamp)

        # Run fusion
        output = sf6.run()

        # Store results
        quaternions[i] = [output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3]
        euler_angles[i] = output.orientation  # [yaw, pitch, roll]
        gravity_vectors[i] = output.gravity

    # Check final values
    final_quat = quaternions[-1]
    final_euler = euler_angles[-1]
    final_gravity = gravity_vectors[-1]

    print(f"\nFinal quaternion: [{final_quat[0]:.6f}, {final_quat[1]:.6f}, "
          f"{final_quat[2]:.6f}, {final_quat[3]:.6f}]")
    print(f"Final Euler [yaw, pitch, roll]: [{final_euler[0]:.3f}°, {final_euler[1]:.3f}°, "
          f"{final_euler[2]:.3f}°]")
    print(f"Final gravity: [{final_gravity[0]:.3f}, {final_gravity[1]:.3f}, "
          f"{final_gravity[2]:.3f}] m/s²")

    # Validation checks
    quat_identity_diff = np.linalg.norm(final_quat - np.array([1, 0, 0, 0]))
    euler_zero_diff = np.linalg.norm(final_euler)
    gravity_expected = np.array([0.0, 0.0, -9.80665])
    gravity_diff = np.linalg.norm(final_gravity - gravity_expected)

    print(f"\nValidation:")
    print(f"  Quaternion deviation from identity: {quat_identity_diff:.6f}")
    print(f"  Euler angle deviation from zero: {euler_zero_diff:.3f}°")
    print(f"  Gravity deviation from [0, 0, -9.81]: {gravity_diff:.3f} m/s²")

    # Plot results
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    time = np.arange(num_samples) / sample_rate

    # Quaternions
    axes[0].plot(time, quaternions[:, 0], 'r-', label='q0 (w)', linewidth=2)
    axes[0].plot(time, quaternions[:, 1], 'g-', label='q1 (x)', linewidth=2)
    axes[0].plot(time, quaternions[:, 2], 'b-', label='q2 (y)', linewidth=2)
    axes[0].plot(time, quaternions[:, 3], 'm-', label='q3 (z)', linewidth=2)
    axes[0].set_ylabel('Quaternion')
    axes[0].set_title('6-Axis Fusion: Static Test')
    axes[0].legend(loc='right')
    axes[0].grid(True, alpha=0.3)

    # Euler angles
    axes[1].plot(time, euler_angles[:, 0], 'r-', label='Yaw', linewidth=2)
    axes[1].plot(time, euler_angles[:, 1], 'g-', label='Pitch', linewidth=2)
    axes[1].plot(time, euler_angles[:, 2], 'b-', label='Roll', linewidth=2)
    axes[1].set_ylabel('Angle (degrees)')
    axes[1].legend(loc='right')
    axes[1].grid(True, alpha=0.3)

    # Gravity
    axes[2].plot(time, gravity_vectors[:, 0], 'r-', label='X', linewidth=2)
    axes[2].plot(time, gravity_vectors[:, 1], 'g-', label='Y', linewidth=2)
    axes[2].plot(time, gravity_vectors[:, 2], 'b-', label='Z', linewidth=2)
    axes[2].set_ylabel('Gravity (m/s²)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='right')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('plt/test_6axis_static.png', dpi=150, bbox_inches='tight')
    print(f"\n✓ Plot saved: plt/test_6axis_static.png")
    plt.close()

    # Pass/fail
    if quat_identity_diff < 0.1 and euler_zero_diff < 5.0 and gravity_diff < 1.0:
        print("\n✅ TEST PASSED: 6-axis static fusion")
        return True
    else:
        print("\n❌ TEST FAILED: Deviations too large")
        return False


# ============================================================================
# Test 3: 6-Axis with Rotation
# ============================================================================

def test_6axis_rotation():
    """Test 6-axis fusion with constant rotation"""
    print("\n" + "="*70)
    print("TEST 3: 6-AXIS SENSOR FUSION (ROTATION)")
    print("="*70)

    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0

    sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Simulate 2 seconds of rotation about Z-axis at 45 deg/s
    num_samples = 200
    sample_rate = 100  # Hz
    rotation_rate = 45.0  # deg/s about Z

    quaternions = np.zeros((num_samples, 4))
    euler_angles = np.zeros((num_samples, 3))

    print(f"\nSimulating {rotation_rate} deg/s rotation about Z-axis for 2 seconds")
    print(f"Expected: Yaw should increase from 0° to ~90°")

    for i in range(num_samples):
        timestamp = i * (1000000000 // sample_rate)

        # Level accelerometer
        acc_data = np.array([0, 0, -16384], dtype=np.int16)

        # Constant rotation about Z
        gyro_data = np.array([0, 0, int(rotation_rate * 131)], dtype=np.int16)

        sf6.preprocess_sensor_data(SID6.ACC, acc_data, timestamp)
        sf6.preprocess_sensor_data(SID6.GYRO, gyro_data, timestamp)

        output = sf6.run()

        quaternions[i] = [output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3]
        euler_angles[i] = output.orientation

    # Check final yaw
    final_yaw = euler_angles[-1, 0]
    expected_yaw = rotation_rate * (num_samples / sample_rate)
    yaw_error = abs(final_yaw - expected_yaw)

    print(f"\nFinal yaw: {final_yaw:.2f}°")
    print(f"Expected yaw: {expected_yaw:.2f}°")
    print(f"Error: {yaw_error:.2f}°")

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    time = np.arange(num_samples) / sample_rate

    # Quaternions
    axes[0].plot(time, quaternions[:, 0], 'r-', label='q0 (w)', linewidth=2)
    axes[0].plot(time, quaternions[:, 1], 'g-', label='q1 (x)', linewidth=2)
    axes[0].plot(time, quaternions[:, 2], 'b-', label='q2 (y)', linewidth=2)
    axes[0].plot(time, quaternions[:, 3], 'm-', label='q3 (z)', linewidth=2)
    axes[0].set_ylabel('Quaternion')
    axes[0].set_title(f'6-Axis Fusion: Rotation Test ({rotation_rate} deg/s about Z)')
    axes[0].legend(loc='right')
    axes[0].grid(True, alpha=0.3)

    # Euler angles
    axes[1].plot(time, euler_angles[:, 0], 'r-', label='Yaw', linewidth=2)
    axes[1].plot(time, euler_angles[:, 1], 'g-', label='Pitch', linewidth=2)
    axes[1].plot(time, euler_angles[:, 2], 'b-', label='Roll', linewidth=2)
    axes[1].plot(time, expected_yaw * (time / time[-1]), 'r--', label='Expected Yaw',
                 linewidth=1, alpha=0.7)
    axes[1].set_ylabel('Angle (degrees)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(loc='right')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('plt/test_6axis_rotation.png', dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved: plt/test_6axis_rotation.png")
    plt.close()

    # Pass/fail (allow 10% error)
    if yaw_error < expected_yaw * 0.1:
        print(f"\n✅ TEST PASSED: 6-axis rotation (error < 10%)")
        return True
    else:
        print(f"\n❌ TEST FAILED: Yaw error too large ({yaw_error:.2f}° > {expected_yaw*0.1:.2f}°)")
        return False


# ============================================================================
# Test 4: 9-Axis Sensor Fusion
# ============================================================================

def test_9axis_static():
    """Test 9-axis fusion with magnetometer"""
    print("\n" + "="*70)
    print("TEST 4: 9-AXIS SENSOR FUSION (STATIC WITH MAG)")
    print("="*70)

    ACC_SCALE = 1.0 / 16384.0
    GYRO_SCALE = 1.0 / 131.0
    MAG_SCALE = 0.15

    sf9 = SensorFusion9Axis(ACC_SCALE, GYRO_SCALE, MAG_SCALE)

    num_samples = 200
    sample_rate = 100

    quaternions = np.zeros((num_samples, 4))
    euler_angles = np.zeros((num_samples, 3))

    print(f"\nProcessing {num_samples} samples with magnetometer")
    print("Expected: Stable heading from magnetometer")

    # Simulate Earth's field: 50 µT pointing North at 45° inclination
    mag_field_ut = 50.0
    mag_north = mag_field_ut * np.cos(np.radians(45))
    mag_down = -mag_field_ut * np.sin(np.radians(45))

    for i in range(num_samples):
        timestamp = i * (1000000000 // sample_rate)

        acc_data = np.array([0, 0, -16384], dtype=np.int16)
        gyro_data = np.array([0, 0, 0], dtype=np.int16)
        mag_data = np.array([
            int(mag_north / MAG_SCALE),
            0,
            int(mag_down / MAG_SCALE)
        ], dtype=np.int16)

        sf9.preprocess_sensor_data(SID6.ACC, acc_data, timestamp)
        sf9.preprocess_sensor_data(SID6.GYRO, gyro_data, timestamp)
        sf9.preprocess_sensor_data(SID6.MAG, mag_data, timestamp)

        output = sf9.run()

        quaternions[i] = [output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3]
        euler_angles[i] = output.orientation

    final_quat = quaternions[-1]
    final_euler = euler_angles[-1]

    print(f"\nFinal quaternion: [{final_quat[0]:.6f}, {final_quat[1]:.6f}, "
          f"{final_quat[2]:.6f}, {final_quat[3]:.6f}]")
    print(f"Final Euler [yaw, pitch, roll]: [{final_euler[0]:.3f}°, {final_euler[1]:.3f}°, "
          f"{final_euler[2]:.3f}°]")

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    time = np.arange(num_samples) / sample_rate

    axes[0].plot(time, quaternions[:, 0], 'r-', label='q0 (w)', linewidth=2)
    axes[0].plot(time, quaternions[:, 1], 'g-', label='q1 (x)', linewidth=2)
    axes[0].plot(time, quaternions[:, 2], 'b-', label='q2 (y)', linewidth=2)
    axes[0].plot(time, quaternions[:, 3], 'm-', label='q3 (z)', linewidth=2)
    axes[0].set_ylabel('Quaternion')
    axes[0].set_title('9-Axis Fusion: Static Test with Magnetometer')
    axes[0].legend(loc='right')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time, euler_angles[:, 0], 'r-', label='Yaw', linewidth=2)
    axes[1].plot(time, euler_angles[:, 1], 'g-', label='Pitch', linewidth=2)
    axes[1].plot(time, euler_angles[:, 2], 'b-', label='Roll', linewidth=2)
    axes[1].set_ylabel('Angle (degrees)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(loc='right')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('plt/test_9axis_static.png', dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved: plt/test_9axis_static.png")
    plt.close()

    # Basic sanity checks
    euler_zero_diff = np.linalg.norm(final_euler[1:3])  # pitch and roll should be near zero
    if euler_zero_diff < 10.0:
        print(f"\n✅ TEST PASSED: 9-axis static fusion (pitch/roll < 10°)")
        return True
    else:
        print(f"\n❌ TEST FAILED: Pitch/roll error too large")
        return False


# ============================================================================
# Main Test Runner
# ============================================================================

def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("SENSOR FUSION PYTHON IMPLEMENTATION - TEST SUITE")
    print("="*70)
    print(f"\nOutput directory: plt/")
    print(f"All plots will be saved with 150 DPI")

    # Create output directory
    os.makedirs('plt', exist_ok=True)

    results = []

    # Run tests
    try:
        results.append(("Quaternion Math", test_quatmath()))
    except Exception as e:
        print(f"\n❌ ERROR in Quaternion Math: {e}")
        import traceback
        traceback.print_exc()
        results.append(("Quaternion Math", False))

    try:
        results.append(("6-Axis Static", test_6axis_static()))
    except Exception as e:
        print(f"\n❌ ERROR in 6-Axis Static: {e}")
        import traceback
        traceback.print_exc()
        results.append(("6-Axis Static", False))

    try:
        results.append(("6-Axis Rotation", test_6axis_rotation()))
    except Exception as e:
        print(f"\n❌ ERROR in 6-Axis Rotation: {e}")
        import traceback
        traceback.print_exc()
        results.append(("6-Axis Rotation", False))

    try:
        results.append(("9-Axis Static", test_9axis_static()))
    except Exception as e:
        print(f"\n❌ ERROR in 9-Axis Static: {e}")
        import traceback
        traceback.print_exc()
        results.append(("9-Axis Static", False))

    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {test_name}")

    print(f"\nTotal: {passed}/{total} tests passed ({100*passed//total}%)")

    if passed == total:
        print("\n🎉 ALL TESTS PASSED!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
