#!/usr/bin/env python3
"""
Integration Test for Sensor Fusion Implementation

Tests the complete workflow of the sensor fusion system:
1. Platform configuration
2. Sensor data preprocessing
3. 6-axis fusion (accel + gyro)
4. 9-axis fusion (accel + gyro + mag)
5. Quaternion output validation

Author: Production integration test
Date: 2025-10-14
"""

import numpy as np
import sys
import os

# Add pycode to path
sys.path.insert(0, os.path.dirname(__file__))

from sensor_fusion_6axis import SensorFusion6Axis
from sensor_fusion_9axis import SensorFusion9Axis
from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE


def test_platform_configuration():
    """Test platform configuration loading"""
    print("Testing platform configuration...")

    # Test Invensense platform
    inv = SensorPlatform(INVENSENSE)
    assert inv.accel_scale_factor > 0, "Invensense accel scale invalid"
    assert inv.gyro_scale_factor > 0, "Invensense gyro scale invalid"
    assert inv.mag_scale_factor > 0, "Invensense mag scale invalid"

    # Test Freescale platform
    free = SensorPlatform(FREESCALE)
    assert free.accel_scale_factor > 0, "Freescale accel scale invalid"
    assert free.gyro_scale_factor > 0, "Freescale gyro scale invalid"
    assert free.mag_scale_factor > 0, "Freescale mag scale invalid"

    print("  ✓ Platform configuration valid")


def test_6axis_initialization():
    """Test 6-axis fusion initialization"""
    print("Testing 6-axis fusion initialization...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Check initial state
    assert sf.quat_post is not None, "Quaternion not initialized"
    quat_arr = sf.quat_post.to_array()
    assert len(quat_arr) == 4, "Quaternion wrong size"

    # Check initial quaternion is normalized
    quat_norm = np.linalg.norm(quat_arr)
    assert abs(quat_norm - 1.0) < 0.01, f"Initial quaternion not normalized: {quat_norm}"

    print("  ✓ 6-axis fusion initialized correctly")


def test_9axis_initialization():
    """Test 9-axis fusion initialization"""
    print("Testing 9-axis fusion initialization...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Check initial state
    assert sf.quat_post is not None, "Quaternion not initialized"
    quat_arr = sf.quat_post.to_array()
    assert len(quat_arr) == 4, "Quaternion wrong size"

    # Check bias limiting parameters are set
    assert sf.motion_threshold_slow == 12.0, "Motion threshold slow incorrect"
    assert sf.motion_threshold_fast == 45.0, "Motion threshold fast incorrect"
    assert sf.max_bias_rate_moderate == 0.07, "Bias rate moderate incorrect"
    assert sf.max_bias_rate_fast == 0.015, "Bias rate fast incorrect"

    print("  ✓ 9-axis fusion initialized correctly")


def test_6axis_static_convergence():
    """Test 6-axis fusion on static data"""
    print("Testing 6-axis fusion static convergence...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion6Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor
    )

    # Simulate static sensor data (1g downward, no rotation)
    # Assume sensor Z-axis points down in body frame
    accel_1g = 1.0 / platform.accel_scale_factor  # counts for 1g
    gyro_zero = 0.0

    timestamp = 0
    dt_ns = 10_000_000  # 10ms = 100Hz

    # Run 100 iterations
    for i in range(100):
        timestamp += dt_ns

        # Static: accel = [0, 0, -1g], gyro = [0, 0, 0]
        accel_counts = np.array([0.0, 0.0, accel_1g])
        gyro_counts = np.array([0.0, 0.0, 0.0])

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

    # After 100 iterations, quaternion should still be normalized
    quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
    quat_norm = np.linalg.norm(quat)
    assert abs(quat_norm - 1.0) < 0.01, f"Quaternion not normalized after static: {quat_norm}"

    # Bias should converge near zero
    bias_magnitude = np.linalg.norm(sf.bias_post_s)
    assert bias_magnitude < 5.0, f"Bias too large on static: {bias_magnitude} dps"

    print(f"  ✓ Static convergence successful (bias: {bias_magnitude:.3f} dps)")


def test_9axis_static_convergence():
    """Test 9-axis fusion on static data with magnetometer"""
    print("Testing 9-axis fusion static convergence...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Simulate static sensor data
    accel_1g = 1.0 / platform.accel_scale_factor
    mag_50uT = 50.0 / platform.mag_scale_factor  # 50µT north

    timestamp = 0
    dt_ns = 10_000_000  # 10ms

    # Run 200 iterations (need more for mag convergence)
    for i in range(200):
        timestamp += dt_ns

        # Static: accel down, gyro zero, mag north
        accel_counts = np.array([0.0, 0.0, accel_1g])
        gyro_counts = np.array([0.0, 0.0, 0.0])
        mag_counts = np.array([mag_50uT, 0.0, 0.0])

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

    # Check quaternion normalization
    quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
    quat_norm = np.linalg.norm(quat)
    assert abs(quat_norm - 1.0) < 0.01, f"Quaternion not normalized: {quat_norm}"

    # Check bias convergence
    bias_magnitude = np.linalg.norm(sf.bias_post_s)
    assert bias_magnitude < 5.0, f"Bias too large: {bias_magnitude} dps"

    print(f"  ✓ 9-axis static convergence successful (bias: {bias_magnitude:.3f} dps)")


def test_bias_rate_limiting():
    """Test motion-adaptive bias rate limiting"""
    print("Testing motion-adaptive bias rate limiting...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Test _apply_bias_rate_limit method exists and works correctly
    # NOTE: omega is in dps (degrees per second), not radians

    # Test case 1: Slow motion (<12 dps) - no limiting
    sf.omega = np.array([5.0, 0.0, 0.0])  # 5 dps (slow)
    bias_correction = np.array([10.0, 0.0, 0.0])  # Large correction
    limited = sf._apply_bias_rate_limit(bias_correction)
    assert np.allclose(limited, bias_correction), "Slow motion should not limit bias"

    # Test case 2: Moderate motion (12-45 dps) - moderate limiting
    sf.omega = np.array([20.0, 0.0, 0.0])  # 20 dps (moderate)
    bias_correction = np.array([10.0, 0.0, 0.0])  # Large correction
    limited = sf._apply_bias_rate_limit(bias_correction)
    assert np.allclose(limited, [0.07, 0.0, 0.0]), f"Moderate motion should limit to ±0.07"

    # Test case 3: Fast motion (>45 dps) - strong limiting
    sf.omega = np.array([50.0, 0.0, 0.0])  # 50 dps (fast)
    bias_correction = np.array([10.0, 0.0, 0.0])  # Large correction
    limited = sf._apply_bias_rate_limit(bias_correction)
    assert np.allclose(limited, [0.015, 0.0, 0.0]), f"Fast motion should limit to ±0.015"

    print("  ✓ Bias rate limiting thresholds correct")


def test_quaternion_output_range():
    """Test that quaternion outputs stay in valid range"""
    print("Testing quaternion output ranges...")

    platform = SensorPlatform(INVENSENSE)
    sf = SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor
    )

    # Simulate rotation
    accel_1g = 1.0 / platform.accel_scale_factor
    gyro_20dps = 20.0 / platform.gyro_scale_factor
    mag_50uT = 50.0 / platform.mag_scale_factor

    timestamp = 0
    dt_ns = 10_000_000

    for i in range(100):
        timestamp += dt_ns

        # Rotating around X-axis at 20 dps
        accel_counts = np.array([0.0, 0.0, accel_1g])
        gyro_counts = np.array([gyro_20dps, 0.0, 0.0])
        mag_counts = np.array([mag_50uT, 0.0, 0.0])

        ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
        ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
        ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

        if ready_gyro & 0x2:
            output = sf.run()

            # Check each quaternion component is in valid range
            assert -1.0 <= output.quat.q0 <= 1.0, f"q0 out of range: {output.quat.q0}"
            assert -1.0 <= output.quat.q1 <= 1.0, f"q1 out of range: {output.quat.q1}"
            assert -1.0 <= output.quat.q2 <= 1.0, f"q2 out of range: {output.quat.q2}"
            assert -1.0 <= output.quat.q3 <= 1.0, f"q3 out of range: {output.quat.q3}"

            # Check normalization
            quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])
            quat_norm = np.linalg.norm(quat)
            assert abs(quat_norm - 1.0) < 0.01, f"Quaternion not normalized: {quat_norm}"

    print("  ✓ Quaternion outputs remain valid during rotation")


def main():
    """Run all integration tests"""
    print("=" * 70)
    print("SENSOR FUSION INTEGRATION TESTS")
    print("=" * 70)
    print()

    tests = [
        test_platform_configuration,
        test_6axis_initialization,
        test_9axis_initialization,
        test_6axis_static_convergence,
        test_9axis_static_convergence,
        test_bias_rate_limiting,
        test_quaternion_output_range,
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"  ✗ FAILED: {e}")
            failed += 1
        except Exception as e:
            print(f"  ✗ ERROR: {e}")
            failed += 1
        print()

    print("=" * 70)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 70)

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
