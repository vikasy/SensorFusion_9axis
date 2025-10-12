"""
Example usage of 6-axis and 9-axis sensor fusion Python implementations

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
from sensor_fusion_6axis import SensorFusion6Axis, SensorID
from sensor_fusion_9axis import SensorFusion9Axis

# ============================================================================
# Example 1: 6-Axis Sensor Fusion (Accelerometer + Gyroscope)
# ============================================================================

def example_6axis():
    """Example usage of 6-axis sensor fusion"""
    print("=" * 70)
    print("6-AXIS SENSOR FUSION EXAMPLE")
    print("=" * 70)

    # Sensor specifications (MPU9250)
    ACC_SCALE = 1.0 / 16384.0  # g/count (±2g range)
    GYRO_SCALE = 1.0 / 131.0   # dps/count (±250 dps range)

    # Initialize 6-axis fusion
    sf6 = SensorFusion6Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE)

    # Simulate sensor data
    sample_rate = 100  # Hz
    duration = 1.0  # seconds
    num_samples = int(sample_rate * duration)

    print(f"\nProcessing {num_samples} samples at {sample_rate} Hz...")

    for i in range(num_samples):
        timestamp = i * (1000000000 // sample_rate)  # nanoseconds

        # Simulated accelerometer data (raw counts)
        # Assume device is stationary and level (gravity in -Z)
        acc_data = np.array([0, 0, -16384])  # 1g in -Z direction

        # Simulated gyroscope data (raw counts)
        # Slowly rotating about Z-axis at 10 deg/s
        gyro_data = np.array([0, 0, 10 * 131])  # 10 dps in Z

        # Preprocess sensor data
        sf6.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
        sf6.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)

        # Run fusion algorithm
        output = sf6.run()

        # Print output every 25 samples
        if i % 25 == 0:
            print(f"\nSample {i}:")
            print(f"  Quaternion: [{output.quat.q0:.4f}, {output.quat.q1:.4f}, "
                  f"{output.quat.q2:.4f}, {output.quat.q3:.4f}]")
            print(f"  Orientation [yaw, pitch, roll]: "
                  f"[{output.orientation[0]:.2f}°, {output.orientation[1]:.2f}°, "
                  f"{output.orientation[2]:.2f}°]")
            print(f"  Gravity: [{output.gravity[0]:.3f}, {output.gravity[1]:.3f}, "
                  f"{output.gravity[2]:.3f}] m/s²")

    print("\n✓ 6-axis fusion completed successfully")


# ============================================================================
# Example 2: 9-Axis Sensor Fusion (Accelerometer + Gyroscope + Magnetometer)
# ============================================================================

def example_9axis():
    """Example usage of 9-axis sensor fusion"""
    print("\n" + "=" * 70)
    print("9-AXIS SENSOR FUSION EXAMPLE")
    print("=" * 70)

    # Sensor specifications (MPU9250 + AK8963)
    ACC_SCALE = 1.0 / 16384.0    # g/count (±2g range)
    GYRO_SCALE = 1.0 / 131.0     # dps/count (±250 dps range)
    MAG_SCALE = 0.15             # µT/count (4912 µT range / 32760)

    # Initialize 9-axis fusion
    sf9 = SensorFusion9Axis(acc_scale=ACC_SCALE, gyro_scale=GYRO_SCALE, mag_scale=MAG_SCALE)

    # Simulate sensor data
    sample_rate = 100  # Hz
    duration = 1.0  # seconds
    num_samples = int(sample_rate * duration)

    print(f"\nProcessing {num_samples} samples at {sample_rate} Hz...")

    for i in range(num_samples):
        timestamp = i * (1000000000 // sample_rate)  # nanoseconds

        # Simulated accelerometer data (raw counts)
        acc_data = np.array([0, 0, -16384])  # 1g in -Z

        # Simulated gyroscope data (raw counts)
        # Slowly rotating about Z-axis at 10 deg/s
        gyro_data = np.array([0, 0, 10 * 131])

        # Simulated magnetometer data (raw counts)
        # Earth's field ~50 µT pointing North-East at 45° inclination
        # Assuming sensor aligned with NED frame
        mag_field_ut = 50.0  # µT
        mag_north = mag_field_ut * np.cos(np.radians(45))
        mag_down = -mag_field_ut * np.sin(np.radians(45))
        mag_data = np.array([
            int(mag_north / MAG_SCALE),
            0,
            int(mag_down / MAG_SCALE)
        ])

        # Preprocess sensor data
        sf9.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
        sf9.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)
        sf9.preprocess_sensor_data(SensorID.MAG, mag_data, timestamp)

        # Run fusion algorithm
        output = sf9.run()

        # Print output every 25 samples
        if i % 25 == 0:
            print(f"\nSample {i}:")
            print(f"  Quaternion: [{output.quat.q0:.4f}, {output.quat.q1:.4f}, "
                  f"{output.quat.q2:.4f}, {output.quat.q3:.4f}]")
            print(f"  Orientation [yaw, pitch, roll]: "
                  f"[{output.orientation[0]:.2f}°, {output.orientation[1]:.2f}°, "
                  f"{output.orientation[2]:.2f}°]")
            print(f"  Gravity: [{output.gravity[0]:.3f}, {output.gravity[1]:.3f}, "
                  f"{output.gravity[2]:.3f}] m/s²")

    print("\n✓ 9-axis fusion completed successfully")


# ============================================================================
# Example 3: Load and Process Real IMU Data
# ============================================================================

def example_load_csv():
    """Example of loading and processing CSV data"""
    print("\n" + "=" * 70)
    print("LOAD CSV DATA EXAMPLE")
    print("=" * 70)

    try:
        import pandas as pd

        # Example CSV format:
        # timestamp_ns, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z

        print("\nCSV Format:")
        print("  Column 0: timestamp (nanoseconds)")
        print("  Columns 1-3: accelerometer [x, y, z] (raw counts)")
        print("  Columns 4-6: gyroscope [x, y, z] (raw counts)")
        print("  Columns 7-9: magnetometer [x, y, z] (raw counts)")

        # Initialize fusion
        ACC_SCALE = 1.0 / 16384.0
        GYRO_SCALE = 1.0 / 131.0
        MAG_SCALE = 0.15

        sf9 = SensorFusion9Axis(ACC_SCALE, GYRO_SCALE, MAG_SCALE)

        # Example code to process CSV (uncomment when you have data)
        """
        data = pd.read_csv('sensor_data.csv')

        for idx, row in data.iterrows():
            timestamp = int(row['timestamp_ns'])
            acc_data = np.array([row['accel_x'], row['accel_y'], row['accel_z']])
            gyro_data = np.array([row['gyro_x'], row['gyro_y'], row['gyro_z']])
            mag_data = np.array([row['mag_x'], row['mag_y'], row['mag_z']])

            sf9.preprocess_sensor_data(SensorID.ACC, acc_data, timestamp)
            sf9.preprocess_sensor_data(SensorID.GYRO, gyro_data, timestamp)
            sf9.preprocess_sensor_data(SensorID.MAG, mag_data, timestamp)

            output = sf9.run()

            # Store or process output
            print(f"Sample {idx}: Yaw={output.orientation[0]:.2f}°")
        """

        print("\nNote: Uncomment the CSV processing code when you have real data")

    except ImportError:
        print("\nNote: pandas not installed. Install with: pip install pandas")


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("SENSOR FUSION PYTHON IMPLEMENTATION - EXAMPLES")
    print("=" * 70)

    # Run examples
    example_6axis()
    example_9axis()
    example_load_csv()

    print("\n" + "=" * 70)
    print("ALL EXAMPLES COMPLETED")
    print("=" * 70)
    print("\nNext steps:")
    print("  1. Test with real sensor data from test/data/datasets/")
    print("  2. Compare outputs with C implementation")
    print("  3. Validate quaternion and angle outputs")
    print("=" * 70 + "\n")
