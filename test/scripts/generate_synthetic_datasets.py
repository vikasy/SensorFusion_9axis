#!/usr/bin/env python3
"""
Synthetic IMU Dataset Generator for SensorFusion Testing

Generates realistic synthetic IMU data (accelerometer, gyroscope, magnetometer)
for testing 6-axis and 9-axis sensor fusion algorithms.

Features:
- Multiple motion scenarios (static, rotation, translation, complex)
- Realistic sensor specifications (MPU9250, AK8963)
- Configurable noise and bias
- Ground truth orientation for validation
- CSV output format compatible with E2E tests

Author: Vikas Yadav / Claude Code
Date: 2025-10-12
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import pandas as pd
from pathlib import Path
import argparse
from datetime import datetime

# ============================================================================
# SENSOR SPECIFICATIONS (MPU9250 + AK8963)
# ============================================================================

class SensorSpec:
    """MPU9250 + AK8963 sensor specifications"""

    # Accelerometer (MPU9250) - MATCHING C CODE CONFIG (±4g range)
    ACCEL_RANGE_G = 4.0                    # ±4g range (from sensor_spec_agm.h line 65)
    ACCEL_RESOLUTION_BITS = 16             # 16-bit ADC
    ACCEL_COUNTS_PER_G = 32768 / ACCEL_RANGE_G  # 8192 counts/g
    ACCEL_G_PER_COUNT = ACCEL_RANGE_G / 32768   # ~0.0001220703125 g/count
    ACCEL_NOISE_DENSITY = 300e-6           # 300 µg/√Hz (from sensor_spec_agm.h line 83)
    ACCEL_BIAS_STABILITY = 0.01            # ±0.01 g (typical for MPU9250)

    # Gyroscope (MPU9250) - MATCHING C CODE CONFIG (±1000 dps range)
    GYRO_RANGE_DPS = 1000.0                # ±1000 dps range (from sensor_spec_agm.h line 99)
    GYRO_RESOLUTION_BITS = 16              # 16-bit ADC
    GYRO_COUNTS_PER_DPS = 32768 / GYRO_RANGE_DPS  # 32.768 counts/dps
    GYRO_DPS_PER_COUNT = GYRO_RANGE_DPS / 32768   # ~0.030518 dps/count
    GYRO_NOISE_DENSITY = 0.01              # 0.01 dps/√Hz (from sensor_spec_agm.h line 117)
    GYRO_BIAS_STABILITY = 0.5              # ±0.5 dps (typical for MPU9250)

    # Magnetometer (AK8963)
    MAG_RANGE_UT = 4800.0                  # ±4800 µT range
    MAG_RESOLUTION_BITS = 16               # 16-bit ADC
    MAG_COUNTS_PER_UT = 32768 / MAG_RANGE_UT      # ~6.8 counts/µT
    MAG_UT_PER_COUNT = MAG_RANGE_UT / 32768       # ~0.15 µT/count
    MAG_NOISE_RMS = 0.6                    # 0.6 µT RMS

    # Earth magnetic field (typical values)
    MAG_FIELD_STRENGTH_UT = 50.0           # ~50 µT (varies by location)
    MAG_INCLINATION_DEG = 60.0             # Magnetic inclination angle
    MAG_DECLINATION_DEG = 0.0              # Magnetic declination

    # Gravity
    GRAVITY_MPS2 = 9.81                    # m/s²

    # Sampling
    SAMPLE_RATE_HZ = 100.0                 # 100 Hz
    SAMPLE_PERIOD_S = 1.0 / SAMPLE_RATE_HZ # 0.01 s


# ============================================================================
# MOTION SCENARIOS
# ============================================================================

class MotionScenario:
    """Define various motion scenarios for testing"""

    @staticmethod
    def static(duration_s, sample_rate_hz):
        """Static scenario - no movement"""
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)

        # Identity rotation (no orientation change)
        orientations = [R.identity() for _ in range(num_samples)]

        # No angular velocity
        angular_velocities = np.zeros((num_samples, 3))

        # No linear acceleration
        linear_accelerations = np.zeros((num_samples, 3))

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def constant_rotation(duration_s, sample_rate_hz, axis='z', rate_dps=30.0):
        """Constant rotation around specified axis"""
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        # Constant angular velocity
        omega = np.zeros((num_samples, 3))
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        omega[:, axis_idx] = np.radians(rate_dps)  # Convert to rad/s

        # Integrate to get orientation
        orientations = []
        current_rot = R.identity()
        for i in range(num_samples):
            if i > 0:
                angle = np.linalg.norm(omega[i]) * dt
                if angle > 1e-9:
                    axis_vec = omega[i] / np.linalg.norm(omega[i])
                    delta_rot = R.from_rotvec(angle * axis_vec)
                    current_rot = delta_rot * current_rot
            orientations.append(current_rot)

        # No linear acceleration
        linear_accelerations = np.zeros((num_samples, 3))

        return timestamps, orientations, omega, linear_accelerations

    @staticmethod
    def smooth_rotation_sequence(duration_s, sample_rate_hz, waypoints):
        """Smooth rotation through waypoint orientations using SLERP"""
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)

        # Create waypoint rotations
        waypoint_times = np.linspace(0, duration_s, len(waypoints))
        waypoint_rots = [R.from_euler('xyz', wp, degrees=True) for wp in waypoints]

        # Interpolate using SLERP
        slerp = Slerp(waypoint_times, R.from_quat([r.as_quat() for r in waypoint_rots]))
        orientations = [R.from_quat(slerp(t).as_quat()) for t in timestamps]

        # Compute angular velocities (numerical differentiation)
        angular_velocities = np.zeros((num_samples, 3))
        dt = 1.0 / sample_rate_hz
        for i in range(1, num_samples):
            r_prev = orientations[i-1]
            r_curr = orientations[i]
            delta_rot = r_curr * r_prev.inv()
            rotvec = delta_rot.as_rotvec()
            angular_velocities[i] = rotvec / dt

        # No linear acceleration
        linear_accelerations = np.zeros((num_samples, 3))

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def vibration(duration_s, sample_rate_hz, frequency_hz=5.0, amplitude_g=0.5):
        """Vibration scenario with sinusoidal acceleration"""
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)

        # Static orientation
        orientations = [R.identity() for _ in range(num_samples)]

        # No angular velocity
        angular_velocities = np.zeros((num_samples, 3))

        # Sinusoidal linear acceleration
        linear_accelerations = np.zeros((num_samples, 3))
        linear_accelerations[:, 2] = amplitude_g * np.sin(2 * np.pi * frequency_hz * timestamps)

        return timestamps, orientations, angular_velocities, linear_accelerations


# ============================================================================
# IMU DATA GENERATOR
# ============================================================================

class IMUDataGenerator:
    """Generate synthetic IMU data with realistic noise and bias"""

    def __init__(self, sensor_spec=SensorSpec(), seed=None):
        self.spec = sensor_spec
        if seed is not None:
            np.random.seed(seed)

        # Generate random but stable sensor biases
        self.accel_bias = np.random.uniform(-self.spec.ACCEL_BIAS_STABILITY,
                                           self.spec.ACCEL_BIAS_STABILITY, 3)
        self.gyro_bias = np.random.uniform(-self.spec.GYRO_BIAS_STABILITY,
                                          self.spec.GYRO_BIAS_STABILITY, 3)

    def generate_accelerometer(self, orientations, linear_accelerations,
                              noise_scale=1.0, bias_scale=1.0):
        """Generate accelerometer data (gravity + linear acceleration)"""
        num_samples = len(orientations)
        accel_data = np.zeros((num_samples, 3))

        # Gravity vector in global frame (NED: pointing down)
        gravity_global = np.array([0, 0, self.spec.GRAVITY_MPS2])

        for i in range(num_samples):
            # Transform gravity to sensor frame
            gravity_sensor = orientations[i].inv().apply(gravity_global)

            # Add linear acceleration (already in sensor frame)
            total_accel = gravity_sensor + linear_accelerations[i] * self.spec.GRAVITY_MPS2

            # Add bias (in g) - convert to m/s² by multiplying by GRAVITY_MPS2
            total_accel += self.accel_bias * bias_scale * self.spec.GRAVITY_MPS2

            # Add noise
            noise = np.random.normal(0, self.spec.ACCEL_NOISE_DENSITY * noise_scale, 3)
            total_accel += noise

            accel_data[i] = total_accel

        return accel_data  # in m/s²

    def generate_gyroscope(self, angular_velocities, noise_scale=1.0, bias_scale=1.0):
        """Generate gyroscope data (angular velocity in sensor frame)"""
        num_samples = len(angular_velocities)
        gyro_data = np.zeros((num_samples, 3))

        for i in range(num_samples):
            # Angular velocity is already in sensor frame (rad/s)
            omega_sensor = angular_velocities[i]

            # Convert to dps
            omega_dps = np.degrees(omega_sensor)

            # Add bias
            omega_dps += self.gyro_bias * bias_scale

            # Add noise
            noise = np.random.normal(0, self.spec.GYRO_NOISE_DENSITY * noise_scale, 3)
            omega_dps += noise

            gyro_data[i] = omega_dps

        return gyro_data  # in dps

    def generate_magnetometer(self, orientations, noise_scale=1.0):
        """Generate magnetometer data (Earth's magnetic field)"""
        num_samples = len(orientations)
        mag_data = np.zeros((num_samples, 3))

        # Earth's magnetic field in NED frame
        inclination_rad = np.radians(self.spec.MAG_INCLINATION_DEG)
        declination_rad = np.radians(self.spec.MAG_DECLINATION_DEG)

        # Magnetic field components in NED
        mag_north = self.spec.MAG_FIELD_STRENGTH_UT * np.cos(inclination_rad)
        mag_down = self.spec.MAG_FIELD_STRENGTH_UT * np.sin(inclination_rad)

        # Apply declination (rotation in horizontal plane)
        mag_x = mag_north * np.cos(declination_rad)
        mag_y = mag_north * np.sin(declination_rad)
        mag_z = mag_down

        mag_global = np.array([mag_x, mag_y, mag_z])

        for i in range(num_samples):
            # Transform to sensor frame
            mag_sensor = orientations[i].inv().apply(mag_global)

            # Add noise
            noise = np.random.normal(0, self.spec.MAG_NOISE_RMS * noise_scale, 3)
            mag_sensor += noise

            mag_data[i] = mag_sensor

        return mag_data  # in µT

    def to_counts(self, accel_mps2, gyro_dps, mag_ut):
        """Convert physical units to sensor counts"""
        accel_counts = (accel_mps2 / self.spec.GRAVITY_MPS2 *
                       self.spec.ACCEL_COUNTS_PER_G).astype(np.int16)
        gyro_counts = (gyro_dps * self.spec.GYRO_COUNTS_PER_DPS).astype(np.int16)
        mag_counts = (mag_ut * self.spec.MAG_COUNTS_PER_UT).astype(np.int16)

        return accel_counts, gyro_counts, mag_counts

    def generate_dataset(self, timestamps, orientations, angular_velocities,
                        linear_accelerations, noise_scale=1.0, bias_scale=1.0):
        """Generate complete IMU dataset"""

        # Generate sensor data
        accel_mps2 = self.generate_accelerometer(orientations, linear_accelerations,
                                                 noise_scale, bias_scale)
        gyro_dps = self.generate_gyroscope(angular_velocities, noise_scale, bias_scale)
        mag_ut = self.generate_magnetometer(orientations, noise_scale)

        # Convert to counts
        accel_counts, gyro_counts, mag_counts = self.to_counts(accel_mps2, gyro_dps, mag_ut)

        # Extract ground truth orientation (Euler angles)
        euler_angles = np.array([r.as_euler('xyz', degrees=True) for r in orientations])

        # Extract ground truth quaternions
        quaternions = np.array([r.as_quat(scalar_first=True) for r in orientations])

        # Create timestamps in nanoseconds
        timestamps_ns = (timestamps * 1e9).astype(np.int64)

        return {
            'timestamps_ns': timestamps_ns,
            'timestamps_s': timestamps,
            'accel_counts': accel_counts,
            'gyro_counts': gyro_counts,
            'mag_counts': mag_counts,
            'accel_mps2': accel_mps2,
            'gyro_dps': gyro_dps,
            'mag_ut': mag_ut,
            'euler_deg': euler_angles,  # [roll, pitch, yaw]
            'quaternions': quaternions,  # [w, x, y, z]
        }


# ============================================================================
# DATASET GENERATION
# ============================================================================

def save_dataset_csv(dataset, filepath, include_ground_truth=True):
    """Save dataset to CSV file"""

    df_data = {
        'timestamp_ns': dataset['timestamps_ns'],
        'accel_x_counts': dataset['accel_counts'][:, 0],
        'accel_y_counts': dataset['accel_counts'][:, 1],
        'accel_z_counts': dataset['accel_counts'][:, 2],
        'gyro_x_counts': dataset['gyro_counts'][:, 0],
        'gyro_y_counts': dataset['gyro_counts'][:, 1],
        'gyro_z_counts': dataset['gyro_counts'][:, 2],
        'mag_x_counts': dataset['mag_counts'][:, 0],
        'mag_y_counts': dataset['mag_counts'][:, 1],
        'mag_z_counts': dataset['mag_counts'][:, 2],
    }

    if include_ground_truth:
        df_data.update({
            'gt_roll_deg': dataset['euler_deg'][:, 0],
            'gt_pitch_deg': dataset['euler_deg'][:, 1],
            'gt_yaw_deg': dataset['euler_deg'][:, 2],
            'gt_quat_w': dataset['quaternions'][:, 0],
            'gt_quat_x': dataset['quaternions'][:, 1],
            'gt_quat_y': dataset['quaternions'][:, 2],
            'gt_quat_z': dataset['quaternions'][:, 3],
        })

    df = pd.DataFrame(df_data)
    df.to_csv(filepath, index=False)

    print(f"✓ Saved {len(df)} samples to: {filepath}")


def generate_all_scenarios(output_dir):
    """Generate datasets for all test scenarios"""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    generator = IMUDataGenerator(seed=42)
    spec = SensorSpec()

    print("=" * 80)
    print("Generating Synthetic IMU Datasets")
    print("=" * 80)
    print(f"Output directory: {output_dir}")
    print(f"Sample rate: {spec.SAMPLE_RATE_HZ} Hz")
    print(f"Sensor: MPU9250 + AK8963")
    print()

    scenarios = []

    # 1. Static scenario (calibration)
    print("1. Static scenario (10 seconds)...")
    t, r, w, a = MotionScenario.static(10.0, spec.SAMPLE_RATE_HZ)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "static_10s.csv")
    scenarios.append(("static_10s", dataset))

    # 2. Constant rotation around Z-axis (yaw)
    print("\n2. Constant Z-axis rotation (30 dps, 10 seconds)...")
    t, r, w, a = MotionScenario.constant_rotation(10.0, spec.SAMPLE_RATE_HZ,
                                                   axis='z', rate_dps=30.0)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "rotation_z_30dps_10s.csv")
    scenarios.append(("rotation_z_30dps", dataset))

    # 3. Constant rotation around X-axis (roll)
    print("\n3. Constant X-axis rotation (20 dps, 10 seconds)...")
    t, r, w, a = MotionScenario.constant_rotation(10.0, spec.SAMPLE_RATE_HZ,
                                                   axis='x', rate_dps=20.0)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "rotation_x_20dps_10s.csv")
    scenarios.append(("rotation_x_20dps", dataset))

    # 4. Constant rotation around Y-axis (pitch)
    print("\n4. Constant Y-axis rotation (15 dps, 10 seconds)...")
    t, r, w, a = MotionScenario.constant_rotation(10.0, spec.SAMPLE_RATE_HZ,
                                                   axis='y', rate_dps=15.0)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "rotation_y_15dps_10s.csv")
    scenarios.append(("rotation_y_15dps", dataset))

    # 5. Smooth rotation sequence through waypoints
    print("\n5. Smooth rotation sequence (15 seconds)...")
    waypoints = [
        [0, 0, 0],      # Level
        [30, 0, 0],     # Roll 30°
        [30, 30, 0],    # Roll 30°, Pitch 30°
        [0, 30, 90],    # Pitch 30°, Yaw 90°
        [0, 0, 180],    # Yaw 180°
        [0, 0, 0],      # Back to level
    ]
    t, r, w, a = MotionScenario.smooth_rotation_sequence(15.0, spec.SAMPLE_RATE_HZ, waypoints)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "rotation_sequence_15s.csv")
    scenarios.append(("rotation_sequence", dataset))

    # 6. Vibration scenario
    print("\n6. Vibration scenario (5 Hz, 10 seconds)...")
    t, r, w, a = MotionScenario.vibration(10.0, spec.SAMPLE_RATE_HZ,
                                          frequency_hz=5.0, amplitude_g=0.5)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "vibration_5hz_10s.csv")
    scenarios.append(("vibration_5hz", dataset))

    # 7. High noise scenario
    print("\n7. High noise scenario (10 seconds)...")
    t, r, w, a = MotionScenario.static(10.0, spec.SAMPLE_RATE_HZ)
    dataset = generator.generate_dataset(t, r, w, a, noise_scale=5.0)
    save_dataset_csv(dataset, output_dir / "static_high_noise_10s.csv")
    scenarios.append(("static_high_noise", dataset))

    # 8. High bias scenario
    print("\n8. High bias scenario (10 seconds)...")
    t, r, w, a = MotionScenario.static(10.0, spec.SAMPLE_RATE_HZ)
    dataset = generator.generate_dataset(t, r, w, a, bias_scale=3.0)
    save_dataset_csv(dataset, output_dir / "static_high_bias_10s.csv")
    scenarios.append(("static_high_bias", dataset))

    # 9. Complex motion (for advanced testing)
    print("\n9. Complex motion scenario (20 seconds)...")
    waypoints = [
        [0, 0, 0],
        [45, 0, 0],
        [45, 45, 0],
        [0, 45, 90],
        [-30, 30, 90],
        [-30, -30, 180],
        [0, 0, 270],
        [15, 15, 315],
        [0, 0, 360],
    ]
    t, r, w, a = MotionScenario.smooth_rotation_sequence(20.0, spec.SAMPLE_RATE_HZ, waypoints)
    dataset = generator.generate_dataset(t, r, w, a, noise_scale=1.5)
    save_dataset_csv(dataset, output_dir / "complex_motion_20s.csv")
    scenarios.append(("complex_motion", dataset))

    # 10. Long static for drift analysis (60 seconds)
    print("\n10. Long static for drift analysis (60 seconds)...")
    t, r, w, a = MotionScenario.static(60.0, spec.SAMPLE_RATE_HZ)
    dataset = generator.generate_dataset(t, r, w, a)
    save_dataset_csv(dataset, output_dir / "static_60s.csv")
    scenarios.append(("static_60s", dataset))

    print("\n" + "=" * 80)
    print(f"✓ Generated {len(scenarios)} synthetic datasets")
    print("=" * 80)

    # Generate README
    readme_content = f"""# Synthetic IMU Datasets

**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
**Sensor:** MPU9250 (Accel + Gyro) + AK8963 (Magnetometer)
**Sample Rate:** {spec.SAMPLE_RATE_HZ} Hz

## Sensor Specifications

### Accelerometer (MPU9250)
- Range: ±{spec.ACCEL_RANGE_G} g
- Resolution: {spec.ACCEL_RESOLUTION_BITS} bits
- Scale Factor: {spec.ACCEL_COUNTS_PER_G:.2f} counts/g
- Noise Density: {spec.ACCEL_NOISE_DENSITY*1e6:.0f} µg/√Hz
- Bias Stability: ±{spec.ACCEL_BIAS_STABILITY} g

### Gyroscope (MPU9250)
- Range: ±{spec.GYRO_RANGE_DPS} dps
- Resolution: {spec.GYRO_RESOLUTION_BITS} bits
- Scale Factor: {spec.GYRO_COUNTS_PER_DPS:.2f} counts/dps
- Noise Density: {spec.GYRO_NOISE_DENSITY} dps/√Hz
- Bias Stability: ±{spec.GYRO_BIAS_STABILITY} dps

### Magnetometer (AK8963)
- Range: ±{spec.MAG_RANGE_UT} µT
- Resolution: {spec.MAG_RESOLUTION_BITS} bits
- Scale Factor: {spec.MAG_COUNTS_PER_UT:.2f} counts/µT
- Noise: {spec.MAG_NOISE_RMS} µT RMS
- Earth Field Strength: {spec.MAG_FIELD_STRENGTH_UT} µT
- Magnetic Inclination: {spec.MAG_INCLINATION_DEG}°

## Dataset Files

"""

    for name, dataset in scenarios:
        num_samples = len(dataset['timestamps_s'])
        duration = dataset['timestamps_s'][-1]
        readme_content += f"### {name}.csv\n"
        readme_content += f"- Samples: {num_samples}\n"
        readme_content += f"- Duration: {duration:.2f} seconds\n"
        readme_content += f"- Description: {name.replace('_', ' ').title()}\n\n"

    readme_content += """
## CSV Format

Each CSV file contains the following columns:

### Sensor Data (Counts)
- `timestamp_ns`: Timestamp in nanoseconds
- `accel_x_counts`, `accel_y_counts`, `accel_z_counts`: Accelerometer readings (int16)
- `gyro_x_counts`, `gyro_y_counts`, `gyro_z_counts`: Gyroscope readings (int16)
- `mag_x_counts`, `mag_y_counts`, `mag_z_counts`: Magnetometer readings (int16)

### Ground Truth (for validation)
- `gt_roll_deg`, `gt_pitch_deg`, `gt_yaw_deg`: True Euler angles in degrees
- `gt_quat_w`, `gt_quat_x`, `gt_quat_y`, `gt_quat_z`: True quaternion (scalar-first)

## Usage with Tests

### 6-Axis Tests (Accel + Gyro)
```bash
./build/bin/test_6axis_e2e test/datasets/synthetic/static_10s.csv
./build/bin/test_6axis_e2e test/datasets/synthetic/rotation_z_30dps_10s.csv
```

### 9-Axis Tests (Accel + Gyro + Mag)
```bash
./build/bin/test_9axis_e2e test/datasets/synthetic/static_10s.csv
./build/bin/test_9axis_e2e test/datasets/synthetic/rotation_sequence_15s.csv
```

## Regenerating Datasets

To regenerate all datasets:
```bash
cd scripts
python3 generate_synthetic_datasets.py
```

To generate specific scenarios with custom parameters:
```bash
python3 generate_synthetic_datasets.py --duration 30 --sample-rate 200
```
"""

    readme_path = output_dir / "README.md"
    with open(readme_path, 'w') as f:
        f.write(readme_content)

    print(f"\n✓ Generated README: {readme_path}")
    print("\nNext steps:")
    print("  1. Update E2E tests to use these datasets")
    print("  2. Run: ./build/bin/test_6axis_e2e test/datasets/synthetic/static_10s.csv")
    print("  3. Run: ./build/bin/test_9axis_e2e test/datasets/synthetic/rotation_z_30dps_10s.csv")


def main():
    parser = argparse.ArgumentParser(
        description='Generate synthetic IMU datasets for sensor fusion testing'
    )
    parser.add_argument(
        '--output', '-o',
        default='../test/datasets/synthetic',
        help='Output directory for datasets (default: ../test/datasets/synthetic)'
    )

    args = parser.parse_args()

    # Resolve output path relative to script location
    script_dir = Path(__file__).parent
    output_dir = (script_dir / args.output).resolve()

    generate_all_scenarios(output_dir)

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
