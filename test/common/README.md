# Test Fixtures and Utilities

This directory contains helper functions and synthetic data generators for testing the SensorFusion library.

## Files

### `test_helpers.h` / `test_helpers.c`
General-purpose test utilities for quaternions, matrices, vectors, angles, and statistics.

**Key Functions:**
- `test_quat_equals()` - Compare quaternions with tolerance
- `test_matrix_is_orthogonal()` - Verify rotation matrix properties
- `test_vector_normalize()` - Vector operations
- `test_angle_wrap_180()` - Angle wrapping utilities
- `test_compute_statistics()` - Statistical analysis (mean, std dev, RMSE)
- `test_timer_start()` / `test_timer_stop()` - Performance measurement

### `test_data_generator.h` / `test_data_generator.c`
Synthetic IMU data generator for controlled testing scenarios.

**Key Functions:**
- `generate_static_sample()` - Single static IMU sample at given orientation
- `generate_rotating_sample()` - Rotating IMU with angular velocity
- `generate_imu_sequence()` - Full sequence of IMU data
- `add_imu_noise()` - Add Gaussian noise to IMU data

**Supported Scenarios:**
- `SCENARIO_STATIC` - Stationary IMU
- `SCENARIO_CONSTANT_ROTATION` - Rotating at constant rate
- `SCENARIO_TILT_FORWARD` - Slowly tilting forward
- `SCENARIO_FIGURE_EIGHT` - Figure-8 motion pattern
- `SCENARIO_FREE_FALL` - Zero-g free fall

## Usage Examples

### Example 1: Generate Static IMU Data
```c
#include "test_data_generator.h"

test_imu_sample_t sample;
generate_static_sample(0.0, 30.0, 0.0, &sample);  // 30° pitch

printf("Accel: [%.3f, %.3f, %.3f] m/s^2\n",
       sample.accel[0], sample.accel[1], sample.accel[2]);
```

### Example 2: Generate Test Sequence
```c
test_data_config_t config = {
    .scenario = SCENARIO_STATIC,
    .duration_sec = 2.0,
    .sample_rate_hz = 100.0,
    .noise_accel = 0.01,    // m/s^2
    .noise_gyro = 0.001,    // rad/s
    .noise_mag = 0.5,       // uT
    .bias_gyro = {0.01, -0.01, 0.005},
    .initial_orientation = {0.0, 0.0, 0.0}
};

test_imu_sample_t samples[200];
uint32_t count = generate_imu_sequence(&config, samples, 200);
```

### Example 3: Quaternion Testing
```c
#include "test_helpers.h"

quaternion_double_t q1 = {1.0, 0.0, 0.0, 0.0};
quaternion_double_t q2 = {0.9999, 0.0001, 0.0, 0.0};

if (test_quat_equals(&q1, &q2, 1e-3)) {
    test_report_pass("Quaternion comparison");
}

if (test_quat_is_normalized(&q1, 1e-6)) {
    test_report_pass("Quaternion is normalized");
}
```

### Example 4: Performance Measurement
```c
test_timer_t timer;

test_timer_start(&timer);
// ... code to measure ...
test_timer_stop(&timer);

test_timer_print("Algorithm runtime", &timer);
// Output: Algorithm runtime: 1.234 ms (1234.5 us)
```

### Example 5: Statistical Analysis
```c
double orientation_data[100];
double ground_truth[100];

// ... collect data ...

test_statistics_t stats;
test_compute_statistics(orientation_data, 100, ground_truth, &stats);

printf("RMSE: %.3f degrees\n", stats.rmse);
printf("Mean: %.3f, Std Dev: %.3f\n", stats.mean, stats.std_dev);
```

## Demo Program

Compile and run the demo:
```bash
cd test/fixtures
gcc -std=c99 -I. -I../../code/algo/inc \
    test_data_generator.c test_data_generator_demo.c -lm -o demo
./demo
```

## Integration with Tests

These utilities are automatically available to all test files:

```cmake
# In build/CMakeLists.txt
include_directories(
    ${TEST_FIXTURE_DIR}
)
```

Simply include the headers in your test files:
```c
#include "test_helpers.h"
#include "test_data_generator.h"
```

## Coordinate Frames

### Accelerometer
- Measures specific force (not acceleration!)
- At rest: reads -gravity in sensor frame
- Values in m/s²

### Gyroscope
- Measures angular velocity in sensor frame
- Values in rad/s

### Magnetometer
- Measures Earth's magnetic field in sensor frame
- Typical values: 25-65 µT
- Points toward magnetic north (not true north!)

### Orientation Convention
- **Roll:** Rotation around X-axis (forward)
- **Pitch:** Rotation around Y-axis (right)
- **Yaw:** Rotation around Z-axis (down)
- **Units:** Degrees

## Notes

- All functions are thread-safe (no global state)
- Random number generation uses local seeds
- Noise generation uses Box-Muller transform (Gaussian)
- Coordinate frame: NED (North-East-Down) convention
- Gravity: 9.81 m/s²
- Earth magnetic field: ~50 µT (approximate)

## Future Enhancements

- [ ] Non-uniform magnetic field simulation
- [ ] Temperature-dependent sensor bias
- [ ] Vibration and shock simulation
- [ ] Multi-IMU synchronization
- [ ] Real dataset loader (EuRoC, BROAD)
