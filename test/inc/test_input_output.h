/* Common test data structures for sensor fusion algorithm validation */
#ifndef TEST_INPUT_OUTPUT_H
#define TEST_INPUT_OUTPUT_H

#include <stdint.h>

typedef struct {
    uint32_t id;      // Sensor ID: 0=accel, 1=gyro, 2=mag
    int16_t x, y, z;  // Sensor readings
    uint64_t ts;      // Timestamp (nanoseconds)
} test_sensor_sample_t;

typedef struct {
    float q0, q1, q2, q3;  // Expected quaternion output
    float roll, pitch, yaw; // Expected Euler angles (degrees)  
    uint64_t ts;            // Timestamp
} test_expected_output_t;

#endif /* TEST_INPUT_OUTPUT_H */
