#ifndef TEST_DATA_GENERATOR_H
#define TEST_DATA_GENERATOR_H

#include <stdint.h>
#include "algo_sf_types.h"

/*******************************************************************************
 * Synthetic IMU Data Generator for Testing
 *
 * Generates realistic IMU sensor data for controlled testing scenarios
 * Author: Vikas Yadav
 * Date: 2025-10-11
 ******************************************************************************/

// IMU sample structure
typedef struct {
    double accel[3];     // Accelerometer data (m/s^2)
    double gyro[3];      // Gyroscope data (rad/s)
    double mag[3];       // Magnetometer data (uT)
    uint64_t timestamp;  // Timestamp (nanoseconds)
} test_imu_sample_t;

// Test scenario types
typedef enum {
    SCENARIO_STATIC,           // Stationary IMU
    SCENARIO_CONSTANT_ROTATION, // Rotating at constant rate
    SCENARIO_TILT_FORWARD,     // Tilting forward slowly
    SCENARIO_FIGURE_EIGHT,     // Figure-8 motion
    SCENARIO_FREE_FALL         // Free fall (zero g)
} test_scenario_t;

// Configuration for synthetic data generation
typedef struct {
    test_scenario_t scenario;
    double duration_sec;       // Total duration
    double sample_rate_hz;     // Sampling frequency
    double noise_accel;        // Accelerometer noise std dev (m/s^2)
    double noise_gyro;         // Gyroscope noise std dev (rad/s)
    double noise_mag;          // Magnetometer noise std dev (uT)
    double bias_accel[3];      // Accelerometer bias
    double bias_gyro[3];       // Gyroscope bias
    double initial_orientation[3];  // Initial roll, pitch, yaw (degrees)
} test_data_config_t;

/**
 * @brief Generate synthetic IMU data sequence
 *
 * @param config Configuration for data generation
 * @param samples Output buffer for IMU samples (caller allocates)
 * @param max_samples Maximum number of samples to generate
 * @return Number of samples generated
 */
uint32_t generate_imu_sequence(const test_data_config_t *config,
                               test_imu_sample_t *samples,
                               uint32_t max_samples);

/**
 * @brief Generate single static IMU sample
 *
 * @param roll_deg Roll angle (degrees)
 * @param pitch_deg Pitch angle (degrees)
 * @param yaw_deg Yaw angle (degrees)
 * @param sample Output IMU sample
 */
void generate_static_sample(double roll_deg, double pitch_deg, double yaw_deg,
                            test_imu_sample_t *sample);

/**
 * @brief Generate rotating IMU sample
 *
 * @param angular_velocity Angular velocity vector (rad/s)
 * @param dt Time step (seconds)
 * @param prev_orientation Previous orientation (degrees)
 * @param sample Output IMU sample
 */
void generate_rotating_sample(const double angular_velocity[3],
                              double dt,
                              double prev_orientation[3],
                              test_imu_sample_t *sample);

/**
 * @brief Add noise to IMU sample
 *
 * @param sample IMU sample to add noise to
 * @param noise_accel Accelerometer noise std dev
 * @param noise_gyro Gyroscope noise std dev
 * @param noise_mag Magnetometer noise std dev
 * @param seed Random seed
 */
void add_imu_noise(test_imu_sample_t *sample,
                   double noise_accel,
                   double noise_gyro,
                   double noise_mag,
                   uint32_t seed);

/**
 * @brief Get expected ground truth for static scenario
 *
 * @param roll_deg Expected roll (degrees)
 * @param pitch_deg Expected pitch (degrees)
 * @param yaw_deg Expected yaw (degrees)
 */
void get_static_ground_truth(double *roll_deg, double *pitch_deg, double *yaw_deg);

#endif // TEST_DATA_GENERATOR_H
