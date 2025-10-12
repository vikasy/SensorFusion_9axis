/*******************************************************************************
 * Synthetic IMU Data Generator for Testing
 *
 * Generates realistic IMU sensor data for controlled testing scenarios
 * Author: Vikas Yadav
 * Date: 2025-10-11
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "test_data_generator.h"

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Standard gravity (m/s^2)
#define GRAVITY 9.81

// Earth's magnetic field (approximate, uT)
#define MAG_FIELD_STRENGTH 50.0

// Simple random number generator (0.0 to 1.0)
static double rand_uniform(uint32_t *seed) {
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    return (double)(*seed) / (double)0x7fffffff;
}

// Box-Muller transform for Gaussian noise
static double rand_gaussian(uint32_t *seed, double mean, double stddev) {
    double u1 = rand_uniform(seed);
    double u2 = rand_uniform(seed);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + z0 * stddev;
}

// Rotation matrix from Euler angles (ZYX convention)
static void euler_to_rotation_matrix(double roll_rad, double pitch_rad, double yaw_rad,
                                     double R[3][3]) {
    double sr = sin(roll_rad);
    double cr = cos(roll_rad);
    double sp = sin(pitch_rad);
    double cp = cos(pitch_rad);
    double sy = sin(yaw_rad);
    double cy = cos(yaw_rad);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

// Matrix-vector multiplication
static void matvec_multiply(const double R[3][3], const double v[3], double result[3]) {
    result[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    result[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    result[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

/**
 * @brief Generate single static IMU sample
 */
void generate_static_sample(double roll_deg, double pitch_deg, double yaw_deg,
                            test_imu_sample_t *sample) {
    // Convert to radians
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // Get rotation matrix (body to global)
    double R[3][3];
    euler_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad, R);

    // Gravity in global frame (NED: down is positive Z)
    double gravity_global[3] = {0.0, 0.0, GRAVITY};

    // Transform gravity to body frame (inverse rotation = transpose)
    // Accelerometer measures -gravity in body frame
    sample->accel[0] = -(R[0][0] * gravity_global[0] + R[1][0] * gravity_global[1] + R[2][0] * gravity_global[2]);
    sample->accel[1] = -(R[0][1] * gravity_global[0] + R[1][1] * gravity_global[1] + R[2][1] * gravity_global[2]);
    sample->accel[2] = -(R[0][2] * gravity_global[0] + R[1][2] * gravity_global[1] + R[2][2] * gravity_global[2]);

    // Gyroscope: zero for static
    sample->gyro[0] = 0.0;
    sample->gyro[1] = 0.0;
    sample->gyro[2] = 0.0;

    // Magnetometer: Earth's field pointing north (NED: north is positive X)
    double mag_global[3] = {MAG_FIELD_STRENGTH, 0.0, 0.0};

    // Transform to body frame
    sample->mag[0] = R[0][0] * mag_global[0] + R[1][0] * mag_global[1] + R[2][0] * mag_global[2];
    sample->mag[1] = R[0][1] * mag_global[0] + R[1][1] * mag_global[1] + R[2][1] * mag_global[2];
    sample->mag[2] = R[0][2] * mag_global[0] + R[1][2] * mag_global[1] + R[2][2] * mag_global[2];

    sample->timestamp = 0;
}

/**
 * @brief Generate rotating IMU sample
 */
void generate_rotating_sample(const double angular_velocity[3],
                              double dt,
                              double prev_orientation[3],
                              test_imu_sample_t *sample) {
    // Update orientation using Euler integration
    double new_orientation[3];
    new_orientation[0] = prev_orientation[0] + angular_velocity[0] * dt * 180.0 / M_PI;  // rad/s to deg
    new_orientation[1] = prev_orientation[1] + angular_velocity[1] * dt * 180.0 / M_PI;
    new_orientation[2] = prev_orientation[2] + angular_velocity[2] * dt * 180.0 / M_PI;

    // Generate static sample at new orientation
    generate_static_sample(new_orientation[0], new_orientation[1], new_orientation[2], sample);

    // Add gyroscope reading (body frame angular velocity)
    sample->gyro[0] = angular_velocity[0];
    sample->gyro[1] = angular_velocity[1];
    sample->gyro[2] = angular_velocity[2];

    // Update previous orientation
    prev_orientation[0] = new_orientation[0];
    prev_orientation[1] = new_orientation[1];
    prev_orientation[2] = new_orientation[2];
}

/**
 * @brief Add noise to IMU sample
 */
void add_imu_noise(test_imu_sample_t *sample,
                   double noise_accel,
                   double noise_gyro,
                   double noise_mag,
                   uint32_t seed) {
    uint32_t rng_seed = seed;

    // Add Gaussian noise to accelerometer
    sample->accel[0] += rand_gaussian(&rng_seed, 0.0, noise_accel);
    sample->accel[1] += rand_gaussian(&rng_seed, 0.0, noise_accel);
    sample->accel[2] += rand_gaussian(&rng_seed, 0.0, noise_accel);

    // Add Gaussian noise to gyroscope
    sample->gyro[0] += rand_gaussian(&rng_seed, 0.0, noise_gyro);
    sample->gyro[1] += rand_gaussian(&rng_seed, 0.0, noise_gyro);
    sample->gyro[2] += rand_gaussian(&rng_seed, 0.0, noise_gyro);

    // Add Gaussian noise to magnetometer
    sample->mag[0] += rand_gaussian(&rng_seed, 0.0, noise_mag);
    sample->mag[1] += rand_gaussian(&rng_seed, 0.0, noise_mag);
    sample->mag[2] += rand_gaussian(&rng_seed, 0.0, noise_mag);
}

/**
 * @brief Generate synthetic IMU data sequence
 */
uint32_t generate_imu_sequence(const test_data_config_t *config,
                               test_imu_sample_t *samples,
                               uint32_t max_samples) {
    if (config == NULL || samples == NULL || max_samples == 0) {
        return 0;
    }

    double dt = 1.0 / config->sample_rate_hz;
    uint32_t total_samples = (uint32_t)(config->duration_sec * config->sample_rate_hz);

    if (total_samples > max_samples) {
        total_samples = max_samples;
    }

    double orientation[3] = {
        config->initial_orientation[0],
        config->initial_orientation[1],
        config->initial_orientation[2]
    };

    for (uint32_t i = 0; i < total_samples; i++) {
        test_imu_sample_t *sample = &samples[i];

        switch (config->scenario) {
            case SCENARIO_STATIC:
                // Stationary IMU
                generate_static_sample(orientation[0], orientation[1], orientation[2], sample);
                break;

            case SCENARIO_CONSTANT_ROTATION: {
                // Rotating at 10 deg/s around Z-axis
                double angular_vel[3] = {0.0, 0.0, 10.0 * M_PI / 180.0};  // rad/s
                generate_rotating_sample(angular_vel, dt, orientation, sample);
                break;
            }

            case SCENARIO_TILT_FORWARD: {
                // Slowly tilting forward (pitch)
                double angular_vel[3] = {0.0, 5.0 * M_PI / 180.0, 0.0};  // 5 deg/s pitch
                generate_rotating_sample(angular_vel, dt, orientation, sample);
                break;
            }

            case SCENARIO_FIGURE_EIGHT: {
                // Figure-8 motion
                double t = i * dt;
                double angular_vel[3] = {
                    5.0 * sin(2.0 * M_PI * 0.5 * t) * M_PI / 180.0,  // Roll oscillation
                    3.0 * cos(2.0 * M_PI * 0.5 * t) * M_PI / 180.0,  // Pitch oscillation
                    2.0 * sin(2.0 * M_PI * 0.25 * t) * M_PI / 180.0  // Yaw rotation
                };
                generate_rotating_sample(angular_vel, dt, orientation, sample);
                break;
            }

            case SCENARIO_FREE_FALL:
                // Free fall: zero acceleration
                sample->accel[0] = 0.0;
                sample->accel[1] = 0.0;
                sample->accel[2] = 0.0;
                sample->gyro[0] = 0.0;
                sample->gyro[1] = 0.0;
                sample->gyro[2] = 0.0;
                // Magnetometer still works in free fall
                generate_static_sample(orientation[0], orientation[1], orientation[2], sample);
                sample->accel[0] = 0.0;  // Override with zero-g
                sample->accel[1] = 0.0;
                sample->accel[2] = 0.0;
                break;

            default:
                generate_static_sample(orientation[0], orientation[1], orientation[2], sample);
                break;
        }

        // Add sensor biases
        sample->accel[0] += config->bias_accel[0];
        sample->accel[1] += config->bias_accel[1];
        sample->accel[2] += config->bias_accel[2];

        sample->gyro[0] += config->bias_gyro[0];
        sample->gyro[1] += config->bias_gyro[1];
        sample->gyro[2] += config->bias_gyro[2];

        // Add noise
        if (config->noise_accel > 0.0 || config->noise_gyro > 0.0 || config->noise_mag > 0.0) {
            add_imu_noise(sample, config->noise_accel, config->noise_gyro, config->noise_mag, i + 1);
        }

        // Set timestamp
        sample->timestamp = (uint64_t)(i * dt * 1e9);  // nanoseconds
    }

    return total_samples;
}

/**
 * @brief Get expected ground truth for static scenario
 */
void get_static_ground_truth(double *roll_deg, double *pitch_deg, double *yaw_deg) {
    // For static scenario, ground truth is the initial orientation
    // This is a placeholder - in real implementation, this would track
    // the actual orientation throughout the sequence
    *roll_deg = 0.0;
    *pitch_deg = 0.0;
    *yaw_deg = 0.0;
}
