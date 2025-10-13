/*
 * kalman_9dof.h
 *
 * 9-axis (accelerometer + gyroscope + magnetometer) sensor fusion
 * using an error-state Kalman filter to estimate orientation,
 * quaternion, rotation matrix, and derived quantities.
 *
 * Based on Freescale/NXP application notes:
 *  - AN5018: Basic Kalman Filter Theory
 *  - AN5021: Calculation of Orientation Matrices
 *  - AN5022: Quaternion Algebra and Rotations
 *  - AN5023: Sensor Fusion Kalman Filter
 *
 * The implementation maintains a 6x1 error state consisting of the
 * small-angle attitude error (rad) and gyroscope bias error (rad/s)
 * and fuses normalized accelerometer and magnetometer measurements.
 */

#ifndef KALMAN_9DOF_H
#define KALMAN_9DOF_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double accel[3];  /* Specific force in m/s^2 */
    double gyro[3];   /* Angular rate in rad/s   */
    double mag[3];    /* Magnetic field in uT    */
    double dt;        /* Sample period in seconds */
} sf9dof_sample_t;

typedef struct {
    double gyro_noise;       /* Gyro measurement noise spectral density (rad/s)^2 */
    double gyro_bias_noise;  /* Gyro bias random walk (rad/s)^2 */
    double accel_noise;      /* Accelerometer noise variance for normalized measurement */
    double mag_noise;        /* Magnetometer noise variance for normalized measurement */
    double gravity;          /* Gravitational acceleration (m/s^2) */
} sf9dof_config_t;

typedef struct {
    bool   initialised;
    sf9dof_config_t config;

    /* Orientation state */
    double quat[4];       /* w, x, y, z */
    double rot[3][3];     /* Rotation matrix: global(NED) -> sensor */
    double euler[3];      /* Roll, Pitch, Yaw in radians */

    /* Bias estimate */
    double gyro_bias[3];

    /* Covariance matrix (6x6) and error state (6x1) */
    double P[6][6];
    double x_err[6];

    /* Reference magnetic field in global frame (unit vector) */
    double mag_ref[3];

    /* Outputs */
    double gravity_s[3];  /* Gravity vector in sensor frame (m/s^2) */
    double gravity_g[3];  /* Gravity vector in global frame (m/s^2) */
    double lin_accel_s[3];
    double lin_accel_g[3];
    double heading;       /* Compass heading (rad) */
} sf9dof_state_t;

/**
 * Fill |cfg| with conservative default parameters.
 */
void sf9dof_default_config(sf9dof_config_t *cfg);

/**
 * Reset the filter state.
 */
void sf9dof_reset(sf9dof_state_t *state, const sf9dof_config_t *cfg);

/**
 * Run one Kalman prediction / correction step.
 *
 * The sample must provide non-zero dt. Accelerometer and magnetometer
 * measurements are internally normalised; zero-norm inputs are ignored.
 */
void sf9dof_update(sf9dof_state_t *state, const sf9dof_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_9DOF_H */
