/*******************************************************************************
 * Test Helper Utilities Implementation
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "test_helpers.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*******************************************************************************
 * Quaternion Utilities
 ******************************************************************************/

bool test_quat_equals(const quaternion_double_t *q1,
                      const quaternion_double_t *q2,
                      double tolerance) {
    return (fabs(q1->q0 - q2->q0) < tolerance &&
            fabs(q1->q1 - q2->q1) < tolerance &&
            fabs(q1->q2 - q2->q2) < tolerance &&
            fabs(q1->q3 - q2->q3) < tolerance);
}

void test_quat_print(const char *name, const quaternion_double_t *q) {
    printf("%s: [%.6f, %.6f, %.6f, %.6f]\n", name, q->q0, q->q1, q->q2, q->q3);
}

bool test_quat_is_normalized(const quaternion_double_t *q, double tolerance) {
    double mag = sqrt(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    return fabs(mag - 1.0) < tolerance;
}

/*******************************************************************************
 * Matrix Utilities
 ******************************************************************************/

bool test_matrix_equals(const double m1[3][3],
                        const double m2[3][3],
                        double tolerance) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(m1[i][j] - m2[i][j]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

void test_matrix_print(const char *name, const double m[3][3]) {
    printf("%s:\n", name);
    for (int i = 0; i < 3; i++) {
        printf("  [%.6f, %.6f, %.6f]\n", m[i][0], m[i][1], m[i][2]);
    }
}

bool test_matrix_is_identity(const double m[3][3], double tolerance) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double expected = (i == j) ? 1.0 : 0.0;
            if (fabs(m[i][j] - expected) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool test_matrix_is_orthogonal(const double m[3][3], double tolerance) {
    // Check if M * M^T = I
    double result[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                result[i][j] += m[i][k] * m[j][k];  // M * M^T
            }
        }
    }

    return test_matrix_is_identity(result, tolerance);
}

/*******************************************************************************
 * Vector Utilities
 ******************************************************************************/

bool test_vector_equals(const double v1[3],
                        const double v2[3],
                        double tolerance) {
    return (fabs(v1[0] - v2[0]) < tolerance &&
            fabs(v1[1] - v2[1]) < tolerance &&
            fabs(v1[2] - v2[2]) < tolerance);
}

void test_vector_print(const char *name, const double v[3]) {
    printf("%s: [%.6f, %.6f, %.6f]\n", name, v[0], v[1], v[2]);
}

double test_vector_magnitude(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

double test_vector_dot(const double v1[3], const double v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void test_vector_cross(const double v1[3], const double v2[3], double result[3]) {
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void test_vector_normalize(const double v[3], double result[3]) {
    double mag = test_vector_magnitude(v);
    if (mag > 1e-9) {
        result[0] = v[0] / mag;
        result[1] = v[1] / mag;
        result[2] = v[2] / mag;
    } else {
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
    }
}

/*******************************************************************************
 * Angle Utilities
 ******************************************************************************/

bool test_angle_equals(double angle1_deg, double angle2_deg, double tolerance_deg) {
    // Wrap both angles to [-180, 180]
    double a1 = test_angle_wrap_180(angle1_deg);
    double a2 = test_angle_wrap_180(angle2_deg);

    return fabs(a1 - a2) < tolerance_deg;
}

double test_angle_wrap_180(double angle_deg) {
    // Wrap angle to [-180, 180]
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

double test_angle_wrap_360(double angle_deg) {
    // Wrap angle to [0, 360]
    while (angle_deg >= 360.0) angle_deg -= 360.0;
    while (angle_deg < 0.0) angle_deg += 360.0;
    return angle_deg;
}

/*******************************************************************************
 * Statistical Utilities
 ******************************************************************************/

void test_compute_statistics(const double *data, uint32_t count,
                             const double *reference,
                             test_statistics_t *stats) {
    if (count == 0 || stats == NULL) {
        return;
    }

    memset(stats, 0, sizeof(test_statistics_t));
    stats->sample_count = count;
    stats->min = data[0];
    stats->max = data[0];

    // Compute mean and find min/max
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        sum += data[i];
        if (data[i] < stats->min) stats->min = data[i];
        if (data[i] > stats->max) stats->max = data[i];
    }
    stats->mean = sum / (double)count;

    // Compute standard deviation
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        double diff = data[i] - stats->mean;
        sum_sq += diff * diff;
    }
    stats->std_dev = sqrt(sum_sq / (double)count);

    // Compute RMSE if reference provided
    if (reference != NULL) {
        double sum_sq_error = 0.0;
        for (uint32_t i = 0; i < count; i++) {
            double error = data[i] - reference[i];
            sum_sq_error += error * error;
        }
        stats->rmse = sqrt(sum_sq_error / (double)count);
    }
}

/*******************************************************************************
 * Test Data Generation
 ******************************************************************************/

void test_generate_identity_quat(quaternion_double_t *q) {
    q->q0 = 1.0;
    q->q1 = 0.0;
    q->q2 = 0.0;
    q->q3 = 0.0;
}

void test_generate_random_quat(quaternion_double_t *q, uint32_t seed) {
    // Simple random quaternion generation (not uniformly distributed)
    srand(seed);
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX * 2.0 * M_PI;
    double u3 = (double)rand() / RAND_MAX * 2.0 * M_PI;

    q->q0 = sqrt(1.0 - u1) * sin(u2);
    q->q1 = sqrt(1.0 - u1) * cos(u2);
    q->q2 = sqrt(u1) * sin(u3);
    q->q3 = sqrt(u1) * cos(u3);
}

void test_generate_rotation_quat(double roll_deg, double pitch_deg, double yaw_deg,
                                 quaternion_double_t *q) {
    // Convert to radians
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    // ZYX Euler to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q->q0 = cr * cp * cy + sr * sp * sy;
    q->q1 = sr * cp * cy - cr * sp * sy;
    q->q2 = cr * sp * cy + sr * cp * sy;
    q->q3 = cr * cp * sy - sr * sp * cy;
}

void test_generate_identity_matrix(double m[3][3]) {
    memset(m, 0, sizeof(double) * 9);
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
}

void test_generate_rotation_matrix_x(double angle_rad, double m[3][3]) {
    double c = cos(angle_rad);
    double s = sin(angle_rad);

    m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
    m[1][0] = 0.0; m[1][1] = c;   m[1][2] = -s;
    m[2][0] = 0.0; m[2][1] = s;   m[2][2] = c;
}

void test_generate_rotation_matrix_y(double angle_rad, double m[3][3]) {
    double c = cos(angle_rad);
    double s = sin(angle_rad);

    m[0][0] = c;   m[0][1] = 0.0; m[0][2] = s;
    m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
    m[2][0] = -s;  m[2][1] = 0.0; m[2][2] = c;
}

void test_generate_rotation_matrix_z(double angle_rad, double m[3][3]) {
    double c = cos(angle_rad);
    double s = sin(angle_rad);

    m[0][0] = c;   m[0][1] = -s;  m[0][2] = 0.0;
    m[1][0] = s;   m[1][1] = c;   m[1][2] = 0.0;
    m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
}

// Note: IMU data generation functions moved to test_data_generator.c
// Use generate_static_sample() and generate_imu_sequence() instead

/*******************************************************************************
 * Performance Utilities
 ******************************************************************************/

void test_timer_start(test_timer_t *timer) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    timer->start_time_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void test_timer_stop(test_timer_t *timer) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    timer->end_time_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    timer->elapsed_ns = timer->end_time_ns - timer->start_time_ns;
    timer->elapsed_us = (double)timer->elapsed_ns / 1000.0;
    timer->elapsed_ms = (double)timer->elapsed_ns / 1000000.0;
}

void test_timer_print(const char *label, const test_timer_t *timer) {
    printf("%s: %.3f ms (%.1f us)\n", label, timer->elapsed_ms, timer->elapsed_us);
}

/*******************************************************************************
 * Validation Utilities
 ******************************************************************************/

bool test_check_no_nan_quat(const quaternion_double_t *q) {
    return !isnan(q->q0) && !isnan(q->q1) && !isnan(q->q2) && !isnan(q->q3);
}

bool test_check_no_nan_vector(const double v[3]) {
    return !isnan(v[0]) && !isnan(v[1]) && !isnan(v[2]);
}

bool test_check_no_nan_matrix(const double m[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (isnan(m[i][j])) return false;
        }
    }
    return true;
}

bool test_has_nan(const double *data, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        if (isnan(data[i])) {
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * Reporting Utilities
 ******************************************************************************/

void test_report_pass(const char *test_name) {
    printf("✓ PASS: %s\n", test_name);
}

void test_report_fail(const char *test_name, const char *reason) {
    printf("✗ FAIL: %s - %s\n", test_name, reason);
}

void test_report_statistics(const char *dataset_name, const test_statistics_t *stats) {
    printf("\n=== Statistics: %s ===\n", dataset_name);
    printf("  Samples:  %u\n", stats->sample_count);
    printf("  Mean:     %.6f\n", stats->mean);
    printf("  Std Dev:  %.6f\n", stats->std_dev);
    printf("  Min:      %.6f\n", stats->min);
    printf("  Max:      %.6f\n", stats->max);
    if (stats->rmse > 0.0) {
        printf("  RMSE:     %.6f\n", stats->rmse);
    }
    printf("========================\n\n");
}
