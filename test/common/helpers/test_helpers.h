#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdint.h>
#include <stdbool.h>
#include "algo_sf_types.h"
#include "algo_sf_quatmath.h"

/*******************************************************************************
 * Test Helper Utilities for SensorFusion Testing
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 ******************************************************************************/

// Tolerance for floating point comparisons
#define TEST_FLOAT_TOLERANCE_TIGHT   1e-6
#define TEST_FLOAT_TOLERANCE_NORMAL  1e-4
#define TEST_FLOAT_TOLERANCE_LOOSE   1e-2
#define TEST_ANGLE_TOLERANCE_DEG     0.5    // degrees

// Quaternion test utilities
bool test_quat_equals(const quaternion_double_t *q1,
                      const quaternion_double_t *q2,
                      double tolerance);

void test_quat_print(const char *name, const quaternion_double_t *q);

bool test_quat_is_normalized(const quaternion_double_t *q, double tolerance);

// Matrix test utilities
bool test_matrix_equals(const double m1[3][3],
                        const double m2[3][3],
                        double tolerance);

void test_matrix_print(const char *name, const double m[3][3]);

bool test_matrix_is_identity(const double m[3][3], double tolerance);

bool test_matrix_is_orthogonal(const double m[3][3], double tolerance);

// Vector test utilities
bool test_vector_equals(const double v1[3],
                        const double v2[3],
                        double tolerance);

void test_vector_print(const char *name, const double v[3]);

double test_vector_magnitude(const double v[3]);

double test_vector_dot(const double v1[3], const double v2[3]);

void test_vector_cross(const double v1[3], const double v2[3], double result[3]);

void test_vector_normalize(const double v[3], double result[3]);

// Angle test utilities
bool test_angle_equals(double angle1_deg, double angle2_deg, double tolerance_deg);

double test_angle_wrap_180(double angle_deg);

double test_angle_wrap_360(double angle_deg);

// Statistical utilities for dataset validation
typedef struct {
    double mean;
    double std_dev;
    double min;
    double max;
    double rmse;
    uint32_t sample_count;
} test_statistics_t;

void test_compute_statistics(const double *data, uint32_t count,
                             const double *reference,
                             test_statistics_t *stats);

// Test data generation utilities
void test_generate_identity_quat(quaternion_double_t *q);

void test_generate_random_quat(quaternion_double_t *q, uint32_t seed);

void test_generate_rotation_quat(double roll_deg, double pitch_deg, double yaw_deg,
                                 quaternion_double_t *q);

void test_generate_identity_matrix(double m[3][3]);

void test_generate_rotation_matrix_x(double angle_rad, double m[3][3]);

void test_generate_rotation_matrix_y(double angle_rad, double m[3][3]);

void test_generate_rotation_matrix_z(double angle_rad, double m[3][3]);

// Note: IMU data generation moved to test_data_generator.h
// Use generate_static_sample() and generate_imu_sequence() instead

// Performance measurement utilities
typedef struct {
    uint64_t start_time_ns;
    uint64_t end_time_ns;
    uint64_t elapsed_ns;
    double elapsed_us;
    double elapsed_ms;
} test_timer_t;

void test_timer_start(test_timer_t *timer);

void test_timer_stop(test_timer_t *timer);

void test_timer_print(const char *label, const test_timer_t *timer);

// Memory and resource checking
bool test_check_no_nan_quat(const quaternion_double_t *q);

bool test_check_no_nan_vector(const double v[3]);

bool test_check_no_nan_matrix(const double m[3][3]);

bool test_has_nan(const double *data, uint32_t count);

// Test result reporting
void test_report_pass(const char *test_name);

void test_report_fail(const char *test_name, const char *reason);

void test_report_statistics(const char *dataset_name, const test_statistics_t *stats);

#endif // TEST_HELPERS_H
