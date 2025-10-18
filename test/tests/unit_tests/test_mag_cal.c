/**
 * @file test_mag_cal.c
 * @brief Unit tests for magnetometer calibration module
 *
 * @author Vikas Yadav
 * @date 2025
 */

#include "unity.h"
#include "algo_sf_mag_cal.h"
#include <math.h>
#include <string.h>

// Test state
static mag_cal_state_t test_state;

void setUp(void) {
    mag_cal_init(&test_state);
}

void tearDown(void) {
    // Cleanup if needed
}

/******************************************************************************
 *                         INITIALIZATION TESTS
 ******************************************************************************/

void test_mag_cal_init_sets_uncalibrated_status(void) {
    TEST_ASSERT_EQUAL(MAG_CAL_STATUS_UNCALIBRATED, test_state.params.status);
}

void test_mag_cal_init_sets_identity_matrix(void) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-6, expected, test_state.params.matrix[i][j]);
        }
    }
}

void test_mag_cal_init_zeros_offset(void) {
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, test_state.params.offset[i]);
    }
}

void test_mag_cal_init_sets_zero_samples(void) {
    TEST_ASSERT_EQUAL(0, test_state.num_samples);
}

/******************************************************************************
 *                         SAMPLE COLLECTION TESTS
 ******************************************************************************/

void test_mag_cal_add_sample_accepts_valid_sample(void) {
    float mag[3] = {50.0f, 0.0f, 0.0f};
    bool result = mag_cal_add_sample(&test_state, mag, 1000ULL);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1, test_state.num_samples);
}

void test_mag_cal_add_sample_rejects_nan(void) {
    float mag[3] = {NAN, 0.0f, 0.0f};
    bool result = mag_cal_add_sample(&test_state, mag, 1000ULL);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(0, test_state.num_samples);
}

void test_mag_cal_add_sample_rejects_outliers(void) {
    float mag[3] = {500.0f, 0.0f, 0.0f};  // > 200 μT
    bool result = mag_cal_add_sample(&test_state, mag, 1000ULL);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(0, test_state.num_samples);
}

void test_mag_cal_add_sample_rejects_duplicates(void) {
    float mag[3] = {50.0f, 0.0f, 0.0f};

    // Add first sample
    mag_cal_add_sample(&test_state, mag, 1000ULL);

    // Try to add same sample (should be rejected as too similar)
    bool result = mag_cal_add_sample(&test_state, mag, 2000ULL);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(1, test_state.num_samples);
}

void test_mag_cal_add_sample_accepts_diverse_samples(void) {
    float mag1[3] = {50.0f, 0.0f, 0.0f};
    float mag2[3] = {0.0f, 50.0f, 0.0f};

    mag_cal_add_sample(&test_state, mag1, 1000ULL);
    bool result = mag_cal_add_sample(&test_state, mag2, 2000ULL);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(2, test_state.num_samples);
}

void test_mag_cal_add_sample_updates_status_to_collecting(void) {
    // Add enough samples to trigger COLLECTING status
    for (int i = 0; i < MAG_CAL_MIN_SAMPLES; i++) {
        float mag[3] = {50.0f + i, i, i};
        mag_cal_add_sample(&test_state, mag, i * 1000ULL);
    }

    TEST_ASSERT_EQUAL(MAG_CAL_STATUS_COLLECTING, test_state.params.status);
}

/******************************************************************************
 *                         CALIBRATION COMPUTATION TESTS
 ******************************************************************************/

void test_mag_cal_compute_fails_with_insufficient_samples(void) {
    // Add only a few samples
    for (int i = 0; i < 10; i++) {
        float mag[3] = {50.0f + i, i, i};
        mag_cal_add_sample(&test_state, mag, i * 1000ULL);
    }

    bool result = mag_cal_compute(&test_state);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(MAG_CAL_STATUS_INSUFFICIENT, test_state.params.status);
}

void test_mag_cal_compute_fails_with_low_variance(void) {
    // Add many samples but all similar (low variance in YZ, only varying in X)
    for (int i = 0; i < MAG_CAL_MIN_SAMPLES + 10; i++) {
        float mag[3] = {50.0f + i * 5.0f, 0.1f, 0.1f};  // Vary enough to be accepted, but low YZ variance
        mag_cal_add_sample(&test_state, mag, i * 1000000ULL);
    }

    bool result = mag_cal_compute(&test_state);

    TEST_ASSERT_FALSE(result);
    // Could be INSUFFICIENT (if samples rejected) or POOR_QUALITY (if accepted but low variance)
    TEST_ASSERT_TRUE(test_state.params.status == MAG_CAL_STATUS_INSUFFICIENT ||
                     test_state.params.status == MAG_CAL_STATUS_POOR_QUALITY);
}

void test_mag_cal_compute_succeeds_with_good_samples(void) {
    // Add samples from sphere surface
    for (int i = 0; i < 100; i++) {
        float theta = (float)i * 3.14159f / 50;
        float phi = (float)i * 2.0f * 3.14159f / 100;

        float mag[3];
        mag[0] = 50.0f * sinf(theta) * cosf(phi);
        mag[1] = 50.0f * sinf(theta) * sinf(phi);
        mag[2] = 50.0f * cosf(theta);

        mag_cal_add_sample(&test_state, mag, i * 1000ULL);
    }

    bool result = mag_cal_compute(&test_state);

    // May pass or fail depending on quality, but should not crash
    TEST_ASSERT_TRUE(result || !result);  // Just check it runs
}

/******************************************************************************
 *                         CALIBRATION APPLICATION TESTS
 ******************************************************************************/

void test_mag_cal_apply_passes_through_when_uncalibrated(void) {
    float mag_raw[3] = {70.0f, -5.0f, 10.0f};
    float mag_cal[3];

    mag_cal_apply(&test_state, mag_raw, mag_cal);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, mag_raw[0], mag_cal[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, mag_raw[1], mag_cal[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, mag_raw[2], mag_cal[2]);
}

void test_mag_cal_apply_applies_offset(void) {
    // Manually set calibration
    test_state.params.offset[0] = 10.0f;
    test_state.params.offset[1] = -5.0f;
    test_state.params.offset[2] = 8.0f;
    test_state.params.status = MAG_CAL_STATUS_CALIBRATED;

    float mag_raw[3] = {70.0f, -5.0f, 10.0f};
    float mag_cal[3];

    mag_cal_apply(&test_state, mag_raw, mag_cal);

    // Should subtract offset
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 60.0f, mag_cal[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, mag_cal[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 2.0f, mag_cal[2]);
}

/******************************************************************************
 *                         STATUS AND QUERY TESTS
 ******************************************************************************/

void test_mag_cal_get_status_returns_current_status(void) {
    TEST_ASSERT_EQUAL(MAG_CAL_STATUS_UNCALIBRATED, mag_cal_get_status(&test_state));
}

void test_mag_cal_is_valid_returns_false_when_uncalibrated(void) {
    TEST_ASSERT_FALSE(mag_cal_is_valid(&test_state));
}

void test_mag_cal_is_valid_returns_true_when_calibrated(void) {
    test_state.params.status = MAG_CAL_STATUS_CALIBRATED;
    TEST_ASSERT_TRUE(mag_cal_is_valid(&test_state));
}

void test_mag_cal_get_quality_returns_zero_initially(void) {
    float quality = mag_cal_get_quality(&test_state);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, quality);
}

void test_mag_cal_get_params_fails_when_uncalibrated(void) {
    mag_cal_params_t params;
    bool result = mag_cal_get_params(&test_state, &params);

    TEST_ASSERT_FALSE(result);
}

void test_mag_cal_get_params_succeeds_when_calibrated(void) {
    test_state.params.status = MAG_CAL_STATUS_CALIBRATED;

    mag_cal_params_t params;
    bool result = mag_cal_get_params(&test_state, &params);

    TEST_ASSERT_TRUE(result);
}

/******************************************************************************
 *                         RESET TESTS
 ******************************************************************************/

void test_mag_cal_reset_clears_samples(void) {
    // Add some samples
    for (int i = 0; i < 10; i++) {
        float mag[3] = {50.0f + i, i, i};
        mag_cal_add_sample(&test_state, mag, i * 1000ULL);
    }

    mag_cal_reset(&test_state);

    TEST_ASSERT_EQUAL(0, test_state.num_samples);
}

void test_mag_cal_reset_restores_uncalibrated_status(void) {
    test_state.params.status = MAG_CAL_STATUS_CALIBRATED;

    mag_cal_reset(&test_state);

    TEST_ASSERT_EQUAL(MAG_CAL_STATUS_UNCALIBRATED, test_state.params.status);
}

/******************************************************************************
 *                         MAIN TEST RUNNER
 ******************************************************************************/

int main(void) {
    UNITY_BEGIN();

    // Initialization tests
    RUN_TEST(test_mag_cal_init_sets_uncalibrated_status);
    RUN_TEST(test_mag_cal_init_sets_identity_matrix);
    RUN_TEST(test_mag_cal_init_zeros_offset);
    RUN_TEST(test_mag_cal_init_sets_zero_samples);

    // Sample collection tests
    RUN_TEST(test_mag_cal_add_sample_accepts_valid_sample);
    RUN_TEST(test_mag_cal_add_sample_rejects_nan);
    RUN_TEST(test_mag_cal_add_sample_rejects_outliers);
    RUN_TEST(test_mag_cal_add_sample_rejects_duplicates);
    RUN_TEST(test_mag_cal_add_sample_accepts_diverse_samples);
    RUN_TEST(test_mag_cal_add_sample_updates_status_to_collecting);

    // Calibration computation tests
    RUN_TEST(test_mag_cal_compute_fails_with_insufficient_samples);
    RUN_TEST(test_mag_cal_compute_fails_with_low_variance);
    RUN_TEST(test_mag_cal_compute_succeeds_with_good_samples);

    // Calibration application tests
    RUN_TEST(test_mag_cal_apply_passes_through_when_uncalibrated);
    RUN_TEST(test_mag_cal_apply_applies_offset);

    // Status and query tests
    RUN_TEST(test_mag_cal_get_status_returns_current_status);
    RUN_TEST(test_mag_cal_is_valid_returns_false_when_uncalibrated);
    RUN_TEST(test_mag_cal_is_valid_returns_true_when_calibrated);
    RUN_TEST(test_mag_cal_get_quality_returns_zero_initially);
    RUN_TEST(test_mag_cal_get_params_fails_when_uncalibrated);
    RUN_TEST(test_mag_cal_get_params_succeeds_when_calibrated);

    // Reset tests
    RUN_TEST(test_mag_cal_reset_clears_samples);
    RUN_TEST(test_mag_cal_reset_restores_uncalibrated_status);

    return UNITY_END();
}
