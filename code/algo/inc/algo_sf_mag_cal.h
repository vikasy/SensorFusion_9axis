/**
 * @file algo_sf_mag_cal.h
 * @brief Magnetometer calibration using ellipsoid fitting method
 *
 * This module implements online magnetometer calibration to correct for:
 * - Hard iron distortion (constant magnetic offset from device electronics)
 * - Soft iron distortion (magnetic field warping from ferromagnetic materials)
 * - Sensor scaling errors
 *
 * Method: Least Squares Ellipsoid Fit
 * - Collects magnetometer samples over time
 * - Fits samples to ellipsoid equation: (m - c)^T * A * (m - c) = 1
 * - Extracts hard iron offset (c) and soft iron matrix (A^-1/2)
 *
 * The ideal magnetic field forms a sphere centered at origin with radius equal
 * to local magnetic field strength. Distortions cause this to become an ellipsoid
 * offset from origin. This algorithm recovers the transformation.
 *
 * @author Vikas Yadav
 * @date 2025
 */

#ifndef _ALGO_SF_MAG_CAL_H_
#define _ALGO_SF_MAG_CAL_H_

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                         CALIBRATION CONSTANTS
 ******************************************************************************/

/**
 * @brief Minimum number of samples required for calibration
 * @note Need enough samples to fit 9 parameters (3 offsets, 6 matrix elements)
 */
#define MAG_CAL_MIN_SAMPLES 50

/**
 * @brief Maximum number of samples to store (circular buffer)
 * @note Larger buffer = better accuracy but more memory
 */
#define MAG_CAL_MAX_SAMPLES 200

/**
 * @brief Minimum variation in samples to accept calibration
 * @note Prevents calibration from samples in same orientation
 * @note Value in μT² - samples should span different orientations
 */
#define MAG_CAL_MIN_VARIANCE 100.0

/**
 * @brief Quality threshold for accepting calibration (0.0 to 1.0)
 * @note 1.0 = perfect sphere, 0.0 = terrible fit
 * @note Typical good calibration: > 0.95 (full SVD), > 0.85 (bounding box)
 */
#define MAG_CAL_QUALITY_THRESHOLD 0.85

/******************************************************************************
 *                         CALIBRATION STATUS
 ******************************************************************************/

/**
 * @brief Calibration status codes
 */
typedef enum {
    MAG_CAL_STATUS_UNCALIBRATED = 0,  /**< No valid calibration available */
    MAG_CAL_STATUS_COLLECTING = 1,     /**< Collecting samples for calibration */
    MAG_CAL_STATUS_CALIBRATED = 2,     /**< Calibration complete and valid */
    MAG_CAL_STATUS_POOR_QUALITY = 3,   /**< Calibration failed quality check */
    MAG_CAL_STATUS_INSUFFICIENT = 4     /**< Not enough samples yet */
} mag_cal_status_t;

/******************************************************************************
 *                         CALIBRATION DATA STRUCTURES
 ******************************************************************************/

/**
 * @brief Calibration parameters applied to raw magnetometer data
 */
typedef struct {
    // Hard iron offset (constant bias) in μT
    float offset[3];  // [offset_x, offset_y, offset_z]

    // Soft iron correction matrix (symmetric 3x3)
    // Stored as 6 unique elements: [a11, a12, a13, a22, a23, a33]
    float matrix[3][3];

    // Calibration quality metric (0.0 to 1.0)
    // 1.0 = perfect sphere fit, lower = worse fit
    float quality;

    // Magnetic field magnitude (μT) - expected radius of sphere
    float field_magnitude;

    // Timestamp of last calibration update
    uint64_t timestamp;

    // Calibration status
    mag_cal_status_t status;

} mag_cal_params_t;

/**
 * @brief Internal state for online calibration algorithm
 * @note This structure is opaque to users - access via API only
 */
typedef struct {
    // Circular buffer of magnetometer samples
    float samples[MAG_CAL_MAX_SAMPLES][3];

    // Number of samples currently stored
    uint32_t num_samples;

    // Index for next sample (circular buffer)
    uint32_t sample_index;

    // Current calibration parameters
    mag_cal_params_t params;

    // Flag: calibration needs update
    bool needs_update;

    // Variance of collected samples (for quality check)
    float sample_variance[3];

} mag_cal_state_t;

/******************************************************************************
 *                         CALIBRATION API
 ******************************************************************************/

/**
 * @brief Initialize magnetometer calibration module
 *
 * @param state Pointer to calibration state structure
 *
 * @note Sets default identity calibration (no correction)
 */
void mag_cal_init(mag_cal_state_t *state);

/**
 * @brief Reset calibration and clear all samples
 *
 * @param state Pointer to calibration state structure
 *
 * @note Useful when device is moved to new location (different field strength)
 */
void mag_cal_reset(mag_cal_state_t *state);

/**
 * @brief Add new magnetometer sample for calibration
 *
 * @param state Pointer to calibration state structure
 * @param mag_raw Raw magnetometer reading [mx, my, mz] in μT
 * @param timestamp Timestamp of sample (nanoseconds)
 *
 * @return true if sample was added, false if rejected (e.g., too similar to existing)
 *
 * @note Samples are added to circular buffer. When buffer fills, oldest is replaced.
 * @note Algorithm automatically triggers recalibration when enough new samples added.
 */
bool mag_cal_add_sample(mag_cal_state_t *state, const float mag_raw[3], uint64_t timestamp);

/**
 * @brief Compute calibration parameters from collected samples
 *
 * @param state Pointer to calibration state structure
 *
 * @return true if calibration successful, false if failed (quality/variance check)
 *
 * @note Uses least squares ellipsoid fitting
 * @note Updates state->params with new calibration
 * @note Checks quality and variance before accepting calibration
 */
bool mag_cal_compute(mag_cal_state_t *state);

/**
 * @brief Apply calibration to raw magnetometer reading
 *
 * @param state Pointer to calibration state structure
 * @param mag_raw Raw magnetometer reading [mx, my, mz] in μT
 * @param mag_cal Output calibrated magnetometer reading [mx, my, mz] in μT
 *
 * @note Applies transformation: mag_cal = A^-1/2 * (mag_raw - offset)
 * @note If uncalibrated, returns raw data unchanged
 */
void mag_cal_apply(const mag_cal_state_t *state, const float mag_raw[3], float mag_cal[3]);

/**
 * @brief Get current calibration parameters
 *
 * @param state Pointer to calibration state structure
 * @param params Output calibration parameters
 *
 * @return true if calibration is valid, false if uncalibrated
 */
bool mag_cal_get_params(const mag_cal_state_t *state, mag_cal_params_t *params);

/**
 * @brief Set calibration parameters manually (e.g., from saved config)
 *
 * @param state Pointer to calibration state structure
 * @param params Calibration parameters to apply
 *
 * @note Useful for loading calibration from non-volatile storage
 */
void mag_cal_set_params(mag_cal_state_t *state, const mag_cal_params_t *params);

/**
 * @brief Get calibration status
 *
 * @param state Pointer to calibration state structure
 *
 * @return Current calibration status
 */
mag_cal_status_t mag_cal_get_status(const mag_cal_state_t *state);

/**
 * @brief Check if calibration is valid and ready to use
 *
 * @param state Pointer to calibration state structure
 *
 * @return true if calibration is valid, false otherwise
 */
bool mag_cal_is_valid(const mag_cal_state_t *state);

/**
 * @brief Get calibration quality metric (0.0 to 1.0)
 *
 * @param state Pointer to calibration state structure
 *
 * @return Quality metric (1.0 = perfect, lower = worse fit)
 */
float mag_cal_get_quality(const mag_cal_state_t *state);

#endif /* _ALGO_SF_MAG_CAL_H_ */
