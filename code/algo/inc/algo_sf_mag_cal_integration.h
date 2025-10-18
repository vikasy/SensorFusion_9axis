/**
 * @file algo_sf_mag_cal_integration.h
 * @brief Integration helpers for magnetometer calibration with 9-axis fusion
 *
 * This module provides convenience functions to integrate the magnetometer
 * calibration module (algo_sf_mag_cal) with the 9-axis sensor fusion algorithm
 * (algo_sf_9x_sensor_fusion).
 *
 * @author Vikas Yadav
 * @date 2025
 */

#ifndef _ALGO_SF_MAG_CAL_INTEGRATION_H_
#define _ALGO_SF_MAG_CAL_INTEGRATION_H_

#include "algo_sf_mag_cal.h"
#include "algo_sf_9x_sensor_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Apply calibration from mag_cal module to 9-axis state vector
 *
 * @param sf_9x_state Pointer to 9-axis fusion state vector
 * @param mag_cal_state Pointer to calibration state
 *
 * @note Updates MagCalOffset, MagCalMatrix, and MagCalValid fields
 */
void mag_cal_sync_to_9axis(state_vec_9XAGM_t *sf_9x_state,
                            const mag_cal_state_t *mag_cal_state);

/**
 * @brief Calibrate magnetometer reading before fusion
 *
 * @param mag_cal_state Pointer to calibration state
 * @param mag_raw Raw magnetometer reading [mx, my, mz] in μT
 * @param mag_calibrated Output calibrated reading [mx, my, mz] in μT
 *
 * @return true if calibration was applied, false if using raw data
 */
bool mag_cal_calibrate_reading(const mag_cal_state_t *mag_cal_state,
                                 const float mag_raw[3],
                                 float mag_calibrated[3]);

/**
 * @brief Check if calibration should be updated
 *
 * Determines if enough new samples have been collected to warrant
 * recomputing calibration parameters.
 *
 * @param mag_cal_state Pointer to calibration state
 *
 * @return true if should recompute, false otherwise
 */
bool mag_cal_should_update(const mag_cal_state_t *mag_cal_state);

/**
 * @brief Get calibration status string for logging
 *
 * @param status Calibration status code
 *
 * @return Human-readable status string
 */
const char* mag_cal_status_string(mag_cal_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* _ALGO_SF_MAG_CAL_INTEGRATION_H_ */
