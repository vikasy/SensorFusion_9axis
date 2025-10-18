/**
 * @file algo_sf_mag_cal_integration.c
 * @brief Integration helpers for magnetometer calibration with 9-axis fusion
 *
 * @author Vikas Yadav
 * @date 2025
 */

#include "algo_sf_mag_cal_integration.h"
#include <string.h>

void mag_cal_sync_to_9axis(state_vec_9XAGM_t *sf_9x_state,
                            const mag_cal_state_t *mag_cal_state) {
    if (!sf_9x_state || !mag_cal_state) return;

    // Check if calibration is valid
    if (mag_cal_is_valid(mag_cal_state)) {
        // Copy offset
        for (int i = 0; i < 3; i++) {
            sf_9x_state->MagCalOffset[i] = (double)mag_cal_state->params.offset[i];
        }

        // Copy soft iron matrix
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                sf_9x_state->MagCalMatrix[i][j] = (double)mag_cal_state->params.matrix[i][j];
            }
        }

        // Mark calibration as valid
        sf_9x_state->MagCalValid = 1;
    } else {
        // No valid calibration - use identity
        for (int i = 0; i < 3; i++) {
            sf_9x_state->MagCalOffset[i] = 0.0;
            for (int j = 0; j < 3; j++) {
                sf_9x_state->MagCalMatrix[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        sf_9x_state->MagCalValid = 0;
    }
}

bool mag_cal_calibrate_reading(const mag_cal_state_t *mag_cal_state,
                                 const float mag_raw[3],
                                 float mag_calibrated[3]) {
    if (!mag_cal_state || !mag_raw || !mag_calibrated) return false;

    mag_cal_apply(mag_cal_state, mag_raw, mag_calibrated);

    return mag_cal_is_valid(mag_cal_state);
}

bool mag_cal_should_update(const mag_cal_state_t *mag_cal_state) {
    if (!mag_cal_state) return false;

    return mag_cal_state->needs_update;
}

const char* mag_cal_status_string(mag_cal_status_t status) {
    switch (status) {
        case MAG_CAL_STATUS_UNCALIBRATED:
            return "Uncalibrated";
        case MAG_CAL_STATUS_COLLECTING:
            return "Collecting samples";
        case MAG_CAL_STATUS_CALIBRATED:
            return "Calibrated";
        case MAG_CAL_STATUS_POOR_QUALITY:
            return "Poor quality";
        case MAG_CAL_STATUS_INSUFFICIENT:
            return "Insufficient samples";
        default:
            return "Unknown";
    }
}
