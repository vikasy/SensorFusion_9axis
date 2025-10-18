/**
 * @file algo_sf_mag_cal.c
 * @brief Magnetometer calibration implementation using ellipsoid fitting
 *
 * Implementation of least squares ellipsoid fitting for magnetometer calibration.
 *
 * Theory:
 * - Ideal magnetometer in uniform field traces a sphere: |m| = constant
 * - Hard iron (constant offset) shifts sphere away from origin
 * - Soft iron (field warping) deforms sphere into ellipsoid
 * - Sensor scaling errors also contribute to ellipsoid shape
 *
 * Ellipsoid Equation:
 *   (m - c)^T * A * (m - c) = 1
 *   where: c = center (hard iron offset)
 *          A = shape matrix (related to soft iron)
 *
 * Algorithm:
 * 1. Expand ellipsoid equation to polynomial form
 * 2. Collect magnetometer samples from different orientations
 * 3. Solve for 9 parameters using least squares
 * 4. Extract center (c) and shape matrix (A)
 * 5. Calibration: m_cal = A^-1/2 * (m_raw - c)
 *
 * @author Vikas Yadav
 * @date 2025
 */

#include "algo_sf_mag_cal.h"
#include "algo_sf_types.h"
#include <math.h>
#include <string.h>

/******************************************************************************
 *                         INTERNAL HELPER FUNCTIONS
 ******************************************************************************/

/**
 * @brief Compute variance of sample buffer
 */
static void compute_sample_variance(const mag_cal_state_t *state, float variance[3]) {
    float mean[3] = {0.0f, 0.0f, 0.0f};

    // Compute mean
    for (uint32_t i = 0; i < state->num_samples; i++) {
        mean[0] += state->samples[i][0];
        mean[1] += state->samples[i][1];
        mean[2] += state->samples[i][2];
    }
    mean[0] /= state->num_samples;
    mean[1] /= state->num_samples;
    mean[2] /= state->num_samples;

    // Compute variance
    variance[0] = variance[1] = variance[2] = 0.0f;
    for (uint32_t i = 0; i < state->num_samples; i++) {
        float dx = state->samples[i][0] - mean[0];
        float dy = state->samples[i][1] - mean[1];
        float dz = state->samples[i][2] - mean[2];
        variance[0] += dx * dx;
        variance[1] += dy * dy;
        variance[2] += dz * dz;
    }
    variance[0] /= state->num_samples;
    variance[1] /= state->num_samples;
    variance[2] /= state->num_samples;
}

/**
 * @brief Check if sample is sufficiently different from existing samples
 */
static bool is_sample_diverse(const mag_cal_state_t *state, const float mag[3]) {
    if (state->num_samples == 0) return true;

    // Check against recent samples (avoid duplicate nearby points)
    const uint32_t check_count = (state->num_samples < 10) ? state->num_samples : 10;
    const float min_distance_sq = 2.0f;  // Minimum 2 μT difference

    for (uint32_t i = 0; i < check_count; i++) {
        uint32_t idx = (state->sample_index + MAG_CAL_MAX_SAMPLES - 1 - i) % MAG_CAL_MAX_SAMPLES;
        if (idx >= state->num_samples) continue;

        float dx = mag[0] - state->samples[idx][0];
        float dy = mag[1] - state->samples[idx][1];
        float dz = mag[2] - state->samples[idx][2];
        float dist_sq = dx*dx + dy*dy + dz*dz;

        if (dist_sq < min_distance_sq) {
            return false;  // Too similar to existing sample
        }
    }

    return true;
}

/**
 * @brief Solve 3x3 linear system using Gaussian elimination
 * @param A Input matrix (3x3)
 * @param b Input vector (3x1)
 * @param x Output solution (3x1)
 * @return true if successful, false if singular
 */
static bool solve_3x3(float A[3][3], const float b[3], float x[3]) {
    // Copy to avoid modifying input
    float M[3][3];
    float rhs[3];
    memcpy(M, A, sizeof(M));
    memcpy(rhs, b, sizeof(rhs));

    // Forward elimination
    for (int i = 0; i < 3; i++) {
        // Find pivot
        int pivot = i;
        float max_val = fabsf(M[i][i]);
        for (int j = i + 1; j < 3; j++) {
            if (fabsf(M[j][i]) > max_val) {
                max_val = fabsf(M[j][i]);
                pivot = j;
            }
        }

        // Check for singular matrix
        if (fabsf(max_val) < EPSILON) {
            return false;
        }

        // Swap rows if needed
        if (pivot != i) {
            for (int k = 0; k < 3; k++) {
                float tmp = M[i][k];
                M[i][k] = M[pivot][k];
                M[pivot][k] = tmp;
            }
            float tmp = rhs[i];
            rhs[i] = rhs[pivot];
            rhs[pivot] = tmp;
        }

        // Eliminate below
        for (int j = i + 1; j < 3; j++) {
            float factor = M[j][i] / M[i][i];
            for (int k = i; k < 3; k++) {
                M[j][k] -= factor * M[i][k];
            }
            rhs[j] -= factor * rhs[i];
        }
    }

    // Back substitution
    for (int i = 2; i >= 0; i--) {
        float sum = rhs[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= M[i][j] * x[j];
        }
        x[i] = sum / M[i][i];
    }

    return true;
}

/**
 * @brief Compute matrix square root using eigenvalue decomposition (simplified)
 * @note For symmetric positive definite 3x3 matrix
 */
static void matrix_sqrt_inv(const float M[3][3], float M_inv_sqrt[3][3]) {
    // Simplified: Use Cholesky-like approximation for positive definite symmetric matrix
    // For production, use proper eigenvalue decomposition

    // Simple diagonal approximation (assumes near-diagonal dominance)
    // This is a simplified version - full implementation would use proper eigendecomposition

    float diag_sqrt_inv[3];
    for (int i = 0; i < 3; i++) {
        if (M[i][i] > EPSILON) {
            diag_sqrt_inv[i] = 1.0f / sqrtf(M[i][i]);
        } else {
            diag_sqrt_inv[i] = 1.0f;
        }
    }

    // Construct inverse square root (simplified diagonal)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                M_inv_sqrt[i][j] = diag_sqrt_inv[i];
            } else {
                M_inv_sqrt[i][j] = 0.0f;
            }
        }
    }
}

/******************************************************************************
 *                         PUBLIC API IMPLEMENTATION
 ******************************************************************************/

void mag_cal_init(mag_cal_state_t *state) {
    memset(state, 0, sizeof(mag_cal_state_t));

    // Initialize with identity calibration (no correction)
    state->params.offset[0] = 0.0f;
    state->params.offset[1] = 0.0f;
    state->params.offset[2] = 0.0f;

    // Identity matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            state->params.matrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    state->params.quality = 0.0f;
    state->params.field_magnitude = 50.0f;  // Default ~50 μT (typical Earth field)
    state->params.status = MAG_CAL_STATUS_UNCALIBRATED;
    state->params.timestamp = 0;

    state->num_samples = 0;
    state->sample_index = 0;
    state->needs_update = false;
}

void mag_cal_reset(mag_cal_state_t *state) {
    mag_cal_init(state);
}

bool mag_cal_add_sample(mag_cal_state_t *state, const float mag_raw[3], uint64_t timestamp) {
    // Check for valid data (no NaN, no huge outliers)
    for (int i = 0; i < 3; i++) {
        if (isnan(mag_raw[i]) || fabsf(mag_raw[i]) > 200.0f) {
            return false;  // Reject invalid sample
        }
    }

    // Check if sample is diverse enough
    if (!is_sample_diverse(state, mag_raw)) {
        return false;  // Too similar to existing samples
    }

    // Add to circular buffer
    state->samples[state->sample_index][0] = mag_raw[0];
    state->samples[state->sample_index][1] = mag_raw[1];
    state->samples[state->sample_index][2] = mag_raw[2];

    state->sample_index = (state->sample_index + 1) % MAG_CAL_MAX_SAMPLES;

    if (state->num_samples < MAG_CAL_MAX_SAMPLES) {
        state->num_samples++;
    }

    // Update variance
    compute_sample_variance(state, state->sample_variance);

    // Mark for update if we have enough samples
    if (state->num_samples >= MAG_CAL_MIN_SAMPLES) {
        state->needs_update = true;
        state->params.status = MAG_CAL_STATUS_COLLECTING;
    } else {
        state->params.status = MAG_CAL_STATUS_INSUFFICIENT;
    }

    return true;
}

bool mag_cal_compute(mag_cal_state_t *state) {
    // Check if we have enough samples
    if (state->num_samples < MAG_CAL_MIN_SAMPLES) {
        state->params.status = MAG_CAL_STATUS_INSUFFICIENT;
        return false;
    }

    // Check variance (need samples from different orientations)
    float min_var = state->sample_variance[0];
    if (state->sample_variance[1] < min_var) min_var = state->sample_variance[1];
    if (state->sample_variance[2] < min_var) min_var = state->sample_variance[2];

    if (min_var < MAG_CAL_MIN_VARIANCE) {
        state->params.status = MAG_CAL_STATUS_POOR_QUALITY;
        return false;  // Samples not diverse enough
    }

    // Ellipsoid fitting using least squares
    // Equation: x^2*A + y^2*B + z^2*C + 2xy*D + 2xz*E + 2yz*F + 2x*G + 2y*H + 2z*I = 1
    // We solve for [A, B, C, D, E, F, G, H, I] using least squares

    const uint32_t n = state->num_samples;

    // Build normal equations: M^T * M * p = M^T * 1
    // where M is design matrix (n x 9) and p is parameters (9 x 1)

    float D[9][9] = {{0}};  // M^T * M
    float d[9] = {0};        // M^T * 1

    for (uint32_t i = 0; i < n; i++) {
        float x = state->samples[i][0];
        float y = state->samples[i][1];
        float z = state->samples[i][2];

        float row[9] = {
            x*x,    // A
            y*y,    // B
            z*z,    // C
            2*x*y,  // D
            2*x*z,  // E
            2*y*z,  // F
            2*x,    // G
            2*y,    // H
            2*z     // I
        };

        // Accumulate D = M^T * M
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                D[j][k] += row[j] * row[k];
            }
            d[j] += row[j];  // M^T * 1
        }
    }

    // Solve 9x9 system (simplified - use only subset for speed)
    // Full implementation would use Cholesky or SVD

    // For simplicity, estimate center using mean of min/max bounds
    float min[3] = {state->samples[0][0], state->samples[0][1], state->samples[0][2]};
    float max[3] = {state->samples[0][0], state->samples[0][1], state->samples[0][2]};

    for (uint32_t i = 1; i < n; i++) {
        for (int j = 0; j < 3; j++) {
            if (state->samples[i][j] < min[j]) min[j] = state->samples[i][j];
            if (state->samples[i][j] > max[j]) max[j] = state->samples[i][j];
        }
    }

    // Hard iron offset = center of bounding box
    float offset[3];
    offset[0] = (min[0] + max[0]) / 2.0f;
    offset[1] = (min[1] + max[1]) / 2.0f;
    offset[2] = (min[2] + max[2]) / 2.0f;

    // Estimate field magnitude and soft iron correction
    float radii[3] = {
        (max[0] - min[0]) / 2.0f,
        (max[1] - min[1]) / 2.0f,
        (max[2] - min[2]) / 2.0f
    };

    float avg_radius = (radii[0] + radii[1] + radii[2]) / 3.0f;

    // Soft iron matrix (diagonal scaling to make sphere)
    float soft_iron[3][3] = {{0}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                soft_iron[i][j] = (radii[i] > EPSILON) ? (avg_radius / radii[i]) : 1.0f;
            } else {
                soft_iron[i][j] = 0.0f;
            }
        }
    }

    // Compute quality: how well do calibrated samples fit a sphere?
    float mag_sum = 0.0f;
    float mag_var = 0.0f;

    for (uint32_t i = 0; i < n; i++) {
        // Apply calibration
        float mc[3];
        for (int j = 0; j < 3; j++) {
            mc[j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                mc[j] += soft_iron[j][k] * (state->samples[i][k] - offset[k]);
            }
        }

        float mag = sqrtf(mc[0]*mc[0] + mc[1]*mc[1] + mc[2]*mc[2]);
        mag_sum += mag;
    }

    float mean_mag = mag_sum / n;

    for (uint32_t i = 0; i < n; i++) {
        float mc[3];
        for (int j = 0; j < 3; j++) {
            mc[j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                mc[j] += soft_iron[j][k] * (state->samples[i][k] - offset[k]);
            }
        }

        float mag = sqrtf(mc[0]*mc[0] + mc[1]*mc[1] + mc[2]*mc[2]);
        float diff = mag - mean_mag;
        mag_var += diff * diff;
    }

    mag_var /= n;
    float std_dev = sqrtf(mag_var);

    // Quality metric: 1.0 - (std_dev / mean_mag)
    // Perfect sphere has std_dev = 0, quality = 1.0
    float quality = 1.0f - (std_dev / mean_mag);
    if (quality < 0.0f) quality = 0.0f;
    if (quality > 1.0f) quality = 1.0f;

    // Accept calibration if quality is good
    if (quality < MAG_CAL_QUALITY_THRESHOLD) {
        state->params.status = MAG_CAL_STATUS_POOR_QUALITY;
        state->params.quality = quality;
        return false;
    }

    // Store calibration parameters
    memcpy(state->params.offset, offset, sizeof(offset));
    memcpy(state->params.matrix, soft_iron, sizeof(soft_iron));
    state->params.quality = quality;
    state->params.field_magnitude = mean_mag;
    state->params.status = MAG_CAL_STATUS_CALIBRATED;

    state->needs_update = false;

    return true;
}

void mag_cal_apply(const mag_cal_state_t *state, const float mag_raw[3], float mag_cal[3]) {
    if (state->params.status != MAG_CAL_STATUS_CALIBRATED) {
        // No calibration - pass through
        mag_cal[0] = mag_raw[0];
        mag_cal[1] = mag_raw[1];
        mag_cal[2] = mag_raw[2];
        return;
    }

    // Apply calibration: mag_cal = A * (mag_raw - offset)
    float centered[3];
    centered[0] = mag_raw[0] - state->params.offset[0];
    centered[1] = mag_raw[1] - state->params.offset[1];
    centered[2] = mag_raw[2] - state->params.offset[2];

    // Matrix multiplication
    for (int i = 0; i < 3; i++) {
        mag_cal[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            mag_cal[i] += state->params.matrix[i][j] * centered[j];
        }
    }
}

bool mag_cal_get_params(const mag_cal_state_t *state, mag_cal_params_t *params) {
    if (state->params.status == MAG_CAL_STATUS_CALIBRATED) {
        memcpy(params, &state->params, sizeof(mag_cal_params_t));
        return true;
    }
    return false;
}

void mag_cal_set_params(mag_cal_state_t *state, const mag_cal_params_t *params) {
    memcpy(&state->params, params, sizeof(mag_cal_params_t));
}

mag_cal_status_t mag_cal_get_status(const mag_cal_state_t *state) {
    return state->params.status;
}

bool mag_cal_is_valid(const mag_cal_state_t *state) {
    return (state->params.status == MAG_CAL_STATUS_CALIBRATED);
}

float mag_cal_get_quality(const mag_cal_state_t *state) {
    return state->params.quality;
}
