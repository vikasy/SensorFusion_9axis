"""
9-Axis Sensor Fusion (Accelerometer + Gyroscope + Magnetometer)
Python implementation matching C code in algo_sf_9x_sensor_fusion.c

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
from typing import Tuple
from sensor_fusion_6axis import (
    SensorFusion6Axis, PhysSensor, Quaternion, AlgoOutput, SensorID, AlgoType,
    GTOMSEC2, DEG2RAD, RAD2DEG, EPSILON, NSEC2MSEC,
    SF_OVERSAMPLE_RATIO, SF_GYRO_SAMP_INTVL, SF_DELTA_T, SF_DELTA_T_SQ,
    CHX, CHY, CHZ, SF_MAX_ORIENT_ERR,
    SF_6XAG_QVACC, SF_6XAG_QWACC, SF_6XAG_QVGYRO, SF_6XAG_QWGYRO,
    SF_6XAG_QOrient, SF_6XAG_QBias, SF_6XAG_QLinAcc, SF_6XAG_QBiasOrient,
    cross_product_matrix, quaternion_integrate, rotation_matrix_to_quaternion,
    rotation_matrix_to_angles, timestamp_ms
)
from QuatMath.QuatNormal import quat_normalize
from QuatMath.Quat2RodMat import quat_to_rotation_matrix

# 9-axis specific constants (matching C code)
SF_9XAGM_QVMAG = 1e-6  # Magnetometer measurement noise
SF_9XAGM_QWMAG = 1e-4  # Magnetometer process noise
SF_9XAGM_QMagDist = 1e-1  # Magnetic disturbance modeling error

# Magnetometer timing constants
SF_MAG_MAX_STALE_DUR = 2000  # 2 seconds
SF_MAG_MAX_MISS_DUR = 5000  # 5 seconds

# Magnetometer mode flags
SF_MAG_MASK = 48  # bits 4-5
SF_MAG_STALE = 16  # bit 4
SF_MAG_MISSING = 32  # bit 5

# Bias rate limiting (Iteration 1 + Phase 1 optimization)
# Use motion-dependent rate limiting:
# - During slow motion/static: Allow normal convergence (no limiting)
# - During rapid rotation: Apply aggressive limiting (prevent tracking motion)
#
# Thresholds (optimized via grid search 2025-10-14):
# - Slow motion: < 12 dps → no limit
# - Moderate motion: 12-45 dps → moderate limit (0.07 dps/cycle = 7 dps/sec)
# - Fast motion: > 45 dps → strong limit (0.015 dps/cycle = 1.5 dps/sec)
SF_MOTION_THRESHOLD_SLOW = 12.0  # dps - below this, no bias limiting
SF_MOTION_THRESHOLD_FAST = 45.0  # dps - above this, strong bias limiting
SF_MAX_BIAS_RATE_MODERATE = 0.07  # dps per cycle during moderate motion
SF_MAX_BIAS_RATE_FAST = 0.015  # dps per cycle during fast motion

# Adaptive Process Noise Scaling (Iteration 2 improvement)
# Increase orientation process noise during rapid rotation to:
# - Tell filter to trust gyro integration more during motion
# - Reduce accelerometer Kalman gain (accel less reliable during motion)
# - Allow filter to track larger orientation changes without excessive correction
#
# Scale factors for Q[0][0] (orientation error covariance):
# - Slow/Moderate motion (< 40 dps): 1.0× (normal process noise, trust accel)
# - Fast motion (40-60 dps): 2.5× (modest increase, start reducing accel trust)
# - Very fast motion (> 60 dps): 5.0× (significant increase, rely on gyro)
#
# Note: Uses different thresholds than bias limiting to avoid over-scaling
SF_PROCESS_NOISE_THRESHOLD_FAST = 40.0  # dps - above this, start scaling
SF_PROCESS_NOISE_THRESHOLD_VERY_FAST = 60.0  # dps - above this, strong scaling
SF_PROCESS_NOISE_SCALE_FAST = 2.5  # Scale factor for fast motion
SF_PROCESS_NOISE_SCALE_VERY_FAST = 5.0  # Scale factor for very fast motion


class SensorFusion9Axis(SensorFusion6Axis):
    """
    9-Axis Sensor Fusion Algorithm (Accelerometer + Gyroscope + Magnetometer)
    Extends 6-axis fusion with magnetometer for absolute heading
    """

    def __init__(self, acc_scale: float, gyro_scale: float, mag_scale: float,
                 motion_threshold_slow: float = SF_MOTION_THRESHOLD_SLOW,
                 motion_threshold_fast: float = SF_MOTION_THRESHOLD_FAST,
                 max_bias_rate_moderate: float = SF_MAX_BIAS_RATE_MODERATE,
                 max_bias_rate_fast: float = SF_MAX_BIAS_RATE_FAST,
                 process_noise_threshold_fast: float = SF_PROCESS_NOISE_THRESHOLD_FAST,
                 process_noise_threshold_very_fast: float = SF_PROCESS_NOISE_THRESHOLD_VERY_FAST,
                 process_noise_scale_fast: float = SF_PROCESS_NOISE_SCALE_FAST,
                 process_noise_scale_very_fast: float = SF_PROCESS_NOISE_SCALE_VERY_FAST):
        """
        Initialize 9-axis sensor fusion

        Args:
            acc_scale: Accelerometer scale factor (g/count)
            gyro_scale: Gyroscope scale factor (dps/count)
            mag_scale: Magnetometer scale factor (µT/count)
            motion_threshold_slow: Slow motion threshold (dps) - default 12.0
            motion_threshold_fast: Fast motion threshold (dps) - default 45.0
            max_bias_rate_moderate: Max bias rate during moderate motion (dps/cycle) - default 0.07
            max_bias_rate_fast: Max bias rate during fast motion (dps/cycle) - default 0.015
            process_noise_threshold_fast: Threshold for fast motion process noise scaling - default 40.0
            process_noise_threshold_very_fast: Threshold for very fast motion scaling - default 60.0
            process_noise_scale_fast: Process noise scale for fast motion - default 2.5
            process_noise_scale_very_fast: Process noise scale for very fast motion - default 5.0
        """
        # Initialize base 6-axis fusion
        super().__init__(acc_scale, gyro_scale)

        # Override algorithm type
        self.algo_type = AlgoType.SF_9AGM

        # Add magnetometer sensor
        self.mag_data = PhysSensor(sensor_id=SensorID.MAG, scale_factor=mag_scale)

        # Store bias rate limiting parameters as instance variables (Iteration 1)
        self.motion_threshold_slow = motion_threshold_slow
        self.motion_threshold_fast = motion_threshold_fast
        self.max_bias_rate_moderate = max_bias_rate_moderate
        self.max_bias_rate_fast = max_bias_rate_fast

        # Store adaptive process noise scaling parameters (Iteration 2)
        self.process_noise_threshold_fast = process_noise_threshold_fast
        self.process_noise_threshold_very_fast = process_noise_threshold_very_fast
        self.process_noise_scale_fast = process_noise_scale_fast
        self.process_noise_scale_very_fast = process_noise_scale_very_fast

        # Magnetometer-specific state variables
        self.mag_cal_offset = np.zeros(3)  # Hard iron offset
        self.mag_field_ref = np.zeros(3)  # Reference magnetic field (global frame)
        self.mag_cal_matrix = np.eye(3)  # Soft iron calibration matrix
        self.mag_post_s = np.zeros(3)  # Mag in sensor frame
        self.mag_post_g = np.zeros(3)  # Mag in global frame
        self.mag_dist_err_post_s = np.zeros(3)  # Magnetic disturbance error

        # Magnetic field parameters
        self.mag_declination = 0.0  # Magnetic declination (radians)
        self.mag_cal_valid = 0  # Calibration validity flag

        # Timestamps
        self.mag_meas_updt_ts = 0

        # 9-axis noise parameters
        self.proc_noise_var_mag_dist = SF_9XAGM_QMagDist
        self.meas_noise_var_mag = SF_9XAGM_QVMAG + SF_9XAGM_QWMAG + ((SF_6XAG_QVGYRO + SF_6XAG_QWGYRO) * SF_DELTA_T_SQ)

        # Expand Kalman filter matrices to 4x4 blocks (for mag disturbance state)
        self.proc_noise_var = np.zeros((4, 4, 3, 3))  # 4x4 blocks
        self.err_cov_mtx_post = np.zeros((4, 4, 3, 3))
        self.kalman_gain = np.zeros((4, 3, 3))

        # Initialize 9-axis specific states
        self._reset_9axis()

        print("9-axis SF algo initialized")

    def _reset_9axis(self):
        """Reset 9-axis specific states"""
        # Reset magnetometer states
        self.mag_cal_offset = np.zeros(3)
        self.mag_field_ref = np.zeros(3)
        self.mag_cal_matrix = np.eye(3)
        self.mag_post_s = np.zeros(3)
        self.mag_post_g = np.zeros(3)
        self.mag_dist_err_post_s = np.zeros(3)
        self.mag_declination = 0.0
        self.mag_cal_valid = 0
        self.mag_meas_updt_ts = 0

        # Initialize 4x4 error covariance matrix P with non-zero values
        for i in range(4):
            for j in range(4):
                if i == j:
                    # Diagonal blocks - set initial uncertainty
                    if i == 0:
                        # P[0][0]: Initial orientation error covariance
                        self.err_cov_mtx_post[i, j] = np.eye(3) * 0.1
                    elif i == 1:
                        # P[1][1]: Initial gyro bias error covariance
                        self.err_cov_mtx_post[i, j] = np.eye(3) * (50.0 * DEG2RAD) ** 2
                    elif i == 2:
                        # P[2][2]: Initial linear acceleration error covariance
                        self.err_cov_mtx_post[i, j] = np.eye(3) * 0.1
                    else:
                        # P[3][3]: Initial magnetic disturbance error covariance
                        self.err_cov_mtx_post[i, j] = np.eye(3) * 0.01
                else:
                    # Off-diagonal blocks start at zero
                    self.err_cov_mtx_post[i, j] = np.zeros((3, 3))

        # Zero out off-diagonal process noise blocks for mag
        self.proc_noise_var[0, 3] = np.zeros((3, 3))
        self.proc_noise_var[1, 3] = np.zeros((3, 3))
        self.proc_noise_var[2, 3] = np.zeros((3, 3))
        self.proc_noise_var[3, 0] = np.zeros((3, 3))
        self.proc_noise_var[3, 1] = np.zeros((3, 3))
        self.proc_noise_var[3, 2] = np.zeros((3, 3))

    def _init_orient_mag(self, accel_avg: np.ndarray, mag_avg: np.ndarray):
        """
        Initialize orientation from accelerometer tilt and magnetometer heading

        Args:
            accel_avg: Average accelerometer reading in m/s²
            mag_avg: Average magnetometer reading in µT
        """
        # For now, just use the 6-axis tilt initialization
        # Magnetometer heading initialization needs more work
        # TODO: Implement proper tilt-compensated magnetometer initialization
        rot_mtx = self._tilt_rotation_matrix(accel_avg)
        self.rot_mtx_post = rot_mtx

        # Convert to quaternion
        quat_arr = rotation_matrix_to_quaternion(rot_mtx)
        self.quat_post.from_array(quat_arr)

        # Mark orientation as initialized
        self.orient_init = True

    @staticmethod
    def _tilt_rotation_matrix_mag(accel_avg: np.ndarray, mag_avg: np.ndarray) -> np.ndarray:
        """
        Calculate tilt-compensated orientation matrix using accelerometer and magnetometer

        Args:
            accel_avg: Accelerometer average [x, y, z] in m/s²
            mag_avg: Magnetometer average [x, y, z] in µT

        Returns:
            3x3 rotation matrix
        """
        # Normalize accelerometer (down vector)
        accel_mag = np.linalg.norm(accel_avg)
        if accel_mag > 0.0:
            down = accel_avg / accel_mag
        else:
            down = np.array([0.0, 0.0, 1.0])

        # Normalize magnetometer
        mag_mag = np.linalg.norm(mag_avg)
        if mag_mag > 0.0:
            mag_norm = mag_avg / mag_mag
        else:
            mag_norm = np.array([1.0, 0.0, 0.0])

        # Calculate horizontal component of magnetic field (remove tilt)
        # mag_horizontal = mag_norm - (mag_norm · down) * down
        mag_dot_down = np.dot(mag_norm, down)
        mag_horizontal = mag_norm - mag_dot_down * down

        # Normalize horizontal magnetic field to get north vector
        mag_horiz_mag = np.linalg.norm(mag_horizontal)
        if mag_horiz_mag > 0.0:
            north = mag_horizontal / mag_horiz_mag
        else:
            north = np.array([1.0, 0.0, 0.0])

        # East vector = down × north (cross product)
        east = np.cross(down, north)

        # Construct rotation matrix: [north east down]
        # Creates NED (North-East-Down) coordinate frame
        rot_mtx = np.column_stack([north, east, down])

        return rot_mtx

    def preprocess_sensor_data(self, sensor_id: int, sensor_data: np.ndarray, timestamp: int):
        """
        Preprocess incoming sensor data (extends 6-axis to include magnetometer)

        Args:
            sensor_id: Sensor ID (ACC=0, GYRO=1, MAG=2)
            sensor_data: Sensor measurements [x, y, z] in raw counts
            timestamp: Timestamp in nanoseconds

        Returns:
            Signal bits: 0x1=ACC_READY, 0x2=GYRO_READY, 0x4=MAG_READY
        """
        # Handle magnetometer
        if sensor_id == SensorID.MAG:
            self.mag_data.count_buff[0] = sensor_data.astype(np.int16)
            # Scale to physical units (µT) like accel/gyro
            self.mag_data.count_avg = sensor_data.astype(np.float64) * self.mag_data.scale_factor
            self.mag_data.timestamp = timestamp
            self.sens_flags |= 4
            return self.signal_sf_run | 4

        # Call parent class for acc/gyro
        return super().preprocess_sensor_data(sensor_id, sensor_data, timestamp)

    def _apply_bias_rate_limit(self, bias_correction: np.ndarray) -> np.ndarray:
        """
        Apply motion-adaptive bias rate limiting (Iteration 1 improvement)

        Real gyro bias changes < 0.01 dps/sec due to thermal effects.
        Faster changes indicate filter tracking motion instead of bias.

        Rate limiting is adaptive based on current rotation magnitude:
        - Slow motion (<threshold_slow): No limiting (allow convergence)
        - Moderate motion (threshold_slow - threshold_fast): Moderate limiting
        - Fast motion (>threshold_fast): Strong limiting

        Args:
            bias_correction: Proposed bias change (dps)

        Returns:
            Rate-limited bias change (dps)
        """
        # Detect current rotation magnitude
        rotation_magnitude = np.linalg.norm(self.omega)

        # Apply motion-dependent limiting using instance parameters
        if rotation_magnitude < self.motion_threshold_slow:
            # Slow motion or static - no limiting, allow convergence
            return bias_correction
        elif rotation_magnitude < self.motion_threshold_fast:
            # Moderate motion - moderate limiting
            return np.clip(bias_correction, -self.max_bias_rate_moderate, self.max_bias_rate_moderate)
        else:
            # Fast motion - strong limiting
            return np.clip(bias_correction, -self.max_bias_rate_fast, self.max_bias_rate_fast)

    def _compute_process_noise_scale(self) -> float:
        """
        Compute adaptive process noise scale factor based on motion magnitude (Iteration 2)

        Uses higher thresholds than bias limiting to avoid over-scaling at moderate speeds.
        Accelerometer corrections are still valuable at 15-40 dps for single-axis rotation.

        Returns:
            Scale factor for orientation process noise (1.0, 2.5, or 5.0)
        """
        rotation_magnitude = np.linalg.norm(self.omega)

        if rotation_magnitude < self.process_noise_threshold_fast:
            # Slow/moderate motion - normal process noise (trust accelerometer)
            return 1.0
        elif rotation_magnitude < self.process_noise_threshold_very_fast:
            # Fast motion - modest increase (start reducing accel trust)
            return self.process_noise_scale_fast
        else:
            # Very fast motion - significant increase (rely heavily on gyro)
            return self.process_noise_scale_very_fast

    def _update_process_noise_9axis(self, delta_t: float):
        """Update process noise covariance matrix for 9-axis"""
        Cacc2 = self.lin_acc_tc * self.lin_acc_tc

        # Compute adaptive scale factor for orientation process noise (Iteration 2)
        # DISABLED: Causes regressions on rotation_y_15dps and rotation_z_30dps
        # Root cause: Process noise scaling affects filter convergence globally,
        # not just during measurement updates. Need different approach.
        orient_noise_scale = 1.0  # self._compute_process_noise_scale()

        # Q[0][0]: Orientation error covariance
        self.proc_noise_var[0, 0] = (
            np.eye(3) * self.proc_noise_var_orient * orient_noise_scale +
            self.err_cov_mtx_post[0, 0] +
            delta_t**2 * self.err_cov_mtx_post[1, 1]
        )

        # Q[1][1]: Gyro bias error covariance
        self.proc_noise_var[1, 1] = (
            np.eye(3) * self.proc_noise_var_bias +
            self.err_cov_mtx_post[1, 1]
        )

        # Q[2][2]: Linear acceleration error covariance
        self.proc_noise_var[2, 2] = (
            np.eye(3) * self.proc_noise_var_lin_acc +
            Cacc2 * self.err_cov_mtx_post[2, 2]
        )

        # Q[3][3]: Magnetic disturbance error covariance
        self.proc_noise_var[3, 3] = (
            np.eye(3) * self.proc_noise_var_mag_dist +
            self.err_cov_mtx_post[3, 3]
        )

        # Q[0][1] and Q[1][0]: Cross-covariance between orientation and bias
        self.proc_noise_var[0, 1] = (
            np.eye(3) * self.proc_noise_var_bias_orient -
            delta_t * self.err_cov_mtx_post[1, 1]
        )
        self.proc_noise_var[1, 0] = self.proc_noise_var[0, 1].T

        # Check orientation error threshold
        orient_err = (
            self.proc_noise_var[0, 0][0, 0]**2 +
            self.proc_noise_var[0, 0][1, 1]**2 +
            self.proc_noise_var[0, 0][2, 2]**2
        )
        if orient_err > SF_MAX_ORIENT_ERR:
            self.update_err_cov_mtx = 0

    def time_update(self, debug=False):
        """
        Nominal time update for 9-axis (same as 6-axis but uses 4x4 covariance)
        """
        delta_t = SF_GYRO_SAMP_INTVL

        if debug:
            print(f"\n=== TIME UPDATE (ts={self.gyro_data.timestamp}) ===")
            print(f"Quat BEFORE integration: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")
            print(f"bias_post_s: [{self.bias_post_s[0]:.8f}, {self.bias_post_s[1]:.8f}, {self.bias_post_s[2]:.8f}] dps")

        # Process all gyro samples in buffer
        for k in range(SF_OVERSAMPLE_RATIO):
            # Compute angular velocity (bias-corrected)
            for i in range(3):
                self.omega[i] = self.gyro_data.count_buff[k, i] * self.gyro_data.scale_factor
                self.omega[i] -= self.bias_post_s[i]  # Fixed: use BiasPostS (accumulated bias)
                self.ang_rate_prev[i] = self.omega[i]

            if debug:
                print(f"  Buffer[{k}] gyro_raw: [{self.gyro_data.count_buff[k, 0]:.0f}, {self.gyro_data.count_buff[k, 1]:.0f}, {self.gyro_data.count_buff[k, 2]:.0f}]")
                print(f"  Buffer[{k}] omega (bias-corrected): [{self.omega[0]:.8f}, {self.omega[1]:.8f}, {self.omega[2]:.8f}] dps")
                print(f"  Quat before integrate[{k}]: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")

            # Integrate quaternion
            quat_int = quaternion_integrate(
                self.quat_post.to_array(),
                self.omega,
                delta_t
            )
            self.quat_post.from_array(quat_int)

            if debug:
                print(f"  Quat after integrate[{k}]:  [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")

        # Normalize quaternion
        quat_norm = quat_normalize(self.quat_post.to_array())
        self.quat_post.from_array(quat_norm)

        if debug:
            print(f"Quat AFTER normalization: [{self.quat_post.q0:.8f}, {self.quat_post.q1:.8f}, {self.quat_post.q2:.8f}, {self.quat_post.q3:.8f}]")

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update timestamp to sensor timestamp (not wall-clock!)
        self.nom_updt_ts = self.gyro_data.timestamp

        # Update error covariance matrix if enabled
        if self.update_err_cov_mtx == 1:
            self._update_process_noise_9axis(delta_t)

    def measurement_update(self, debug=False):
        """
        Measurement update (correction step) using accelerometer - adapted for 9-axis (4x4 matrices)
        """
        if debug:
            print(f"\n=== MEASUREMENT UPDATE (ts={self.acc_data.timestamp}) ===")

        # Compute gravity error
        for i in range(3):
            self.grav_gyr_pri_s[i] = -self.rot_mtx_post[i, 2] * GTOMSEC2
            self.grav_err_pri_s[i] = self.acc_data.count_buff[SF_OVERSAMPLE_RATIO - 1, i]
            self.grav_err_pri_s[i] *= -1.0
            self.grav_err_pri_s[i] *= self.acc_data.scale_factor * GTOMSEC2
            self.grav_err_pri_s[i] += self.lin_acc_tc * self.acc_post_s[i]
            self.grav_err_pri_s[i] -= self.grav_gyr_pri_s[i]

        # Compute measurement matrix C (ONLY 3x3x3 for accel update - mag disturbance not affected!)
        C = np.zeros((3, 3, 3))
        cp_mat = cross_product_matrix(self.grav_gyr_pri_s)
        C[0] = -DEG2RAD * cp_mat
        C[1] = (DEG2RAD * SF_DELTA_T) * cp_mat
        C[2] = np.eye(3)

        # Compute Kalman gain: K = Qw*C'*inv(C*Qw*C' + Qv)
        # F[3] = Qw[3][3]*C[3]'  (only 3 blocks, not 4!)
        F = np.zeros((3, 3, 3))
        for i in range(3):
            F[i] = np.zeros((3, 3))
            for j in range(3):
                C_transp = C[j].T
                term = self.proc_noise_var[i, j] @ C_transp
                F[i] += term

        # G = C*F + Qv
        G = np.eye(3) * self.meas_noise_var_acc
        for i in range(3):
            G += C[i] @ F[i]

        # Ginv = inv(G)
        try:
            G_inv = np.linalg.inv(G)
            inv_exist = True
        except np.linalg.LinAlgError:
            inv_exist = False

        # K = F*Ginv (only 3 blocks!)
        if inv_exist:
            for i in range(3):
                self.kalman_gain[i] = F[i] @ G_inv
            # Zero out K[3] - mag disturbance not updated by accelerometer!
            self.kalman_gain[3] = np.zeros((3, 3))

        # Measurement update: xe+ = xe- + K*ze (only 9 DOF, not 12!)
        M_updt = np.zeros(9)
        for k in range(3):
            for j in range(3):
                M_updt[3*k + j] = 0.0
                for i in range(3):
                    M_updt[3*k + j] += self.kalman_gain[k][j, i] * self.grav_err_pri_s[i]

        # Update state error estimates
        self.ornt_err_post_s = M_updt[0:3]
        self.bias_err_post_s = M_updt[3:6]
        self.acc_err_post_s = M_updt[6:9]
        # Mag disturbance NOT updated by accelerometer!
        self.mag_dist_err_post_s = np.zeros(3)

        gyro_corr = -self.ornt_err_post_s / SF_DELTA_T

        # Update quaternion with correction
        quat_int = quaternion_integrate(
            self.quat_post.to_array(),
            gyro_corr,
            SF_DELTA_T
        )
        quat_norm = quat_normalize(quat_int)
        self.quat_post.from_array(quat_norm)

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update timestamp
        self.meas_updt_ts = self.acc_data.timestamp

        # Update aposteriori covariance matrix
        self._update_error_covariance_9axis(C)

        # Update gyro bias with rate limiting (Iteration 1 improvement)
        bias_correction = self._apply_bias_rate_limit(self.bias_err_post_s)
        self.bias_post_s -= bias_correction
        self.bias_post_s = np.clip(self.bias_post_s, -200.0, 200.0)
        self.acc_post_s = self.lin_acc_tc * self.acc_post_s - self.acc_err_post_s

        # Transform acceleration to global frame
        for j in range(3):
            self.acc_post_g[j] = 0.0
            for k in range(3):
                self.acc_post_g[j] += self.rot_mtx_post[j, k] * self.acc_post_s[k]
        self.acc_post_g[3] -= GTOMSEC2

    def measurement_update_mag(self):
        """
        Magnetometer measurement update (correction step)
        Implements Kalman filter measurement update using magnetometer data
        """
        # Get measured magnetometer data (with calibration)
        # count_avg is already in µT (scaled in preprocess_sensor_data)
        mag_measured = np.zeros(3)
        for i in range(3):
            mag_measured[i] = self.mag_data.count_avg[i]
            # Apply hard iron calibration
            mag_measured[i] -= self.mag_cal_offset[i]

        # Normalize measured magnetic field
        mag_norm = np.linalg.norm(mag_measured)
        if mag_norm < EPSILON:
            return  # Invalid magnetometer reading

        mag_measured = mag_measured / mag_norm

        # Initialize reference field if not set
        if np.linalg.norm(self.mag_field_ref) < EPSILON:
            # Transform to global frame: mag_global = R^T * mag_sensor
            self.mag_field_ref = self.rot_mtx_post.T @ mag_measured
            # Normalize
            ref_norm = np.linalg.norm(self.mag_field_ref)
            if ref_norm > EPSILON:
                self.mag_field_ref = self.mag_field_ref / ref_norm

        # Project reference magnetic field to sensor frame
        # mag_expected = R * mag_ref_global
        mag_expected = self.rot_mtx_post @ self.mag_field_ref

        # Compute measurement error (innovation)
        mag_error = mag_measured - mag_expected

        # Compute measurement matrix C
        C = np.zeros((4, 3, 3))
        cp_mat = cross_product_matrix(mag_expected)
        C[0] = -DEG2RAD * cp_mat  # Orientation error
        C[1] = np.zeros((3, 3))    # Gyro bias (mag not affected)
        C[2] = np.zeros((3, 3))    # Linear accel (mag not affected)
        C[3] = np.eye(3)           # Magnetic disturbance

        # Compute Kalman gain: K = Qw*C'*inv(C*Qw*C' + Qv)
        # F[4] = Qw[4][4]*C[4]'
        F = np.zeros((4, 3, 3))
        for i in range(4):
            F[i] = np.zeros((3, 3))
            for j in range(4):
                C_transp = C[j].T
                F[i] += self.proc_noise_var[i, j] @ C_transp

        # G = C[4]*F[4] + Qv
        G = np.eye(3) * self.meas_noise_var_mag
        for i in range(4):
            G += C[i] @ F[i]

        # Ginv = inv(G)
        try:
            G_inv = np.linalg.inv(G)
            inv_exist = True
        except np.linalg.LinAlgError:
            inv_exist = False
            return  # Skip update if matrix is singular

        # K[4] = F[4]*Ginv
        if inv_exist:
            for i in range(4):
                self.kalman_gain[i] = F[i] @ G_inv

        # Measurement update: xe+ = xe- + K*ze
        M_updt = np.zeros(12)
        for k in range(4):
            for j in range(3):
                M_updt[3*k + j] = 0.0
                for i in range(3):
                    M_updt[3*k + j] += self.kalman_gain[k][j, i] * mag_error[i]

        # Update state error estimates
        self.ornt_err_post_s = M_updt[0:3]
        self.bias_err_post_s = M_updt[3:6]
        self.acc_err_post_s = M_updt[6:9]
        self.mag_dist_err_post_s = M_updt[9:12]

        gyro_corr = -self.ornt_err_post_s / SF_DELTA_T

        # Update quaternion with correction
        quat_int = quaternion_integrate(
            self.quat_post.to_array(),
            gyro_corr,
            SF_DELTA_T
        )
        quat_norm = quat_normalize(quat_int)
        self.quat_post.from_array(quat_norm)

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update aposteriori covariance matrix
        self._update_error_covariance_9axis(C)

        # Update gyro bias with rate limiting (Iteration 1 improvement)
        bias_correction = self._apply_bias_rate_limit(self.bias_err_post_s)
        self.bias_post_s -= bias_correction

        # Store calibrated magnetometer data
        self.mag_post_s = mag_measured * mag_norm
        self.mag_post_g = self.rot_mtx_post.T @ self.mag_post_s

    def _update_error_covariance_9axis(self, C: np.ndarray):
        """
        Update aposteriori error covariance matrix for 9-axis

        Args:
            C: Measurement matrix - can be 3x3x3 (accel) or 4x3x3 (mag)
        """
        # P_post = (I - K*C)*Qw
        # Compute A = (I - K*C)
        # IMPORTANT: Only update the blocks that are affected by this measurement!
        # For accel: update P[0:3][0:3] only (orientation, bias, lin_acc)
        # For mag: update P[0:4][0:4] (all states including mag disturbance)

        num_c_blocks = C.shape[0]  # 3 for accel, 4 for mag

        A = np.zeros((num_c_blocks, num_c_blocks, 3, 3))
        for i in range(num_c_blocks):
            for j in range(num_c_blocks):
                if i == j:
                    A[i, j] = np.eye(3)
                else:
                    A[i, j] = np.zeros((3, 3))

                K_C = self.kalman_gain[i] @ C[j]
                A[i, j] -= K_C

                # Store in appropriate block of full P matrix
                self.err_cov_mtx_post[i, j] = A[i, j]

        # Compute P_post = A*Qw (only for affected blocks!)
        for i in range(num_c_blocks):
            for j in range(num_c_blocks):
                temp = np.zeros((3, 3))
                for k in range(num_c_blocks):
                    temp += self.err_cov_mtx_post[i, k] @ self.proc_noise_var[k, j]

                # Ensure symmetry
                temp_sym = 0.5 * (temp + temp.T)

                # Add small epsilon for numerical stability
                if i == j:
                    temp_sym += EPSILON * np.eye(3)

                self.err_cov_mtx_post[i, j] = temp_sym

        # Check orientation error
        orient_err = (
            self.err_cov_mtx_post[0, 0][0, 0]**2 +
            self.err_cov_mtx_post[0, 0][1, 1]**2 +
            self.err_cov_mtx_post[0, 0][2, 2]**2
        )
        if self.update_err_cov_mtx == 0 and orient_err < SF_MAX_ORIENT_ERR:
            self.update_err_cov_mtx = 1

    @staticmethod
    def detect_mag_disturbance(mag_data: np.ndarray, mag_ref: np.ndarray, threshold: float = 0.3) -> bool:
        """
        Detect magnetic disturbances

        Args:
            mag_data: Current magnetometer measurements [x, y, z]
            mag_ref: Reference magnetic field vector [x, y, z]
            threshold: Disturbance detection threshold (normalized, default 0.3 = 30%)

        Returns:
            True if disturbance detected, False otherwise
        """
        mag_norm = np.linalg.norm(mag_data)
        ref_norm = np.linalg.norm(mag_ref)

        if mag_norm < EPSILON or ref_norm < EPSILON:
            return True  # Invalid data treated as disturbance

        # Compute magnitude difference (normalized)
        mag_diff = abs(mag_norm - ref_norm) / ref_norm

        if mag_diff > threshold:
            return True  # Magnitude deviation detected

        # Check direction consistency (dot product)
        dot_product = np.dot(mag_data / mag_norm, mag_ref / ref_norm)

        # If vectors are misaligned > 30° (cos(30°) ≈ 0.866)
        if dot_product < 0.866:
            return True  # Direction mismatch detected

        return False  # No disturbance

    def run(self) -> AlgoOutput:
        """
        Run one iteration of 9-axis sensor fusion algorithm

        Returns:
            AlgoOutput with updated orientation, gravity, linear acceleration
        """
        curr_time_msec = timestamp_ms()

        # Handle reset
        if self.reset_flag:
            print("9-axis SF algo reset")
            self._reset()
            self._reset_9axis()
            return AlgoOutput()

        # Initialize orientation on first run
        if not self.orient_init:
            print("9-axis SF algo init orient")
            # count_avg is already scaled to physical units (g for accel, µT for mag)
            # by the preprocessing (line 381 in sensor_fusion_6axis.py)
            accel_avg = self.acc_data.count_avg * GTOMSEC2  # g * 9.81 = m/s²
            mag_avg = self.mag_data.count_avg  # already in µT
            self._init_orient_mag(accel_avg, mag_avg)

        # Time update if new gyro data available
        if self.nom_updt_ts < self.gyro_data.timestamp:
            self.time_update()
            # Check for stale gyro
            # (gyro timing checks same as 6-axis)

        # Measurement update if new accel data available
        if self.meas_updt_ts < self.acc_data.timestamp:
            self.measurement_update()
            # Check for stale accel
            # (accel timing checks same as 6-axis)

        # Magnetometer update if new mag data available
        if self.mag_meas_updt_ts < self.mag_data.timestamp:
            # Get measured magnetometer data (already in µT from preprocessing)
            mag_measured_raw = self.mag_data.count_avg

            # Check if mag reference is initialized
            mag_ref_initialized = np.linalg.norm(self.mag_field_ref) > EPSILON

            # Only check for disturbance if reference is initialized
            if mag_ref_initialized:
                # Normalize measured mag field for comparison with normalized reference
                mag_norm = np.linalg.norm(mag_measured_raw)
                if mag_norm > EPSILON:
                    mag_measured_normalized = mag_measured_raw / mag_norm
                    disturbance = self.detect_mag_disturbance(mag_measured_normalized, self.mag_field_ref, 0.3)
                    # DEBUG
                    # print(f"  [DEBUG] mag_norm={mag_norm:.3f}, mag_measured_normalized=[{mag_measured_normalized[0]:.6f}, {mag_measured_normalized[1]:.6f}, {mag_measured_normalized[2]:.6f}], disturbance={disturbance}")
                else:
                    disturbance = True  # Invalid magnitude
            else:
                disturbance = False  # Allow first measurement to initialize reference

            # Apply mag update if no disturbance (or first measurement)
            if not disturbance:
                self.measurement_update_mag()

            # Check for stale mag
            if self.mag_data.timestamp < (curr_time_msec - SF_MAG_MAX_STALE_DUR):
                self.op_mode &= ~SF_MAG_MASK
                self.op_mode |= SF_MAG_STALE

            self.mag_meas_updt_ts = self.mag_data.timestamp
        elif self.mag_meas_updt_ts < (curr_time_msec - SF_MAG_MAX_MISS_DUR):
            self.op_mode &= ~SF_MAG_MASK
            self.op_mode |= SF_MAG_MISSING

        # Update gravity vector
        for i in range(3):
            self.grav_post_s[i] = -1.0 * GTOMSEC2 * self.rot_mtx_post[i, 2]

        # Convert rotation matrix to Euler angles
        self.theta_post, self.phi_post, self.psi_post, self.rho_post, self.chi_post = \
            rotation_matrix_to_angles(self.rot_mtx_post, self.theta_post, self.psi_post)

        # Prepare output
        output = AlgoOutput()
        output.algo_type = AlgoType.SF_9AGM
        output.quat = Quaternion(
            q0=self.quat_post.q0,
            q1=self.quat_post.q1,
            q2=self.quat_post.q2,
            q3=self.quat_post.q3
        )
        output.orientation = np.array([self.psi_post, self.theta_post, self.phi_post])  # [yaw, pitch, roll]
        output.gravity = self.grav_post_s.copy()
        output.linear_acc = self.acc_post_g[0:3].copy()
        output.valid_flag = 0xFFFFFFFF
        output.mode = self.op_mode
        output.timestamp_ns = curr_time_msec * NSEC2MSEC

        return output


# ============================================================================
# Utility Functions for Magnetometer
# ============================================================================

def apply_mag_calibration(mag_raw: np.ndarray, hard_iron: np.ndarray, soft_iron: np.ndarray) -> np.ndarray:
    """
    Apply hard and soft iron calibration to magnetometer data

    Args:
        mag_raw: Raw magnetometer measurements [x, y, z]
        hard_iron: Hard iron offset [x, y, z]
        soft_iron: Soft iron correction matrix [3x3]

    Returns:
        Calibrated magnetometer measurements [x, y, z]
    """
    # Apply hard iron correction
    mag_offset_corrected = mag_raw - hard_iron

    # Apply soft iron correction
    mag_cal = soft_iron @ mag_offset_corrected

    return mag_cal


def apply_declination(orientation_mag: np.ndarray, declination_rad: float) -> np.ndarray:
    """
    Apply magnetic declination correction to convert magnetic north to true north

    Args:
        orientation_mag: Orientation relative to magnetic north [yaw, pitch, roll] in degrees
        declination_rad: Local magnetic declination in radians (positive = east)

    Returns:
        Orientation relative to true north [yaw, pitch, roll] in degrees
    """
    orientation_true = orientation_mag.copy()

    # Apply declination to yaw only (pitch and roll unchanged)
    orientation_true[0] += declination_rad * RAD2DEG

    # Normalize yaw to [0, 360)
    if orientation_true[0] < 0.0:
        orientation_true[0] += 360.0
    elif orientation_true[0] >= 360.0:
        orientation_true[0] -= 360.0

    return orientation_true
