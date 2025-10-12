"""
6-Axis Sensor Fusion (Accelerometer + Gyroscope)
Python implementation matching C code in algo_sf_6x_sensor_fusion.c

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass, field
from enum import IntEnum

# Import quaternion and matrix math functions
from QuatMath.QuatNormal import quat_normalize
from QuatMath.QuatProduct import quat_product
from QuatMath.Quat2RodMat import quat_to_rotation_matrix
from QuatMath.Deg2Rad import deg_to_rad
from QuatMath.Rad2Deg import rad_to_deg


# Constants (matching C code)
class SensorID(IntEnum):
    ACC = 0
    GYRO = 1
    MAG = 2


class AlgoType(IntEnum):
    SF_6AG = 0
    SF_9AGM = 1


# Physical constants
GTOMSEC2 = 9.80665  # Standard gravity m/s²
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi
EPSILON = 1e-12
NSEC2MSEC = 1000000

# 6-axis noise parameters (matching C code)
SF_6XAG_QVACC = 2e-6
SF_6XAG_QWACC = 1e-4
SF_6XAG_QVGYRO = 0.01  # Radians
SF_6XAG_QWGYRO = 1e-9  # Radians
SF_6XAG_QOrient = 1e-1
SF_6XAG_QBias = 1e1
SF_6XAG_QLinAcc = 1e1
SF_6XAG_QBiasOrient = 1e-1

# Timing constants
SF_GYRO_MAX_STALE_DUR = 1000  # ms
SF_GYRO_MAX_MISS_DUR = 5000  # ms
SF_ACCEL_MAX_STALE_DUR = 1000  # ms
SF_ACCEL_MAX_MISS_DUR = 5000  # ms

# Operation mode flags
SF_GYRO_MASK = 3
SF_GYRO_STALE = 1
SF_GYRO_MISSING = 2
SF_ACC_MASK = 12
SF_ACC_STALE = 4
SF_ACC_MISSING = 8

SF_MAX_ORIENT_ERR = 100

# Sampling constants
SF_OVERSAMPLE_RATIO = 4  # Ratio of gyro/accel sampling frequency (must match C code)
SF_GYRO_FS = 100  # Hz
SF_GYRO_SAMP_INTVL = 1.0 / SF_GYRO_FS  # seconds
SF_DELTA_T = SF_GYRO_SAMP_INTVL
SF_DELTA_T_SQ = SF_DELTA_T * SF_DELTA_T

# Channel indices
CHX, CHY, CHZ = 0, 1, 2


@dataclass
class PhysSensor:
    """Physical sensor data structure"""
    sensor_id: int = 0
    scale_factor: float = 0.0
    count_buff: np.ndarray = field(default_factory=lambda: np.zeros((SF_OVERSAMPLE_RATIO, 3), dtype=np.int16))
    last_index: int = 0
    count_avg: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.int16))
    timestamp: int = 0  # nanoseconds


@dataclass
class Quaternion:
    """Quaternion structure"""
    q0: float = 1.0  # w (scalar)
    q1: float = 0.0  # x
    q2: float = 0.0  # y
    q3: float = 0.0  # z

    def to_array(self) -> np.ndarray:
        return np.array([self.q0, self.q1, self.q2, self.q3])

    def from_array(self, arr: np.ndarray):
        self.q0, self.q1, self.q2, self.q3 = arr[0], arr[1], arr[2], arr[3]


@dataclass
class AlgoOutput:
    """Algorithm output structure"""
    algo_type: int = AlgoType.SF_6AG
    quat: Quaternion = field(default_factory=Quaternion)
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [yaw, pitch, roll]
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -GTOMSEC2]))
    linear_acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    valid_flag: int = 0xFFFFFFFF
    mode: int = 0
    timestamp_ns: int = 0


class SensorFusion6Axis:
    """
    6-Axis Sensor Fusion Algorithm (Accelerometer + Gyroscope)
    Implements Extended Kalman Filter for orientation estimation
    """

    def __init__(self, acc_scale: float, gyro_scale: float):
        """
        Initialize 6-axis sensor fusion

        Args:
            acc_scale: Accelerometer scale factor (g/count)
            gyro_scale: Gyroscope scale factor (dps/count)
        """
        # Algorithm identification
        self.algo_type = AlgoType.SF_6AG

        # Sensor data
        self.acc_data = PhysSensor(sensor_id=SensorID.ACC, scale_factor=acc_scale)
        self.gyro_data = PhysSensor(sensor_id=SensorID.GYRO, scale_factor=gyro_scale)

        # State variables
        self.reset_flag = False
        self.orient_init = False
        self.op_mode = 0
        self.sens_flags = 0

        # Timestamps
        self.nom_updt_ts = 0
        self.meas_updt_ts = 0

        # Angular velocity and orientation
        self.ang_rate_prev = np.zeros(3)
        self.rot_mtx_post = np.eye(3)
        self.quat_post = Quaternion()
        self.omega = np.zeros(3)

        # Sample counters for oversampling buffer
        self.acc_count = 0
        self.gyro_count = 0
        self.signal_sf_run = 0  # Bit 0=ACC_READY, Bit 1=GYRO_READY

        # Orientation angles (degrees)
        self.phi_post = 0.0    # Roll
        self.theta_post = 0.0  # Pitch
        self.psi_post = 0.0    # Yaw
        self.rho_post = 0.0    # Compass heading
        self.chi_post = 0.0    # Tilt angle

        # Gravity and acceleration
        self.grav_gyr_pri_s = np.zeros(3)
        self.grav_err_pri_s = np.zeros(3)
        self.grav_post_s = np.array([0.0, 0.0, -GTOMSEC2])
        self.acc_post_s = np.zeros(3)
        self.acc_post_g = np.zeros(4)

        # Bias estimates
        self.bias_post_s = np.zeros(3)
        self.bias_err_post_s = np.zeros(3)

        # Error states
        self.ornt_err_post_s = np.zeros(3)
        self.acc_err_post_s = np.zeros(3)

        # Noise parameters
        self.proc_noise_var_orient = SF_6XAG_QOrient
        self.proc_noise_var_bias = SF_6XAG_QBias
        self.proc_noise_var_lin_acc = SF_6XAG_QLinAcc
        self.proc_noise_var_bias_orient = SF_6XAG_QBiasOrient
        self.meas_noise_var_acc = SF_6XAG_QVACC + SF_6XAG_QWACC + ((SF_6XAG_QVGYRO + SF_6XAG_QWGYRO) * SF_DELTA_T_SQ)

        # Linear acceleration time constant
        self.lin_acc_tc = 0.5

        # Kalman filter matrices (3x3 blocks for 6-axis: orientation, bias, lin_acc)
        self.proc_noise_var = np.zeros((3, 3, 3, 3))  # 3x3 blocks
        self.err_cov_mtx_post = np.zeros((3, 3, 3, 3))
        self.kalman_gain = np.zeros((3, 3, 3))
        self.update_err_cov_mtx = 1

        # Initialize
        self._reset()

        print("6-axis SF algo initialized")

    def _reset(self):
        """Reset algorithm state"""
        # Initialize quaternion to identity
        self.quat_post = Quaternion(q0=1.0, q1=0.0, q2=0.0, q3=0.0)
        self.rot_mtx_post = np.eye(3)

        # Initialize error covariance matrix to zero
        for i in range(3):
            for j in range(3):
                self.err_cov_mtx_post[i, j] = np.zeros((3, 3))

        # Zero out off-diagonal process noise blocks
        self.proc_noise_var[0, 2] = np.zeros((3, 3))
        self.proc_noise_var[1, 2] = np.zeros((3, 3))
        self.proc_noise_var[2, 0] = np.zeros((3, 3))
        self.proc_noise_var[2, 1] = np.zeros((3, 3))

        self.update_err_cov_mtx = 1

        # Initialize angles to zero
        self.phi_post = 0.0
        self.theta_post = 0.0
        self.psi_post = 0.0
        self.rho_post = 0.0
        self.chi_post = 0.0

        # Initialize gravity vector
        self.grav_post_s = np.array([0.0, 0.0, -GTOMSEC2])

        # Initialize linear acceleration to zero
        self.acc_post_g = np.zeros(4)
        self.acc_post_s = np.zeros(3)

        # Initialize gyro bias and errors to zero
        self.bias_post_s = np.zeros(3)
        self.bias_err_post_s = np.zeros(3)
        self.ornt_err_post_s = np.zeros(3)
        self.acc_err_post_s = np.zeros(3)
        self.omega = np.zeros(3)
        self.ang_rate_prev = np.zeros(3)

        # Initialize operation mode
        self.op_mode = 0
        self.sens_flags = 0
        self.nom_updt_ts = 0
        self.meas_updt_ts = 0
        self.orient_init = False

        # Clear reset flag
        self.reset_flag = False

    def _init_orient(self, accel_avg: np.ndarray):
        """
        Initialize orientation from accelerometer tilt

        Args:
            accel_avg: Average accelerometer reading in m/s²
        """
        # Initialize rotation matrix from tilt
        rot_mtx = self._tilt_rotation_matrix(accel_avg)
        self.rot_mtx_post = rot_mtx

        # Convert to quaternion
        quat_arr = rotation_matrix_to_quaternion(rot_mtx)
        self.quat_post.from_array(quat_arr)

        # Mark orientation as initialized
        self.orient_init = True

    @staticmethod
    def _tilt_rotation_matrix(accel_avg: np.ndarray) -> np.ndarray:
        """
        Calculate orientation matrix based on accelerometer tilt

        Args:
            accel_avg: Accelerometer average [x, y, z] in m/s²

        Returns:
            3x3 rotation matrix
        """
        # Normalize accelerometer (gravity direction in sensor frame)
        mag_grav = np.linalg.norm(accel_avg)
        if mag_grav < EPSILON:
            return np.eye(3)  # Return identity if no acceleration

        # Normalized gravity vector in sensor frame
        # For device at rest and level: gravity = [0, 0, -9.81]
        # which means down in sensor frame is +Z direction
        grav_norm = accel_avg / mag_grav

        # We want to find rotation matrix that transforms the standard
        # gravity vector [0, 0, -1] to the measured gravity direction

        # Third column of rotation matrix is the down direction
        # For level device with gravity in +Z: R[:,2] = grav_norm
        V3 = grav_norm

        # Find orthogonal vectors for X and Y axes
        # Choose X axis to be perpendicular to gravity, favoring [1,0,0]
        if abs(grav_norm[0]) < 0.9:
            # Use X axis as reference if not aligned with gravity
            V1 = np.array([1.0, 0.0, 0.0])
        else:
            # Use Y axis as reference if gravity is mostly in X direction
            V1 = np.array([0.0, 1.0, 0.0])

        # Make V1 perpendicular to V3 using Gram-Schmidt
        V1 = V1 - np.dot(V1, V3) * V3
        V1_norm = np.linalg.norm(V1)
        if V1_norm > EPSILON:
            V1 = V1 / V1_norm
        else:
            V1 = np.array([1.0, 0.0, 0.0])

        # V2 = V3 × V1 (cross product for right-handed system)
        V2 = np.cross(V3, V1)

        # Construct rotation matrix: R = [V1 V2 V3]
        rot_mtx = np.column_stack([V1, V2, V3])

        return rot_mtx

    def preprocess_sensor_data(self, sensor_id: int, sensor_data: np.ndarray, timestamp: int) -> int:
        """
        Preprocess incoming sensor data - implements oversampling buffer
        Collects SF_OVERSAMPLE_RATIO samples before signaling ready

        Args:
            sensor_id: Sensor ID (ACC=0, GYRO=1)
            sensor_data: Sensor measurements [x, y, z] in raw counts
            timestamp: Timestamp in nanoseconds

        Returns:
            Signal bits: 0x1=ACC_READY, 0x2=GYRO_READY, 0x3=BOTH_READY
        """
        ACC_READY_BIT = 1
        GYRO_READY_BIT = 2

        if sensor_id == SensorID.ACC:
            # Buffer accelerometer sample (C code: lines 66-92)
            self.acc_data.count_buff[self.acc_count] = sensor_data.astype(np.int16)
            if self.acc_count == 0:
                # Save timestamp of first sample in buffer
                self.acc_data.timestamp = timestamp

            self.acc_count += 1
            if self.acc_count == SF_OVERSAMPLE_RATIO:
                # Buffer full - compute average and signal ready
                self.acc_count = 0
                self.signal_sf_run |= ACC_READY_BIT

                # Compute weighted average (matching C code lines 79-91)
                avg_wt = self.acc_data.scale_factor / SF_OVERSAMPLE_RATIO
                for k in range(3):
                    self.acc_data.count_avg[k] = 0
                    for i in range(SF_OVERSAMPLE_RATIO):
                        self.acc_data.count_avg[k] += self.acc_data.count_buff[i, k]
                    # Rounding (matching C)
                    if self.acc_data.count_avg[k] > 0:
                        self.acc_data.count_avg[k] += SF_OVERSAMPLE_RATIO // 2
                    else:
                        self.acc_data.count_avg[k] -= SF_OVERSAMPLE_RATIO // 2
                    self.acc_data.count_avg[k] *= avg_wt

        elif sensor_id == SensorID.GYRO:
            # Buffer gyroscope sample (C code: lines 96-109)
            self.gyro_data.count_buff[self.gyro_count] = sensor_data.astype(np.int16)
            if self.gyro_count == 0:
                # Save timestamp of first sample in buffer
                self.gyro_data.timestamp = timestamp

            self.gyro_count += 1
            if self.gyro_count == SF_OVERSAMPLE_RATIO:
                # Buffer full - signal ready
                self.gyro_count = 0
                self.signal_sf_run |= GYRO_READY_BIT

        return self.signal_sf_run

    def time_update(self):
        """
        Nominal time update (prediction step) using gyroscope data
        Implements Kalman filter time propagation
        """
        delta_t = SF_GYRO_SAMP_INTVL

        # Process all gyro samples in buffer
        for k in range(SF_OVERSAMPLE_RATIO):
            # Compute angular velocity (bias-corrected) in deg/s
            for i in range(3):
                self.omega[i] = self.gyro_data.count_buff[k, i] * self.gyro_data.scale_factor
                self.omega[i] -= self.bias_post_s[i]
                self.ang_rate_prev[i] = self.omega[i]

            # Integrate quaternion
            quat_int = quaternion_integrate(
                self.quat_post.to_array(),
                self.omega,
                delta_t
            )
            self.quat_post.from_array(quat_int)

        # Normalize quaternion
        quat_norm = quat_normalize(self.quat_post.to_array())
        self.quat_post.from_array(quat_norm)

        # Update rotation matrix
        self.rot_mtx_post = quat_to_rotation_matrix(self.quat_post.to_array())

        # Update timestamp to match sensor timestamp
        self.nom_updt_ts = self.gyro_data.timestamp

        # Update error covariance matrix if enabled
        if self.update_err_cov_mtx == 1:
            self._update_process_noise(delta_t)

    def _update_process_noise(self, delta_t: float):
        """Update process noise covariance matrix"""
        Cacc2 = self.lin_acc_tc * self.lin_acc_tc

        # Q[0][0]: Orientation error covariance
        self.proc_noise_var[0, 0] = (
            np.eye(3) * self.proc_noise_var_orient +
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

        # Q[0][1] and Q[1][0]: Cross-covariance
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

    def measurement_update(self):
        """
        Measurement update (correction step) using accelerometer data
        Implements Kalman filter measurement update
        """
        # Compute gravity error
        # Following C code exactly:
        # GravErr = -AccCounts * Scale * G + LinAccTC * AccPostS - GravGyrPri
        for i in range(3):
            # Gravity from gyro (expected gravity in sensor frame)
            self.grav_gyr_pri_s[i] = -self.rot_mtx_post[i, 2] * GTOMSEC2

            # Compute gravity error directly (matching C code line-by-line)
            self.grav_err_pri_s[i] = self.acc_data.count_buff[SF_OVERSAMPLE_RATIO - 1, i]
            self.grav_err_pri_s[i] *= -1.0
            self.grav_err_pri_s[i] *= self.acc_data.scale_factor * GTOMSEC2
            self.grav_err_pri_s[i] += self.lin_acc_tc * self.acc_post_s[i]
            self.grav_err_pri_s[i] -= self.grav_gyr_pri_s[i]

        # Compute measurement matrix C using cross product matrix
        C = np.zeros((3, 3, 3))
        cp_mat = cross_product_matrix(self.grav_gyr_pri_s)
        C[0] = -DEG2RAD * cp_mat
        C[1] = (DEG2RAD * SF_DELTA_T) * cp_mat
        C[2] = np.eye(3)

        # Compute Kalman gain: K = Qw*C'*inv(C*Qw*C' + Qv)
        # F[3] = Qw[3][3]*C[3]'
        F = np.zeros((3, 3, 3))
        for i in range(3):
            F[i] = np.zeros((3, 3))
            for j in range(3):
                C_transp = C[j].T
                F[i] += self.proc_noise_var[i, j] @ C_transp

        # G = C[3]*F[3] + Qv
        G = np.eye(3) * self.meas_noise_var_acc
        for i in range(3):
            G += C[i] @ F[i]

        # Ginv = inv(G)
        try:
            G_inv = np.linalg.inv(G)
            inv_exist = True
        except np.linalg.LinAlgError:
            inv_exist = False

        # K[3] = F[3]*Ginv
        if inv_exist:
            for i in range(3):
                self.kalman_gain[i] = F[i] @ G_inv

        # Measurement update: xe+ = xe- + K*ze
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

        # Update timestamp to match sensor timestamp
        self.meas_updt_ts = self.acc_data.timestamp

        # Update aposteriori covariance matrix
        self._update_error_covariance(C)

        # Update gyro bias and linear acceleration
        self.bias_post_s -= self.bias_err_post_s
        self.acc_post_s = self.lin_acc_tc * self.acc_post_s - self.acc_err_post_s

        # Transform acceleration to global frame
        for j in range(3):
            self.acc_post_g[j] = 0.0
            for k in range(3):
                self.acc_post_g[j] += self.rot_mtx_post[j, k] * self.acc_post_s[k]
        self.acc_post_g[3] -= GTOMSEC2

    def _update_error_covariance(self, C: np.ndarray):
        """Update aposteriori error covariance matrix"""
        # P_post = (I - K*C)*Qw
        # Compute A = (I - K*C)
        A = np.zeros((3, 3, 3, 3))
        for i in range(3):
            for j in range(3):
                if i == j:
                    A[i, j] = np.eye(3)
                else:
                    A[i, j] = np.zeros((3, 3))

                K_C = self.kalman_gain[i] @ C[j]
                A[i, j] -= K_C
                self.err_cov_mtx_post[i, j] = A[i, j]

        # Compute P_post = A*Qw
        for i in range(3):
            for j in range(3):
                temp = np.zeros((3, 3))
                for k in range(3):
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

    def run(self) -> AlgoOutput:
        """
        Run one iteration of 6-axis sensor fusion algorithm

        Returns:
            AlgoOutput with updated orientation, gravity, linear acceleration
        """
        curr_time_msec = timestamp_ms()

        # Handle reset
        if self.reset_flag:
            print("6-axis SF algo reset")
            self._reset()
            return AlgoOutput()

        # Initialize orientation on first run
        if not self.orient_init:
            print("6-axis SF algo initial orientation lock")
            accel_avg = self.acc_data.count_avg * self.acc_data.scale_factor * GTOMSEC2
            self._init_orient(accel_avg)

        # Time update if new gyro data available
        if self.nom_updt_ts < self.gyro_data.timestamp:
            self.time_update()
            if self.gyro_data.timestamp < (curr_time_msec - SF_GYRO_MAX_STALE_DUR):
                self.op_mode &= ~SF_GYRO_MASK
                self.op_mode |= SF_GYRO_STALE
        elif self.nom_updt_ts < (curr_time_msec - SF_GYRO_MAX_MISS_DUR):
            self.op_mode &= ~SF_GYRO_MASK
            self.op_mode |= SF_GYRO_MISSING

        # Measurement update if new accel data available
        if self.meas_updt_ts < self.acc_data.timestamp:
            self.measurement_update()
            if self.acc_data.timestamp < (curr_time_msec - SF_ACCEL_MAX_STALE_DUR):
                self.op_mode &= ~SF_ACC_MASK
                self.op_mode |= SF_ACC_STALE
        elif self.meas_updt_ts < (curr_time_msec - SF_ACCEL_MAX_MISS_DUR):
            self.op_mode &= ~SF_ACC_MASK
            self.op_mode |= SF_ACC_MISSING

        # Update gravity vector
        for i in range(3):
            self.grav_post_s[i] = -1.0 * GTOMSEC2 * self.rot_mtx_post[i, 2]

        # Convert rotation matrix to Euler angles
        self.theta_post, self.phi_post, self.psi_post, self.rho_post, self.chi_post = \
            rotation_matrix_to_angles(self.rot_mtx_post, self.theta_post, self.psi_post)

        # Prepare output
        output = AlgoOutput()
        output.algo_type = AlgoType.SF_6AG
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

        # Clear signal flags after processing
        self.signal_sf_run = 0

        return output


# ============================================================================
# Helper Functions
# ============================================================================

def cross_product_matrix(vec: np.ndarray) -> np.ndarray:
    """
    Create skew-symmetric cross product matrix from vector

    Args:
        vec: 3D vector [x, y, z]

    Returns:
        3x3 skew-symmetric matrix
    """
    return np.array([
        [0, -vec[2], vec[1]],
        [vec[2], 0, -vec[0]],
        [-vec[1], vec[0], 0]
    ])


def quaternion_integrate(quat: np.ndarray, omega: np.ndarray, delta_t: float) -> np.ndarray:
    """
    Integrate quaternion with angular velocity

    Args:
        quat: Quaternion [q0, q1, q2, q3]
        omega: Angular velocity [wx, wy, wz] in deg/s
        delta_t: Time step in seconds

    Returns:
        Integrated quaternion [q0, q1, q2, q3]
    """
    # Convert angular velocity to radians
    omega_rad = omega * DEG2RAD

    # Compute half angle
    half_angle = 0.5 * np.linalg.norm(omega_rad) * delta_t

    if half_angle < EPSILON:
        return quat.copy()

    # Compute sine and cosine
    s = np.sin(half_angle)
    c = np.cos(half_angle)

    # Normalize omega
    omega_norm = omega_rad / np.linalg.norm(omega_rad)

    # Create rotation quaternion
    dq = np.array([c, s * omega_norm[0], s * omega_norm[1], s * omega_norm[2]])

    # Quaternion multiplication: q_new = q * dq
    q_new = quat_product(quat, dq)

    return q_new


def rotation_matrix_to_quaternion(rot_mtx: np.ndarray) -> np.ndarray:
    """
    Convert rotation matrix to quaternion

    Args:
        rot_mtx: 3x3 rotation matrix

    Returns:
        Quaternion [q0, q1, q2, q3]
    """
    trace = rot_mtx[0, 0] + rot_mtx[1, 1] + rot_mtx[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q0 = 0.25 / s
        q1 = (rot_mtx[2, 1] - rot_mtx[1, 2]) * s
        q2 = (rot_mtx[0, 2] - rot_mtx[2, 0]) * s
        q3 = (rot_mtx[1, 0] - rot_mtx[0, 1]) * s
    elif rot_mtx[0, 0] > rot_mtx[1, 1] and rot_mtx[0, 0] > rot_mtx[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_mtx[0, 0] - rot_mtx[1, 1] - rot_mtx[2, 2])
        q0 = (rot_mtx[2, 1] - rot_mtx[1, 2]) / s
        q1 = 0.25 * s
        q2 = (rot_mtx[0, 1] + rot_mtx[1, 0]) / s
        q3 = (rot_mtx[0, 2] + rot_mtx[2, 0]) / s
    elif rot_mtx[1, 1] > rot_mtx[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_mtx[1, 1] - rot_mtx[0, 0] - rot_mtx[2, 2])
        q0 = (rot_mtx[0, 2] - rot_mtx[2, 0]) / s
        q1 = (rot_mtx[0, 1] + rot_mtx[1, 0]) / s
        q2 = 0.25 * s
        q3 = (rot_mtx[1, 2] + rot_mtx[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot_mtx[2, 2] - rot_mtx[0, 0] - rot_mtx[1, 1])
        q0 = (rot_mtx[1, 0] - rot_mtx[0, 1]) / s
        q1 = (rot_mtx[0, 2] + rot_mtx[2, 0]) / s
        q2 = (rot_mtx[1, 2] + rot_mtx[2, 1]) / s
        q3 = 0.25 * s

    return np.array([q0, q1, q2, q3])


def rotation_matrix_to_angles(rot_mtx: np.ndarray, prev_theta: float, prev_psi: float) -> Tuple[float, float, float, float, float]:
    """
    Convert rotation matrix to Euler angles with gimbal lock handling

    Args:
        rot_mtx: 3x3 rotation matrix
        prev_theta: Previous theta (pitch) for gimbal lock resolution
        prev_psi: Previous psi (yaw) for gimbal lock resolution

    Returns:
        Tuple of (theta, phi, psi, rho, chi) in degrees
    """
    MAX_POS_PITCH_DEG = 179.9999

    # Pitch angle [-90, 90) - AN5017 NED: θ = asin(-R[0][2])
    theta = np.arcsin(-rot_mtx[0, 2]) * RAD2DEG

    # Initialize with previous values
    phi = prev_theta  # using prev_theta as temporary storage for roll
    psi = prev_psi

    # Roll and yaw angles (avoiding gimbal lock) - AN5017 NED
    if (rot_mtx[0, 2] < (1 - EPSILON)) and (rot_mtx[0, 2] > -(1 - EPSILON)):
        phi = np.arctan2(rot_mtx[1, 2], rot_mtx[2, 2]) * RAD2DEG  # AN5017: φ = atan2(R[1][2], R[2][2])
        psi = np.arctan2(rot_mtx[0, 1], rot_mtx[0, 0]) * RAD2DEG  # AN5017: ψ = atan2(R[0][1], R[0][0])
    else:
        # Gimbal lock at pitch = ±90° (AN5017 Section 2.6, Eqs 23-24)
        angle_val = np.arctan2(rot_mtx[1, 0], rot_mtx[1, 1]) * RAD2DEG
        if rot_mtx[0, 2] <= -(1 - EPSILON):  # pitch = +90°
            psi = angle_val - phi  # tan(ψ - φ) = R[1][0]/R[1][1]
        else:  # pitch = -90°
            psi = angle_val + phi  # tan(ψ + φ) = -R[1][0]/R[1][1]

    # Normalize angles
    if theta > MAX_POS_PITCH_DEG:
        theta = -180.0

    if psi < 0.0:
        psi += 360.0
    elif psi >= 360.0:
        psi = 0.0

    # Compass heading
    rho = psi

    # Tilt angle [0, 180]
    chi = np.arccos(rot_mtx[2, 2]) * RAD2DEG

    return theta, phi, psi, rho, chi


def timestamp_ms() -> int:
    """Get current timestamp in milliseconds"""
    import time
    return int(time.time() * 1000)
