
/**
 * @file algo_sf_9x_sensor_fusion.c
 * @brief 9-Axis Extended Kalman Filter for Sensor Fusion (Accel + Gyro + Magnetometer)
 * @author Vikas Yadav
 * @date 2020
 *
 * @section ALGORITHM_OVERVIEW Algorithm Overview
 *
 * This module implements a 9-axis Extended Kalman Filter (EKF) for sensor fusion
 * combining accelerometer, gyroscope, and magnetometer measurements to estimate
 * device orientation with absolute heading (yaw) reference, gravity vector,
 * linear acceleration, gyroscope bias, and magnetic field disturbance.
 *
 * @subsection STATE_VECTOR State Vector (12 DOF)
 * The filter estimates a 12-dimensional state vector:
 * - x[0:2]: Orientation error (3 DOF) - rotation angles in degrees
 * - x[3:5]: Gyroscope bias (3 DOF) - bias error in deg/s
 * - x[6:8]: Linear acceleration (3 DOF) - in m/s²
 * - x[9:11]: Magnetic field disturbance (3 DOF) - in μT
 *
 * @subsection QUATERNION_REPRESENTATION Quaternion Representation
 * Orientation is represented internally using unit quaternions to avoid gimbal lock
 * and provide efficient rotation updates. The quaternion is converted to Euler angles
 * and rotation matrix for output.
 *
 * @subsection EKF_EQUATIONS Extended Kalman Filter Equations
 *
 * Time Update (Prediction):
 * - x_k = F * x_{k-1} + w_k
 * - P_k = F * P_{k-1} * F^T + Q
 *
 * Measurement Update (Correction):
 * - K = P * H^T * (H * P * H^T + R)^{-1}
 * - x_k = x_k + K * (z - h(x_k))
 * - P_k = (I - K * H) * P
 *
 * Where:
 * - F: State transition matrix
 * - Q: Process noise covariance matrix (4x4 blocks for 9-axis)
 * - H: Measurement matrix
 * - R: Measurement noise covariance matrix
 * - K: Kalman gain
 * - P: Error covariance matrix
 *
 * @subsection SENSOR_FUSION Sensor Fusion Strategy
 * - Gyroscope: Used for time update (high frequency, drift accumulation)
 * - Accelerometer: Used for measurement update (gravity reference, tilt correction)
 * - Magnetometer: Used for measurement update (heading reference, yaw correction)
 * - Magnetic disturbance detection prevents corruption from local magnetic fields
 * - Fusion provides drift-free 3D orientation with absolute heading
 *
 * @subsection MAGNETOMETER_HANDLING Magnetometer Processing
 * - Hard iron calibration: Removes constant magnetic offsets
 * - Soft iron calibration: Corrects for sensor axis misalignment
 * - Magnetic declination: Converts magnetic north to true north
 * - Disturbance detection: 30% magnitude deviation threshold
 * - Adaptive gain: Reduces magnetometer weight during motion/disturbance
 *
 * @subsection REFERENCES References
 * - AN5023: NXP 9-Axis Sensor Fusion Implementation
 * - AN5017: NXP Sensor Fusion Mathematics
 * - Freescale Sensor Fusion Library
 */

#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"

/*==============================================================================
 * Private Constants
 *============================================================================*/

/** @brief Maximum positive pitch angle before wrapping (degrees) */
#define SF_9XAGM_MAX_POS_PITCH_DEG          (179.9999)

/** @brief Gyroscope bias time averaging window (1 hour at gyro sample rate) */
#define SF_9XAGM_GYRO_BIAS_TIME_AVG_LEN     (60*60*SF_GYRO_FS)

/** @brief Magnitude check threshold for zero-detection */
#define SF_9XAGM_MAGNITUDE_THRESHOLD        (1e-6)

/** @brief Symmetry coefficient for covariance matrix averaging */
#define SF_9XAGM_COVARIANCE_SYMMETRY_FACTOR (0.5)

/** @brief Time constant for linear acceleration estimation model (seconds) */
#define SF_9XAGM_LINEAR_ACC_TIME_CONSTANT   (0.5)

/** @brief Magnetic disturbance angular deviation threshold (cos(30°) ≈ 0.866) */
#define SF_9XAGM_MAG_DIRECTION_THRESHOLD    (0.866)

/** @brief Angular velocity threshold for adaptive mag gain (rad/s) */
#define SF_9XAGM_GYRO_MOTION_THRESHOLD      (0.5)

/** @brief Linear acceleration deviation threshold for adaptive mag gain (g) */
#define SF_9XAGM_ACCEL_DEVIATION_THRESHOLD  (0.3)

/** @brief Exponential decay factor for adaptive gain computation */
#define SF_9XAGM_ADAPTIVE_GAIN_DECAY_FACTOR (5.0)

/*==============================================================================
 * Private Function Prototypes
 *============================================================================*/

/* State Initialization Functions */
static void sf_9xagm_algo_reset(state_vec_9XAGM_t *ptr_state_vec_9XAGM);
static void sf_9xagm_algo_init_orient(state_vec_9XAGM_t *ptr_state_vec_9XAGM,
	                                   phys_sensor_t     *ptr_accel_data,
	                                   phys_sensor_t     *ptr_mag_data);
static void sf_9xagm_algo_tilt_rotmtx(const double accel_avg[3],
	                                   const double mag_avg[3],
	                                   double       RotMtx[3][3]);
static void sf_9xagm_algo_measupdate_mag(state_vec_9XAGM_t *ptr_state_vec_9XAGM,
                                          phys_sensor_t     *ptr_mag_data);

/*==============================================================================
 * State Initialization Functions
 *============================================================================*/

/**
 * @brief Resets the 9-axis sensor fusion algorithm state to initial conditions
 *
 * This function initializes all state variables to their default values:
 * - Rotation matrix and quaternion to identity
 * - Error covariance matrix P to zero (4x4 blocks for 9-axis)
 * - Process noise covariance matrix Q to zero (off-diagonal blocks)
 * - Magnetometer calibration parameters to defaults
 * - Hard iron offset to zero, soft iron matrix to identity
 * - Magnetic field reference to zero (initialized on first measurement)
 * - All orientation angles (roll, pitch, yaw) to zero
 * - Gravity vector to standard gravity (-9.80665 m/s² in Z-direction)
 * - Linear acceleration and magnetic disturbance to zero
 * - Gyroscope bias and error states to zero
 * - Operation mode flags and timestamps to initial state
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 *
 * @return None
 *
 * @note This function is called both during initialization and when a reset
 *       is explicitly requested via the Reset flag
 * @note The quaternion and rotation matrix are initialized to identity, but
 *       will be properly initialized with real sensor data during the first
 *       algorithm run via sf_9xagm_algo_init_orient()
 * @note For 9-axis, the state vector is 12 DOF (orientation, bias, lin_acc, mag_dist)
 *       requiring a 4x4 block matrix for error covariance
 */
static void sf_9xagm_algo_reset(state_vec_9XAGM_t *ptr_state_vec_9XAGM)
{
    // initialize rotation matrix and quaternion to 1
	// another real data based initialization of rot mtx and quat happens during the first run of the algo
	ptr_state_vec_9XAGM->QuatPost.q0 = 1.0f;
	ptr_state_vec_9XAGM->RotMtxPost[CHX][CHX] = 1.0f;
	ptr_state_vec_9XAGM->RotMtxPost[CHY][CHY] = ptr_state_vec_9XAGM->RotMtxPost[CHX][CHX];
	ptr_state_vec_9XAGM->RotMtxPost[CHZ][CHZ] = ptr_state_vec_9XAGM->RotMtxPost[CHX][CHX];

	// initialize P0, error cov matrix (4x4 for 9-axis including magnetometer)	
	for(int32_t i = 0; i < 4; i++) {
		for(int32_t j = 0; j < 4; j++) {
			ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
		}
	}

	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[0][3]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[1][3]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[2][3]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[3][0]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[3][1]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[3][2]));

	ptr_state_vec_9XAGM->update_ErrCovMtx = 1;


	// Initialize magnetometer calibration parameters
	// These would typically be loaded from calibration data
	ptr_state_vec_9XAGM->MagCalOffset[0] = 0.0f;
	ptr_state_vec_9XAGM->MagCalOffset[1] = 0.0f;
	ptr_state_vec_9XAGM->MagCalOffset[2] = 0.0f;
	
	// Initialize local magnetic field reference (to be updated during initialization)
	ptr_state_vec_9XAGM->MagFieldRef[0] = 0.0f;  // North component
	ptr_state_vec_9XAGM->MagFieldRef[1] = 0.0f;  // East component
	ptr_state_vec_9XAGM->MagFieldRef[2] = 0.0f;  // Down component

	// Initialize soft iron calibration matrix to identity
	for(int32_t i = 0; i < 3; i++) {
		for(int32_t j = 0; j < 3; j++) {
			ptr_state_vec_9XAGM->MagCalMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
	
	// Initialize additional 9-axis specific parameters
	ptr_state_vec_9XAGM->MagDeclination = 0.0f;  // To be set based on location
	ptr_state_vec_9XAGM->MagCalValid = 0;        // Calibration not yet valid

	// Initialize algorithm output-related state variables with reasonable defaults
	// Initialize orientation angles to zero (no rotation)
	ptr_state_vec_9XAGM->PhiPost = 0.0;     // roll angle
	ptr_state_vec_9XAGM->ThetaPost = 0.0;   // pitch angle  
	ptr_state_vec_9XAGM->PsiPost = 0.0;     // yaw angle
	ptr_state_vec_9XAGM->RhoPost = 0.0;     // compass heading
	ptr_state_vec_9XAGM->ChiPost = 0.0;     // tilt angle

	// Initialize gravity vector (pointing down in sensor frame)
	ptr_state_vec_9XAGM->GravPostS[0] = 0.0;
	ptr_state_vec_9XAGM->GravPostS[1] = 0.0; 
	ptr_state_vec_9XAGM->GravPostS[2] = -9.80665; // standard gravity in m/s²

	// Initialize linear acceleration to zero
	for(int32_t i = 0; i < 4; i++) {
		ptr_state_vec_9XAGM->AccPostG[i] = 0.0;
	}
	for(int32_t i = 0; i < 3; i++) {
		ptr_state_vec_9XAGM->AccPostS[i] = 0.0;
		ptr_state_vec_9XAGM->MagPostS[i] = 0.0;  // sensor frame mag data
		ptr_state_vec_9XAGM->MagPostG[i] = 0.0;  // global frame mag data
	}

	// Initialize gyro bias and error states to zero
	for(int32_t i = 0; i < 3; i++) {
		ptr_state_vec_9XAGM->BiasPostS[i] = 0.0;
		ptr_state_vec_9XAGM->BiasErrPostS[i] = 0.0;
		ptr_state_vec_9XAGM->OrntErrPostS[i] = 0.0;
		ptr_state_vec_9XAGM->AccErrPostS[i] = 0.0;
		ptr_state_vec_9XAGM->MagDistErrPostS[i] = 0.0;
		ptr_state_vec_9XAGM->Omega[i] = 0.0;
		ptr_state_vec_9XAGM->AngRatePrev[i] = 0.0;
		ptr_state_vec_9XAGM->RotVec[i] = 0.0;
	}

	// Initialize operation mode flags and timestamps
	ptr_state_vec_9XAGM->OpMode = 0;
	ptr_state_vec_9XAGM->SensFlags = 0;
	ptr_state_vec_9XAGM->NomupdtTS = 0;
	ptr_state_vec_9XAGM->MeasupdtTS = 0;
	ptr_state_vec_9XAGM->MagMeasupdtTS = 0;
	ptr_state_vec_9XAGM->OrientInit = false;

	// clear the reset flag
	ptr_state_vec_9XAGM->Reset = false;

} /* sf_9xagm_algo_reset */

/*==============================================================================
 * Public Interface Functions
 *============================================================================*/

/**
 * @brief Initializes the 9-axis sensor fusion algorithm and allocates state memory
 *
 * This function is the interface between the sensor fusion algorithm and algorithm
 * manager. It creates a new 9-axis (accelerometer + gyroscope + magnetometer) sensor
 * fusion algorithm instance that outputs gravity, linear acceleration, rotation
 * vector, orientation angles with absolute heading (yaw), and magnetic field data.
 *
 * The function performs the following initialization steps:
 * 1. Validates input parameters
 * 2. Allocates memory for algorithm state data structure
 * 3. Configures sensor specifications (scale factors, sensor IDs)
 * 4. Initializes Kalman filter noise parameters (Q and R matrices)
 * 5. Sets magnetometer-specific noise parameters
 * 6. Sets linear acceleration time constant
 * 7. Resets all state variables to initial conditions
 *
 * @param[in] algo_init_data Pointer to algorithm initialization data containing:
 *                           - Acc_GPERCOUNT: Accelerometer scale factor (g per count)
 *                           - Gyro_DPSPERCOUNT: Gyroscope scale factor (deg/s per count)
 *                           - Mag_UTPERCOUNT: Magnetometer scale factor (μT per count)
 *
 * @return Unique algorithm ID (pointer to state structure) for subsequent API calls
 * @retval 0 If initialization failed (invalid input or memory allocation failure)
 * @retval non-zero Valid algorithm ID to be used for all future API communications
 *
 * @note The returned algorithm ID must be saved by the algorithm manager and used
 *       for all subsequent operations (data input, algorithm run, stop)
 * @note Memory is dynamically allocated using calloc() and must be freed using
 *       sf_9xagm_algo_stop() when algorithm is no longer needed
 * @note Process noise matrix Q tuning parameters (9-axis):
 *       - ProcNoiseVarOrient: Orientation error variance
 *       - ProcNoiseVarBias: Gyroscope bias variance
 *       - ProcNoiseVarBiasOrient: Cross-correlation between bias and orientation
 *       - ProcNoiseVarLinAcc: Linear acceleration variance
 *       - ProcNoiseVarMagDist: Magnetic disturbance variance (9-axis specific)
 * @note Measurement noise matrices R are computed for both accelerometer and
 *       magnetometer from sensor noise characteristics plus discretization error
 */
uintptr_t sf_9xagm_algo_init(sf_algo_init_data_t *algo_init_data)
{
	state_vec_9XAGM_t *ptr_state_vec_9XAGM;

	// check for valid input
	if (algo_init_data == NULL) {
		printf("init failed invalid input \n");
		return 0;
	}

	// allocate memory for algo state data
	ptr_state_vec_9XAGM = calloc(1, sizeof(state_vec_9XAGM_t));
	if (ptr_state_vec_9XAGM == NULL) {
		printf("init failed no memory \n");
		return 0;
	}
	// set algo id to the start address of state vec memory
	ptr_state_vec_9XAGM->AlgoID = (uintptr_t)ptr_state_vec_9XAGM;

	// specify the type of algorithm (9-axis Accel + Gyro + Mag)
	ptr_state_vec_9XAGM->AlgoType = SF_9AGM;

	// add sensor specifications for Accel (0), Gyro (1), and Mag (2)
	ptr_state_vec_9XAGM->AccData.ScaleFactor = algo_init_data->Acc_GPERCOUNT;
	ptr_state_vec_9XAGM->AccData.SensorID = ACC;
	ptr_state_vec_9XAGM->GyroData.ScaleFactor = algo_init_data->Gyro_DPSPERCOUNT;
	ptr_state_vec_9XAGM->GyroData.SensorID = GYRO;
	// Add magnetometer sensor specification
	ptr_state_vec_9XAGM->MagData.ScaleFactor = algo_init_data->Mag_UTPERCOUNT;
	ptr_state_vec_9XAGM->MagData.SensorID = MAG;

	// initialize parameters for model noise matrix Q 
	ptr_state_vec_9XAGM->ProcNoiseVarOrient = SF_6XAG_QOrient;
	ptr_state_vec_9XAGM->ProcNoiseVarBias = SF_6XAG_QBias;
	ptr_state_vec_9XAGM->ProcNoiseVarBiasOrient = SF_6XAG_QBiasOrient;
	ptr_state_vec_9XAGM->ProcNoiseVarLinAcc = SF_6XAG_QLinAcc;
	// Add magnetometer-specific noise parameters
	ptr_state_vec_9XAGM->ProcNoiseVarMagDist = SF_9XAGM_QMagDist;

	// initialize parameter for measurement noise matrix R (including magnetometer)
	ptr_state_vec_9XAGM->MeasNoiseVarAcc = SF_6XAG_QVACC + SF_6XAG_QWACC + ((SF_6XAG_QVGYRO + SF_6XAG_QWGYRO) * SF_DELTA_T_SQ);
	ptr_state_vec_9XAGM->MeasNoiseVarMag = SF_9XAGM_QVMAG + SF_9XAGM_QWMAG + ((SF_6XAG_QVGYRO + SF_6XAG_QWGYRO) * SF_DELTA_T_SQ);

	// set time constant for linear acceleration estimation model
	ptr_state_vec_9XAGM->LinAccTC = SF_9XAGM_LINEAR_ACC_TIME_CONSTANT;

	// set the reset flag
	sf_9xagm_algo_reset(ptr_state_vec_9XAGM);

    printf("\n9-axis SF algo initialized \n");
	printf("start 9-axis SF algo with algo id = %zu (0x%zx)\n", ptr_state_vec_9XAGM->AlgoID, ptr_state_vec_9XAGM->AlgoID);
	printf("ptr_state_vec_9XAGM address = %p\n", ptr_state_vec_9XAGM);
	printf("sizeof(ptr_state_vec_9XAGM) = %zu bytes\n", sizeof(ptr_state_vec_9XAGM));
	printf("sizeof(*ptr_state_vec_9XAGM) = %zu bytes\n", sizeof(*ptr_state_vec_9XAGM));

	return ptr_state_vec_9XAGM->AlgoID;

} /* sf_9xagm_algo_init */

/**
 * @brief Initializes orientation state using accelerometer and magnetometer tilt/heading measurement
 *
 * This function performs the initial orientation lock using accelerometer and magnetometer
 * data to establish the gravity reference frame and magnetic heading. It is called once
 * during the first algorithm run to initialize the rotation matrix and quaternion from
 * real sensor measurements rather than using the identity matrix.
 *
 * The initialization process:
 * 1. Converts accelerometer and magnetometer counts to physical units
 * 2. Computes tilt-compensated heading rotation matrix using both sensors
 * 3. Converts rotation matrix to unit quaternion
 * 4. Sets OrientInit flag to prevent re-initialization
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 * @param[in] ptr_accel_data Pointer to accelerometer sensor data containing:
 *                           - CountAvg: Averaged accelerometer counts [3]
 *                           - ScaleFactor: Accelerometer scale factor (g per count)
 * @param[in] ptr_mag_data Pointer to magnetometer sensor data containing:
 *                         - CountAvg: Averaged magnetometer counts [3]
 *                         - ScaleFactor: Magnetometer scale factor (μT per count)
 *
 * @return None
 *
 * @note This function modifies RotMtxPost and QuatPost in the state structure
 * @note The function assumes gravity is the dominant acceleration during initialization
 * @note The magnetometer provides absolute heading reference (yaw), unlike 6-axis
 * @note Uses tilt-compensated magnetometer to establish North-East-Down (NED) frame
 */
static void sf_9xagm_algo_init_orient(state_vec_9XAGM_t *ptr_state_vec_9XAGM,
	                                 phys_sensor_t    *ptr_accel_data,
	                                 phys_sensor_t    *ptr_mag_data)
{
	double      accel_avg[3];
	double      mag_avg[3];
	uint32_t    i;

	for(i = CHX; i <= CHZ; i++) {
		accel_avg[i] = (ptr_accel_data->CountAvg[i])*(ptr_accel_data->ScaleFactor)*GTOMSEC2;
		mag_avg[i] = (ptr_mag_data->CountAvg[i])*(ptr_mag_data->ScaleFactor);
	}

	// initialize the a posteriori orientation state vector to the tilt-compensated orientation
	sf_9xagm_algo_tilt_rotmtx(accel_avg, mag_avg, ptr_state_vec_9XAGM->RotMtxPost);

	RotMtx2Quat(ptr_state_vec_9XAGM->RotMtxPost, &(ptr_state_vec_9XAGM->QuatPost));

	// clear the reset flag
	ptr_state_vec_9XAGM->OrientInit = true;

} /* sf_9xagm_algo_init_orient */

/**
 * @brief Main algorithm execution function (internal implementation)
 *
 * This is the core execution function that runs the 9-axis sensor fusion algorithm.
 * It implements the complete Extended Kalman Filter cycle including time updates
 * (prediction) and measurement updates (correction) for both accelerometer and
 * magnetometer.
 *
 * Algorithm execution flow:
 * 1. Check for reset request and reinitialize if needed
 * 2. Perform one-time orientation initialization using accelerometer+magnetometer
 * 3. Apply nominal time update when new gyroscope data is available
 * 4. Apply accelerometer measurement update when new data is available
 * 5. Apply magnetometer measurement update when new data is available (with disturbance check)
 * 6. Check for stale or missing magnetometer data and update operation mode
 * 7. Compute gravity vector from updated rotation matrix
 * 8. Convert rotation matrix to Euler angles
 * 9. Populate output structure with all estimated states
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 * @param[out] ptr_algo_out Pointer to algorithm output structure containing:
 *                          - algo_type: Algorithm type identifier (SF_9AGM)
 *                          - quat: Orientation quaternion (q0, q1, q2, q3)
 *                          - orientation: Euler angles [yaw, pitch, roll] in degrees
 *                          - gravity: Gravity vector in sensor frame [x, y, z] in m/s²
 *                          - linear_acc: Linear acceleration in global frame [x, y, z] in m/s²
 *                          - valid_flag: Output validity indicator
 *                          - mode: Current operation mode flags
 *                          - timestamp_ns: Output timestamp in nanoseconds
 *
 * @return None
 *
 * @note Time update is performed at gyroscope sampling rate (high frequency)
 * @note Accelerometer measurement update at accelerometer sampling rate
 * @note Magnetometer measurement update at magnetometer sampling rate (typically lower)
 * @note Magnetic disturbance detection uses 30% magnitude deviation threshold
 * @note Magnetometer updates are skipped during detected disturbances
 * @note Function maintains timestamp tracking to detect stale/missing sensor data
 */
static void sf_9xagm_algo_run_orig(state_vec_9XAGM_t *ptr_state_vec_9XAGM,
	                               sf_algo_output_t *ptr_algo_out)
{
	int64_t     curr_time_msec;
	uint32_t    i;

	// do a reset and return if requested
	if (ptr_state_vec_9XAGM->Reset) {
		printf("9-axis SF algo reset \n");
		sf_9xagm_algo_reset(ptr_state_vec_9XAGM);
		return;
	}

	// do a once-only orientation lock to accelerometer tilt 
	if (!ptr_state_vec_9XAGM->OrientInit) {
		printf("9-axis SF algo init orient \n");
		sf_9xagm_algo_init_orient(ptr_state_vec_9XAGM, &(ptr_state_vec_9XAGM->AccData), &(ptr_state_vec_9XAGM->MagData));
	}

	//printf("running SF...");

	curr_time_msec = (int64_t)clock();

	// if new gyro data is available, apply nominal time udpate
	if ( ptr_state_vec_9XAGM->NomupdtTS < ptr_state_vec_9XAGM->GyroData.timestamp ) {
		sf_9xagm_algo_nom_timeupdate(ptr_state_vec_9XAGM);
	}

	// if new accel data is availabel apply measurement update
	if ( ptr_state_vec_9XAGM->MeasupdtTS < ptr_state_vec_9XAGM->AccData.timestamp ) {
		sf_9xagm_algo_measupdate(ptr_state_vec_9XAGM);
	}

	// if new magnetometer data is available, apply magnetometer measurement update
	if ( ptr_state_vec_9XAGM->MagMeasupdtTS < ptr_state_vec_9XAGM->MagData.timestamp ) {
		// Check for magnetic disturbances before applying update
		double mag_measured[3];
		for(uint32_t idx = CHX; idx <= CHZ; idx++) {
			mag_measured[idx] = ptr_state_vec_9XAGM->MagData.CountAvg[idx] *
			                   ptr_state_vec_9XAGM->MagData.ScaleFactor;
		}

		// Detect magnetic disturbance (threshold: 30% magnitude deviation)
		int disturbance = sf_9xagm_detect_mag_disturbance(mag_measured,
		                                                   ptr_state_vec_9XAGM->MagFieldRef,
		                                                   SF_9XAGM_ACCEL_DEVIATION_THRESHOLD);

		// Only apply magnetometer update if no disturbance detected
		if (!disturbance) {
			sf_9xagm_algo_measupdate_mag(ptr_state_vec_9XAGM, &(ptr_state_vec_9XAGM->MagData));
		}

		if (ptr_state_vec_9XAGM->MagData.timestamp < (curr_time_msec - SF_MAG_MAX_STALE_DUR)) {
			// if magnetometer data used is stale, mark mode_op to degraded mag mode
			ptr_state_vec_9XAGM->OpMode &= ~(SF_MAG_MASK);
			ptr_state_vec_9XAGM->OpMode |= SF_MAG_STALE;
		}
		ptr_state_vec_9XAGM->MagMeasupdtTS = ptr_state_vec_9XAGM->MagData.timestamp;
	}
	// if current time is greater than last mag update time by max miss duration, 
	// mark missing magnetometer data
	else if (ptr_state_vec_9XAGM->MagMeasupdtTS < ((curr_time_msec - SF_MAG_MAX_MISS_DUR))) {
		// if magnetometer data used is not available for a long time, mark mode_op to degraded mag mode
		ptr_state_vec_9XAGM->OpMode &= ~(SF_MAG_MASK);
		ptr_state_vec_9XAGM->OpMode |= SF_MAG_MISSING;
	}

	for(i = 0; i < 3; i++) {
		ptr_state_vec_9XAGM->GravPostS[i] = -1.0 * GTOMSEC2 * ptr_state_vec_9XAGM->RotMtxPost[i][2];
	}

	// Convert quaternions to angles
	sf_9xagm_algo_rotmtx2angles(ptr_state_vec_9XAGM->RotMtxPost, &(ptr_state_vec_9XAGM->ThetaPost), 
		                        &(ptr_state_vec_9XAGM->PhiPost), &(ptr_state_vec_9XAGM->PsiPost), 
		                        &(ptr_state_vec_9XAGM->RhoPost), &(ptr_state_vec_9XAGM->ChiPost),
		                        ptr_state_vec_9XAGM->ThetaPost, ptr_state_vec_9XAGM->PsiPost);

	// Assign outputs
	ptr_algo_out->algo_type = SF_9AGM;
	ptr_algo_out->quat.q0 = (float)ptr_state_vec_9XAGM->QuatPost.q0;
	ptr_algo_out->quat.q1 = (float)ptr_state_vec_9XAGM->QuatPost.q1;
	ptr_algo_out->quat.q2 = (float)ptr_state_vec_9XAGM->QuatPost.q2;
	ptr_algo_out->quat.q3 = (float)ptr_state_vec_9XAGM->QuatPost.q3;
	ptr_algo_out->orientation[0] = (float)ptr_state_vec_9XAGM->PsiPost;
	ptr_algo_out->orientation[1] = (float)ptr_state_vec_9XAGM->ThetaPost;
	ptr_algo_out->orientation[2] = (float)ptr_state_vec_9XAGM->PhiPost;
	ptr_algo_out->gravity[0] = (float)ptr_state_vec_9XAGM->GravPostS[0];
	ptr_algo_out->gravity[1] = (float)ptr_state_vec_9XAGM->GravPostS[1];
	ptr_algo_out->gravity[2] = (float)ptr_state_vec_9XAGM->GravPostS[2];
	ptr_algo_out->linear_acc[0] = (float)ptr_state_vec_9XAGM->AccPostG[0];
	ptr_algo_out->linear_acc[1] = (float)ptr_state_vec_9XAGM->AccPostG[1];
	ptr_algo_out->linear_acc[2] = (float)ptr_state_vec_9XAGM->AccPostG[2];
	ptr_algo_out->valid_flag = 0xFFFFFFFF;
	ptr_algo_out->mode = ptr_state_vec_9XAGM->OpMode;
	ptr_algo_out->timestamp_ns = curr_time_msec / NSEC2MSEC;

}

/*==============================================================================
 * Orientation Computation Functions
 *============================================================================*/

/**
 * @brief Computes tilt-compensated heading rotation matrix from accelerometer and magnetometer
 *
 * This function calculates an initial orientation (rotation matrix) based on both
 * accelerometer and magnetometer data. The algorithm establishes a North-East-Down (NED)
 * coordinate frame by using the accelerometer to determine tilt and the magnetometer
 * to determine heading.
 *
 * Algorithm steps:
 * 1. Normalize gravity vector from accelerometer (down vector, Z-axis)
 * 2. Normalize magnetometer reading
 * 3. Remove tilt from magnetic field: project onto horizontal plane
 * 4. Normalize horizontal magnetic field to get north vector (X-axis)
 * 5. Compute east vector: east = down × north (cross product, Y-axis)
 * 6. Construct rotation matrix R = [north east down]
 *
 * This establishes a full 3-DOF orientation including absolute heading (yaw),
 * unlike the 6-axis case which can only determine tilt (roll and pitch).
 *
 * @param[in] accel_avg Averaged accelerometer measurements [x, y, z] in m/s²
 *                      Expected to measure gravity vector (nominally -9.81 m/s² in Z)
 * @param[in] mag_avg Averaged magnetometer measurements [x, y, z] in μT
 *                    Expected to measure Earth's magnetic field
 * @param[out] RotMtx Computed 3x3 rotation matrix representing device orientation
 *                    Columns: [north east down] in NED frame
 *
 * @return None
 *
 * @note The function returns identity matrix if either sensor has invalid magnitude
 * @note Tilt compensation is critical: horizontal mag field = mag - (mag·down)*down
 * @note This provides absolute heading reference, eliminating yaw drift
 * @note The rotation matrix maps global NED frame to sensor frame
 * @note Fallback orientations are used if computed vectors are invalid
 */
static void sf_9xagm_algo_tilt_rotmtx(const double accel_avg[3],
	                                 const double mag_avg[3],
	                                 double       RotMtx[3][3])
{
	double mag_norm[3];
	double east[3], north[3], down[3];
	double accel_mag, mag_mag;
	double mag_horizontal[3], mag_horiz_mag;
	uint32_t i;

	// Normalize accelerometer reading (down vector)
	accel_mag = sqrt(accel_avg[0]*accel_avg[0] + accel_avg[1]*accel_avg[1] + accel_avg[2]*accel_avg[2]);
	if (accel_mag > 0.0) {
		for(i = CHX; i <= CHZ; i++) {
			down[i] = accel_avg[i] / accel_mag;  // Down vector (Z-axis)
		}
	} else {
		// Default orientation if accelerometer data is invalid
		down[0] = 0.0; down[1] = 0.0; down[2] = 1.0;
	}

	// Normalize magnetometer reading
	mag_mag = sqrt(mag_avg[0]*mag_avg[0] + mag_avg[1]*mag_avg[1] + mag_avg[2]*mag_avg[2]);
	if (mag_mag > 0.0) {
		for(i = CHX; i <= CHZ; i++) {
			mag_norm[i] = mag_avg[i] / mag_mag;
		}
	} else {
		// Default magnetic field pointing north if mag data invalid
		mag_norm[0] = 1.0; mag_norm[1] = 0.0; mag_norm[2] = 0.0;
	}

	// Calculate horizontal component of magnetic field (remove tilt)
	// mag_horizontal = mag_norm - (mag_norm . down) * down
	double mag_dot_down = mag_norm[0]*down[0] + mag_norm[1]*down[1] + mag_norm[2]*down[2];
	for(i = CHX; i <= CHZ; i++) {
		mag_horizontal[i] = mag_norm[i] - mag_dot_down * down[i];
	}

	// Normalize horizontal magnetic field to get north vector
	mag_horiz_mag = sqrt(mag_horizontal[0]*mag_horizontal[0] + mag_horizontal[1]*mag_horizontal[1] + mag_horizontal[2]*mag_horizontal[2]);
	if (mag_horiz_mag > 0.0) {
		for(i = CHX; i <= CHZ; i++) {
			north[i] = mag_horizontal[i] / mag_horiz_mag;  // North vector (X-axis)
		}
	} else {
		// Fallback if horizontal mag field is zero
		north[0] = 1.0; north[1] = 0.0; north[2] = 0.0;
	}

	// East vector = down × north (cross product)
	east[0] = down[1]*north[2] - down[2]*north[1];  // East vector (Y-axis)
	east[1] = down[2]*north[0] - down[0]*north[2];
	east[2] = down[0]*north[1] - down[1]*north[0];

	// Construct rotation matrix: [north east down]
	// Note: This creates NED (North-East-Down) coordinate frame
	for(i = CHX; i <= CHZ; i++) {
		RotMtx[i][0] = north[i];  // X-axis points north
		RotMtx[i][1] = east[i];   // Y-axis points east  
		RotMtx[i][2] = down[i];   // Z-axis points down
	}

}

/**
 * @brief Converts rotation matrix to Euler angles (roll, pitch, yaw) with gimbal lock handling
 *
 * This function extracts Euler angles from a rotation matrix using the NED (North-East-Down)
 * convention. It handles gimbal lock singularities at roll = ±90° by using previous angle
 * values to resolve ambiguities. The function provides compass heading and tilt angle in
 * addition to standard Euler angles.
 *
 * Euler angle extraction (NED convention):
 * - roll (φ): φ = asin(R[0][2])
 * - pitch (θ): θ = atan2(-R[1][2], R[2][2])
 * - yaw (ψ): ψ = atan2(-R[0][1], R[0][0])
 *
 * Gimbal lock handling (when |R[0][2]| ≈ 1):
 * - At roll = +90°: ψ - θ = atan2(R[1][0], R[1][1])
 * - At roll = -90°: ψ + θ = atan2(R[1][0], R[1][1])
 * - Alternates between solving for ψ and θ using previous values
 *
 * Output angle ranges:
 * - roll (φ): [-90°, +90°]
 * - pitch (θ): [-180°, +180°)
 * - yaw (ψ): [0°, 360°)
 * - compass heading (ρ): [0°, 360°) - same as yaw for magnetic reference
 * - tilt (χ): [0°, 180°] - angle from vertical
 *
 * @param[in] RotMtx 3x3 rotation matrix (sensor to global frame)
 * @param[out] theta Pitch angle in degrees [-180, 180)
 * @param[out] phi Roll angle in degrees [-90, 90]
 * @param[out] psi Yaw angle in degrees [0, 360) - magnetic heading
 * @param[out] rho Compass heading in degrees [0, 360) (same as psi for 9-axis)
 * @param[out] chi Tilt angle from vertical in degrees [0, 180]
 * @param[in] prev_theta Previous pitch angle for gimbal lock resolution
 * @param[in] prev_psi Previous yaw angle for gimbal lock resolution
 *
 * @return None
 *
 * @note Gimbal lock occurs when |R[0][2]| > (1 - EPSILON), handled using previous values
 * @note The function uses atan2_safe() for numerically stable arctangent computation
 * @note Tilt angle χ = acos(R[2][2]) represents total tilt from vertical axis
 * @note For 9-axis, yaw (ψ) has absolute reference from magnetometer
 * @note Maximum positive pitch is clamped to SF_9XAGM_MAX_POS_PITCH_DEG (179.9999°)
 */
void sf_9xagm_algo_rotmtx2angles(const double RotMtx[3][3],
                                       double       *theta,
                                       double       *phi,
                                       double       *psi,
                                       double       *rho,
                                       double       *chi,
	                                   double       prev_theta,
	                                   double       prev_psi)
{
	static uint32_t prev_theta_used = 0;
	double          angle_val;
	uint32_t        angle_vld;

	/*  roll angle [-90,90) */
	*phi = asin(RotMtx[0][2]) * RAD2DEG;

	/*  pitch angle [-180,180)  and yaw angle [0, 360)    */
	*theta = prev_theta;
	*psi = prev_psi;
	if( (RotMtx[0][2] < (1 - EPSILON)) && (RotMtx[0][2] > -(1 - EPSILON)) ) {
		angle_val = atan2_safe(-RotMtx[1][2], RotMtx[2][2], &angle_vld);
		if(angle_vld == 1) {
			*theta = angle_val * RAD2DEG;
		}
		angle_val = atan2_safe(-RotMtx[0][1], RotMtx[0][0], &angle_vld);
		if(angle_vld == 1) {
			*psi = angle_val * RAD2DEG;
		}
	}
	/*  and if roll = 90 or -90 , resolve gimbal lock first using prev values */
	else {
		angle_val = atan2_safe(RotMtx[1][0], RotMtx[1][1], &angle_vld);
		if(angle_vld == 1) {
			if (prev_theta_used == 0) {
				if ((RotMtx[0][2] >= (1 - EPSILON))) {
					*psi = (angle_val * RAD2DEG) - *theta;
				}
				else {
					*psi = (angle_val * RAD2DEG) + *theta;
				}
				prev_theta_used = 1;
			}
			else {
				if ((RotMtx[0][2] >= (1 - EPSILON))) {
					*theta = (angle_val * RAD2DEG) - *psi;
				}
				else {
					*theta = (angle_val * RAD2DEG) + *psi;
				}
				prev_theta_used = 0;
			}
		}
	}

	if (*theta > SF_9XAGM_MAX_POS_PITCH_DEG) {
		*theta = -180.0;
	}
	if (*psi < 0.0) {
		*psi += 360.0;
	}
	else if (*psi >= 360.0) {
		*psi = 0.0;
	}

	/* compass heading angle */
	*rho = *psi;

	/* tilt angle chi [0,180] */
	*chi = acos(RotMtx[2][2]) * RAD2DEG;

}

/*==============================================================================
 * Kalman Filter Time Update (Prediction) Functions
 *============================================================================*/

/**
 * @brief Performs Kalman filter time update (prediction step) using gyroscope data
 *
 * This function implements the Extended Kalman Filter (EKF) time update (prediction step)
 * using gyroscope measurements. It propagates the state forward in time and updates the
 * error covariance matrix. For 9-axis, this handles a 12-dimensional state vector.
 *
 * Time Update Equations:
 * 1. State prediction: x_k = F * x_{k-1} + B * u_k
 *    - Orientation: Quaternion integration using bias-corrected gyro rates
 *    - Gyro bias: Propagated with process noise (random walk model)
 *    - Linear acceleration: Decayed using time constant
 *    - Magnetic disturbance: Propagated with process noise
 *
 * 2. Covariance prediction: P_k = F * P_{k-1} * F^T + Q
 *    - Q: Process noise covariance (adaptive based on previous P)
 *
 * Algorithm steps:
 * 1. Apply bias correction to gyroscope measurements
 * 2. Integrate quaternion for all oversampled gyro measurements
 * 3. Normalize quaternion to maintain unit constraint
 * 4. Update rotation matrix from quaternion
 * 5. Check for missing gyro data and update operation mode
 * 6. Update process noise covariance Q as function of previous P (4x4 blocks)
 * 7. Check orientation error threshold for covariance update control
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 *
 * @return None
 *
 * @note Gyroscope measurements are processed with SF_OVERSAMPLE_RATIO samples per update
 * @note Quaternion integration uses small angle approximation for efficiency
 * @note Process noise Q is adaptive: Q = f(P) + Q_init, where f(P) accounts for
 *       coupling between orientation, bias, linear acceleration, and magnetic disturbance
 * @note If gyro data is missing for > SF_GYRO_MAX_MISS_DUR, covariance update is skipped
 * @note Orientation error threshold SF_MAX_ORIENT_ERR triggers covariance freeze
 * @note For 9-axis: 4x4 block matrix structure for P and Q (vs 3x3 for 6-axis)
 */
void sf_9xagm_algo_nom_timeupdate(state_vec_9XAGM_t *ptr_state_vec_9XAGM)
{
#ifdef GYRO_BIAS_TIME_AVG
	#define GYRO_BIAS_TIME_AVG_LEN   (60*60*SF_GYRO_FS) // 1 hour 
	static uint32_t gyro_bias_time_avg_len = 1;
#endif /* GYRO_BIAS_TIME_AVG */

	quaternion_double_t  QuatInt;
	double               delta_T;
	double               Cacc2;
	double               orient_err;
	blk_mtx_3x3_t        Atemp, Btemp, Ctemp;
	uint32_t             i, k;

	// apply nominal time update for all samples in Gyro buffer
	//delta_T = (double)(ptr_state_vec_9XAGM->GyroData.timestamp - ptr_state_vec_9XAGM->NomupdtTS);
	delta_T = SF_GYRO_SAMP_INTVL;
#if 0	
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][0], ptr_state_vec_9XAGM->GyroData.ScaleFactor,
		(ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][0] * ptr_state_vec_9XAGM->GyroData.ScaleFactor), ptr_state_vec_9XAGM->BiasPostS[0]);
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][1], ptr_state_vec_9XAGM->GyroData.ScaleFactor,
		(ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][1] * ptr_state_vec_9XAGM->GyroData.ScaleFactor), ptr_state_vec_9XAGM->BiasPostS[1]);
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][2], ptr_state_vec_9XAGM->GyroData.ScaleFactor,
		(ptr_state_vec_9XAGM->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][2] * ptr_state_vec_9XAGM->GyroData.ScaleFactor), ptr_state_vec_9XAGM->BiasPostS[2]);
#endif // 0

	for(k = 0; k < SF_OVERSAMPLE_RATIO; k++) {
		for(i = CHX; i <= CHZ; i++) {
#ifdef GYRO_BIAS_TIME_AVG
			ptr_state_vec_9XAGM->BiasErrPostS[i] *= gyro_bias_time_avg_len;
			ptr_state_vec_9XAGM->BiasErrPostS[i] += ptr_state_vec_9XAGM->GyroData.CountBuff[k][i];
			ptr_state_vec_9XAGM->BiasErrPostS[i] /= (gyro_bias_time_avg_len++);
			if(gyro_bias_time_avg_len > GYRO_BIAS_TIME_AVG_LEN) 
				gyro_bias_time_avg_len = GYRO_BIAS_TIME_AVG_LEN;
#endif /* GYRO_BIAS_TIME_AVG */
			ptr_state_vec_9XAGM->Omega[i] = ptr_state_vec_9XAGM->GyroData.CountBuff[k][i] * ptr_state_vec_9XAGM->GyroData.ScaleFactor;
			ptr_state_vec_9XAGM->Omega[i] -= ptr_state_vec_9XAGM->BiasErrPostS[i];
			ptr_state_vec_9XAGM->AngRatePrev[i] = ptr_state_vec_9XAGM->Omega[i];
		}
		QuatIntegrate(&(ptr_state_vec_9XAGM->QuatPost), ptr_state_vec_9XAGM->Omega, delta_T, &QuatInt);
		ptr_state_vec_9XAGM->QuatPost.q0 = QuatInt.q0;
		ptr_state_vec_9XAGM->QuatPost.q1 = QuatInt.q1;
		ptr_state_vec_9XAGM->QuatPost.q2 = QuatInt.q2;
		ptr_state_vec_9XAGM->QuatPost.q3 = QuatInt.q3;
		//printf("quat tup =%f, %f, %f, %f \n", ptr_state_vec_9XAGM->QuatPost.q0, ptr_state_vec_9XAGM->QuatPost.q1, ptr_state_vec_9XAGM->QuatPost.q2, ptr_state_vec_9XAGM->QuatPost.q3);
	}
	// Normalize quaternion
	QuatNormal(&QuatInt, &(ptr_state_vec_9XAGM->QuatPost));
	// Update rotation matrix as well
	Quat2RotMtx( &(ptr_state_vec_9XAGM->QuatPost), ptr_state_vec_9XAGM->RotMtxPost);
	// Use gyro timestamp to maintain sync with sensor data
	// Check for missing gyro data
	if( ptr_state_vec_9XAGM->GyroData.timestamp - ptr_state_vec_9XAGM->NomupdtTS > SF_GYRO_MAX_MISS_DUR) {
		ptr_state_vec_9XAGM->update_ErrCovMtx = 1;
		// if gyro data used is not available for a long time, mark mode_op to degraded gyro mode
		ptr_state_vec_9XAGM->OpMode &= ~(SF_GYRO_MASK);
		ptr_state_vec_9XAGM->OpMode |= SF_GYRO_MISSING;
	}
	else {
		ptr_state_vec_9XAGM->update_ErrCovMtx = 1;
		// if gyro data used is available, mark mode_op to normal gyro mode
		ptr_state_vec_9XAGM->OpMode &= ~(SF_GYRO_MASK);
	}
	ptr_state_vec_9XAGM->NomupdtTS = ptr_state_vec_9XAGM->GyroData.timestamp;
	//printf("nomupdt_ts =%d\n", ptr_state_vec_9XAGM->NomupdtTS);

	if (ptr_state_vec_9XAGM->update_ErrCovMtx == 1) {
		// Update posteriori covariance matrix P_pri = A' *P_post *A + Qw
		// In other words, update Qw based on aposteriori error covariance matrix
		// where Qw = a_fn(P_post) = f(P_post) + Qinit
		// For 9-axis, this is a 4x4 block matrix (orientation, bias, lin_acc, mag_dist)
		Cacc2 = ptr_state_vec_9XAGM->LinAccTC * ptr_state_vec_9XAGM->LinAccTC;

		// Q[0][0]: Orientation error covariance
		IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[0][0]));
		ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->ProcNoiseVarOrient, &(ptr_state_vec_9XAGM->ProcNoiseVar[0][0]), &Atemp);
		ScaleBlkMtx_3x3(delta_T*delta_T, &(ptr_state_vec_9XAGM->ErrCovMtxPost[1][1]), &Btemp);
		AddBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[0][0]), &Ctemp);
		AddBlkMtx_3x3(&Ctemp, &Btemp, &(ptr_state_vec_9XAGM->ProcNoiseVar[0][0]));

		// Q[1][1]: Gyro bias error covariance
		IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[1][1]));
		ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->ProcNoiseVarBias, &(ptr_state_vec_9XAGM->ProcNoiseVar[1][1]), &Atemp);
		AddBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[1][1]), &(ptr_state_vec_9XAGM->ProcNoiseVar[1][1]));

		// Q[2][2]: Linear acceleration error covariance
		IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[2][2]));
		ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->ProcNoiseVarLinAcc, &(ptr_state_vec_9XAGM->ProcNoiseVar[2][2]), &Atemp);
		ScaleBlkMtx_3x3(Cacc2, &(ptr_state_vec_9XAGM->ErrCovMtxPost[2][2]), &Btemp);
		AddBlkMtx_3x3(&Atemp, &Btemp, &(ptr_state_vec_9XAGM->ProcNoiseVar[2][2]));

		// Q[3][3]: Magnetic disturbance error covariance
		IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[3][3]));
		ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->ProcNoiseVarMagDist, &(ptr_state_vec_9XAGM->ProcNoiseVar[3][3]), &Atemp);
		AddBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[3][3]), &(ptr_state_vec_9XAGM->ProcNoiseVar[3][3]));

		// Q[0][1] and Q[1][0]: Cross-covariance between orientation and bias
		IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[0][1]));
		ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->ProcNoiseVarBiasOrient, &(ptr_state_vec_9XAGM->ProcNoiseVar[0][1]), &Atemp);
		ScaleBlkMtx_3x3(-1 * delta_T, &(ptr_state_vec_9XAGM->ErrCovMtxPost[1][1]), &Btemp);
		SetBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ProcNoiseVar[0][1]));
		TranspBlkMtx_3x3(&Atemp, &Btemp);
		SetBlkMtx_3x3(&Btemp, &(ptr_state_vec_9XAGM->ProcNoiseVar[1][0]));

		orient_err = ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[0][0] * ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[0][0];
		orient_err += ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[1][1] * ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[1][1];
		orient_err += ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[2][2] * ptr_state_vec_9XAGM->ProcNoiseVar[0][0].elem[2][2];
		if(orient_err > SF_MAX_ORIENT_ERR) {
			ptr_state_vec_9XAGM->update_ErrCovMtx = 0;
		}
	}

}

/*==============================================================================
 * Kalman Filter Measurement Update (Correction) Functions
 *============================================================================*/

/**
 * @brief Performs Kalman filter measurement update (correction step) using accelerometer data
 *
 * This function implements the Extended Kalman Filter (EKF) measurement update (correction step)
 * using accelerometer measurements. It compares predicted gravity with measured acceleration
 * to correct orientation, gyro bias, linear acceleration, and (for 9-axis) magnetic disturbance
 * estimates.
 *
 * Measurement Update Equations:
 * 1. Innovation (measurement residual): z = y_measured - h(x_predicted)
 *    - Gravity error: z_g = accel_measured - gravity_predicted
 *
 * 2. Kalman Gain: K = P * H^T * (H * P * H^T + R)^{-1}
 *    - H: Measurement matrix (linearized observation model)
 *    - R: Measurement noise covariance
 *
 * 3. State correction: x = x + K * z
 *    - Orientation correction via quaternion rotation
 *    - Gyro bias correction
 *    - Linear acceleration update with time constant decay
 *
 * 4. Covariance correction: P = (I - K * H) * P
 *    - Joseph form for numerical stability and symmetry enforcement
 *
 * Algorithm steps:
 * 1. Compute gravity prediction from current rotation matrix
 * 2. Compute innovation (gravity error) including linear acceleration compensation
 * 3. Compute measurement matrix C (cross-product form for orientation)
 * 4. Compute Kalman gain K = Q_w * C^T * inv(C * Q_w * C^T + Q_v)
 * 5. Compute state correction vector
 * 6. Update quaternion using correction (integrate and normalize)
 * 7. Update rotation matrix from corrected quaternion
 * 8. Update gyro bias
 * 9. Update linear acceleration estimate
 * 10. Update error covariance matrix with symmetry enforcement
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 *
 * @return None
 *
 * @note Measurement matrix C[0] = -[g×] relates orientation error to gravity error
 * @note Measurement matrix C[1] = δt*[g×] relates bias error to gravity error
 * @note Measurement matrix C[2] = I relates linear acceleration to gravity error
 * @note Linear acceleration uses first-order lag with time constant LinAccTC
 * @note Covariance update uses Joseph form: P = (I-KC)*Q_w*(I-KC)^T + K*R*K^T
 *       simplified to: P = (I-KC)*Q_w for computational efficiency
 * @note Symmetry is enforced: P = 0.5*(P + P^T) + ε*I
 * @note Missing accelerometer data for > SF_ACCEL_MAX_MISS_DUR sets degraded mode flag
 * @note For 9-axis: measurement does not directly observe magnetic disturbance state
 */
void sf_9xagm_algo_measupdate(state_vec_9XAGM_t *ptr_state_vec_9XAGM)
{
	
	blk_mtx_3x3_t        Atemp, Btemp;
	blk_mtx_3x3_t        Ftemp[3], Gtemp, Ginv;
	blk_mtx_3x3_t        Cmat[3];
	blk_mtx_3x3_t        Qv_mat;
	quaternion_double_t  QuatInt;
	double               Mupdt[9];
	double               gyro_corr[3];
	double               orient_err;
	uint32_t             inv_exist;
	uint32_t             i, j, k;

	inv_exist = 1;

	// Compute error in Gravity Vector
	for(i = CHX; i <= CHZ; i++) {
		ptr_state_vec_9XAGM->GravGyrPriS[i] = -ptr_state_vec_9XAGM->RotMtxPost[i][2] * GTOMSEC2;
		//printf("g=%f, h=%f\n\n", ptr_state_vec_9XAGM->GravGyrPriS[i], ptr_state_vec_9XAGM->RotMtxPost[i][2]);
		ptr_state_vec_9XAGM->GravErrPriS[i] = ptr_state_vec_9XAGM->AccData.CountBuff[SF_OVERSAMPLE_RATIO - 1][i];
		ptr_state_vec_9XAGM->GravErrPriS[i] *= -1.0;
		ptr_state_vec_9XAGM->GravErrPriS[i] *= ptr_state_vec_9XAGM->AccData.ScaleFactor * GTOMSEC2;
		ptr_state_vec_9XAGM->GravErrPriS[i] += ptr_state_vec_9XAGM->LinAccTC *ptr_state_vec_9XAGM->AccPostS[i];
		ptr_state_vec_9XAGM->GravErrPriS[i] -= ptr_state_vec_9XAGM->GravGyrPriS[i];
		//printf("j=%d, k=%f, l=%f, m=%f\n\n", ptr_state_vec_9XAGM->AccData.CountBuff[SF_OVERSAMPLE_RATIO - 1][i], ptr_state_vec_9XAGM->AccData.ScaleFactor,
		//	ptr_state_vec_9XAGM->AccData.CountBuff[SF_OVERSAMPLE_RATIO - 1][i] * ptr_state_vec_9XAGM->AccData.ScaleFactor *GTOMSEC2, ptr_state_vec_9XAGM->AccPostS[i]);
		//printf("n = %f \n", ptr_state_vec_9XAGM->GravErrPriS[i]);
	}

	// Compute the measurement matrix, C using a priori gravity values 
	CrossPdctMtx_3x3(ptr_state_vec_9XAGM->GravGyrPriS, &Atemp);
	ScaleBlkMtx_3x3(-DEG2RAD, &Atemp, &Cmat[0]);
	ScaleBlkMtx_3x3((DEG2RAD * SF_DELTA_T), &Atemp, &Cmat[1]);
	IdentityBlkMtx_3x3(&Cmat[2]);
	//PrintBlkMtx_3x3(&Cmat[0],"Cmat0");
	//PrintBlkMtx_3x3(&Cmat[1],"Cmat1");
	//PrintBlkMtx_3x3(&Cmat[2],"Cmat2");

	/* Compute Kalman gain, K = Qw*C'*inv(C*Qw*C' + Qv) 
	 * F[3] = Qw[3][3]*C[3]'
	 * G = C[3]*F[3] + Qv
	 * Ginv = inv(G)
	 * K[3] = F[3]*Ginv
	 */
	
	// F[3] = Qw[3][3]*C[3]'
	for( i = CHX; i <= CHZ; i++) {
		ZeroBlkMtx_3x3(&Ftemp[i]);
		for( j = CHX; j <= CHZ; j++) {
			TranspBlkMtx_3x3(&Cmat[j], &Atemp);
			MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[i][j]), &Atemp, &Btemp);
			//PrintBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[i][j]),"Qwij");
			//PrintBlkMtx_3x3(&Btemp,"Qwij*Cmati'");
			AddBlkMtx_3x3(&Ftemp[i], &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &Ftemp[i]);
			//PrintBlkMtx_3x3(&Ftemp[i],"Fi");
		}
	}
	
	// G = C[3]*F[3] + Qv
	IdentityBlkMtx_3x3(&Qv_mat);
	ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->MeasNoiseVarAcc, &Qv_mat, &Gtemp);
	//PrintBlkMtx_3x3(&Gtemp,"Qv");
	for( i = CHX; i <= CHZ; i++) {
		MultBlkMtx_3x3(&Cmat[i], &Ftemp[i], &Atemp);
		//PrintBlkMtx_3x3(&Atemp,"Ci*Fi");
		AddBlkMtx_3x3( &Atemp, &Gtemp, &Btemp );
		SetBlkMtx_3x3( &Btemp, &Gtemp );
		//PrintBlkMtx_3x3(&Gtemp,"Qv+Ci*Fi");
	}
	
	// Ginv = inv(G)
	inv_exist = InvBlkSymMtx1_3x3(&Gtemp, &Ginv);

	// K[3] = F[3]*Ginv
	if (inv_exist == 1) {
		for(  i = CHX; i <= CHZ; i++) {
			MultBlkMtx_3x3(&Ftemp[i], &Ginv, &(ptr_state_vec_9XAGM->KalmanGain[i]));
		}
	}
#if 0	
	printf("\nKgain %f, %f, %f, %f, %f, %f, %f, %f,%f; %f, %f, %f,%f, %f, %f, %f,%f, %f; %f, %f,%f, %f, %f, %f, %f, %f, %f\n\n",
		ptr_state_vec_9XAGM->KalmanGain[0].elem[0][0], ptr_state_vec_9XAGM->KalmanGain[0].elem[0][1], ptr_state_vec_9XAGM->KalmanGain[0].elem[0][2],
		ptr_state_vec_9XAGM->KalmanGain[0].elem[1][0], ptr_state_vec_9XAGM->KalmanGain[0].elem[1][1], ptr_state_vec_9XAGM->KalmanGain[0].elem[1][2],
		ptr_state_vec_9XAGM->KalmanGain[0].elem[2][0], ptr_state_vec_9XAGM->KalmanGain[0].elem[2][1], ptr_state_vec_9XAGM->KalmanGain[0].elem[2][2],
		ptr_state_vec_9XAGM->KalmanGain[1].elem[0][0], ptr_state_vec_9XAGM->KalmanGain[1].elem[0][1], ptr_state_vec_9XAGM->KalmanGain[1].elem[0][2],
		ptr_state_vec_9XAGM->KalmanGain[1].elem[1][0], ptr_state_vec_9XAGM->KalmanGain[1].elem[1][1], ptr_state_vec_9XAGM->KalmanGain[1].elem[1][2],
		ptr_state_vec_9XAGM->KalmanGain[1].elem[2][0], ptr_state_vec_9XAGM->KalmanGain[1].elem[2][1], ptr_state_vec_9XAGM->KalmanGain[1].elem[2][2],
		ptr_state_vec_9XAGM->KalmanGain[2].elem[0][0], ptr_state_vec_9XAGM->KalmanGain[2].elem[0][1], ptr_state_vec_9XAGM->KalmanGain[2].elem[0][2],
		ptr_state_vec_9XAGM->KalmanGain[2].elem[1][0], ptr_state_vec_9XAGM->KalmanGain[2].elem[1][1], ptr_state_vec_9XAGM->KalmanGain[2].elem[1][2],
		ptr_state_vec_9XAGM->KalmanGain[2].elem[2][0], ptr_state_vec_9XAGM->KalmanGain[2].elem[2][1], ptr_state_vec_9XAGM->KalmanGain[2].elem[2][2]);
#endif // 0

	// Measurement Update of state vector, xe+ = xe- + K*ze 
	// where z is input measurement with ze = C*xe = [(gA- - gG-); (mM- - mG-)] 
	// Mupdt = K[3]*state_var.GravErrPriS;
#if 0
	for( k = CHX; k <= CHZ; k++) {
		Mupdt[3*k] = 0.0;
		Mupdt[3*k+1] = 0.0;
		Mupdt[3*k+2] = 0.0;
		for( i = CHX; i <= CHZ; i++) {
			Mupdt[3*k] += ptr_state_vec_9XAGM->KalmanGain[k].elem[0][i] * ptr_state_vec_9XAGM->GravErrPriS[i];
			Mupdt[3*k+1] += ptr_state_vec_9XAGM->KalmanGain[k].elem[1][i] * ptr_state_vec_9XAGM->GravErrPriS[i];
			Mupdt[3*k+2] += ptr_state_vec_9XAGM->KalmanGain[k].elem[2][i] * ptr_state_vec_9XAGM->GravErrPriS[i];
		}
	}
#endif // 0
	for (k = CHX; k <= CHZ; k++) {
		for (j = CHX; j <= CHZ; j++)
		{
			Mupdt[3*k + j] = 0.0;
			for (i = CHX; i <= CHZ; i++) {
				Mupdt[3*k + j] += ptr_state_vec_9XAGM->KalmanGain[k].elem[j][i] * ptr_state_vec_9XAGM->GravErrPriS[i];
			}
		}
	}

	// Update the following state information for next iteration
	for(  j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->OrntErrPostS[j] = Mupdt[j];
		ptr_state_vec_9XAGM->BiasErrPostS[j] = Mupdt[j+3];
		ptr_state_vec_9XAGM->AccErrPostS[j] = Mupdt[j+6];
		gyro_corr[j] = -Mupdt[j] / SF_DELTA_T;
	}

	// Update the rotation matrix by rotating it back to remove error, as estimated
	// by OrntErrPostS (error in orientation angles based on measurement update) 
	// Integrate quaternion
	QuatIntegrate(&(ptr_state_vec_9XAGM->QuatPost), gyro_corr, SF_DELTA_T, &QuatInt);
	// Normalize quaternion
	QuatNormal(&QuatInt, &(ptr_state_vec_9XAGM->QuatPost));
	// Update rotation matrix
	Quat2RotMtx(&(ptr_state_vec_9XAGM->QuatPost), ptr_state_vec_9XAGM->RotMtxPost);
	// Use accelerometer timestamp to maintain sync with sensor data
	// Check for missing accel data
	if( ptr_state_vec_9XAGM->AccData.timestamp - ptr_state_vec_9XAGM->MeasupdtTS > SF_ACCEL_MAX_MISS_DUR) {
		// if accel data used is not available for a long time, mark mode_op to degraded accel mode
		ptr_state_vec_9XAGM->OpMode &= ~(SF_ACC_MASK);
		ptr_state_vec_9XAGM->OpMode |= SF_ACC_MISSING;
	}
	else {
		// if accel data used is available, mark mode_op to normal accel mode
		ptr_state_vec_9XAGM->OpMode &= ~(SF_ACC_MASK);
	}
	ptr_state_vec_9XAGM->MeasupdtTS = ptr_state_vec_9XAGM->AccData.timestamp;

	// Update aposteriori covariance matrix, P_post = (I9 - K*C)*Qw
	//  Compute A= (I9 - K*C)
	for(  i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			if (i == j) {
				IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
			}
			else {
				ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
			}
			MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->KalmanGain[i]), &Cmat[j], &Atemp);
			ScaleBlkMtx_3x3(-1.0, &Atemp, &Btemp);
			AddBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]), &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
		}
	}
	// Compuate P_post = A*Qw
	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			ZeroBlkMtx_3x3(&Atemp);
			for( k = 0; k < 3; k++) {
				MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][k]), &(ptr_state_vec_9XAGM->ProcNoiseVar[k][j]), &Btemp);
				AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
				SetBlkMtx_3x3(&Gtemp, &Atemp);
			}
			TranspBlkMtx_3x3(&Atemp, &Btemp);
			AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
			ScaleBlkMtx_3x3(SF_9XAGM_COVARIANCE_SYMMETRY_FACTOR, &Gtemp, &Atemp);
			if (i == j) {
				IdentityBlkMtx_3x3(&Gtemp);
				ScaleBlkMtx_3x3(EPSILON, &Gtemp, &Btemp);
			}
			else {
				ZeroBlkMtx_3x3(&Btemp);
			}
			AddBlkMtx_3x3( &Atemp, &Btemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
		}
	}

	orient_err = ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[0][0] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[0][0];
	orient_err += ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[1][1] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[1][1];
	orient_err += ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[2][2] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[2][2];
	if((ptr_state_vec_9XAGM->update_ErrCovMtx == 0) && (orient_err < SF_MAX_ORIENT_ERR)) {
		ptr_state_vec_9XAGM->update_ErrCovMtx = 1;
	}

	// Update gyro bias and lin acc based on meas update
	for( j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->BiasPostS[j] -= ptr_state_vec_9XAGM->BiasErrPostS[j];
		ptr_state_vec_9XAGM->AccPostS[j] *= ptr_state_vec_9XAGM->LinAccTC;
		ptr_state_vec_9XAGM->AccPostS[j] -= ptr_state_vec_9XAGM->AccErrPostS[j];
		//printf("u = %f and v = %f \n", ptr_state_vec_9XAGM->AccErrPostS[j], ptr_state_vec_9XAGM->AccPostS[j]);
	}
	for( j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->AccPostG[j] = 0.0;
		for( k = CHX; k <= CHZ; k++) {
			ptr_state_vec_9XAGM->AccPostG[j] += (ptr_state_vec_9XAGM->RotMtxPost[j][k] * ptr_state_vec_9XAGM->AccPostS[k]);
		}
	}
	ptr_state_vec_9XAGM->AccPostG[3] -= GTOMSEC2;

}

/**
 * @brief Performs Kalman filter measurement update (correction step) using magnetometer data
 *
 * This function implements the Extended Kalman Filter (EKF) measurement update for magnetometer
 * observations, providing absolute heading (yaw) correction and magnetic disturbance estimation.
 * This is the key differentiator between 6-axis and 9-axis sensor fusion.
 *
 * Magnetometer Measurement Model:
 * - Measures Earth's magnetic field in sensor frame
 * - Normalized for direction-only comparison (magnitude-invariant)
 * - Compares measured field to expected field from current orientation
 * - Provides absolute yaw reference, eliminating gyro drift in heading
 *
 * Algorithm steps:
 * 1. Apply hard iron calibration offset to raw magnetometer data
 * 2. Normalize measured magnetic field for direction comparison
 * 3. Initialize or use existing magnetic field reference in global frame
 * 4. Project reference field to sensor frame using current orientation
 * 5. Compute measurement innovation (magnetic field error)
 * 6. Build measurement matrix C for magnetometer observation model:
 *    - C[0]: Orientation error coupling (cross-product with mag field)
 *    - C[1]: Gyro bias coupling (zero - mag not affected by gyro bias)
 *    - C[2]: Linear acceleration coupling (zero - mag not affected by accel)
 *    - C[3]: Magnetic disturbance coupling (identity)
 * 7. Compute Kalman gain K = Q_w * C^T * inv(C * Q_w * C^T + R_mag)
 * 8. Apply state correction for all 12 states
 * 9. Update quaternion and rotation matrix
 * 10. Update error covariance matrix (4x4 blocks)
 * 11. Store calibrated magnetometer data in sensor and global frames
 *
 * @param[in,out] ptr_state_vec_9XAGM Pointer to 9-axis algorithm state structure
 * @param[in] ptr_mag_data Pointer to magnetometer sensor data containing:
 *                         - CountAvg: Averaged magnetometer counts [3]
 *                         - ScaleFactor: Magnetometer scale factor (μT per count)
 *
 * @return None
 *
 * @note First magnetometer measurement initializes magnetic field reference
 * @note Magnetometer provides direction only; magnitude variations are ignored
 * @note Measurement is skipped if magnetometer magnitude < EPSILON (invalid data)
 * @note Reference field is transformed to sensor frame: B_sensor = R * B_global
 * @note Magnetic disturbance state absorbs local field variations
 * @note This update affects all states due to coupling in error covariance
 * @note Measurement noise R_mag accounts for sensor noise and model uncertainty
 * @note Covariance symmetry enforced: P = 0.5*(P + P^T) + ε*I
 * @note Reference: AN5023 for 9-axis magnetometer fusion implementation
 */
static void sf_9xagm_algo_measupdate_mag(state_vec_9XAGM_t *ptr_state_vec_9XAGM,
                                          phys_sensor_t     *ptr_mag_data)
{
	blk_mtx_3x3_t        Atemp, Btemp;
	blk_mtx_3x3_t        Ftemp[4], Gtemp, Ginv;
	blk_mtx_3x3_t        Cmat[4];
	blk_mtx_3x3_t        Qv_mat;
	quaternion_double_t  QuatInt;
	double               Mupdt[12];
	double               gyro_corr[3];
	double               mag_measured[3], mag_expected[3];
	double               mag_error[3];
	double               mag_norm;
	double               orient_err;
	uint32_t             inv_exist;
	uint32_t             i, j, k;

	inv_exist = 1;

	// Get measured magnetometer data (average of buffered samples)
	for(i = CHX; i <= CHZ; i++) {
		mag_measured[i] = ptr_mag_data->CountAvg[i] * ptr_mag_data->ScaleFactor;
		// Apply hard iron calibration offset
		mag_measured[i] -= ptr_state_vec_9XAGM->MagCalOffset[i];
	}

	// Normalize measured magnetic field for direction-only comparison
	mag_norm = sqrt(mag_measured[0]*mag_measured[0] +
	               mag_measured[1]*mag_measured[1] +
	               mag_measured[2]*mag_measured[2]);

	if (mag_norm < EPSILON) {
		// Invalid magnetometer reading, skip update
		return;
	}

	for(i = CHX; i <= CHZ; i++) {
		mag_measured[i] /= mag_norm;
	}

	// Calculate expected magnetic field in sensor frame using current orientation
	// mag_expected_sensor = R * mag_reference_global
	// Initialize reference field if not set (first run)
	if (ptr_state_vec_9XAGM->MagFieldRef[0] == 0.0 &&
	    ptr_state_vec_9XAGM->MagFieldRef[1] == 0.0 &&
	    ptr_state_vec_9XAGM->MagFieldRef[2] == 0.0) {
		// Use current measurement as initial reference (normalized)
		// Transform to global frame: mag_global = R^T * mag_sensor
		for(i = CHX; i <= CHZ; i++) {
			ptr_state_vec_9XAGM->MagFieldRef[i] = 0.0;
			for(j = CHX; j <= CHZ; j++) {
				ptr_state_vec_9XAGM->MagFieldRef[i] += ptr_state_vec_9XAGM->RotMtxPost[j][i] * mag_measured[j];
			}
		}
		// Normalize reference field
		double ref_norm = sqrt(ptr_state_vec_9XAGM->MagFieldRef[0]*ptr_state_vec_9XAGM->MagFieldRef[0] +
		                      ptr_state_vec_9XAGM->MagFieldRef[1]*ptr_state_vec_9XAGM->MagFieldRef[1] +
		                      ptr_state_vec_9XAGM->MagFieldRef[2]*ptr_state_vec_9XAGM->MagFieldRef[2]);
		if (ref_norm > EPSILON) {
			for(i = CHX; i <= CHZ; i++) {
				ptr_state_vec_9XAGM->MagFieldRef[i] /= ref_norm;
			}
		}
	}

	// Project reference magnetic field to sensor frame using current orientation
	// mag_expected = R * mag_ref_global
	for(i = CHX; i <= CHZ; i++) {
		mag_expected[i] = 0.0;
		for(j = CHX; j <= CHZ; j++) {
			mag_expected[i] += ptr_state_vec_9XAGM->RotMtxPost[i][j] * ptr_state_vec_9XAGM->MagFieldRef[j];
		}
	}

	// Compute measurement error (innovation)
	for(i = CHX; i <= CHZ; i++) {
		mag_error[i] = mag_measured[i] - mag_expected[i];
	}

	// Compute the measurement matrix C using expected magnetic field
	// C[0] corresponds to orientation error: dB/dθ = [B × ] (cross product matrix)
	CrossPdctMtx_3x3(mag_expected, &Atemp);
	ScaleBlkMtx_3x3(-DEG2RAD, &Atemp, &Cmat[0]);

	// C[1] corresponds to gyro bias: dB/db ≈ 0 (magnetometer not affected by gyro bias directly)
	ZeroBlkMtx_3x3(&Cmat[1]);

	// C[2] corresponds to linear acceleration: dB/da ≈ 0 (mag not affected by accel)
	ZeroBlkMtx_3x3(&Cmat[2]);

	// C[3] corresponds to magnetic disturbance: dB/dd = I (identity)
	IdentityBlkMtx_3x3(&Cmat[3]);

	/* Compute Kalman gain, K = Qw*C'*inv(C*Qw*C' + Qv)
	 * F[4] = Qw[4][4]*C[4]'
	 * G = C[4]*F[4] + Qv
	 * Ginv = inv(G)
	 * K[4] = F[4]*Ginv
	 */

	// F[4] = Qw[4][4]*C[4]'
	for(i = 0; i < 4; i++) {
		ZeroBlkMtx_3x3(&Ftemp[i]);
		for(j = 0; j < 4; j++) {
			TranspBlkMtx_3x3(&Cmat[j], &Atemp);
			MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->ProcNoiseVar[i][j]), &Atemp, &Btemp);
			AddBlkMtx_3x3(&Ftemp[i], &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &Ftemp[i]);
		}
	}

	// G = C[4]*F[4] + Qv
	IdentityBlkMtx_3x3(&Qv_mat);
	ScaleBlkMtx_3x3(ptr_state_vec_9XAGM->MeasNoiseVarMag, &Qv_mat, &Gtemp);

	for(i = 0; i < 4; i++) {
		MultBlkMtx_3x3(&Cmat[i], &Ftemp[i], &Atemp);
		AddBlkMtx_3x3(&Atemp, &Gtemp, &Btemp);
		SetBlkMtx_3x3(&Btemp, &Gtemp);
	}

	// Ginv = inv(G)
	inv_exist = InvBlkSymMtx1_3x3(&Gtemp, &Ginv);

	// K[4] = F[4]*Ginv
	if (inv_exist == 1) {
		for(i = 0; i < 4; i++) {
			MultBlkMtx_3x3(&Ftemp[i], &Ginv, &(ptr_state_vec_9XAGM->KalmanGain[i]));
		}
	} else {
		// Matrix inversion failed, skip this update
		return;
	}

	// Measurement Update of state vector, xe+ = xe- + K*ze
	// Mupdt = K[4]*mag_error
	for(k = 0; k < 4; k++) {
		for(j = CHX; j <= CHZ; j++) {
			Mupdt[3*k + j] = 0.0;
			for(i = CHX; i <= CHZ; i++) {
				Mupdt[3*k + j] += ptr_state_vec_9XAGM->KalmanGain[k].elem[j][i] * mag_error[i];
			}
		}
	}

	// Update the state error estimates
	for(j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->OrntErrPostS[j] = Mupdt[j];
		ptr_state_vec_9XAGM->BiasErrPostS[j] = Mupdt[j+3];
		ptr_state_vec_9XAGM->AccErrPostS[j] = Mupdt[j+6];
		ptr_state_vec_9XAGM->MagDistErrPostS[j] = Mupdt[j+9];
		gyro_corr[j] = -Mupdt[j] / SF_DELTA_T;
	}

	// Update the rotation matrix by rotating it back to remove error
	// Integrate quaternion with correction
	QuatIntegrate(&(ptr_state_vec_9XAGM->QuatPost), gyro_corr, SF_DELTA_T, &QuatInt);
	// Normalize quaternion
	QuatNormal(&QuatInt, &(ptr_state_vec_9XAGM->QuatPost));
	// Update rotation matrix
	Quat2RotMtx(&(ptr_state_vec_9XAGM->QuatPost), ptr_state_vec_9XAGM->RotMtxPost);

	// Update aposteriori covariance matrix, P_post = (I12 - K*C)*Qw
	//  Compute A = (I12 - K*C)
	for(i = 0; i < 4; i++) {
		for(j = 0; j < 4; j++) {
			if (i == j) {
				IdentityBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
			} else {
				ZeroBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
			}
			MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->KalmanGain[i]), &Cmat[j], &Atemp);
			ScaleBlkMtx_3x3(-1.0, &Atemp, &Btemp);
			AddBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]), &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
		}
	}

	// Compute P_post = A*Qw (ensure symmetry)
	for(i = 0; i < 4; i++) {
		for(j = 0; j < 4; j++) {
			ZeroBlkMtx_3x3(&Atemp);
			for(k = 0; k < 4; k++) {
				MultBlkMtx_3x3(&(ptr_state_vec_9XAGM->ErrCovMtxPost[i][k]), &(ptr_state_vec_9XAGM->ProcNoiseVar[k][j]), &Btemp);
				AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
				SetBlkMtx_3x3(&Gtemp, &Atemp);
			}
			TranspBlkMtx_3x3(&Atemp, &Btemp);
			AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
			ScaleBlkMtx_3x3(SF_9XAGM_COVARIANCE_SYMMETRY_FACTOR, &Gtemp, &Atemp);
			if (i == j) {
				IdentityBlkMtx_3x3(&Gtemp);
				ScaleBlkMtx_3x3(EPSILON, &Gtemp, &Btemp);
			} else {
				ZeroBlkMtx_3x3(&Btemp);
			}
			AddBlkMtx_3x3(&Atemp, &Btemp, &(ptr_state_vec_9XAGM->ErrCovMtxPost[i][j]));
		}
	}

	// Check orientation error for covariance update control
	orient_err = ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[0][0] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[0][0];
	orient_err += ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[1][1] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[1][1];
	orient_err += ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[2][2] * ptr_state_vec_9XAGM->ErrCovMtxPost[0][0].elem[2][2];
	if((ptr_state_vec_9XAGM->update_ErrCovMtx == 0) && (orient_err < SF_MAX_ORIENT_ERR)) {
		ptr_state_vec_9XAGM->update_ErrCovMtx = 1;
	}

	// Update gyro bias (magnetometer provides additional constraints)
	for(j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->BiasPostS[j] -= ptr_state_vec_9XAGM->BiasErrPostS[j];
	}

	// Store calibrated magnetometer data in sensor and global frames
	for(j = CHX; j <= CHZ; j++) {
		ptr_state_vec_9XAGM->MagPostS[j] = mag_measured[j] * mag_norm;
		ptr_state_vec_9XAGM->MagPostG[j] = 0.0;
		for(k = CHX; k <= CHZ; k++) {
			ptr_state_vec_9XAGM->MagPostG[j] += ptr_state_vec_9XAGM->RotMtxPost[k][j] * ptr_state_vec_9XAGM->MagPostS[k];
		}
	}
}

/*==============================================================================
 * Magnetometer Calibration and Utility Functions
 *============================================================================*/

/**
 * @brief Applies magnetometer calibration (hard iron and soft iron correction)
 *
 * This function performs comprehensive magnetometer calibration to correct for both
 * hard iron and soft iron distortions. Hard iron effects are constant magnetic offsets
 * from ferromagnetic materials near the sensor. Soft iron effects are sensor axis
 * misalignments and scale factor errors from ferromagnetic materials that distort
 * the magnetic field.
 *
 * Calibration equation:
 * B_calibrated = C * (B_raw - B_offset)
 * Where:
 * - B_raw: Raw magnetometer measurements
 * - B_offset: Hard iron offset (3-vector)
 * - C: Soft iron correction matrix (3x3)
 * - B_calibrated: Calibrated magnetometer measurements
 *
 * @param[in] mag_raw Raw magnetometer measurements [x, y, z] in μT (uncalibrated)
 * @param[in] cal_offset Hard iron offset calibration [x, y, z] in μT
 *                       Constant bias from nearby ferromagnetic materials
 * @param[in] cal_matrix Soft iron correction matrix [3x3] (dimensionless)
 *                       Corrects for axis misalignment and scale factors
 * @param[out] mag_cal Calibrated magnetometer measurements [x, y, z] in μT
 *
 * @return None
 *
 * @note Hard iron calibration is applied first: mag = mag_raw - offset
 * @note Soft iron calibration is applied second: mag_cal = matrix * mag
 * @note Calibration parameters should be determined through calibration procedure
 * @note Typical hard iron offsets range from -50 to +50 μT
 * @note Soft iron matrix is typically close to identity for quality sensors
 * @note For uncalibrated sensors, use offset=[0,0,0] and matrix=I
 */
void sf_9xagm_apply_mag_calibration(const double mag_raw[3],
                                    const double cal_offset[3],
                                    const double cal_matrix[3][3],
                                    double       mag_cal[3])
{
	double mag_offset_corrected[3];
	uint32_t i, j;

	// Apply hard iron offset correction
	for(i = CHX; i <= CHZ; i++) {
		mag_offset_corrected[i] = mag_raw[i] - cal_offset[i];
	}

	// Apply soft iron matrix correction: mag_cal = cal_matrix * (mag_raw - cal_offset)
	for(i = CHX; i <= CHZ; i++) {
		mag_cal[i] = 0.0;
		for(j = CHX; j <= CHZ; j++) {
			mag_cal[i] += cal_matrix[i][j] * mag_offset_corrected[j];
		}
	}
}

/**
 * @brief Applies magnetic declination correction to convert magnetic north to true north
 *
 * This function corrects for magnetic declination, which is the angle between magnetic
 * north (measured by magnetometer) and true north (geographic north pole). Declination
 * varies by location and changes over time, ranging from -180° to +180°.
 *
 * The correction adjusts only the yaw angle; pitch and roll remain unchanged as they
 * are not affected by the difference between magnetic and true north.
 *
 * Declination convention:
 * - Positive declination: Magnetic north is EAST of true north
 * - Negative declination: Magnetic north is WEST of true north
 *
 * Example locations (approximate 2020 values):
 * - New York, USA: -13° (west)
 * - London, UK: -1° (west)
 * - Tokyo, Japan: -7° (west)
 * - Sydney, Australia: +12° (east)
 *
 * @param[in] mag_declination Local magnetic declination in radians (positive = east of true north)
 *                            Typical range: -π to +π radians (-180° to +180°)
 * @param[in] orientation_mag Orientation relative to magnetic north [yaw, pitch, roll] in degrees
 *                            - yaw: [0, 360°) relative to magnetic north
 *                            - pitch: [-180°, 180°)
 *                            - roll: [-90°, 90°]
 * @param[out] orientation_true Orientation relative to true north [yaw, pitch, roll] in degrees
 *                              - yaw: [0, 360°) relative to true north
 *                              - pitch: unchanged
 *                              - roll: unchanged
 *
 * @return None
 *
 * @note Only yaw (heading) is affected by declination; pitch and roll are copied unchanged
 * @note Declination is added to magnetic heading: yaw_true = yaw_mag + declination
 * @note Result is normalized to [0, 360°) range
 * @note Declination data can be obtained from NOAA's World Magnetic Model (WMM)
 * @note Declination changes slowly over time (~0.1-0.5° per year at mid-latitudes)
 */
void sf_9xagm_apply_declination(double mag_declination,
                                const double orientation_mag[3],
                                double       orientation_true[3])
{
	// Copy pitch and roll unchanged
	orientation_true[1] = orientation_mag[1];  // pitch (theta)
	orientation_true[2] = orientation_mag[2];  // roll (phi)

	// Apply declination correction to yaw (psi)
	// Positive declination means magnetic north is east of true north
	orientation_true[0] = orientation_mag[0] + (mag_declination * RAD2DEG);

	// Normalize yaw to [0, 360) range
	if (orientation_true[0] < 0.0) {
		orientation_true[0] += 360.0;
	} else if (orientation_true[0] >= 360.0) {
		orientation_true[0] -= 360.0;
	}
}

/**
 * @brief Detects magnetic field disturbances that could corrupt heading estimates
 *
 * This function analyzes magnetometer measurements to detect local magnetic disturbances
 * from ferromagnetic objects, electrical equipment, or other magnetic sources. Such
 * disturbances can temporarily corrupt the heading estimate and should be rejected.
 *
 * Detection strategy (two-stage):
 * 1. Magnitude check: Compare field magnitude to reference (threshold deviation)
 * 2. Direction check: Compare field direction to reference (angular deviation)
 *
 * A disturbance is flagged if EITHER:
 * - Magnitude deviation > threshold (e.g., 30% = 0.3)
 * - Angular deviation > SF_9XAGM_MAG_DIRECTION_THRESHOLD (30°)
 *
 * Common disturbance sources:
 * - Ferromagnetic objects (tools, vehicles, buildings)
 * - Electrical equipment (motors, transformers, power lines)
 * - Electronic devices (speakers, hard drives)
 * - Permanent magnets
 *
 * @param[in] mag_data Current magnetometer measurements [x, y, z] in μT
 * @param[in] mag_ref Reference magnetic field vector [x, y, z] in μT
 *                    Established during calibration or initialization
 * @param[in] threshold Magnitude disturbance detection threshold (normalized, 0.0-1.0)
 *                      Typical value: 0.3 (30% deviation)
 *                      Lower = more sensitive, Higher = more tolerant
 *
 * @return Disturbance detection status
 * @retval 1 Disturbance detected (measurements should be rejected)
 * @retval 0 No disturbance (measurements are clean)
 *
 * @note Invalid data (magnitude < EPSILON) is treated as a disturbance
 * @note Magnitude check: |mag_norm - ref_norm| / ref_norm > threshold
 * @note Direction check: mag·ref / (|mag||ref|) < cos(30°) ≈ SF_9XAGM_MAG_DIRECTION_THRESHOLD
 * @note Direction threshold corresponds to 30° angular deviation
 * @note Earth's magnetic field ranges from ~25-65 μT depending on latitude
 * @note Typical disturbances cause 20-50% magnitude changes
 */
int sf_9xagm_detect_mag_disturbance(const double mag_data[3],
                                    const double mag_ref[3],
                                    double       threshold)
{
	double mag_norm, ref_norm;
	double mag_diff;
	uint32_t i;

	// Compute magnitudes
	mag_norm = sqrt(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] + mag_data[2]*mag_data[2]);
	ref_norm = sqrt(mag_ref[0]*mag_ref[0] + mag_ref[1]*mag_ref[1] + mag_ref[2]*mag_ref[2]);

	// Check for valid measurements
	if (mag_norm < EPSILON || ref_norm < EPSILON) {
		return 1;  // Invalid data treated as disturbance
	}

	// Compute magnitude difference (normalized)
	mag_diff = fabs(mag_norm - ref_norm) / ref_norm;

	// Check if magnitude difference exceeds threshold
	if (mag_diff > threshold) {
		return 1;  // Disturbance detected
	}

	// Also check direction consistency by computing dot product
	// of normalized vectors
	double dot_product = 0.0;
	for(i = CHX; i <= CHZ; i++) {
		dot_product += (mag_data[i] / mag_norm) * (mag_ref[i] / ref_norm);
	}

	// If vectors are significantly misaligned, flag as disturbance
	// dot_product < cos(30°) ≈ SF_9XAGM_MAG_DIRECTION_THRESHOLD indicates > 30° angular difference
	if (dot_product < SF_9XAGM_MAG_DIRECTION_THRESHOLD) {
		return 1;  // Disturbance detected (direction mismatch)
	}

	return 0;  // No disturbance detected
}

/**
 * @brief Computes adaptive magnetometer fusion gain based on motion and stability
 *
 * This function dynamically adjusts the weight given to magnetometer measurements
 * in the sensor fusion algorithm based on device motion and magnetic field stability.
 * The gain is reduced during motion or magnetic disturbances to prevent corrupting
 * the orientation estimate with unreliable magnetometer data.
 *
 * Adaptive gain strategy:
 * - High gain (near 1.0): Stationary device, stable magnetic field
 * - Low gain (near 0.0): High motion, unstable magnetic field
 * - Exponential decay with SF_9XAGM_ADAPTIVE_GAIN_DECAY_FACTOR = 5.0
 *
 * Motion detection criteria:
 * - Angular velocity > SF_9XAGM_GYRO_MOTION_THRESHOLD (0.5 rad/s ≈ 28.6°/s)
 * - Linear acceleration deviation > SF_9XAGM_ACCEL_DEVIATION_THRESHOLD (0.3g)
 *
 * Gain computation:
 * gain = exp(-5*(gyro-0.5)) * exp(-5*(|accel|/g-1-0.3)) * stability
 *
 * @param[in] gyro_magnitude Current angular velocity magnitude in rad/s
 *                           Typical stationary: < 0.1 rad/s
 *                           Typical motion: 0.5-5 rad/s
 * @param[in] accel_magnitude Current linear acceleration magnitude in m/s²
 *                            Typical stationary: ≈9.81 m/s² (1g)
 *                            Typical motion: deviates from 1g
 * @param[in] mag_stability Magnetometer field stability metric (0.0 to 1.0)
 *                          1.0 = perfectly stable field
 *                          0.0 = highly unstable/disturbed field
 *
 * @return Adaptive gain value
 * @retval 0.0 to 1.0 Gain multiplier for magnetometer fusion weight
 *                    1.0 = full trust, 0.0 = no trust
 *
 * @note Gyro threshold SF_9XAGM_GYRO_MOTION_THRESHOLD = 0.5 rad/s (28.6°/s)
 * @note Accel threshold SF_9XAGM_ACCEL_DEVIATION_THRESHOLD = 0.3g
 * @note Decay factor SF_9XAGM_ADAPTIVE_GAIN_DECAY_FACTOR = 5.0 (exponential)
 * @note All factors are multiplied: gain = gyro_factor * accel_factor * stability
 * @note Result is clamped to [0.0, 1.0] range
 * @note This adaptive mechanism prevents magnetic disturbances from corrupting orientation
 * @note Lower gain means less magnetometer influence, more reliance on gyroscope
 */
double sf_9xagm_adaptive_mag_gain(double gyro_magnitude,
                                  double accel_magnitude,
                                  double mag_stability)
{
	double gain = 1.0;

	// Reduce gain during high angular velocity (likely rotating)
	// Threshold: SF_9XAGM_GYRO_MOTION_THRESHOLD rad/s (~28.6 deg/s)
	if (gyro_magnitude > SF_9XAGM_GYRO_MOTION_THRESHOLD) {
		double gyro_factor = exp(-SF_9XAGM_ADAPTIVE_GAIN_DECAY_FACTOR * (gyro_magnitude - SF_9XAGM_GYRO_MOTION_THRESHOLD));
		gain *= gyro_factor;
	}

	// Reduce gain during high linear acceleration (likely experiencing external forces)
	// Threshold: deviation from 1g by more than SF_9XAGM_ACCEL_DEVIATION_THRESHOLD
	double accel_dev = fabs(accel_magnitude / GTOMSEC2 - 1.0);
	if (accel_dev > SF_9XAGM_ACCEL_DEVIATION_THRESHOLD) {
		double accel_factor = exp(-SF_9XAGM_ADAPTIVE_GAIN_DECAY_FACTOR * (accel_dev - SF_9XAGM_ACCEL_DEVIATION_THRESHOLD));
		gain *= accel_factor;
	}

	// Reduce gain if magnetic field is unstable
	// mag_stability should be 1.0 for stable field, 0.0 for unstable
	gain *= mag_stability;

	// Clamp gain to valid range [0.0, 1.0]
	if (gain < 0.0) gain = 0.0;
	if (gain > 1.0) gain = 1.0;

	return gain;
}

void sf_9xagm_algo_stop(uintptr_t sf_algo_id)
{
	state_vec_9XAGM_t *ptr_state_vec_9XAGM;

	// check for input validity
	if (sf_algo_id == (uintptr_t)NULL) {
		printf("invalid input \n");
		return;
	}
	// data integrity check
	ptr_state_vec_9XAGM = (state_vec_9XAGM_t *)sf_algo_id;
	if (ptr_state_vec_9XAGM->AlgoID != sf_algo_id) {
		printf("algo id not matching \n");
		return;
	}

	printf("stop algo with algo id = %zu\n", sf_algo_id);
	free(ptr_state_vec_9XAGM);

	return;

} /* sf_9xagm_algo_stop */


  /**
  * @brief: This function is an interface between SF algorithm and algorithm manager.
  * It is used by algoritm manager to run an existing 6axis (accel+gyro) SF algorithm, after
  * all required sensor data has been collected.
  *
  * @param[in]: sf_algo_id: SF algo id that was provided to algo manager at the time
  *                         of algo initialization (creation)
  *             ptr_algo_out: .
  *
  * @param[out]: None
  *
  */
void sf_9xagm_algo_run(uintptr_t        sf_algo_id,
	sf_algo_output_t *ptr_algo_out)
{

	state_vec_9XAGM_t *ptr_state_vec_9XAGM;

	// check for input validity
	if (sf_algo_id == (uintptr_t)NULL) {
		printf("invalid input \n");
		return;
	}
	if (ptr_algo_out == NULL) {
		printf("invalid output mem \n");
		return;
	}

	ptr_state_vec_9XAGM = (state_vec_9XAGM_t *)sf_algo_id;
	if (ptr_state_vec_9XAGM->AlgoID != sf_algo_id) {
		printf("invalid algo id \n");
		return;
	}

	sf_9xagm_algo_run_orig(ptr_state_vec_9XAGM, ptr_algo_out);

	// unsignal sf run
	algo_sf_9xagm_unsignal_sf_run();

} /* sf_9xagm_algo_run */
