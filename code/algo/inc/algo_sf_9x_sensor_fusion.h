/*****
* Author: Vikas Yadav
* Date: 2020
* 9-Axis Sensor Fusion Header (Accelerometer + Gyroscope + Magnetometer)
*/

#ifndef ALGO_SF_9X_SENSOR_FUSION_H
#define ALGO_SF_9X_SENSOR_FUSION_H

#include "algo_sf_types.h"
#include "algo_sf_interface.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_quatmath.h"
#include "algo_sf_matrixmath.h"
#include "algo_sf_6x_sensor_fusion.h"  // Include 6-axis functions for reuse

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
                9-Axis Sensor Fusion Constants and Definitions
**************************************************************************************/

// 9-axis specific constants and parameters
#define SF_9XAGM_QVMAG            1E-6f      // magnetometer measurement noise variance
#define SF_9XAGM_QWMAG            1E-4f      // magnetometer process noise variance  
#define SF_9XAGM_QMagDist         1E-1f      // magnetic disturbance modeling error

// Magnetometer timing constants
#define SF_MAG_MAX_STALE_DUR      2000       // 2 sec (magnetometer can be updated slower)
#define SF_MAG_MAX_MISS_DUR       5000       // 5 sec

// Magnetometer mode operation flags
#define	SF_MAG_MASK               48          // bits 4-5 for magnetometer status
#define	SF_MAG_STALE              16          // bit 4: magnetometer data is stale
#define	SF_MAG_MISSING            32          // bit 5: magnetometer data is missing

// Sensor ready bit flags for 9-axis
#define ACC_READY_BIT             1          // accelerometer data ready
#define GYRO_READY_BIT            2          // gyroscope data ready  
#define MAG_READY_BIT             4          // magnetometer data ready

// 9DOF Kalman filter state vector structure (accelerometer + gyroscope + magnetometer)
typedef struct state_vec_9XAGM {
	uint32_t                Reset;
	uint32_t                OrientInit;
	algo_type_t             AlgoType;
	uintptr_t               AlgoID; // start address of the state_vec_9XAGM
	uint32_t                SensFlags;
	uint32_t                OpMode;
	uint32_t                NomupdtTS;
	uint32_t                MeasupdtTS;
	uint32_t                MagMeasupdtTS;        // magnetometer measurement update timestamp
	phys_sensor_t           AccData;
	phys_sensor_t           GyroData;
	phys_sensor_t           MagData;
	double                  AngRatePrev[3];
	double                  RotMtxPost[3][3];
	quaternion_double_t     QuatPost;
	double                  Omega[3];
	double                  LinAccTC;
	double                  GravGyrPriS[3];
	double                  GravErrPriS[3];
	double                  MeasNoiseVarAcc;
	double                  MeasNoiseVarMag;
	double                  ProcNoiseVarOrient;
	double                  ProcNoiseVarBias;
	double                  ProcNoiseVarLinAcc;
	double                  ProcNoiseVarBiasOrient;
	double                  ProcNoiseVarMagDist;     // magnetic disturbance process noise
	double                  MagCalOffset[3];         // magnetometer calibration offsets (DEPRECATED - use mag_cal_state)
	double                  MagFieldRef[3];          // reference magnetic field vector
	double                  MagCalMatrix[3][3];      // soft iron calibration matrix (DEPRECATED - use mag_cal_state)
	void*                   mag_cal_state;           // pointer to mag_cal_state_t (online calibration)
	blk_mtx_3x3_t           ProcNoiseVar[4][4];      // 4x4 for 9-axis (includes mag disturbance)
	blk_mtx_3x3_t           ErrCovMtxPost[4][4];     // 4x4 for 9-axis (includes mag disturbance)
	blk_mtx_3x3_t           KalmanGain[4]; 
	uint32_t                update_ErrCovMtx;
	double                  OrntErrPostS[3];
	double                  BiasErrPostS[3];
	double                  AccErrPostS[3];
	double                  MagDistErrPostS[3];      // magnetic disturbance error states
	double                  PhiPost;
	double                  ThetaPost;
	double                  PsiPost;
	double                  RhoPost;
	double                  ChiPost;
	double                  BiasPostS[3];
	double                  GravPostS[3];
	double                  AccPostS[3];
	double                  AccPostG[4];             // 4 elements for gravity compensation
	double                  MagPostS[3];             // magnetometer data in sensor frame
	double                  MagPostG[3];             // magnetometer data in global frame
	double                  RotVec[3];
	double                  MagDeclination;          // local magnetic declination
	uint32_t                MagCalValid;             // magnetometer calibration validity flag
} state_vec_9XAGM_t;



/**
* @brief: Initialize 9-axis sensor fusion algorithm
* Creates and initializes 9-axis (accel+gyro+mag) SF algorithm instance
*
* @param[in]: algo_init_data: pointer to algo initialization data including
*                            magnetometer scale factor and calibration
* @param[out]: sf_algo_id: unique algorithm identifier, 0 if failed
*/
uintptr_t sf_9xagm_algo_init(sf_algo_init_data_t *algo_init_data);

/**
* @brief: Main 9-axis sensor fusion algorithm execution
* Runs one iteration of the 9-axis sensor fusion algorithm incorporating
* accelerometer, gyroscope, and magnetometer data for improved orientation
*
* @param[in]: sf_algo_id: SF algo id
*             ptr_algo_out: pointer to algorithm output structure
* @param[out]: none
*/
void sf_9xagm_algo_run(uintptr_t        sf_algo_id,
                       sf_algo_output_t *ptr_algo_out);

/**
* @brief: Stop and cleanup 9-axis sensor fusion algorithm
* Deallocates memory and cleans up algorithm resources
*
* @param[in]: sf_algo_id: SF algo id
* @param[out]: none
*/
void sf_9xagm_algo_stop(uintptr_t sf_algo_id);

/**
* @brief: Reset sf_run signal for 9-axis algorithm
* Clears sensor data ready flags after algorithm execution
*
* @param[in]: none
* @param[out]: none
*/
void algo_sf_9xagm_unsignal_sf_run(void);

// 9-axis specific algorithm functions

// Internal functions (static in implementation file)
// These are declared here for documentation but implemented as static functions

/**
* @brief: Nominal time update using gyroscope data
* Updates orientation state using gyroscope measurements and Kalman filtering
*
* @param[in]: ptr_state_vec_9XAGM: pointer to 9-axis algorithm state vector
* @param[out]: none (updates state vector with time propagation)
*/
void sf_9xagm_algo_nom_timeupdate(state_vec_9XAGM_t *ptr_state_vec_9XAGM);

/**
* @brief: Measurement update using accelerometer data
* Performs Kalman filter measurement update with accelerometer data
*
* @param[in]: ptr_state_vec_9XAGM: pointer to 9-axis algorithm state vector
* @param[out]: none (updates state vector with measurement correction)
*/
void sf_9xagm_algo_measupdate(state_vec_9XAGM_t *ptr_state_vec_9XAGM);

/**
* @brief: Convert rotation matrix to Euler angles
* Computes orientation angles from rotation matrix with gimbal lock handling
*
* @param[in]: RotMtx: rotation matrix [3x3]
*             prev_theta, prev_psi: previous angle values for gimbal lock resolution
* @param[out]: theta, phi, psi, rho, chi: computed orientation angles (degrees)
*/
void sf_9xagm_algo_rotmtx2angles(const double RotMtx[3][3],
                                       double       *theta,
                                       double       *phi,
                                       double       *psi,
                                       double       *rho,
                                       double       *chi,
	                                   double       prev_theta,
	                                   double       prev_psi);

/**
* @brief: Apply magnetometer calibration
* Applies hard and soft iron calibration to raw magnetometer data
*
* @param[in]: mag_raw: raw magnetometer measurements [3]
*             cal_offset: hard iron offset calibration [3]
*             cal_matrix: soft iron correction matrix [3x3] (optional)
* @param[out]: mag_cal: calibrated magnetometer measurements [3]
*/
void sf_9xagm_apply_mag_calibration(const double mag_raw[3],
                                    const double cal_offset[3],
                                    const double cal_matrix[3][3],
                                    double       mag_cal[3]);

/**
* @brief: Compute magnetic declination correction
* Applies magnetic declination correction to convert magnetic north to true north
*
* @param[in]: mag_declination: local magnetic declination in radians
*             orientation_mag: orientation relative to magnetic north [3] (roll, pitch, yaw)
* @param[out]: orientation_true: orientation relative to true north [3] (roll, pitch, yaw)
*/
void sf_9xagm_apply_declination(double mag_declination,
                                const double orientation_mag[3],
                                double       orientation_true[3]);

/**
* @brief: Detect magnetic disturbances
* Analyzes magnetometer data to detect magnetic field disturbances that
* could affect heading accuracy
*
* @param[in]: mag_data: current magnetometer measurements [3]
*             mag_ref: reference magnetic field vector [3]
*             threshold: disturbance detection threshold
* @param[out]: returns 1 if disturbance detected, 0 if field is clean
*/
int sf_9xagm_detect_mag_disturbance(const double mag_data[3],
                                    const double mag_ref[3],
                                    double       threshold);

/**
* @brief: Adaptive magnetometer fusion gain
* Dynamically adjusts magnetometer fusion gain based on motion state
* and magnetic field stability
*
* @param[in]: gyro_magnitude: current angular velocity magnitude
*             accel_magnitude: current linear acceleration magnitude
*             mag_stability: magnetometer field stability metric
* @param[out]: returns adaptive gain value (0.0 to 1.0)
*/
double sf_9xagm_adaptive_mag_gain(double gyro_magnitude,
                                  double accel_magnitude,
                                  double mag_stability);

#ifdef __cplusplus
}
#endif

#endif // ALGO_SF_9X_SENSOR_FUSION_H