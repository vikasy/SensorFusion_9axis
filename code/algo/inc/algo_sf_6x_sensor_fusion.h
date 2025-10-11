/*****
* Author: Vikas Yadav
* Date: 2020
* 6-Axis Sensor Fusion Header (Accelerometer + Gyroscope)
*/

#ifndef ALGO_SF_6X_SENSOR_FUSION_H
#define ALGO_SF_6X_SENSOR_FUSION_H

#include "algo_sf_types.h"
#include "algo_sf_interface.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_quatmath.h"
#include "algo_sf_matrixmath.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
                6-Axis Sensor Fusion Constants and Definitions
**************************************************************************************/

// 6-axis noise parameters
#define SF_6XAG_QVACC             2E-6f
#define SF_6XAG_QWACC             1E-4f
#define SF_6XAG_QVGYRO            0.01f      //in Radians
#define SF_6XAG_QWGYRO            1E-9f      //in Radians

#define SF_6XAG_QOrient           1E-1f      // modeling error in angles
#define SF_6XAG_QBias             1E1f       // modeling error in bias
#define SF_6XAG_QLinAcc           1E1f       // modeling error in lin acc
#define SF_6XAG_QBiasOrient       1E-1f      // modeling error in bias and angle

// Sensor timing parameters
#define SF_GYRO_MAX_STALE_DUR     1000       // 1 sec
#define SF_GYRO_MAX_MISS_DUR      5000       // 5 sec
#define SF_ACCEL_MAX_STALE_DUR    1000       // 1 sec
#define SF_ACCEL_MAX_MISS_DUR     5000       // 5 sec

// Mode of operation depending on sensor data
#define	SF_GYRO_MASK              3
#define	SF_GYRO_STALE             1
#define	SF_GYRO_MISSING           2
#define	SF_ACC_MASK               12
#define	SF_ACC_STALE              4 
#define	SF_ACC_MISSING            8

#define SF_MAX_ORIENT_ERR         100

// Physical sensor structure definition (shared between 6-axis and 9-axis)
typedef struct phys_sensor {
	uint32_t SensorID;							// sensor id
	float    ScaleFactor;					    // g/count (Acc) or dps/count (Gyro)
	int16_t  CountBuff[SF_OVERSAMPLE_RATIO][3]; // buffered measurements (counts)
	int16_t  last_index;					    // index to most recent unaveraged meas in buff
	int16_t  CountAvg[3];						// averaged measurement (counts)
	int64_t  timestamp;                         // sensor data timestamp (ns)
} phys_sensor_t;

// 6DOF Kalman filter accelerometer and gyroscope state vector structure
typedef struct state_vec_6XAG {
	uint32_t                Reset;
	uint32_t                OrientInit;
	algo_type_t             AlgoType;
	uintptr_t               AlgoID; // start address of the state_vec_6XAG
	uint32_t                SensFlags;
	uint32_t                OpMode;
	uint32_t                NomupdtTS;
	uint32_t                MeasupdtTS;
	phys_sensor_t           AccData;
	phys_sensor_t           GyroData;
	double                  AngRatePrev[3];
	double                  RotMtxPost[3][3];
	quaternion_double_t     QuatPost;
	double                  Omega[3];
	double                  LinAccTC;
	double                  GravGyrPriS[3];
	double                  GravErrPriS[3];
	double                  MeasNoiseVarAcc;
	double                  ProcNoiseVarOrient;
	double                  ProcNoiseVarBias;
	double                  ProcNoiseVarLinAcc;
	double                  ProcNoiseVarBiasOrient;
	blk_mtx_3x3_t           ProcNoiseVar[3][3];  // 3x3 for 6-axis (no magnetometer)
	blk_mtx_3x3_t           ErrCovMtxPost[3][3]; // 3x3 for 6-axis (no magnetometer)
	blk_mtx_3x3_t           KalmanGain[3]; 
	uint32_t                update_ErrCovMtx;
	double                  OrntErrPostS[3];
	double                  BiasErrPostS[3];
	double                  AccErrPostS[3];
	double                  PhiPost;
	double                  ThetaPost;
	double                  PsiPost;
	double                  RhoPost;
	double                  ChiPost;
	double                  BiasPostS[3];
	double                  GravPostS[3];
	double                  AccPostS[3];
	double                  AccPostG[4];  // 4 elements for gravity compensation
	double                  RotVec[3];
} state_vec_6XAG_t;

/**
* @brief: Initialize 6-axis sensor fusion algorithm
* Creates and initializes 6-axis (accel+gyro) SF algorithm instance
*
* @param[in]: algo_init_data: pointer to algo initialization data
* @param[out]: sf_algo_id: unique algorithm identifier, 0 if failed
*/
uintptr_t sf_6xag_algo_init(sf_algo_init_data_t *algo_init_data);

/**
* @brief: Main 6-axis sensor fusion algorithm execution
* Runs one iteration of the 6-axis sensor fusion algorithm
*
* @param[in]: sf_algo_id: SF algo id
*             ptr_algo_out: pointer to algorithm output structure
* @param[out]: none
*/
void sf_6xag_algo_run(uintptr_t        sf_algo_id,
                      sf_algo_output_t *ptr_algo_out);

/**
* @brief: Stop and cleanup 6-axis sensor fusion algorithm
* Deallocates memory and cleans up algorithm resources
*
* @param[in]: sf_algo_id: SF algo id
* @param[out]: none
*/
void sf_6xag_algo_stop(uintptr_t sf_algo_id);

/**
* @brief: Reset sf_run signal for 6-axis algorithm
* Clears sensor data ready flags after algorithm execution
*
* @param[in]: none
* @param[out]: none
*/
void algo_sf_6xag_unsignal_sf_run(void);

// Core 6-axis algorithm functions (shared with 9-axis implementation)

/**
* @brief: 6-axis nominal time update (prediction step)
* Performs Kalman filter time update using gyroscope data
*
* @param[in]: ptr_state_vec_6XAG: pointer to algorithm state vector
* @param[out]: none
*/
void sf_6xag_algo_nom_timeupdate(state_vec_6XAG_t *ptr_state_vec_6XAG);

/**
* @brief: 6-axis measurement update (correction step)
* Performs Kalman filter measurement update using accelerometer data
*
* @param[in]: ptr_state_vec_6XAG: pointer to algorithm state vector
* @param[out]: none
*/
void sf_6xag_algo_measupdate(state_vec_6XAG_t *ptr_state_vec_6XAG);

/**
* @brief: Convert rotation matrix to Euler angles
* Converts rotation matrix representation to roll, pitch, yaw angles
*
* @param[in]: RotMtx: rotation matrix [3x3]
*             ThetaPri: previous theta angle (for unwrapping)
*             PsiPri: previous psi angle (for unwrapping)
* @param[out]: ThetaPost: computed theta angle (pitch)
*             PhiPost: computed phi angle (roll)
*             PsiPost: computed psi angle (yaw)
*             RhoPost: computed rho angle
*             ChiPost: computed chi angle
*/
void sf_6xag_algo_rotmtx2angles(const double (*RotMtx)[3],
								double *theta,
								double *phi,
								double *psi,
								double *rho,
								double *chi,
								double prev_theta,
								double prev_psi);

#ifdef __cplusplus
}
#endif

#endif // ALGO_SF_6X_SENSOR_FUSION_H