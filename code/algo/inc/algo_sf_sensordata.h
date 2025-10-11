#ifndef _ALGO_SF_SENSORDATA_H_
#define _ALGO_SF_SENSORDATA_H_

/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_types.h"

#ifdef USE_6AXIS_FUSION
/**
* @brief: 6-axis sensor fusion data preprocessing
* Interface for processing accelerometer and gyroscope data. This function is called 
* by algorithm manager everytime there is physical sensor data available which is 
* needed by the algo (in this case, accel and gyro sensor data). 
* If the return value is 3, that means both accel and gyro data have been obtained,
* then, algorithm manager can run one pass of 6-axis SF to process input data.
*
* @param[in]: sf_algo_id: SF algo id that was provided to algo manager at the time
*                         of algo initialization (creation)
*             ptr_sensor_raw_data: pointer to raw physical (acc/gyro) sensor data
*
* @param[out]: 0 if invalid input/algo id or if no valid data collected since last SF algo run,
*              1 if only accel data has been collected since last SF algo run
*              2 if only gyro data has been collected since last SF algo run
*              3 if both accel and gyro data have been collected since last SF algo run
*/
uint32_t sf_6xag_data_preproc(uintptr_t        sf_algo_id,
                              sensor_data_t    *ptr_sensor_raw_data);

/**
* @brief: Reset sf_run signal for 6-axis algorithm
* Clears sensor data ready flags after algorithm execution
*
* @param[in]: none
* @param[out]: none
*/
void algo_sf_6xag_unsignal_sf_run(void);
#endif /* USE_6AXIS_FUSION */

#ifdef USE_9AXIS_FUSION
/**
* @brief: 9-axis sensor fusion data preprocessing
* Interface for processing accelerometer, gyroscope, and magnetometer data
*
* @param[in]: sf_algo_id: SF algo id 
*             ptr_sensor_raw_data: pointer to raw physical sensor data
* @param[out]: status flags indicating which sensors have data ready
*              Bit 0: accelerometer ready
*              Bit 1: gyroscope ready  
*              Bit 2: magnetometer ready
*/
uint32_t sf_9xagm_data_preproc(uintptr_t         sf_algo_id,
                               sensor_data_t    *ptr_sensor_raw_data);

/**
* @brief: Reset sf_run signal for 9-axis algorithm
* Clears sensor data ready flags after algorithm execution
*
* @param[in]: none
* @param[out]: none
*/
void algo_sf_9xagm_unsignal_sf_run(void);
#endif /* USE_9AXIS_FUSION */

#endif /* _ALGO_SF_SENSORDATA_H_ */