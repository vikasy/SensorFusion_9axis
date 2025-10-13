/**
 * @file algo_sf_include_all.h
 * @brief Master include file for all sensor fusion algorithm components
 *
 * This convenience header includes all necessary sensor fusion headers and
 * declares external references to global sensor data structures. Include this
 * file to get access to all sensor fusion functionality in one go.
 *
 * @author Vikas Yadav
 * @date 2020
 */

#ifndef _INCLUDE_ALL_H_
#define _INCLUDE_ALL_H_

/* Include all sensor fusion algorithm modules */
#include "approximations.h"
#include "fusion.h"
#include "matrix.h"
#include "orientation.h"
#include "tasks.h"

/******************************************************************************
 *                         EXTERNAL DATA REFERENCES
 ******************************************************************************/

/** @brief Global accelerometer sensor data structure */
extern accel_sensor_t this_accel_data;

/** @brief Global gyroscope sensor data structure */
extern gyro_sensor_t this_gyro_data;

/** @brief Global 6-axis sensor fusion state vector */
extern state_vec_6XAG_t this_state_vec_6XAG;

#endif /* _INCLUDE_ALL_H_ */
