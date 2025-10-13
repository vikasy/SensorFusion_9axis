/**
 * @file algo_sf_tasks.h
 * @brief Task-level interface definitions for sensor fusion operations
 *
 * This header defines the high-level task interface for sensor fusion operations,
 * including sensor data reading, fusion initialization, and data preprocessing.
 * These functions are typically called from application/RTOS task contexts.
 *
 * @author Vikas Yadav
 * @date 2020
 */

#ifndef _TASKS_H_
#define _TASKS_H_

#include "algo_sf_types.h"
#include <stdint.h>

/******************************************************************************
 *                         SENSOR IDENTIFIERS
 ******************************************************************************/

/** @brief Accelerometer sensor identifier */
#define ACC 0

/** @brief Gyroscope sensor identifier */
#define GYR 1

/** @brief Magnetometer sensor identifier */
#define MAG 2

/******************************************************************************
 *                         FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Read sensor data task
 * @return Status code (0 = success, non-zero = error)
 */
int32_t RdSensData_Run(void);

/**
 * @brief Initialize sensor fusion algorithm
 */
void AOP_SF_Fusion_Init(void);

/**
 * @brief Stop sensor fusion algorithm and free resources
 */
void AOP_SF_Fusion_Stop(void);

/**
 * @brief Preprocess sensor data for fusion algorithm
 * @param[in] AOP_acc Accelerometer sensor data
 * @param[in] AOP_gyr Gyroscope sensor data
 * @return Status code indicating data readiness
 */
int32_t AOP_SF_Data_PreProc(sensor_data_t *AOP_acc, sensor_data_t *AOP_gyr);

#endif /* _TASKS_H_ */
