#ifndef _TASKS_H_
#define _TASKS_H_

#include "algo_sf_types.h"
#include <stdint.h>

// Sensor definations
#define ACC 0
#define GYR 1
#define MAG 2

// function prototypes for functions in tasks_func.c
int32_t RdSensData_Run(void);
void AOP_SF_Fusion_Init(void);
void AOP_SF_Fusion_Stop(void);
int32_t AOP_SF_Data_PreProc(sensor_data_t *AOP_acc, sensor_data_t *AOP_gyr);

#endif /* _TASKS_H_ */
