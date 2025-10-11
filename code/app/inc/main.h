#ifndef _MAIN_H_
#define _MAIN_H_

/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_types.h"
#include "algo_sf_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "platform_compat.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

/**************************************************************************************
                 MPU9250 Sensor Configuration
 * Accel: 16bit ±4g | Gyro: 16bit ±1000dps | Mag: 16bit ±4800µT | Sampling: 100Hz
**************************************************************************************/

// ACCELEROMETER: 16-bit ±4g range
#define MPU9250_FGPERCOUNT 	      0.0001220703125F   // g per count (±4g / 32767 counts)
#define MPU9250_COUNTSPERG        8192              // counts per g (1 / 0.0001220703125)
#define MPU9250_ACC_RANGE_G       4                 // ±4g full-scale range

// GYROSCOPE: 16-bit ±1000dps range  
#define MPU9250_FDPSPERCOUNT      0.030487804878F   // dps per count (±1000dps / 32767 counts)
#define MPU9250_COUNTSPERDPS      32                // counts per dps (1 / 0.03125)
#define MPU9250_GYRO_RANGE_DPS    1000              // ±1000dps full-scale range

// MAGNETOMETER: 16-bit ±4800µT range
#define MPU9250_FUTPERCOUNT       0.146489F         // µT per count (±4800µT / 32767 counts)
#define MPU9250_COUNTSPERUT       6.8F              // counts per µT (1 / 0.146489)  
#define MPU9250_MAG_RANGE_UT      4800              // ±4800µT full-scale range

// SAMPLING RATE: 100Hz (defined in algo_sf_types.h)
// Gyro: 100Hz | Accel/Mag: 25Hz (4:1 oversampling) | Kalman: 25Hz


#endif /* _MAIN_H_ */