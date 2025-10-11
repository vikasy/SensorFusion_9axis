#ifndef _ALGO_SF_INTERFACE_H_
#define _ALGO_SF_INTERFACE_H_

/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include <stdint.h>

// Sensor definitions
#define ACC 0
#define GYRO 1
#define MAG 2

/**************************************************************************************
                 Type Definitions
**************************************************************************************/

// data required for creating an algo data structure
typedef struct sf_algo_init_data {
	float Acc_GPERCOUNT;      // acc sensor scale factor (g/count)
	float Gyro_DPSPERCOUNT;   // gyro sensor scale factor (dps/count)
	float Mag_UTPERCOUNT;     // mag sensor scale factor (µT/count)
} sf_algo_init_data_t;

// the algorithm type to run
typedef enum algo_type {
	SF_6AG = 1,   // 6-axis Accel+Gyro
	SF_9AGM,      // 9-axis Accel+gyro+Mag
	SF_NUM_TYPES = SF_9AGM
} algo_type_t;

// various features supported by algorithms
typedef enum algo_feature_type {
	SF_GRAVITY_VECTOR = 1,
	SF_LINEAR_ACC,
	SF_ROTATION_VECTOR,
	SF_GAME_ROTATION_VECTOR,
	SF_ORIENTATION,
	SF_NUM_FEATURES = SF_ORIENTATION
} algo_feature_t;


// quaternion structure definition
typedef struct quaternion
{
	float q0;	// scalar component
	float q1;	// x vector component
	float q2;	// y vector component
	float q3;	// z vector component
} quaternion_t;


typedef union rotation_vector {
	float rot_vec[4];            // A+G+M SF Algo
	float game_rot_vec[4];       // A+G SF Algo
} rotation_vector_t;


// algorithm output data structure
typedef struct sf_algo_output {
	uint64_t          timestamp_ns;   /* timestamp of output calculation in nanosec */
	uint32_t          valid_flag;     /* indicates validity of output fields listed here */
	uint32_t          mode;           /* mode of operation */
	uint32_t          algo_type;      /* type of SF algo e.g. 6X A+G */
	float             gravity[3];     /* gravitation acceleration in device frame */
	float             linear_acc[3];  /* linear acceleration of device in global frame */
	float             orientation[3]; /* pitch, yaw, and roll angles of the device*/
	rotation_vector_t rotation_vec;   /* rotation vec of device with respect to global frame */
	float             cal_acc[3];     /* calibrated accelerometer sensor data = low pass filtered */
	float             cal_gyro[3];    /* calibrated gyro sensor data = bias removed */
	quaternion_t      quat;           /* associated quaterion output */
	uint32_t          reserved[3];    /* future use */
	uint32_t          crc;
} sf_algo_output_t;


// data structure for standardized sensor data with timestamp to send to algo for processing
typedef struct sensor_data {
	int64_t  timestamp;					// 64bit timestamp ns
	uint32_t sensorID;                  // ACC=0, GYRO=1, MAG=2
	int16_t  sensordata[3];			    // 16bit sensor data 3D
	int16_t  accuracy;					// accuracy of sensor data
	int32_t  reserved;                  // reserved for future use
} sensor_data_t;

#endif /* _ALGO_SF_INTERFACE_H_ */
