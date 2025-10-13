/**
 * @file algo_sf_interface.h
 * @brief External API interface for sensor fusion algorithms
 *
 * This header defines the public API for sensor fusion algorithms, including
 * data structures for algorithm initialization, sensor data input, and fusion
 * output. This interface is designed to be platform-independent and supports
 * both 6-axis (accel+gyro) and 9-axis (accel+gyro+mag) sensor fusion.
 *
 * @author Vikas Yadav
 * @date 2020
 */

#ifndef _ALGO_SF_INTERFACE_H_
#define _ALGO_SF_INTERFACE_H_

#include <stdint.h>

/******************************************************************************
 *                         SENSOR IDENTIFIERS
 ******************************************************************************/

/** @brief Accelerometer sensor identifier */
#define ACC 0

/** @brief Gyroscope sensor identifier */
#define GYRO 1

/** @brief Magnetometer sensor identifier */
#define MAG 2

/******************************************************************************
 *                         TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Sensor fusion algorithm initialization data
 *
 * Contains sensor scale factors required to convert raw sensor counts to
 * physical units (g, dps, µT). These values are platform and sensor-specific.
 */
typedef struct sf_algo_init_data {
	float Acc_GPERCOUNT;      /**< Accelerometer scale factor (g/count) */
	float Gyro_DPSPERCOUNT;   /**< Gyroscope scale factor (degrees per second/count) */
	float Mag_UTPERCOUNT;     /**< Magnetometer scale factor (microTesla/count) */
} sf_algo_init_data_t;

/**
 * @brief Sensor fusion algorithm type enumeration
 *
 * Defines the supported sensor fusion algorithms
 */
typedef enum algo_type {
	SF_6AG = 1,              /**< 6-axis: Accelerometer + Gyroscope */
	SF_9AGM,                 /**< 9-axis: Accelerometer + Gyroscope + Magnetometer */
	SF_NUM_TYPES = SF_9AGM   /**< Total number of algorithm types */
} algo_type_t;

/**
 * @brief Sensor fusion algorithm feature types
 *
 * Defines various output features supported by the algorithms
 */
typedef enum algo_feature_type {
	SF_GRAVITY_VECTOR = 1,      /**< Gravity vector estimation */
	SF_LINEAR_ACC,              /**< Linear acceleration (gravity removed) */
	SF_ROTATION_VECTOR,         /**< Absolute rotation vector (with magnetometer) */
	SF_GAME_ROTATION_VECTOR,    /**< Game rotation vector (without magnetometer) */
	SF_ORIENTATION,             /**< Euler angles (roll, pitch, yaw) */
	SF_NUM_FEATURES = SF_ORIENTATION  /**< Total number of features */
} algo_feature_t;

/**
 * @brief Quaternion structure
 *
 * Represents orientation as a unit quaternion (q0, q1, q2, q3)
 * where q0 is the scalar part and (q1, q2, q3) is the vector part
 */
typedef struct quaternion
{
	float q0;	/**< Scalar component (w) */
	float q1;	/**< X vector component */
	float q2;	/**< Y vector component */
	float q3;	/**< Z vector component */
} quaternion_t;

/**
 * @brief Rotation vector union
 *
 * Represents rotation vectors for different fusion types
 */
typedef union rotation_vector {
	float rot_vec[4];           /**< Rotation vector (9-axis: A+G+M) */
	float game_rot_vec[4];      /**< Game rotation vector (6-axis: A+G) */
} rotation_vector_t;


/**
 * @brief Sensor fusion algorithm output data
 *
 * Contains all output data from the sensor fusion algorithm including orientation,
 * linear acceleration, gravity vector, and calibrated sensor values. This structure
 * is populated after each fusion iteration.
 */
typedef struct sf_algo_output {
	uint64_t          timestamp_ns;   /**< Timestamp of output calculation (nanoseconds) */
	uint32_t          valid_flag;     /**< Validity flags for output fields */
	uint32_t          mode;           /**< Current mode of operation */
	uint32_t          algo_type;      /**< Algorithm type (6-axis or 9-axis) */
	float             gravity[3];     /**< Gravity vector in device/sensor frame (m/s²) */
	float             linear_acc[3];  /**< Linear acceleration in global frame (m/s²) */
	float             orientation[3]; /**< Euler angles: [yaw, pitch, roll] (degrees) */
	rotation_vector_t rotation_vec;   /**< Rotation vector relative to global frame */
	float             cal_acc[3];     /**< Calibrated accelerometer data (low-pass filtered) */
	float             cal_gyro[3];    /**< Calibrated gyroscope data (bias removed) */
	quaternion_t      quat;           /**< Orientation quaternion */
	uint32_t          reserved[3];    /**< Reserved for future use */
	uint32_t          crc;            /**< CRC checksum */
} sf_algo_output_t;

/**
 * @brief Standardized sensor data input structure
 *
 * Provides a unified format for passing sensor data to the fusion algorithm.
 * Supports 16-bit sensor data with timestamp and accuracy information.
 */
typedef struct sensor_data {
	int64_t  timestamp;      /**< 64-bit timestamp (nanoseconds) */
	uint32_t sensorID;       /**< Sensor identifier: ACC=0, GYRO=1, MAG=2 */
	int16_t  sensordata[3];  /**< 16-bit 3D sensor data [x, y, z] */
	int16_t  accuracy;       /**< Accuracy/reliability of sensor data */
	int32_t  reserved;       /**< Reserved for future use */
} sensor_data_t;

#endif /* _ALGO_SF_INTERFACE_H_ */
