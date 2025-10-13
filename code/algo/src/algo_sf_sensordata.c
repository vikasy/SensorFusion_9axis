/**
 * @file algo_sf_sensordata.c
 * @brief Sensor data preprocessing and buffering for sensor fusion algorithms
 *
 * This module provides interfaces for collecting and preprocessing raw sensor data
 * (accelerometer, gyroscope, magnetometer) before feeding it to the sensor fusion
 * algorithms. It handles oversampling, averaging, and data ready signaling for both
 * 6-axis (accel+gyro) and 9-axis (accel+gyro+mag) fusion configurations.
 *
 * @author Vikas Yadav
 * @date 2020
 */

#include "algo_sf_sensordata.h"
#include "algo_sf_fusion.h"

/******************************************************************************
 *                         CONSTANT DEFINITIONS
 ******************************************************************************/

/** @brief Bit flag indicating accelerometer data is ready for fusion */
#define SF_ACC_READY_BIT    1

/** @brief Bit flag indicating gyroscope data is ready for fusion */
#define SF_GYRO_READY_BIT   2

/** @brief Bit flag indicating magnetometer data is ready for fusion */
#define SF_MAG_READY_BIT    4

/******************************************************************************
 *                         STATIC VARIABLES
 *******************************************************************************/

// indicates the availability of sensor data for running SF algo
#ifdef USE_6AXIS_FUSION
static uint32_t signal_sf_6xag_run = 0;
#endif

#ifdef USE_9AXIS_FUSION
static uint32_t signal_sf_9xagm_run = 0;
#endif


#ifdef USE_6AXIS_FUSION
/**
 * @brief 6-axis sensor fusion data preprocessing interface
 *
 * This function serves as the interface between the sensor fusion algorithm and
 * algorithm manager for 6-axis fusion. It collects raw accelerometer and gyroscope
 * sensor data, buffers multiple samples for oversampling, computes running averages,
 * and signals when data is ready for fusion processing.
 *
 * @param[in] sf_algo_id SF algorithm ID provided during initialization
 * @param[in] ptr_sensor_raw_data Pointer to raw sensor data (accel or gyro)
 *
 * @return Status flags indicating data availability:
 *         - 0: Invalid input/algo ID or no valid data collected
 *         - SF_ACC_READY_BIT (1): Only accelerometer data ready
 *         - SF_GYRO_READY_BIT (2): Only gyroscope data ready
 *         - 3: Both accelerometer and gyroscope data ready for fusion
 */
uint32_t sf_6xag_data_preproc(uintptr_t        sf_algo_id,
	                          sensor_data_t    *ptr_sensor_raw_data)
{
	static uint32_t  acc_count = 0;
	static uint32_t  gyro_count = 0;
	state_vec_6XAG_t *ptr_state_vec_6XAG;

	// check for input validity
	if (sf_algo_id == (uintptr_t)NULL) {
		printf("invalid input mem pointers \n");
		return 0;
	}
	if (ptr_sensor_raw_data == (sensor_data_t *)NULL) {
		printf("invalid data mem \n");
		return 0;
	}
	
	// data integrity check
	ptr_state_vec_6XAG = (state_vec_6XAG_t *)sf_algo_id;
	
	if (ptr_state_vec_6XAG->AlgoID != sf_algo_id) {
		printf("invalid algo id: expected=0x%zx, got=0x%zx\n", sf_algo_id, ptr_state_vec_6XAG->AlgoID);
		return 0;
	}

	// save instantanous sensor raw data input
	if (ptr_sensor_raw_data->sensorID == ACC) { // accel input
		// hold sensor_data_sem if applicable
		ptr_state_vec_6XAG->AccData.CountBuff[acc_count][0] = (int16_t)ptr_sensor_raw_data->sensordata[0];
		ptr_state_vec_6XAG->AccData.CountBuff[acc_count][1] = (int16_t)ptr_sensor_raw_data->sensordata[1];
		ptr_state_vec_6XAG->AccData.CountBuff[acc_count][2] = (int16_t)ptr_sensor_raw_data->sensordata[2];
		if(acc_count == 0) {
			// save the first timestamp
			ptr_state_vec_6XAG->AccData.timestamp = (int64_t)ptr_sensor_raw_data->timestamp;
		}
		// release sensor_data_sem if held
		//printf("acc count = %d\n", acc_count);
		acc_count = acc_count + 1;
		if(acc_count == SF_OVERSAMPLE_RATIO) {
			acc_count = 0;
			signal_sf_6xag_run = signal_sf_6xag_run | SF_ACC_READY_BIT;
		    float avg_wt = (ptr_state_vec_6XAG->AccData.ScaleFactor) / SF_OVERSAMPLE_RATIO;
			for(int32_t k = CHX; k <= CHZ; k++)
			{
				// Calculate average value periodically before calling SF
				ptr_state_vec_6XAG->AccData.CountAvg[k] = 0;
				for(int32_t i = 0; i < SF_OVERSAMPLE_RATIO; i++)
					ptr_state_vec_6XAG->AccData.CountAvg[k] += ptr_state_vec_6XAG->AccData.CountBuff[i][k];
				// Apply rounding: add half the divisor before integer division
				if (ptr_state_vec_6XAG->AccData.CountAvg[k] > 0)
					ptr_state_vec_6XAG->AccData.CountAvg[k] += SF_OVERSAMPLE_RATIO / 2;
				else
					ptr_state_vec_6XAG->AccData.CountAvg[k] -= SF_OVERSAMPLE_RATIO / 2;
				ptr_state_vec_6XAG->AccData.CountAvg[k] *= avg_wt;
		    }
	    }
	}
	else if (ptr_sensor_raw_data->sensorID == GYRO) { // gyro input preproc
		// hold sensor_data_sem if applicable
		ptr_state_vec_6XAG->GyroData.CountBuff[gyro_count][0] = (int16_t)ptr_sensor_raw_data->sensordata[0];
		ptr_state_vec_6XAG->GyroData.CountBuff[gyro_count][1] = (int16_t)ptr_sensor_raw_data->sensordata[1];
		ptr_state_vec_6XAG->GyroData.CountBuff[gyro_count][2] = (int16_t)ptr_sensor_raw_data->sensordata[2];
		if (gyro_count == 0) {
			// save the first time stamp
			ptr_state_vec_6XAG->GyroData.timestamp = (int64_t)ptr_sensor_raw_data->timestamp;
	    }
		// release sensor_data_sem if held
		//printf("gyro count = %d\n",gyro_count);
		gyro_count = gyro_count + 1;
		if (gyro_count == SF_OVERSAMPLE_RATIO) {
			gyro_count = 0;
			signal_sf_6xag_run = signal_sf_6xag_run | SF_GYRO_READY_BIT;
		}
	}
	
	return signal_sf_6xag_run;
}

/**
 * @brief Reset data ready signal for 6-axis sensor fusion
 *
 * Resets the sensor data ready flag back to 0, indicating no sensor data is
 * available for running the SF algorithm. This function is called by the 6-axis
 * SF algorithm after completing one iteration to prepare for the next cycle.
 *
 * @param None
 * @return None
 */
void algo_sf_6xag_unsignal_sf_run(void)
{
	signal_sf_6xag_run = 0;
}
#endif /* USE_6AXIS_FUSION */

#ifdef USE_9AXIS_FUSION
/**
 * @brief 9-axis sensor fusion data preprocessing interface
 *
 * This function serves as the interface between the sensor fusion algorithm and
 * algorithm manager for 9-axis fusion. It collects raw accelerometer, gyroscope,
 * and magnetometer sensor data, buffers multiple samples for oversampling (accel/gyro),
 * computes running averages, and signals when data is ready for fusion processing.
 * Note: Magnetometer typically has lower sampling rate and is not oversampled.
 *
 * @param[in] sf_algo_id SF algorithm ID provided during initialization
 * @param[in] ptr_sensor_raw_data Pointer to raw sensor data (accel, gyro, or mag)
 *
 * @return Status flags indicating data availability (bitwise OR combination):
 *         - 0: Invalid input/algo ID or no valid data collected
 *         - SF_ACC_READY_BIT (1): Accelerometer data ready
 *         - SF_GYRO_READY_BIT (2): Gyroscope data ready
 *         - 3: Accelerometer and gyroscope data ready
 *         - SF_MAG_READY_BIT (4): Magnetometer data ready
 *         - 5: Accelerometer and magnetometer data ready
 *         - 6: Gyroscope and magnetometer data ready
 *         - 7: All sensors (accel, gyro, mag) data ready for fusion
 */
uint32_t sf_9xagm_data_preproc(uintptr_t        sf_algo_id,
                               sensor_data_t    *ptr_sensor_raw_data)
{
	static uint32_t  acc_count = 0;
	static uint32_t  gyro_count = 0;
	state_vec_9XAGM_t *ptr_state_vec_9XAGM;

	// check for input validity
	if (sf_algo_id == (uintptr_t)NULL) {
		printf("invalid input mem pointers \n");
		return 0;
	}
	if (ptr_sensor_raw_data == (sensor_data_t *)NULL) {
		printf("invalid data mem \n");
		return 0;
	}
	
	// data integrity check
	ptr_state_vec_9XAGM = (state_vec_9XAGM_t *)sf_algo_id;
	
	if (ptr_state_vec_9XAGM->AlgoID != sf_algo_id) {
		printf("invalid algo id: expected=0x%zx, got=0x%zx\n", sf_algo_id, ptr_state_vec_9XAGM->AlgoID);
		return 0;
	}

	// save instantaneous sensor raw data input
	if (ptr_sensor_raw_data->sensorID == ACC) { // accel input
		ptr_state_vec_9XAGM->AccData.CountBuff[acc_count][0] = (int16_t)ptr_sensor_raw_data->sensordata[0];
		ptr_state_vec_9XAGM->AccData.CountBuff[acc_count][1] = (int16_t)ptr_sensor_raw_data->sensordata[1];
		ptr_state_vec_9XAGM->AccData.CountBuff[acc_count][2] = (int16_t)ptr_sensor_raw_data->sensordata[2];
		if(acc_count == 0) {
			ptr_state_vec_9XAGM->AccData.timestamp = (int64_t)ptr_sensor_raw_data->timestamp;
		}
		acc_count = acc_count + 1;
		if(acc_count == SF_OVERSAMPLE_RATIO) {
            acc_count = 0;
			signal_sf_9xagm_run = signal_sf_9xagm_run | SF_ACC_READY_BIT;
			// Calculate average for accelerometer data
			float avg_wt = (ptr_state_vec_9XAGM->AccData.ScaleFactor) / SF_OVERSAMPLE_RATIO;
			for(int32_t k = CHX; k <= CHZ; k++)
			{	
				// calcualte average value periodically before calling SF
				ptr_state_vec_9XAGM->AccData.CountAvg[k] = 0;
				for(int32_t i = 0; i < SF_OVERSAMPLE_RATIO; i++)
					ptr_state_vec_9XAGM->AccData.CountAvg[k] += ptr_state_vec_9XAGM->AccData.CountBuff[i][k];
				if (ptr_state_vec_9XAGM->AccData.CountAvg[k] > 0)
					ptr_state_vec_9XAGM->AccData.CountAvg[k] += SF_OVERSAMPLE_RATIO / 2;
				else
					ptr_state_vec_9XAGM->AccData.CountAvg[k] -= SF_OVERSAMPLE_RATIO / 2;
				ptr_state_vec_9XAGM->AccData.CountAvg[k] *= avg_wt;
		    }
	    }
	}
	else if (ptr_sensor_raw_data->sensorID == GYRO) { // gyro input preproc
		ptr_state_vec_9XAGM->GyroData.CountBuff[gyro_count][0] = (int16_t)ptr_sensor_raw_data->sensordata[0];
		ptr_state_vec_9XAGM->GyroData.CountBuff[gyro_count][1] = (int16_t)ptr_sensor_raw_data->sensordata[1];
		ptr_state_vec_9XAGM->GyroData.CountBuff[gyro_count][2] = (int16_t)ptr_sensor_raw_data->sensordata[2];
		if (gyro_count == 0) {
			ptr_state_vec_9XAGM->GyroData.timestamp = (int64_t)ptr_sensor_raw_data->timestamp;
	    }
		gyro_count = gyro_count + 1;
		if (gyro_count == SF_OVERSAMPLE_RATIO) {
			gyro_count = 0;
			signal_sf_9xagm_run = signal_sf_9xagm_run | SF_GYRO_READY_BIT;
			// Calculate average for gyroscope data
			float avg_wt = (ptr_state_vec_9XAGM->GyroData.ScaleFactor) / SF_OVERSAMPLE_RATIO;
			for(int32_t k = CHX; k <= CHZ; k++)
			{	
				// calcualte average value periodically before calling SF
				ptr_state_vec_9XAGM->GyroData.CountAvg[k] = 0;
				for(int32_t i = 0; i < SF_OVERSAMPLE_RATIO; i++)
					ptr_state_vec_9XAGM->GyroData.CountAvg[k] += ptr_state_vec_9XAGM->GyroData.CountBuff[i][k];
				if (ptr_state_vec_9XAGM->GyroData.CountAvg[k] > 0)
					ptr_state_vec_9XAGM->GyroData.CountAvg[k] += SF_OVERSAMPLE_RATIO / 2;
				else
					ptr_state_vec_9XAGM->GyroData.CountAvg[k] -= SF_OVERSAMPLE_RATIO / 2;
				ptr_state_vec_9XAGM->GyroData.CountAvg[k] *= avg_wt;
		    }
		}		
	}
	else if (ptr_sensor_raw_data->sensorID == MAG) { // magnetometer input preproc
		// Note: Magnetometer typically has lower sampling rate than gyro/accel
		// Store magnetometer data directly without oversampling
		ptr_state_vec_9XAGM->MagData.CountBuff[0][0] = (int16_t)ptr_sensor_raw_data->sensordata[0];
		ptr_state_vec_9XAGM->MagData.CountBuff[0][1] = (int16_t)ptr_sensor_raw_data->sensordata[1];
		ptr_state_vec_9XAGM->MagData.CountBuff[0][2] = (int16_t)ptr_sensor_raw_data->sensordata[2];
		ptr_state_vec_9XAGM->MagData.timestamp = (int64_t)ptr_sensor_raw_data->timestamp;
		
		// Calculate average for magnetometer (single sample)
		ptr_state_vec_9XAGM->MagData.CountAvg[0] = ptr_state_vec_9XAGM->MagData.CountBuff[0][0];
		ptr_state_vec_9XAGM->MagData.CountAvg[1] = ptr_state_vec_9XAGM->MagData.CountBuff[0][1];
		ptr_state_vec_9XAGM->MagData.CountAvg[2] = ptr_state_vec_9XAGM->MagData.CountBuff[0][2];

		signal_sf_9xagm_run = signal_sf_9xagm_run | SF_MAG_READY_BIT;
	}
	
	return signal_sf_9xagm_run;
}

/**
 * @brief Reset data ready signal for 9-axis sensor fusion
 *
 * Resets the sensor data ready flag back to 0, indicating no sensor data is
 * available for running the SF algorithm. This function is called by the 9-axis
 * SF algorithm after completing one iteration to prepare for the next cycle.
 *
 * @param None
 * @return None
 */
void algo_sf_9xagm_unsignal_sf_run(void)
{
	signal_sf_9xagm_run = 0;
}
#endif /* USE_9AXIS_FUSION */

