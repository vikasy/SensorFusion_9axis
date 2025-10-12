
/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_sensordata.h"
#include "algo_sf_fusion.h"

// indicates the availability of sensor data for running SF algo
#ifdef USE_6AXIS_FUSION
static uint32_t signal_sf_6xag_run = 0;
#endif

#ifdef USE_9AXIS_FUSION
static uint32_t signal_sf_9xagm_run = 0;
#endif


#ifdef USE_6AXIS_FUSION
/**
* @brief: This function is an interface between SF algorithm and algorithm manager. 
* It allowes algorithm manager to send sensor data to algo for its use.
* 
* @param[in]: sf_algo_id: SF algo id that was provided to algo manager at the time 
*                         of algo initialization (creation)
*             ptr_sensor_raw_data: pointer to raw physical (acc/gyro) sensor data
* 
* @param[out]: 0 if invalid input/algo id or if no valid data collected since last SF algo run,
*              1 if only accel data has been collected since last SF algo run
*              2 if only gyro data has been collected since last SF algo run
*              3 if both accel and gyro data have been collected since last SF algo run
*
*/
uint32_t sf_6xag_data_preproc(uintptr_t        sf_algo_id,
	                          sensor_data_t    *ptr_sensor_raw_data)
{
    #define ACC_READY_BIT   1
    #define GYRO_READY_BIT  2

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
			signal_sf_6xag_run = signal_sf_6xag_run | ACC_READY_BIT;
		    float avg_wt = (ptr_state_vec_6XAG->AccData.ScaleFactor) / SF_OVERSAMPLE_RATIO;
			for(int32_t k = CHX; k <= CHZ; k++)
			{	
				// calcualte average value periodically before calling SF
				ptr_state_vec_6XAG->AccData.CountAvg[k] = 0;
				for(int32_t i = 0; i < SF_OVERSAMPLE_RATIO; i++)
					ptr_state_vec_6XAG->AccData.CountAvg[k] += ptr_state_vec_6XAG->AccData.CountBuff[i][k];
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
			signal_sf_6xag_run = signal_sf_6xag_run | GYRO_READY_BIT;
		}		
	}
	
	return signal_sf_6xag_run;
}

/**
* @brief: This function allows reseting the state of sf_run signal back to 0, which means no
*          sensor data available for running SF algo. 
* This is called by 6-axis SF algo after completing its run (one iteration)
* @param[in]: none
*
* @ param[out]: none
*
*/
void algo_sf_6xag_unsignal_sf_run(void)
{
	signal_sf_6xag_run = 0;
}
#endif /* USE_6AXIS_FUSION */

#ifdef USE_9AXIS_FUSION
/**
* @brief: This function is an interface between SF algorithm and algorithm manager for 9-axis fusion. 
* It allows algorithm manager to send sensor data to algo for its use (accel/gyro/mag).
* Interface for processing accelerometer, gyroscope, and magnetometer data
* 
* @param[in]: sf_algo_id: SF algo id that was provided to algo manager at the time 
*                         of algo initialization (creation)
*             ptr_sensor_raw_data: pointer to raw physical (acc/gyro/mag) sensor data
* 
* @param[out]: 0 if invalid input/algo id or if no valid data collected since last SF algo run,
*              1 if only accel data has been collected since last SF algo run
*              2 if only gyro data has been collected since last SF algo run
*              3 if accel and gyro data have been collected since last SF algo run
*              4 if only mag data has been collected since last SF algo run
*              5 if accel and mag data have been collected since last SF algo run
*              6 if gyro and mag data have been collected since last SF algo run  
*              7 if all accel, gyro and mag data have been collected since last SF algo run
*
*/
uint32_t sf_9xagm_data_preproc(uintptr_t        sf_algo_id,
                               sensor_data_t    *ptr_sensor_raw_data)
{
    #define ACC_READY_BIT   1
    #define GYRO_READY_BIT  2
    #define MAG_READY_BIT   4

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
			signal_sf_9xagm_run = signal_sf_9xagm_run | ACC_READY_BIT;
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
			signal_sf_9xagm_run = signal_sf_9xagm_run | GYRO_READY_BIT;
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
		
		signal_sf_9xagm_run = signal_sf_9xagm_run | MAG_READY_BIT;
	}
	
	return signal_sf_9xagm_run;
}

/**
* @brief: This function allows resetting the state of sf_run signal back to 0, which means no
*          sensor data available for running SF algo. 
* This is called by 9-axis SF algo after completing its run (one iteration)
* @param[in]: none
*
* @ param[out]: none
*
*/
void algo_sf_9xagm_unsignal_sf_run(void)
{
	signal_sf_9xagm_run = 0;
}
#endif /* USE_9AXIS_FUSION */

