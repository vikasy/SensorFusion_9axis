#ifndef _ALGO_SF_TYPES_H_
#define _ALGO_SF_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "math.h"
#include "stdlib.h"

#include "algo_sf_interface.h"

// vector components
#define CHX 0
#define CHY 1
#define CHZ 2

/*****
* Author: Vikas Yadav
* Date: 2020
*/

// booleans
#define true 1
#define false 0

// useful multiplicative conversion constants
#define PI               3.14159265358979323846 // Pi
#define PI_OVER_2        PI/2.0                 // Pi/2
#define PI_OVER_4        PI/4.0                 // Pi/4
#define PI_OVER_6        PI/6.0                 // Pi/6
#define PI_OVER_12       PI/12.0                 // Pi/12
#define PI_TIMES_2       PI*2                   // 2*Pi
#define ONEOVERSQRT2     0.70710678118654752440	// 1/sqrt(2)
#define ONETHIRD         0.333333333333333	    // 1/3
#define ONESIXTH         0.166666666666667   	// 1/6
#define ONEOVER24        0.0416666666666667     // 1/24
#define ONEOVER48        0.0208333333333333		// 1/48
#define ONEOVER120       0.00833333333333333	// 1/120
#define ONEOVER3840      0.000260416666666667	// 1/3840
#define GTOMSEC2         9.8     				// standard gravity in m/s2
#define DEG2RAD          0.017453292519943295   // deg to rad conversion = pi / 180
#define RAD2DEG          1.0/DEG2RAD            // rad to deg conversion = 180 / pi
#define NSEC2MSEC        1000000
#define DBL_MIN          2.2250738585072014E-308// DBL_MIN
#define EPSILON          1e-9
#define EPSILON_SQ       EPSILON*EPSILON
#define TINY             2.2250738585072014E-30

// SAMPLING RATE CONFIGURATION: 100Hz
#define SF_GYRO_FS 		     100		             // gyro sensor sampling frequency Hz
#define SF_GYRO_SAMP_INTVL   1.0/(double)SF_GYRO_FS  // gyro sensor sampling interval in sec
#define SF_OVERSAMPLE_RATIO  4			             // ratio of gyro sensor samp freq and accel samp freq
#define SF_DELTA_T           (SF_OVERSAMPLE_RATIO * SF_GYRO_SAMP_INTVL) // interval between two KF runs
#define SF_DELTA_T_SQ        (SF_DELTA_T*SF_DELTA_T)

// start time offset to match PC time with recorded data time
extern int64_t start_time_offset_ns;

#endif /* _ALGO_SF_TYPES_H_ */
