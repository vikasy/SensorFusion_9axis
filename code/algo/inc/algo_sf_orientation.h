/*******************************************************************************
 * @file    algo_sf_orientation.h
 * @brief   Orientation computation functions using quaternions and rotation matrices
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef _ORIENTATION_H_
#define _ORIENTATION_H_

#include "algo_sf_types.h"

/****************************************************************************
 * Orientation Constants
 ****************************************************************************/

#define ORIENT_SMALLQ0           0.01F       /* Threshold for small quaternion scalar component */
#define ORIENT_CORRUPTQUAT       0.001F      /* Threshold for corrupt quaternion detection */
#define ORIENT_SMALLMODULUS      0.01F       /* Threshold where rounding errors may appear */

/****************************************************************************
 * 3DOF Tilt Functions
 ****************************************************************************/

/**
 * @brief Compute 3DOF tilt rotation matrix (NED frame)
 *
 * Computes rotation matrix from accelerometer data using NED coordinate system.
 * Handles gimbal lock at 90 deg pitch.
 *
 * @param[out] fR - 3x3 rotation matrix
 * @param[in] fGp - Accelerometer readings [x, y, z] in g
 * @return None
 */
void f3DOFTiltNED(float fR[][3], float fGp[]);

/**
 * @brief Compute 3DOF tilt rotation matrix (Android frame)
 *
 * Identical to NED tilt matrix. Handles gimbal lock at 90 deg roll.
 *
 * @param[out] fR - 3x3 rotation matrix
 * @param[in] fGp - Accelerometer readings [x, y, z] in g
 * @return None
 */
void f3DOFTiltAndroid(float fR[][3], float fGp[]);

/****************************************************************************
 * Angle Extraction Functions
 ****************************************************************************/

/**
 * @brief Extract Android angles from rotation matrix
 *
 * Computes roll, pitch, yaw, compass heading, and tilt angles from rotation matrix.
 *
 * @param[in] R - 3x3 rotation matrix
 * @param[out] pfPhiDeg - Roll angle in degrees [-90, 90]
 * @param[out] pfTheDeg - Pitch angle in degrees [-180, 180]
 * @param[out] pfPsiDeg - Yaw angle in degrees [0, 360)
 * @param[out] pfRhoDeg - Compass heading in degrees [0, 360)
 * @param[out] pfChiDeg - Tilt angle from vertical in degrees [0, 180]
 * @return None
 */
void fAndroidAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
	float *pfRhoDeg, float *pfChiDeg);

/****************************************************************************
 * Quaternion Conversion Functions
 ****************************************************************************/

/**
 * @brief Convert rotation matrix to quaternion
 *
 * @param[in] R - 3x3 rotation matrix
 * @param[out] pq - Output quaternion
 * @return None
 *
 * @note Handles both normal and near-180 degree rotation cases
 */
void fQuaternionFromRotationMatrix(float R[][3], quaternion_t *pq);

/**
 * @brief Convert quaternion to rotation matrix
 *
 * @param[out] R - 3x3 rotation matrix
 * @param[in] pq - Input quaternion (should be normalized)
 * @return None
 */
void fRotationMatrixFromQuaternion(float R[][3], const quaternion_t *pq);

/****************************************************************************
 * Rotation Vector Functions
 ****************************************************************************/

/**
 * @brief Convert rotation matrix to rotation vector (degrees)
 *
 * @param[in] R - 3x3 rotation matrix
 * @param[out] rvecdeg - Rotation vector [x, y, z] in degrees
 * @return None
 *
 * @note Handles 0, 180, and general angle cases
 */
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[]);

/**
 * @brief Convert rotation vector to normalized quaternion
 *
 * @param[out] pq - Output quaternion (normalized)
 * @param[in] rvecdeg - Rotation vector [x, y, z] in degrees
 * @param[in] fscaling - Scaling factor (typically 1.0)
 * @return None
 *
 * @note Uses MacLaurin series for small angles for better accuracy
 */
void fQuaternionFromRotationVectorDeg(quaternion_t *pq, const float rvecdeg[], float fscaling);

/**
 * @brief Convert quaternion to rotation vector (degrees)
 *
 * @param[in] pq - Input quaternion
 * @param[out] rvecdeg - Rotation vector [x, y, z] in degrees
 * @return None
 *
 * @note Angle is mapped to range [-180, 180] degrees
 */
void fRotationVectorDegFromQuaternion(quaternion_t *pq, float rvecdeg[]);

/****************************************************************************
 * Quaternion Arithmetic Functions
 ****************************************************************************/

/**
 * @brief Quaternion multiplication: qA = qB * qC
 *
 * @param[out] pqA - Result quaternion
 * @param[in] pqB - First quaternion
 * @param[in] pqC - Second quaternion
 * @return None
 */
void qAeqBxC(quaternion_t *pqA, const quaternion_t *pqB, const quaternion_t *pqC);

/**
 * @brief Quaternion multiplication: qA = qA * qB (in-place)
 *
 * @param[in,out] pqA - First quaternion (modified in place)
 * @param[in] pqB - Second quaternion
 * @return None
 */
void qAeqAxB(quaternion_t *pqA, const quaternion_t *pqB);

/**
 * @brief Quaternion multiplication with conjugate: result = conj(qA) * qB
 *
 * @param[in] pqA - First quaternion (conjugated)
 * @param[in] pqB - Second quaternion
 * @return Result quaternion
 */
quaternion_t qconjgAxB(const quaternion_t *pqA, const quaternion_t *pqB);

/**
 * @brief Normalize quaternion and ensure q0 is non-negative
 *
 * @param[in,out] pqA - Quaternion to normalize (modified in place)
 * @return None
 *
 * @note Sets to identity quaternion if corrupt (norm too small)
 */
void fqAeqNormqA(quaternion_t *pqA);

/**
 * @brief Set quaternion to identity (unit quaternion)
 *
 * @param[out] pqA - Quaternion to set
 * @return None
 *
 * @note Identity quaternion: q0=1, q1=q2=q3=0
 */
void fqAeq1(quaternion_t *pqA);

#endif /* _ORIENTATION_H_ */
