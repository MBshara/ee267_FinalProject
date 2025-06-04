/** 
 * @file
 * math implementation for :
 * - quaternion complementary filter
 * - euler complementary filter
 */

#pragma once
#include "Quaternion.h"

/**
 * @param[in] acc - current acc values  (ax, ay, az)
 * @returns pitch angle in degrees
 */
double computeAccPitch(double acc[3]);


/**
 * @param[in] acc - current acc values  (ax, ay, az)
 * @returns roll angle in degrees
 */
double computeAccRoll(double acc[3]);


/**
 * get flatland roll from gyro measurements
 * @param[in] flatlandRollGyrPrev - previous flatland roll estimate from gyro readings
 * @param[in] deltaT - time since previous imu reading in s
 * @param[in] gyr - current gyro values (pitch, yaw, roll)
 * @returns new flatland roll from previous flatland roll
 *  and current gyro values
 */
double computeFlatlandRollGyr(double flatlandRollGyrPrev, double gyr[3], double deltaT);


/**
 * gets flatland roll from acc measurements
 * @param[in] acc - current acc values (ax, ay, az)
 * @returns flatland roll from acc values
 */
double computeFlatlandRollAcc(double acc[3]);


/**
 * gets flatland roll by complementary filtering, using
 * gyro and acc values
 * @param[in] flatlandRollCompPrev - previous complementary filter estimate of flatland roll
 * @param[in] gyr - current gyro readings
 * @param[in] flatlandRollAcc - current estimate of flatland roll from acc readings
 *
 * @param[in] deltaT - time since previous imu reading in s
 * @param[in] alpha - complementary filter alpha value
 * @returns new flatland roll estimate from complementary filter
 */
double computeFlatlandRollComp(double flatlandRollCompPrev, double gyr[3],  double flatlandRollAcc, double deltaT, double alpha);


/**
 * update the quaternion estimate with complementary filtering of the
 * gyro and acc values.
 * @param[in, out] q - previous orientation estimate.
 *   should be updated to the new orientation estimate.
 * @param[in] gyr - current gyro values order: (pitch, yaw, roll)
 * @param[in] deltaT - time since previous imu reading in seconds
 * @param[in] alpha - complementary filter alpha value
 */
void updateQuaternionComp(Quaternion& q, double gyr[3], double acc[3], double deltaT, double alpha);

/**
 * Update the quaternion estimate using the **Madgwick** filter algorithm.
 * @param[in, out] q - Previous orientation estimate. Will be updated in-place.
 * @param[in] gyr - Current gyro values (pitch, yaw, roll).
 * @param[in] acc - Current accelerometer values (x, y, z) in m/s^2.
 * @param[in] deltaT - Time since the previous IMU reading, in seconds.
 * @param[in] beta - Gradient-descent step size (Madgwick gain).
 */
void updateQuaternionMadgwick(Quaternion& q,
                              const double gyr[3],
                              const double acc[3],
                              double       deltaT,  
                              double       beta);


/**
 * update the quaternion estimate using imu gyro values
 * @param[in, out] q - previous orientation estimate.
 * should be updated to the new orientation estimate.
 * @param[in] gyr - current gyro values (pitch, yaw, roll)
 * @param[in] deltaT - time since previous imu reading in seconds
 *
 */
void updateQuaternionGyr(Quaternion& q, double gyr[3], double deltaT);

/**
 * Update the quaternion estimate with an **Extended Kalman Filter (EKF)** fusion
 * of gyro and accelerometer data.
 * @param[in, out] q - Previous orientation estimate. Will be updated in-place.
 * @param[in, out] P - 4×4 state-covariance matrix (updated in-place).
 * @param[in] gyr - Current gyro values (pitch, yaw, roll).
 * @param[in] acc - Current accelerometer values (x, y, z) in m/s^2.
 * @param[in] deltaT - Time since the previous IMU reading, in seconds.
 * @param[in] Qnoise - Process-noise variance (gyro model).
 * @param[in] Rnoise - Measurement-noise variance (accelerometer model).
 */
void updateQuaternionEKF(Quaternion& q, 
                        double (&P)[4][4],
                        const double gyr[3],
                        const double acc[3],
                        double deltaT,
                        double Qnoise,
                        double Rnoise);
/**
 * Initialise the EKF state-covariance matrix to a diagonal matrix
 * with the supplied variance.
 * @param[out] P - 4×4 state-covariance matrix to initialise.
 * @param[in] initVar Variance to place on the diagonal elements.
 */
void initializeEKFCovariance(double (&P)[4][4], double initVar);
/**
 * Update the quaternion estimate using the **Mahony** filter algorithm.
 * @param[in, out] q - Previous orientation estimate. Will be updated in-place.
 * @param[in, out] eInt - Integral error term (3-vector) accumulated over time.
 * @param[in] gyr - Current gyro values (pitch, yaw, roll).
 * @param[in] arr - Current accelerometer values (x, y, z) in m/s^2.
 * @param[in] deltaT - Time since the previous IMU reading, in seconds.
 * @param[in] Kp - Proportional gain.
 * @param[in] Ki - Integral gain.
 */
void updateQuaternionMahony(Quaternion& q, double (&eInt)[3], const double gyr[3], const double arr[3], double deltaT, double Kp, double Ki);
