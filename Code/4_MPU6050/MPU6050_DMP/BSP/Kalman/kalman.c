#ifdef MPU_Kalman
#include "kalman.h"
#include "mpu6050.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105


Kalman_t KalmanX = {
    .Q_angle   = 0.001f,
    .Q_bias    = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle   = 0.001f,
    .Q_bias    = 0.003f,
    .R_measure = 0.03f,
};

uint32_t timer;
IMU_Data_t mpu_data;

/**
 * @brief 卡尔曼滤波函数
 */
static double MPU_Kalman_Filter(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

/**
 * @brief MPU6050 数据采样并滤波
 * @param _imu_data IMU 数据结构体
 */
void MPU_Data_Get(IMU_Data_t *_imu_data)
{
    short _accx,_accy,_accz;
    short _gyrox,_gyroy,_gyroz;
    short _tempreture;
    // 采样数据
    MPU_Get_Accelerometer(&_accx, &_accy, &_accz);
    MPU_Get_Gyroscope(&_gyrox, &_gyroy, &_gyroz);
    _tempreture = MPU_Get_Temperature();
    _imu_data->tempreture = _tempreture;
    _imu_data->accel_x = _accx;
    _imu_data->accel_y = _accy;
    _imu_data->accel_z = _accz;
    _imu_data->gyro_x = _gyrox;
    _imu_data->gyro_y = _gyroy;
    _imu_data->gyro_z = _gyroz;

    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer     = HAL_GetTick();
    // roll
    double roll;
    double roll_sqrt = sqrt(_imu_data->accel_x * _imu_data->accel_x + _imu_data->accel_z * _imu_data->accel_z);
    if (roll_sqrt != 0.0) {
        roll = atan(_imu_data->accel_y / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    // pitch
    double pitch = atan2(_imu_data->accel_x, _imu_data->accel_z) * RAD_TO_DEG;
    if ((pitch < -90 && _imu_data->kalman_y > 90) || (pitch > 90 && _imu_data->kalman_y < -90)) {
        KalmanY.angle       = pitch;
        _imu_data->kalman_y = pitch;
    } else {
        _imu_data->kalman_y = MPU_Kalman_Filter(&KalmanY, pitch, _imu_data->gyro_y, dt);
    }
    if (fabs(_imu_data->kalman_y) > 90)
        _imu_data->gyro_x = -_imu_data->gyro_x;
    _imu_data->kalman_x = MPU_Kalman_Filter(&KalmanX, roll, _imu_data->gyro_x, dt);
}
#endif