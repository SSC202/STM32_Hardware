#ifndef __Kalman_H
#define __Kalman_H

#ifdef MPU_Kalman
#include "stm32f1xx.h"
#include "mpu6050.h"
/***********************************IMU数据结构*******************************************/
typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float tempreture;
    float kalman_x;
    float kalman_y;
} IMU_Data_t;
/*********************Kalman滤波相关矩阵**************************/
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;
/*************************函数定义********************************/
void MPU_Data_Get(IMU_Data_t *imu_data);
/****************************************************************/
extern IMU_Data_t mpu_data;
#endif

#endif