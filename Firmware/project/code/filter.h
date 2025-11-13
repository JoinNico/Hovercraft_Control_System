/*
 * filter.h
 *
 *  Created on: 2021年2月16日
 *      Author: sundm
 */

#ifndef USER_FILTER_H_
#define USER_FILTER_H_

#include "zf_common_headfile.h"
#include "imu.h"
#define WIN_SIZE 10
// 状态向量
typedef struct {
    float angle;  // 姿态角度
    float bias;   // 角速度偏差
} State;

// 卡尔曼滤波器结构
typedef struct {
    State state;  // 状态
    float P[2][2]; // 协方差矩阵
    float Q_angle; // 过程噪声协方差
    float Q_bias;
    float R_measure; // 观测噪声协方差
    float dt; // 时间步长
} KalmanFilter;

float SlidingFilter(float array[WIN_SIZE], float value);
float LowPassFilter(float array[WIN_SIZE], float value);
float first_order_lpf(float * lpf, float val, float cut_off_freq, float sample_freq);

float Kalman_Filter(float Accel,float Gyro);
void Yijielvbo(float angle_m, float gyro_m);

#endif /* USER_FILTER_H_ */
