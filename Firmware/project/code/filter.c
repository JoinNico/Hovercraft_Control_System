/*
 * filter.c
 *
 *  Created on: 2021年2月16日
 *      Author: sundm
 */

#include "filter.h"

const float filter_weight[WIN_SIZE] = { 10, 15, 20, 25, 30, 40, 50, 60, 70, 80};

float SlidingFilter(float array[WIN_SIZE], float value)
{
    //窗口滑动
    for(int i = 0; i < WIN_SIZE - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[WIN_SIZE - 1] = value;

    //加权
    float result = 0.0f;
    float weight_sum = 0.0f;
    for(uint8 i = 0; i < WIN_SIZE; i++)
    {
        result += filter_weight[i] * array[i];
        weight_sum += filter_weight[i];
    }
    if(weight_sum != 0)
    {
        result = result / weight_sum;
    }
    return result;
}

const float filter_weight_2[WIN_SIZE] = { 1, 2, 3, 4, 5, 10, 15, 60, 80, 100};

float LowPassFilter(float array[WIN_SIZE], float value)//低通滤波
{
    //窗口滑动
    for(int i = 0; i < WIN_SIZE - 1; i++) {
        array[i] = array[i + 1];
    }
    array[WIN_SIZE - 1] = value;

    //加权
    float result = 0.0f;
    float weight_sum = 0.0f;
    for(uint8 i = 0; i < WIN_SIZE; i++) {
        result += filter_weight_2[i] * array[i];
        weight_sum += filter_weight_2[i];
    }
    if(weight_sum != 0) {
        result = result / weight_sum;
    }
    return result;
}

float first_order_lpf(float * lpf, float val, float cut_off_freq, float sample_freq)
{
    *lpf += (1 / (1 + 1 / (2 * 3.1415 * cut_off_freq * (1 / sample_freq)))) * (val - *lpf);
    return *lpf;
}

/**************************************************************************/
float K1 =0.02;
float  angle_dot;
float angle;
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
float Kalman_Filter(float Accel,float Gyro)
{
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
    return angle;
}

/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}

// 初始化滤波器
void kalman_init(KalmanFilter *kf, float Q_angle, float Q_bias,
                 float R_measure, float dt) {
    kf->state.angle = 0.0;
    kf->state.bias = 0.0;
    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
    kf->dt = dt;
}

// 卡尔曼滤波算法
float kalman_filter(KalmanFilter *kf, float new_angle, float new_Gyro) {
    // 预测
    //先验估计
    kf->state.angle += kf->dt * (new_Gyro - kf->state.bias);
    //协方差估计
    kf->P[0][0] += kf->dt * ( kf->P[0][1]+ kf->P[1][0] )+ kf->Q_angle;
    kf->P[0][1] -= kf->dt * kf->P[1][1];
    kf->P[1][0] -= kf->dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias;

    // 更新
    //角度差
    float y = new_angle - kf->state.angle;
    //卡尔曼增益矩阵计算
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    //后验状态更新
    kf->state.angle += K[0] * y;
    kf->state.bias += K[1] * y;
    //后验协方差误差更新
    kf->P[0][0] -= K[0] * kf->P[0][0];
    kf->P[0][1] -= K[0] * kf->P[0][1];
    kf->P[1][0] -= K[1] * kf->P[0][0];
    kf->P[1][1] -= K[1] * kf->P[0][1];

    return kf->state.angle;
}

//int main() {
//    // 设置卡尔曼滤波器参数
//    float Q_angle = 0.00005;
//    float Q_bias = 0.00015;
//    float R_measure = 0.5;
//    float dt = 0.005;//传感器实际采样频率
//
//    KalmanFilter kf;
//    kalman_init(&kf, Q_angle, Q_bias, R_measure, dt);
//
//    // 陀螺仪和加速度计数据
//    float gyro_data = 0.1;  // 陀螺仪数据，通过I2C读取
//    float accel_data = 0.0; // 加速度计数据，通过I2C读取
//
//    float angle = kalman_filter(&kf, accel_data, gyro_data);
//    printf("Filtered Angle: %.2f\n", angle);
//
//    return 0;
//}
