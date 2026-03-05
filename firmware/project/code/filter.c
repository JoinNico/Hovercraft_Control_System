/*
 * filter.c
 *
 *  Created on: 2021年2月16日
 *      Author: sundm
 */

#include "filter.h"

const float filter_weight[WIN_SIZE] = { 10, 15, 20, 25, 30, 40, 50, 60, 70, 80};

float SlidingFilter(float array[WIN_SIZE], float value)//滑动滤波
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

/**
 * @brief     一阶低通滤波器（基于截止频率）
 *
 * @param lpf           指向滤波器状态变量的指针（上一次滤波输出值）
 * @param val           当前输入值（原始测量值）
 * @param cut_off_freq  截止频率 (Hz)，决定滤波器的响应速度和平滑程度
 *                      - 频率越低，滤波效果越强，延迟越大
 *                      - 频率越高，响应越快，但滤波效果减弱
 * @param sample_freq   采样频率 (Hz)，即函数被调用的频率
 *
 * @return float        滤波后的输出值
 *
 * @note
  *   滤波器计算公式：y[n] = y[n-1] + α * (x[n] - y[n-1])
  *   其中 α = 1 / (1 + 1/(2π * fc * Ts))，Ts = 1/sample_freq
 *
  *   使用示例：
 * static float filtered_value = 0;
 * float raw_data = get_sensor_value();
 * filtered_value = first_order_lpf(&filtered_value, raw_data, 5.0f, 100.0f); // 截止频率5Hz，采样频率100Hz
 *
 *
 * @attention
 * 1. lpf指针必须指向有效的静态变量或全局变量，用于保持滤波器状态
 * 2. 确保sample_freq远大于cut_off_freq（通常至少2倍，建议5-10倍）
 * 3. 滤波器初始化时，建议将*lpf设置为第一次输入值
 */

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


