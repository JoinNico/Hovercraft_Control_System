#include <stdio.h>

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
    kf->P[0][0] += kf->dt * (kf->dt*kf->P[1][1] - kf->P[0][1]
                             + kf->P[1][0] )+ kf->Q_angle;
    kf->P[0][1] -= kf->dt * kf->P[1][1];
    kf->P[1][0] -= kf->dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias;

    // 更新
  	//角度差
    float y = new_angle - kf->state.angle;
    //K增益矩阵计算
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

int main() {
    // 设置卡尔曼滤波器参数
    float Q_angle = 0.00005;
    float Q_bias = 0.00015;
    float R_measure = 0.5;
    float dt = 0.005;//传感器实际采样频率

    KalmanFilter kf;
    kalman_init(&kf, Q_angle, Q_bias, R_measure, dt);

    // 陀螺仪和加速度计数据
    float gyro_data = 0.1;  // 陀螺仪数据，通过I2C读取
    float accel_data = 0.0; // 加速度计数据，通过I2C读取

    float angle = kalman_filter(&kf, accel_data, gyro_data);
    printf("Filtered Angle: %.2f\n", angle);

    return 0;
}