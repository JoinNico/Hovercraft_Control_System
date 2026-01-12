#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

//imu的三轴枚举
enum imu_xyz
{
    imu_x,
    imu_y,
    imu_z,
};

//imu的数据结构
typedef struct
{
    int16_t a_raw[3];//三轴整型（imu的原始数据）
    int16_t g_raw[3];
    int16_t a_offset[3];
    int16_t g_offset[3];
    float a[3];    //加速度项
    float g[3];    //角速度项
    float temp;
    float rol;
    float pit;
    float yaw;
} IMU_TYPE;

//对 SENSER_OFFSET_FLAG 的位的操作
#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //标志位置1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //标志位置0
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取标志位状态
#define MY_ABS(x)              ((x)>0?(x):-(x))
#define Kp_New      0.9f              //互补滤波当前数据的权重
#define Kp_Old      0.1f              //互补滤波历史数据的权重
#define Acc_Gain    0.0001220f        //加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain   0.0609756f        //角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr     0.0010641f        //角速度变成弧度(3.1415/180 * LSBg)
#define G           9.80665f                // m/s^2
#define RadtoDeg    57.324841f              //弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f              //角度到弧度 (角度 * 3.1415/180)
#define BUF_SIZ     20                      //滤波数组大小

extern uint8_t SENSER_OFFSET_FLAG;
extern IMU_TYPE imu;

void imu_updata(void);
#endif
