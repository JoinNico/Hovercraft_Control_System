#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

#define ENCODER_QUADDEC_L    (TIM9_ENCOEDER)          // 正交编码器对应使用的编码器接口 这里使用 TIM1 的编码器功能
#define ENCODER_QUADDEC_A_L  (TIM9_ENCOEDER_MAP3_CH1_D9)   // A 相对应的引脚
#define ENCODER_QUADDEC_B_L  (TIM9_ENCOEDER_MAP3_CH2_D11)   // B 相对应的引脚
#define ENCODER_CONTROL_CYCLE        0.01f       //电机控制周期T：10ms
typedef struct
{
    float ReductionRatio ;                      //电机减速比
    float EncoderLine ;                         //编码器线数=光栅数16*4
    signed int EncoderValue;                    //编码器实时速度
    float DiameterWheel;                        //轮子直径：mm
    uint8_t CloseLoop;                             //开环模式
    uint16_t Counter;                           //线程计数器
}WheelStruct;

extern WheelStruct WheelStr;
extern float distance;
extern float wheel_speed;
extern float voltage;

static inline float get_distance(void)
{
    return distance;
}

static inline float get_speed(void)
{
    return wheel_speed;
}

void   Encoder_Init(void);                           //编码器初始化
void update_speed_and_distance(void);
void update_voltage(void);
#endif
