#include "encoder.h"

float filter_distance, filter_speed;
WheelStruct WheelStr;

void Encoder_Init(void)
{
    // 初始化左编码器（正交解码模式）
    encoder_quad_init(ENCODER_QUADDEC_L, ENCODER_QUADDEC_A_L, ENCODER_QUADDEC_B_L);

    //测速轮模型初始化
    WheelStr.EncoderLine =  1024.0f;                          //编码器线数
    WheelStr.ReductionRatio = 1.0f;                         //无减速齿轮；
    WheelStr.EncoderValue = 0;
    WheelStr.DiameterWheel = 0.050f;                        //50mm，轮子直径:m
    WheelStr.CloseLoop = 1;                              //默认闭环模式
}

void update_speed_and_distance(void)
{
    int16 pulses = 0;
    static int32 total_pulses  = 0;

    // 读取编码器计数
    pulses = encoder_get_count(ENCODER_QUADDEC_L) / 2;
    // 清空编码器计数
    encoder_clear_count(ENCODER_QUADDEC_L);

    // 累积总脉冲数（用于距离计算）
    total_pulses += (int32)pulses;

    static float distance_filter_buf[WIN_SIZE] = { 0 };
    float distance, speed;

    // 距离系数 -> m
    distance = (float)(total_pulses * PI * WheelStr.DiameterWheel)
               / WheelStr.EncoderLine / WheelStr.ReductionRatio;
    filter_distance = SlidingFilter(distance_filter_buf, distance);

    // 速度系数 -> m/s
    speed = (float)(pulses * PI * WheelStr.DiameterWheel)
            / ENCODER_CONTROL_CYCLE / WheelStr.EncoderLine / WheelStr.ReductionRatio;

    filter_speed = first_order_lpf(&filter_speed, speed, 10, 100);
}
