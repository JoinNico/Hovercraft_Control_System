#include "encoder.h"

int16 encoder=0;
int32 int_dis=0;
float distance;
float wheel_speed;

float voltage;
uint16 raw_voltage;

WheelStruct WheelStr;

void Encoder_Init(void)
{
    encoder_quad_init(ENCODER_QUADDEC_L, ENCODER_QUADDEC_A_L, ENCODER_QUADDEC_B_L);   // 初始化左编码器 正交解码编码器模式

    //测速轮模型初始化
    WheelStr.EncoderLine =  1024.0f;                          //编码器线数
    WheelStr.ReductionRatio = 1.0f;                         //无减速齿轮；
    WheelStr.EncoderValue = 0;
    WheelStr.DiameterWheel = 0.050f;                        //50mm，轮子直径:m
    WheelStr.CloseLoop = 1;                              //默认闭环模式
}

void encoder_handler (void)
{
    encoder  = encoder_get_count(ENCODER_QUADDEC_L)/2;               // 获取速度
    encoder_clear_count(ENCODER_QUADDEC_L);                         // 清空编码器计数

    int_dis += (int32)encoder;
}

void update_speed_and_distance(void)
{
    static float win_1[WIN_SIZE] = { 0 };
    float speed;
    encoder_handler();

    //  速度系数    -> m/s
    //  距离系数    -> m
    distance = (float)(int_dis * PI * WheelStr.DiameterWheel)/ WheelStr.EncoderLine / WheelStr.ReductionRatio;

    speed = (float)(encoder * PI * WheelStr.DiameterWheel)/ ENCODER_CONTROL_CYCLE / WheelStr.EncoderLine / WheelStr.ReductionRatio;

    wheel_speed = LowPassFilter(win_1, speed);

    pidStr.vi_FeedBack = encoder;
}

void update_voltage(void)
{
    raw_voltage = adc_mean_filter_convert(ADC1_IN9_B1,10);
    voltage = raw_voltage*19.06/256.0;

}
