#ifndef _motor_h
#define _motor_h

#include "zf_common_headfile.h"
#include "pid.h"

#define MOTOR2_A   TIM2_PWM_MAP0_CH1_A0  //定义3电机正转PWM引脚
#define MOTOR2_B   TIM2_PWM_MAP0_CH2_A1  //定义3电机反转PWM引脚

#define MOTOR1_A   TIM2_PWM_MAP0_CH3_A2  //定义4电机正转PWM引脚
#define MOTOR1_B   TIM2_PWM_MAP0_CH4_A3  //定义4电机反转PWM引脚

#define MOTOR3_A   TIM4_PWM_MAP1_CH1_D12  //定义3电机正转PWM引脚
#define MOTOR3_B   TIM4_PWM_MAP1_CH2_D13  //定义3电机反转PWM引脚

#define MOTOR4_A   TIM4_PWM_MAP1_CH3_D14  //定义4电机正转PWM引脚
#define MOTOR4_B   TIM4_PWM_MAP1_CH4_D15  //定义4电机反转PWM引脚


#define BRUSHLESS_1 TIM3_PWM_MAP3_CH1_C6
#define BRUSHLESS_2 TIM3_PWM_MAP3_CH2_C7


#define PWM_MAX 6000

#define  MOTOR_PWM_MAX              6000        //OCR=95%,禁止满占空比输出，造成MOS损坏
#define  MOTOR_PWM_MIN              -6000       //OCR=95%
#define BRUSHLESS_PWM_MIN           500
#define BRUSHLESS_PWM_MAX           1000
#define  MOTOR_SPEED_MAX            3.0f       //电机最大转速(m/s) (0.017,8.04)


typedef struct
{
    float EncoderLine ;                         //编码器线数=光栅数16*4
    signed int EncoderValue;                    //编码器实时速度
    float DiameterWheel;                        //轮子直径：mm
    uint8_t CloseLoop;                             //开环模式
    uint16_t Counter;                           //线程计数器
}MotorStruct;

extern MotorStruct motorStr;

extern int single_duty;
extern int left_duty;
extern int right_duty;
extern int leftside_duty;
extern int rightside_duty;
extern int brushless_duty;
extern int base_pwm;

void motor_init(void);

static inline int get_left_duty(void) {return left_duty;}
static inline int get_right_duty(void) {return right_duty;}

void MOTOR_SetPwmValue(int pwm_l, int pwm_r);
void MOTOR_Side_SetPwmValue(int pwm_ls, int pwm_rs);
void Brushless_Motor_SetPwmValue(int duty);

int PWM_decision(float angular_velocity);

void motor_control(void);
void side_motor_control(void);
#endif
