#include "motor.h"

int base_pwm = 3500;
int left_duty = 0;
int right_duty = 0;
int leftside_duty = 0;
int rightside_duty = 0;
int brushless_duty = 560;
int motor_start_flag;

#define MOTOR_RDEAD_VAL 200
#define MOTOR_LDEAD_VAL 200//the max of duty 10000
int single_duty;

MotorStruct motorStr;

void motor_init(void)
{
    pwm_init(MOTOR1_A, 17000, 0);
    pwm_init(MOTOR1_B, 17000, 0);
    pwm_init(MOTOR2_A, 17000, 0);
    pwm_init(MOTOR2_B, 17000, 0);
    pwm_init(MOTOR3_A, 17000, 0);
    pwm_init(MOTOR3_B, 17000, 0);
    pwm_init(MOTOR4_A, 17000, 0);
    pwm_init(MOTOR4_B, 17000, 0);

    pwm_init(BRUSHLESS_1, 50, 0);
    pwm_init(BRUSHLESS_2, 50, 0);
}


void left_motor_duty(int duty)
{
    left_duty = duty;
    if(left_duty > 0)
        left_duty += MOTOR_LDEAD_VAL;
    else
        left_duty -= MOTOR_LDEAD_VAL;
    if(left_duty > PWM_MAX)
        left_duty = PWM_MAX;
    else if(left_duty < -PWM_MAX)
        left_duty = -PWM_MAX;

    if(0 <= left_duty) //电机2   正转
    {
        pwm_set_duty(MOTOR2_A, left_duty);
        pwm_set_duty(MOTOR2_B, 0);
    }
    else                //电机2   反转
    {
        pwm_set_duty(MOTOR2_A, 0);
        pwm_set_duty(MOTOR2_B, -left_duty);
    }

}

void leftside_motor_duty(int duty)
{
    leftside_duty = duty;
    if(leftside_duty > 0)
        leftside_duty += MOTOR_LDEAD_VAL;
    else
        leftside_duty -= MOTOR_LDEAD_VAL;
    if(leftside_duty > PWM_MAX)
        leftside_duty = PWM_MAX;
    else if(leftside_duty < -PWM_MAX)
        leftside_duty = -PWM_MAX;

    if(0 <= leftside_duty) //电机3   正转
    {
        pwm_set_duty(MOTOR3_A, leftside_duty);
        pwm_set_duty(MOTOR3_B, 0);

    }
    else                //电机3   反转
    {
        pwm_set_duty(MOTOR3_A, 0);
        pwm_set_duty(MOTOR3_B, -leftside_duty);
    }


}

void right_motor_duty(int duty)
{
    right_duty = duty;
    if(right_duty > 0)
        right_duty += MOTOR_RDEAD_VAL;
    else
        right_duty -= MOTOR_RDEAD_VAL;
    if(right_duty > PWM_MAX)
        right_duty = PWM_MAX;
    else if(right_duty < -PWM_MAX)
        right_duty = -PWM_MAX;

    if(0 <= right_duty) //电机1   正转
    {
        pwm_set_duty(MOTOR1_A, right_duty);
        pwm_set_duty(MOTOR1_B, 0);

    }
    else                //电机1   反转
    {
        pwm_set_duty(MOTOR1_A, 0);
        pwm_set_duty(MOTOR1_B, -right_duty);
    }
}

void rightside_motor_duty(int duty)
{
    rightside_duty = duty;
    if(rightside_duty > 0)
        rightside_duty += MOTOR_RDEAD_VAL;
    else
        rightside_duty -= MOTOR_RDEAD_VAL;
    if(rightside_duty > PWM_MAX)
        rightside_duty = PWM_MAX;
    else if(rightside_duty < -PWM_MAX)
        rightside_duty = -PWM_MAX;

    if(0 <= rightside_duty) //电机4   正转
    {
        pwm_set_duty(MOTOR4_A, rightside_duty);
        pwm_set_duty(MOTOR4_B, 0);
    }
    else                //电机4   反转
    {
        pwm_set_duty(MOTOR4_A, 0);
        pwm_set_duty(MOTOR4_B, -rightside_duty);
    }

}

void Brushless_Motor_SetPwmValue(int duty)
{
    if(duty > BRUSHLESS_PWM_MAX)
        duty = BRUSHLESS_PWM_MAX;
    else if(duty < BRUSHLESS_PWM_MIN)
        duty = BRUSHLESS_PWM_MIN;

    pwm_set_duty(BRUSHLESS_1, duty);
    pwm_set_duty(BRUSHLESS_2, duty);
}

void MOTOR_SetPwmValue(int pwm_l, int pwm_r)
{
    left_motor_duty(pwm_l);
    right_motor_duty(pwm_r);
}

void MOTOR_Side_SetPwmValue(int pwm_ls, int pwm_rs)
{
    leftside_motor_duty(pwm_ls);
    rightside_motor_duty(pwm_rs);
}

