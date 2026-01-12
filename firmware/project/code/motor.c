#include "motor.h"

int base_pwm = 3500;
int left_duty = 0;
int right_duty = 0;
int leftside_duty = 0;
int rightside_duty = 0;
int brushless_duty = 560;
int motor_start_flag;

//#define MOTOR_RDEAD_VAL -200
//#define MOTOR_LDEAD_VAL 800//the max of duty 10000
//
//#define MOTOR_RDEAD_VAL -1000
//#define MOTOR_LDEAD_VAL 1000//the max of duty 10000

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

    motor_pid_init();
    side_motor_pid_init();
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
//    leftside_motor_duty(pwm_l);
    right_motor_duty(pwm_r);
}

void MOTOR_Side_SetPwmValue(int pwm_ls, int pwm_rs)
{
    leftside_motor_duty(pwm_ls);
    rightside_motor_duty(pwm_rs);
}

static void pid_param_init(void)
{
    angle_pid.K_p = ANGLE_KP;
    angle_pid.K_i = ANGLE_KI;
    angle_pid.K_d = ANGLE_KD;

    rate_pid.K_p = RATE_KP;
    rate_pid.K_i = RATE_KI;
    rate_pid.K_d = RATE_KD;

    pidStr.v_Kp = VELOCITY_KP;
    pidStr.v_Ki = VELOCITY_KI;
    pidStr.v_Kd = VELOCITY_KD;

    image_pid.K_p = IMAGE_KP;
    image_pid.K_i = IMAGE_KI;
    image_pid.K_d = IMAGE_KD;

}

int PWM_decision(float angular_velocity)
{
    int pwm;
    pid_param_init();
    change_set_point(&rate_pid, angular_velocity);

//    change_set_point(&rate_pid,90);
    pwm = (int)positional_pid(&rate_pid, imu.g[imu_z]);

    return pwm;
}
/*
 * @brief 行进电机闭环控制函数
 * @param pidStr 行进电机速度环PID
 * @param speed 设定速度
 * @param changepwm 固定打角
 * @note
 */
void MOTOR_ControlLoop(PIDStruct pidStr, float speed, int changepwm)
{

//    if(speed > MOTOR_SPEED_MAX)
//        speed = MOTOR_SPEED_MAX;
//    else if(speed < -MOTOR_SPEED_MAX)
//        speed = -MOTOR_SPEED_MAX;
#define VELOCITY_CONTROL_LOOP 1 //1-速度闭环 0-速度开环
#if VELOCITY_CONTROL_LOOP
    int motor_velocity;
    int leftpwm;
    int rightpwm;
    pid_param_init();

    pidStr.vi_Ref = (float)(speed * ENCODER_CONTROL_CYCLE / WheelStr.DiameterWheel / PI * WheelStr.EncoderLine );

//    dat[0] = (float)(speed * ENCODER_CONTROL_CYCLE / WheelStr.DiameterWheel / PI * WheelStr.EncoderLine );
    motor_velocity =PID_MoveCalculate(&pidStr);
//    dat[1] = motor_velocity;
    //printf("base_pwm=%d\r\n",20*motor_velocity);
//    MOTOR_SetPwmValue(2400+4*motor_velocity,2400+4*motor_velocity);

    leftpwm = 2400+4*motor_velocity + changepwm;
    rightpwm = 2400+4*motor_velocity - changepwm;

    if((changepwm > 0) && (leftpwm > 6000))
    {
        leftpwm = 6000;
        rightpwm = 6000 - 2*changepwm;
    }
    if((changepwm < 0) && (rightpwm > 6000))
    {
        leftpwm = 6000 + 2*changepwm;
        rightpwm = 6000;
    }

    MOTOR_SetPwmValue(leftpwm, rightpwm);
#else
//    MOTOR_SetPwmValue(base_pwm + changepwm, base_pwm - changepwm);
   MOTOR_SetPwmValue(base_pwm, base_pwm);
#endif
}

//void MOTOR_VelocityControlLoop(PIDStruct pidStr, float speed)
//{
//    pid_param_init();
//
//    pidStr.vi_Ref = (float)(speed * ENCODER_CONTROL_CYCLE / WheelStr.DiameterWheel / PI * WheelStr.EncoderLine );
//
//    MOTOR_SetPwmValue(-PID_MoveCalculate(&pidStr),PID_MoveCalculate(&pidStr));
//}

void motor_control(void)
{
//    static int motor_delay_start = 0;
//    if(motor_start_flag == 0)
//    {
//      motor_delay_start = 200;
//      left_motor_duty(0);
//      right_motor_duty(0);
//      return;
//    }
//    if(motor_start_flag == 1)
//    {
//      motor_delay_start--;
//      if(motor_delay_start > 0)
//        return;
//    }


    if(1 == smartcar_status.motor_close_circle)
    {
        if(smartcar_status.motor_on)
        {
            MOTOR_ControlLoop(pidStr, smartcar_status.global_speed, smartcar_status.goal_PWM);
            if((get_image_error() > 10.0f || fabsf(get_curvity()) > 0.7f ) && get_speed() > 0.6f && get_road_type() != CIRCLE_IN)
            {
                Brushless_Motor_SetPwmValue(800);
            }
            else {
                Brushless_Motor_SetPwmValue(brushless_duty);
            }

        }
        else
        {
            MOTOR_SetPwmValue(0, 0);
            Brushless_Motor_SetPwmValue(500);
        }
    }
}

/* @brief 横向电机闭环控制函数
 * @param 横向电机位置式PID
 * @param changepwm pid输出值
 */
void Side_Motor_ControlLoop(Pid image_pid, int changepwm)
{
    pid_param_init();
    if (changepwm > 0)
    {
        MOTOR_Side_SetPwmValue((int)(1100+smartcar_param.K_changepwm * changepwm), (int)(1100+smartcar_param.K_changepwm * changepwm));
    }
    else if(changepwm < 0){
        MOTOR_Side_SetPwmValue((int)(-1100+smartcar_param.K_changepwm * changepwm), (int)(-1100+smartcar_param.K_changepwm * changepwm));
    }
    else {
        MOTOR_Side_SetPwmValue(0,0);
    }

//    printf("%d\r\n",(int)smartcar_param.K_changepwm * changepwm);
}

void side_motor_control(void)
{
    if(smartcar_param.side_motor_flag)
    {
        Side_Motor_ControlLoop(image_pid, smartcar_status.goal_side_PWM);
    }else{
        Side_Motor_ControlLoop(image_pid, 0);

    }
}
