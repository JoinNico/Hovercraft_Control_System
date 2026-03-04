# include "control.h"

_pid angle_pid, rate_pid, image_pid, speed_pid;

void PID_param_init(void)
{

    //行进电机角度环 PID 初始化
    angle_pid.Kp = ANGLE_KP;
    angle_pid.Ki = ANGLE_KI;
    angle_pid.Kd = ANGLE_KD;

    //行进电机角速度环 PID 初始化
    rate_pid.Kp = RATE_KP;
    rate_pid.Ki = RATE_KI;
    rate_pid.Kd = RATE_KD;


    //行进电机速度环 PID 初始化
    rate_pid.Kp = SPEED_KP;
    rate_pid.Ki = SPEED_KI;
    rate_pid.Kd = SPEED_KD;

    //横向电机角度 PID 初始化
    image_pid.Kp = IMAGE_KP;
    image_pid.Ki = IMAGE_KI;
    image_pid.Kd = IMAGE_KD;
}

int PWM_decision(float angular_velocity)
{
    int pwm;
    pid_param_init();
    change_set_point(&rate_pid, angular_velocity);

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


#define VELOCITY_CONTROL_LOOP 1 //1-速度闭环 0-速度开环
#if VELOCITY_CONTROL_LOOP
    int motor_pwm;
    int leftpwm;
    int rightpwm;
    pid_param_init();

    pidStr.vi_Ref = (float)(speed * ENCODER_CONTROL_CYCLE / WheelStr.DiameterWheel / PI * WheelStr.EncoderLine );

    motor_pwm = PID_MoveCalculate(&pidStr);

    leftpwm = 2400+4*motor_pwm + changepwm;
    rightpwm = 2400+4*motor_pwm - changepwm;

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

void motor_control(void)
{

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
        MOTOR_Side_SetPwmValue((int)(1100 + smartcar_param.K_changepwm * changepwm), (int)(1100+smartcar_param.K_changepwm * changepwm));
    }
    else if(changepwm < 0){
        MOTOR_Side_SetPwmValue((int)(-1100 + smartcar_param.K_changepwm * changepwm), (int)(-1100+smartcar_param.K_changepwm * changepwm));
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
        smartcar_status.goal_side_PWM = side_motor_decision(get_image_error());
        Side_Motor_ControlLoop(image_pid, smartcar_status.goal_side_PWM);
    }else{
        Side_Motor_ControlLoop(image_pid, 0);

    }
}
