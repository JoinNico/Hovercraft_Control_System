# include "control.h"

_pid angle_pid, rate_pid, image_pid, speed_pid;

void PID_param_init(void)
{

    //行进电机 角度环 PID 初始化
    angle_pid.Kp = ANGLE_KP;
    angle_pid.Ki = ANGLE_KI;
    angle_pid.Kd = ANGLE_KD;

    //行进电机 角速度环 PID 初始化
    rate_pid.Kp = RATE_KP;
    rate_pid.Ki = RATE_KI;
    rate_pid.Kd = RATE_KD;

    //行进电机 速度环 PID 初始化
    speed_pid.Kp = SPEED_KP;
    speed_pid.Ki = SPEED_KI;
    speed_pid.Kd = SPEED_KD;

    //横向电机 位置环 PID 初始化
    image_pid.Kp = IMAGE_KP;
    image_pid.Ki = IMAGE_KI;
    image_pid.Kd = IMAGE_KD;
}

float OuterLoop_Update(float middle_line_err)
{
    float outerloop_output;

    // 限幅
    middle_line_err = CLAMP(middle_line_err, IMAGE_WIDTH / 2.0f, -IMAGE_WIDTH / 2.0f);

    // 设定 PID 目标值为 0 - 期望中线值
    set_pid_target(&angle_pid, 0);
    // 依据方向宏，利用实际中线值计算外环输出
#if STEER_DIR
    outerloop_output = - positional_pid_realize(&angle_pid, middle_line_err);
#else
    outerloop_output = positional_pid_realize(&angle_pid, middle_line_err);
#endif

    return outerloop_output;
}

float SpeedLoop_Update(void)
{
    float base_throttle;
    // 设定 PID 目标值为全局速度,注意是 smartcar_status
    set_pid_target(&speed_pid, smartcar_status.global_speed);

    // 利用实际速度计算速度环 PID 输出
    base_throttle = incremental_pid_realize(&speed_pid, filter_speed);

    return base_throttle;
}

// TODO
float InnerLoop_Update(float Outer_Result)
{
    float innerloop_output;

    // 设定 PID 目标值为外环输出
    set_pid_target(&angle_pid, Outer_Result);
    // 利用实际角速度计算角速度环 PID 输出
    innerloop_output = positional_pid_realize(&rate_pid, imu.g[imu_z]);

    return innerloop_output;
}

void motor_control(float Outer_Result)
{

    if(1 == smartcar_status.motor_close_circle)
    {
        if(smartcar_status.motor_on)
        {
            int turn_diff = (int)InnerLoop_Update(Outer_Result);
            int rightpwm, leftpwm;
            Brushless_Motor_SetPwmValue(brushless_duty);

            leftpwm  = 2400 + 4 * smartcar_status.base_PWM + turn_diff;
            rightpwm = 2400 + 4 * smartcar_status.base_PWM - turn_diff;

            MOTOR_SetPwmValue(leftpwm, rightpwm);

        }
        else //所有电机关闭
        {
            MOTOR_SetPwmValue(0, 0);
            Brushless_Motor_SetPwmValue(500);
        }
    }
}


static float SideLoop_Update(float middle_line_err)
{
    float goal_sidePWM;

    // 限幅
    middle_line_err = CLAMP(middle_line_err, IMAGE_WIDTH / 2.0f, -IMAGE_WIDTH / 2.0f);

    // 设定 PID 目标值为 0 - 期望中线值
    set_pid_target(&image_pid, 0);
    // 利用实际中线值 计算角侧向电机位置环 PID 输出
    goal_sidePWM = positional_pid_realize(&image_pid, middle_line_err);

    return goal_sidePWM;
}

/* @brief 横向电机闭环控制函数
 */
void side_motor_control(float err)
{
    if(smartcar_param.side_motor_flag)
    {
        smartcar_status.goal_side_PWM = SideLoop_Update(err);

        if (smartcar_status.goal_side_PWM > 0)
        {
            MOTOR_Side_SetPwmValue((int)(1100 + smartcar_param.K_changepwm * smartcar_status.goal_side_PWM),
                                   (int)(1100 + smartcar_param.K_changepwm * smartcar_status.goal_side_PWM));
        }
        else
        {
            MOTOR_Side_SetPwmValue((int)(-1100 + smartcar_param.K_changepwm * smartcar_status.goal_side_PWM),
                                   (int)(-1100 + smartcar_param.K_changepwm * smartcar_status.goal_side_PWM));
        }
    }
    else
    {
        MOTOR_Side_SetPwmValue(0,0);
    }
}
