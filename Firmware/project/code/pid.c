#include "zf_common_headfile.h"
#include "pid.h"

void range_protect(void * value,float upper_limit,float lower_limit,uint8 type)
{
    switch(type)
    {
    case 1://
        {
            uint8 * p = (uint8 *) value;
            if((* p) > (uint8) upper_limit)
                * p = (uint8) upper_limit;
            else if((* p) < (uint8) lower_limit)
                * p = (uint8) lower_limit;
            break;
        }
    case 2://int16_t
        {
            int16 * p = (int16 *) value;
            if((* p) > (int16) upper_limit)
                * p = (int16) upper_limit;
            else if((* p) < (int16) lower_limit)
                * p = (int16) lower_limit;
            break;
        }
    case 3://float
        {
            float * p = (float *) value;
            if((* p) > (float) upper_limit)
                * p = (float) upper_limit;
            else if((* p) < (float) lower_limit)
                * p = (float) lower_limit;
            break;
        }
    }
}

//积分饱和，输出限幅
float positional_pid(Pid *pid, float sensor_val)
{
    float output;
    float error = 0.0f;

    float p_error = 0.0f;//当前误差项
    float i_error = 0.0f;//积分误差项
    float d_error = 0.0f;//微分误差项

    error = pid->set_point - sensor_val;

    p_error = error;
    i_error = pid->sum_error;
    d_error = error - pid->last_error;

    //积分限幅
    range_protect(&pid->sum_error, 5000.0f, -5000.0f, 3);

    output = pid->K_p * p_error + pid->K_i * i_error + pid->K_d * d_error;

    //输出限幅
    if(output > 5000.0f) output = 5000.0f;
    else if(output < -5000.0f) output = -5000.0f;

    pid->last_error = error;
    pid->sum_error += error;

    return output;
}

//积分饱和，输出限幅
int image_positional_pid(Pid *pid, float taget_val, float sensor_val)//位置式pid
{
    float output = 0.0f;
    float error = 0.0f;

    float p_error = 0.0f;//当前误差项
    float i_error = 0.0f;//积分误差项
    float d_error = 0.0f;//微分误差项

    pid->set_point = taget_val;
    error = pid->set_point - sensor_val;

    p_error = error;
    i_error = pid->sum_error;
    d_error = error - pid->last_error;

    output = pid->K_p * p_error + pid->K_i * i_error + pid->K_d * d_error;

    pid->last_error = error;
    pid->sum_error += error;

    return (int)output;
}

//输出限幅
int incremental_pid(Pid *pid, float sensor_val)//增量式pid，无需计算积分项，即积累误差
{
    float output = 0.0f;
    float error = 0.0f;

    float p_error = 0.0f;//比例误差项
    float i_error = 0.0f;//积分误差项
    float d_error = 0.0f;//微分误差项

    error = pid->set_point - sensor_val;

    p_error = error - pid->last_error;
    i_error = error;
    d_error = error - 2 * pid->last_error + pid->last_2_error;

    output = pid->K_p * p_error + pid->K_i * i_error + pid->K_d * d_error;

    pid->last_2_error = pid->last_error;
    pid->last_error = error;
    return  (int)output;

}

Pid angle_pid, rate_pid, image_pid;
PIDStruct pidStr;

void motor_pid_init(void){

    //行进电机角度PID初始化
    angle_pid.set_point = 0;
    angle_pid.K_p = ANGLE_KP;
    angle_pid.K_i = ANGLE_KI;
    angle_pid.K_d = ANGLE_KD;
    angle_pid.sum_error = 0;
    angle_pid.last_error = 0;
    angle_pid.last_2_error = 0;


    //行进电机角速度PID初始化
    rate_pid.set_point = 0;
    rate_pid.K_p = RATE_KP;
    rate_pid.K_i = RATE_KI;
    rate_pid.K_d = RATE_KD;
    rate_pid.sum_error = 0;
    rate_pid.last_error = 0;
    rate_pid.last_2_error = 0;


    //行进电机速度PID初始化
    pidStr.vi_Ref = 0 ;
    pidStr.vi_FeedBack = 0 ;
    pidStr.vi_PreError = 0 ;
    pidStr.vi_PreDerror = 0 ;
    pidStr.v_Kp = VELOCITY_KP;
    pidStr.v_Ki = VELOCITY_KI;
    pidStr.v_Kd = VELOCITY_KD;
    pidStr.vl_PreU = 0;
}

void side_motor_pid_init(void){

    //横向电机角度PID初始化
    image_pid.set_point = 0;
    image_pid.K_p = IMAGE_KP;
    image_pid.K_i = IMAGE_KI;
    image_pid.K_d = IMAGE_KD;
    image_pid.sum_error = 0;
    image_pid.last_error = 0;
    image_pid.last_2_error = 0;
}

#define VV_MAX 3
#define VV_MIN -3

/**
* @brief        PID速控模型
* @param
* @author       sundm
* @note
**/
signed int PID_MoveCalculate(PIDStruct *pp)
{
    float  error,d_error,dd_error;
    float I_error;

    error = pp->vi_Ref - pp->vi_FeedBack;
    d_error = error - pp->vi_PreError;
    dd_error = d_error - pp->vi_PreDerror;

    pp->vi_PreError = error;
    pp->vi_PreDerror = d_error;

    if( ( error < VV_DEADLINE ) && ( error > -VV_DEADLINE ) )
    {
        ;
    }
    else
    {
        pp->vl_PreU += (pp -> v_Kp * d_error + pp -> v_Ki * error + pp->v_Kd*dd_error);
    }

    if( pp->vl_PreU >= MOTOR_PWM_MAX )
    {
        pp->vl_PreU = MOTOR_PWM_MAX;
    }
    else if( pp->vl_PreU <= MOTOR_PWM_MIN )
    {
        pp->vl_PreU = MOTOR_PWM_MIN;
    }

    return (pp->vl_PreU);
}

