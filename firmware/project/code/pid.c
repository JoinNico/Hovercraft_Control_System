#include "zf_common_headfile.h"
#include "pid.h"

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */
float positional_pid_realize(_pid *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    pid->integral += pid->err;

    pid->actual_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_next);

    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->actual_val;
}

/**
  * @brief  增量PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */
float incremental_pid_realize(_pid *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 限定闭环死区*/
    if((pid->err < VV_DEADLINE) && (pid->err > -VV_DEADLINE))
    {
        pid->err = 0.0f;
    }

    /*PID算法实现*/
    pid->actual_val += pid->Kp*(pid->err - pid->err_next)
                       + pid->Ki*pid->err
                       + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
    /*传递误差*/
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    /*返回当前实际值*/
    return pid->actual_val;
}

/**
  * @brief  范围限定
  * @param  上、下限；类型： 1 8位； 2 16位；3 浮点
  * @note   无
  * @retval 无
  */

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
