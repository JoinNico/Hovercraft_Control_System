#ifndef _PID_H
#define _PID_H

typedef struct
{
    float target_val;               //目标值
    float actual_val;               //实际值
    float err;                      //定义当前偏差值
    float err_next;                 //定义下一个偏差值
    float err_last;                 //定义最后一个偏差值
    float Kp,Ki,Kd;                 //定义比例、积分、微分系数
    float integral;                 //定义积分值
    float integral_up_limit ;       //设置PID限幅
    float integral_down_limit ;     //设置PID限幅
} _pid;


#define VV_DEADLINE             (0.2f)  //速度增量式PID，设置死区范围，消抖，静止强硬程度
#define PID_MAX                 (50.0f)
#define PID_MIN                 (0.0f)
/**
  * @brief  设置目标值
  * @param  val 目标值
  * @note   无
  * @retval 无
  */
static inline void set_pid_target(_pid *pid, float temp_val) {
    pid->target_val = temp_val;
}

/**
  * @brief  获取目标值
  * @param  无
  * @note   无
  * @retval 目标值
  */
static inline float get_pid_target(_pid *pid) {
    return pid->target_val;
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
  * @note   无
  * @retval 无
  */

static inline void set_pid_param(_pid *pid, float p, float i, float d)
{
    pid->Kp = p;    // 设置比例系数 P
    pid->Ki = i;    // 设置积分系数 I
    pid->Kd = d;    // 设置微分系数 D
}

float positional_pid_realize(_pid *pid, float actual_val);
float incremental_pid_realize(_pid *pid, float actual_val);

#endif

