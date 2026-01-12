#ifndef _PID_H
#define _PID_H

//位置式PID模型
typedef struct
{
    float set_point;
    float K_p;
    float K_i;
    float K_d;
    float sum_error;
    float last_error;
    float last_2_error;
}Pid;


//PID速控模型
typedef struct
{
    float vi_Ref;                       //速度PID，速度设定值
    float vi_FeedBack;                  //速度PID，速度反馈值
    float vi_PreError;                  //速度PID，速度误差,vi_Ref - vi_FeedBack
    float vi_PreDerror;                 //速度PID，前一次，速度误差之差，d_error-PreDerror;
    float v_Kp;                         //比例系数，Kp = Kp
    float v_Ki;                         //积分系数，Ki = Kp * ( T / Ti )
    float v_Kd;                         //微分系数，Kd = KP * Td * T
    float vl_PreU;                      //PID输出值
}PIDStruct;

#define VV_DEADLINE             0.2                       //速度PID，设置死区范围，消抖，静止强硬程度
#define VELOCITY_KP             smartcar_param.velocity_kp
#define VELOCITY_KI             smartcar_param.velocity_ki
#define VELOCITY_KD             smartcar_param.velocity_kd

#define ANGLE_KP                smartcar_param.angle_kp
#define ANGLE_KI                smartcar_param.angle_ki
#define ANGLE_KD                smartcar_param.angle_kd

#define RATE_KP                 smartcar_param.rate_kp
#define RATE_KI                 smartcar_param.rate_ki
#define RATE_KD                 smartcar_param.rate_kd

#define IMAGE_KP                smartcar_param.image_kp
#define IMAGE_KI                smartcar_param.image_ki
#define IMAGE_KD                smartcar_param.image_kd

#define PID_MAX                 50.0f
#define PID_MIN                 0.0f

extern PIDStruct pidStr;

extern Pid angle_pid, rate_pid, image_pid;

static inline void change_set_point(Pid *pid, float set_point)
{
    pid->set_point = set_point;
}

static inline void set_pid_param(Pid *pid, float P, float I, float D)
{
    pid->K_p = P;
    pid->K_i = I;
    pid->K_d = D;
}

float positional_pid(Pid *pid, float sensor_val);
int incremental_pid(Pid *pid, float sensor_val);
int image_positional_pid(Pid *pid, float taget_val, float sensor_val);

//signed int PID_TurnCalculate(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);//角度串级PID控制
signed int PID_MoveCalculate(PIDStruct *pp);//速度PID控制

void motor_pid_init(void);
void side_motor_pid_init(void);

#endif

