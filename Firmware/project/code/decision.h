#ifndef _DECISION_H
#define _DECISION_H

#include "zf_common_headfile.h"
#define STEER_DIR 1
#define SLOW_SPEED_RATIO 0.8 //慢速上坡比率
#define FAST_SPEED_RATIO 1.1 //快速下坡比率

typedef struct SmartCar_Status     //智能车运行参数
{
        float goal_angular_velocity;
        int goal_PWM;
        int goal_side_PWM;
        unsigned char motor_on;
        unsigned char motor_close_circle;
        float global_speed;
        int left_duty;
        int right_duty;
        int brushless_duty;
} SMARTCAR_STATUS;

extern SMARTCAR_STATUS smartcar_status;
extern Pid steer_pid;

void decision(void);
float angular_velocity_decision(float middle_line);

#endif
