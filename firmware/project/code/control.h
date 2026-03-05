/*
 * control.h
 *
 *  Created on: 2026Äê3ÔÂ2ÈÕ
 *      Author: JoinNico
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "zf_common_headfile.h"

#define SPEED_KP             smartcar_param.velocity_kp
#define SPEED_KI             smartcar_param.velocity_ki
#define SPEED_KD             smartcar_param.velocity_kd

#define ANGLE_KP             smartcar_param.angle_kp
#define ANGLE_KI             smartcar_param.angle_ki
#define ANGLE_KD             smartcar_param.angle_kd

#define RATE_KP              smartcar_param.rate_kp
#define RATE_KI              smartcar_param.rate_ki
#define RATE_KD              smartcar_param.rate_kd

#define IMAGE_KP             smartcar_param.image_kp
#define IMAGE_KI             smartcar_param.image_ki
#define IMAGE_KD             smartcar_param.image_kd

#define STEER_DIR            1
#define CLAMP(x, lo, hi)     ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

extern _pid angle_pid, rate_pid, image_pid, speed_pid;

void PID_param_init(void);
float OuterLoop_Update(float middle_line_err);
float InnerLoop_Update(float Outer_Result);
float SpeedLoop_Update(void);
void motor_control(float Outer_Result);
void side_motor_control(void);
#endif /* CONTROL_H_ */
