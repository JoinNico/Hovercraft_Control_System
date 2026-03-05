#include "decision.h"

float * angle_kp = &(angle_pid.Kp);
float * angle_ki = &(angle_pid.Ki);
float * angle_kd = &(angle_pid.Kd);

SMARTCAR_STATUS smartcar_status;

float straight_acceleration(int global_speed, float ratio, float thresh) {

      if(thresh == 0)
        return global_speed;
      float g_SpeedMax = ratio * global_speed;
      float g_SpeedMin = global_speed;

      float x = 0.0f;

      x = fabs(get_variance() / thresh);

      x = x > 1.0f ? 1.0f : x;

      float a = g_SpeedMin - g_SpeedMax;
      float c = g_SpeedMax;

      float cur_speed = a * x * x + c;

      return cur_speed;
}



uint8 circle_times = 0;

void decision(void)
{
//    static float err = 0.0;

    switch (get_road_type())
    {
        case LONG_STRAIGHT:
            smartcar_status.motor_on = 1;
            smartcar_status.global_speed = smartcar_param.global_speed * 1.2;
            smartcar_status.motor_close_circle =1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;
        case BEND:
            smartcar_status.motor_on = 1;
            if((get_image_error() > 10.0f) || (get_image_error() < -10.0f) )
            {
                smartcar_param.side_motor_flag = 1;
            }
            else {
                smartcar_param.side_motor_flag = 0;
            }
            smartcar_status.global_speed = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;
        case CIRCLE_IN         :
            smartcar_status.motor_on = 1;
            smartcar_param.side_motor_flag = 0;
            smartcar_status.global_speed = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 1.0;
            *angle_ki = ANGLE_KI * 0.5;
            *angle_kd = ANGLE_KD * 0.5;
            break;
        case CIRCLE_FORECAST   :
        case CIRCLE_READY_ENTRY:
        case CIRCLE_ENTRY      :
        case CIRCLE_READY_EXIT :
        case CIRCLE_EXIT       :
            smartcar_status.motor_on = 1;
            smartcar_param.side_motor_flag = 1;
            smartcar_status.global_speed = 0.6f;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 0.8;
            *angle_ki = ANGLE_KI * 0.7;
            *angle_kd = ANGLE_KD * 0.7;
// TODO
//            err = get_circle_modify_err();

            break;
        case BLOCK_IN:
            smartcar_status.motor_on = 1;
            smartcar_status.global_speed = 0.5;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 1.2;
            *angle_ki = ANGLE_KI * 1.2;
            *angle_kd = ANGLE_KD * 1.2;
            break;
        case FINISH:
            smartcar_status.motor_on = 1;
            smartcar_status.global_speed = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;
        case FINISH_STOP:
            smartcar_status.motor_on = 0;
            smartcar_status.global_speed = 0;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            smartcar_status.motor_close_circle = 1;
            smartcar_status.goal_angular_velocity = 0;
            break;
        default:
           break;
   }
}
