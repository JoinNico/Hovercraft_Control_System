#include "decision.h"

float * angle_kp = &(angle_pid.K_p);
float * angle_ki = &(angle_pid.K_i);
float * angle_kd = &(angle_pid.K_d);

SMARTCAR_STATUS smartcar_status;

float angular_velocity_decision(float middle_line)
{
    float angular_velocity;
    angle_pid.set_point = 0;

    if(middle_line > IMAGE_WIDTH / 2)
        middle_line = IMAGE_WIDTH / 2;
    else if(middle_line < -IMAGE_WIDTH / 2)
        middle_line = -IMAGE_WIDTH / 2;

#if STEER_DIR//
    angular_velocity = -positional_pid(&angle_pid, middle_line);
#else
    angular_velocity = positional_pid(&angle_pid, middle_line);
#endif

//    if(angle > STEER_MAX_ANGLE)
//        angle = STEER_MAX_ANGLE;
//    else if(angle < -STEER_MAX_ANGLE)
//        angle = -STEER_MAX_ANGLE;
    return angular_velocity;
}

int side_motor_decision(float middle_line)
{
    float goal_sidePWM;
    image_pid.set_point = 0;

    if(middle_line > IMAGE_WIDTH / 2)
        middle_line = IMAGE_WIDTH / 2;
    else if(middle_line < -IMAGE_WIDTH / 2)
        middle_line = -IMAGE_WIDTH / 2;

    goal_sidePWM = positional_pid(&image_pid, middle_line);


//    if(angle > STEER_MAX_ANGLE)
//        angle = STEER_MAX_ANGLE;
//    else if(angle < -STEER_MAX_ANGLE)
//        angle = -STEER_MAX_ANGLE;
    return goal_sidePWM;
}
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
    static float err = 0.0;

    switch (get_road_type())
    {
//        case SSTART:
//            smartcar_status.steer_on = 1;
//            smartcar_status.motor_on = 1;
//            smartcar_status.left_goal_speed = BARN_SPEED;
//            smartcar_status.right_goal_speed = BARN_SPEED;
//            smartcar_status.global_speed = BARN_SPEED;
//            smartcar_status.motor_close_circle =1;
//            err = get_image_error();
//            *steer_kp = smartcar_param.bend_steer_kp;
//            *steer_kd = smartcar_param.bend_steer_kd;
//            *steer_ki = 0;
//            smartcar_status.goal_angle = angle_decision(err);
//            break;
    case LONG_STRAIGHT:
        smartcar_status.motor_on = 1;
        smartcar_status.global_speed = smartcar_param.global_speed*1.2;
        smartcar_status.motor_close_circle =1;
        *angle_kp = ANGLE_KP;
        *angle_ki = ANGLE_KI;
        *angle_kd = ANGLE_KD;
        err = get_image_error();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);
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
        err = get_image_error();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);
        break;
    case CIRCLE_IN         :
        smartcar_status.motor_on = 1;
        smartcar_param.side_motor_flag = 0;
        smartcar_status.global_speed = smartcar_param.global_speed;
        smartcar_status.motor_close_circle = 1;
        *angle_kp = ANGLE_KP * 1.0;
        *angle_ki = ANGLE_KI * 0.5;
        *angle_kd = ANGLE_KD * 0.5;
        //err = get_circle_modify_err();
        err = get_image_error();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);

        break;
    case CIRCLE_FORECAST   :
//        smartcar_status.motor_on = 1;
//        smartcar_param.side_motor_flag = 0;
//        smartcar_status.global_speed = 0.6f;
//        smartcar_status.motor_close_circle = 1;
//        *angle_kp = ANGLE_KP * 0.8;
//        *angle_ki = ANGLE_KI * 0.7;
//        *angle_kd = ANGLE_KD * 0.7;
//        err = get_circle_modify_err();
//        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
//        smartcar_status.goal_side_PWM = side_motor_decision(err);
//        break;
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
        err = get_circle_modify_err();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);
        break;

//    case BLOCK:
//        smartcar_status.motor_on = 1;
//        smartcar_param.side_motor_flag = 1;
//        smartcar_status.global_speed = smartcar_param.global_speed;
//        smartcar_status.motor_close_circle = 1;
//        *angle_kp = ANGLE_KP;
//        *angle_ki = ANGLE_KI;
//        *angle_kd = ANGLE_KD;
//        err = get_image_error();
//        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
//        smartcar_status.goal_side_PWM = side_motor_decision(err);
//        break;

//    case CROSS:
//        smartcar_status.steer_on = 1;
//        smartcar_status.motor_on = 1;
//		smartcar_status.left_goal_speed = smartcar_param.global_speed;
//        smartcar_status.right_goal_speed = smartcar_param.global_speed;
//        smartcar_status.global_speed = smartcar_param.global_speed;
//        smartcar_status.motor_close_circle = 1;
//        *steer_kp = 6.0;
//        *steer_ki = 0.0;
//        *steer_kd = 36.0;
//        image_weight = 0.0;
////        err=get_image_error();
//        set_inductor_error_co(1.0, 0.15, 0, 0.15, 1.0, 1, 1);
//        err = get_inductor_err();
//        smartcar_status.goal_angle = angle_decision(err);
//        break;
    case BLOCK_IN:
        smartcar_status.motor_on = 1;
        smartcar_status.global_speed = 0.5;
        smartcar_status.motor_close_circle = 1;
        *angle_kp = ANGLE_KP * 1.2;
        *angle_ki = ANGLE_KI * 1.2;
        *angle_kd = ANGLE_KD * 1.2;
        err = get_image_error();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);
        break;
    case FINISH:
        smartcar_status.motor_on = 1;
        smartcar_status.global_speed = smartcar_param.global_speed;
        smartcar_status.motor_close_circle = 1;
        *angle_kp = ANGLE_KP;
        *angle_ki = ANGLE_KI;
        *angle_kd = ANGLE_KD;
        err = get_image_error();
        smartcar_status.goal_angular_velocity = angular_velocity_decision(err);
        smartcar_status.goal_side_PWM = side_motor_decision(err);
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
