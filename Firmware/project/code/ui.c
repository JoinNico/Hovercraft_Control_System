/*
 * ui.c
 *
 *  Created on: 2024/1/24
 *      Author: Misaka, JoeNico
 */
#include "zf_common_headfile.h"
void home_proc(KEY_MSG* msg);
static void home_show(uint8_t update_flag);
void list1_proc(KEY_MSG* msg);
static void list1_show(uint8_t update_flag);
void list2_proc(KEY_MSG* msg);
static void list2_show(uint8_t update_flag);
void list3_proc(KEY_MSG* msg);
static void list3_show(uint8_t update_flag);
void list4_proc(KEY_MSG* msg);
static void list4_show(uint8_t update_flag);
void list5_proc(KEY_MSG* msg);
void list6_proc(KEY_MSG* msg);
void list7_proc(KEY_MSG* msg);
void list8_proc(KEY_MSG* msg);
UI_LIST ui_list[] = //UI表
{
    {E_HOME,   home_proc},
    {E_LIST1,  list1_proc},
    {E_LIST2,  list2_proc},
    {E_LIST3,  list3_proc},
    {E_LIST4,  list4_proc},
    {E_LIST5,  list5_proc},
    {E_LIST6,  list6_proc},
    {E_LIST7,  list7_proc},
    {E_LIST8,  list8_proc},
};

const setting_LIST homeList[]={
        {"Sensor", 16, IPS200_8X16_FONT},
        {"Motor",    16, IPS200_8X16_FONT},
        {"Camera",   16, IPS200_8X16_FONT},
        {"PID",      16, IPS200_8X16_FONT},
        {"Option5",  16, IPS200_8X16_FONT},
        {"Option6", 16, IPS200_8X16_FONT},
        {"Option7", 16, IPS200_8X16_FONT},
        {"Option8", 16, IPS200_8X16_FONT},
};

const setting_LIST Option1[]={
//        {"This is option 1", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option2[]={
        {"base pwm:", 16, IPS200_8X16_FONT},
        {"Brushless speed:", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option3[]={
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option4[]={
        {"Image Kp:", 16, IPS200_8X16_FONT},
        {"Image Ki:", 16, IPS200_8X16_FONT},
        {"Image Kd:", 16, IPS200_8X16_FONT},
        {"Velocity Kp:", 16, IPS200_8X16_FONT},
        {"Velocity Ki:", 16, IPS200_8X16_FONT},
        {"Velocity Kd:", 16, IPS200_8X16_FONT},
        {"Rate Kp:", 16, IPS200_8X16_FONT},
        {"Rate Ki:", 16, IPS200_8X16_FONT},
        {"Rate Kd:", 16, IPS200_8X16_FONT},
        {"Angle Kp:", 16, IPS200_8X16_FONT},
        {"Angle Ki:", 16, IPS200_8X16_FONT},
        {"Angle Kd:", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option5[]={
        {"This is option 5", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option6[]={
        {"This is option 6", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option7[]={
        {"This is option 7", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};

const setting_LIST Option8[]={
        {"This is option 8", 16, IPS200_8X16_FONT},
        {"<BACK>", 16, IPS200_8X16_FONT},
};
char ui_index = 0;//当前UI的选项
int8_t ui_select = 0;//当前选择了哪一个选项
char ui_state = 0;//UI状态机
static char interval = 3; //全局设置，每行菜单之间的间隔

void UI_Init(void)
{
    ips200_init(IPS200_TYPE_PARALLEL8);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_full(RGB565_WHITE);
    ips200_show_string(55, 0, "--=<<Welcome>>=--");

}
/*
 * 主菜单显示与处理部分
 */

void home_proc(KEY_MSG* msg)
{
    uint8_t screen_update_flag = 0;
    if(msg->updated_flag != 0)
    {
        msg->updated_flag = 0;
        switch(msg->key_id)
        {
        case 1:
            if(ui_select + 1 < sizeof(homeList) / sizeof(setting_LIST)) ui_select++;
            screen_update_flag = 1;
            break;
        case -1:
            if(ui_select > 0) ui_select--;
            screen_update_flag = 1;
            break;
        case 2:
            switch(ui_select)
            {
            case 0: ui_index = E_LIST1; break;
            case 1: ui_index = E_LIST2; break;
            case 2: ui_index = E_LIST3; break;
            case 3: ui_index = E_LIST4; break;
            case 4: ui_index = E_LIST5; break;
            case 5: ui_index = E_LIST6; break;
            case 6: ui_index = E_LIST7; break;
            case 7: ui_index = E_LIST8; break;
            }
            ui_select = 0;
            ips200_clear();
        }
    }
    home_show(screen_update_flag);
    if(ui_index != E_HOME)ips200_clear();
}

static void home_show(uint8_t update_flag)
{
    ips200_show_string(55, 0, "--=<<Welcome>>=--");
    ips200_show_float(200, 0, voltage, 2, 1);
    if(update_flag == 1) ips200_clear();
    uint8_t list_len = sizeof(homeList) / sizeof(setting_LIST); //列表最大长度
    for(uint8_t i = 0 ; i < list_len; i++)
    {
        ips200_show_string(20, 20 + i * (homeList[i].height + interval), homeList[i].str);
    }
    ips200_show_char(0, 20 + ui_select * (homeList[0].height + interval), '>');
}

/*
 * 子菜单1显示与处理部分
 */
void list1_proc(KEY_MSG* msg)
{
    uint8_t screen_update_flag = 0;
    if(msg->updated_flag != 0)
    {
        msg->updated_flag = 0;
        switch(msg->key_id)
        {
        case 1:
            if(ui_select + 1 < sizeof(Option1) / sizeof(setting_LIST)) ui_select++;
            screen_update_flag = 1;
            break;
        case -1:
            if(ui_select > 0) ui_select--;
            screen_update_flag = 1;
            break;
        case 2:
            switch(ui_select)
            {
            case 0:ui_index = E_HOME; break;
            case 1:break;
            case 2:break;
            case 3:break;
            case 4:break;
            case 5:break;
            case 6:break;
            case 7:break;
            }
            ips200_clear();
            ui_select = 0;
        }
    }
    list1_show(screen_update_flag);
    if(ui_index != E_LIST1)ips200_clear();
}

static void list1_show(uint8_t update_flag)
{
    ips200_show_string(55, 0, "--=<<Sensor>>=--");
    if(update_flag == 1) ips200_clear();
    uint8_t list_len = sizeof(Option1) / sizeof(setting_LIST); //列表最大长度
    for(uint8_t i = 0 ; i < list_len; i++)
    {
        ips200_show_string(20, 20 + i * (Option1[i].height + interval), Option1[i].str);
    }
    ips200_show_char(0, 20 + ui_select * (Option1[0].height + interval), '>');

//    char roll_data[30] = {0};
//    char pitch_data[30] = {0};
//    char yaw_data[30] = {0};
    char gyro_data_x[30] = {0};
    char gyro_data_y[30] = {0};
    char gyro_data_z[30] = {0};
    char speed_data[30] = {0};
    char distance_data[30] = {0};

//    sprintf(yaw_data,   "Yaw = %.2f",   imu.yaw);
//    sprintf(pitch_data, "Pitch = %.2f", imu.pit);
//    sprintf(roll_data,  "Roll = %.2f",  imu.rol);

    sprintf(gyro_data_x, "gyro_x = %.3f", imu.g[imu_x]);
    sprintf(gyro_data_y, "gyro_y = %.3f", imu.g[imu_y]);
    sprintf(gyro_data_z, "gyro_z = %.3f", imu.g[imu_z]);

    sprintf(speed_data, "speed m/s = %.2f", wheel_speed);
    sprintf(distance_data, "distance m = %.3f", distance);

//    ips200_show_string(0, 50, yaw_data);
//    ips200_show_string(0, 70, pitch_data);
//    ips200_show_string(0, 90, roll_data);

    ips200_show_string(0, 110, gyro_data_x);
    ips200_show_string(0, 130, gyro_data_y);
    ips200_show_string(0, 150, gyro_data_z);

    ips200_show_string(0, 170, speed_data);
    ips200_show_string(0, 190, distance_data);
}

/*
 * 子菜单2显示与处理部分
 */
void list2_proc(KEY_MSG* msg)
{
    static uint8_t is_speed_control = 0;
    static uint8_t brush_control = 0;
    static uint8_t burshless_control = 0;
    uint8_t screen_update_flag = 0;
    if(msg->updated_flag != 0)
    {
        msg->updated_flag = 0;
        if(!is_speed_control)
        {
            switch(msg->key_id)
            {
            case 1:
                if(ui_select + 1 < sizeof(Option2) / sizeof(setting_LIST)) ui_select++;
                screen_update_flag = 1;
                break;
            case -1:
                if(ui_select > 0) ui_select--;
                screen_update_flag = 1;
                break;
            case 2:
                switch(ui_select)
                {
                    case 0:is_speed_control = 1; brush_control = 1; break;
                    case 1:is_speed_control = 1; burshless_control = 1; break;
                    case 2:ui_index = E_HOME; ui_select = 0;break;
                    case 3:break;
                    case 4:break;
                    case 5:break;
                    case 6:break;
                    case 7:break;
                }
                ips200_clear();
//                ui_select = 0;
            }
        }
        else {
            switch(msg->key_id)
            {
            case FORDWARD_ROTATION:
                if(brush_control)
                {
                    base_pwm += 50;
                    if(base_pwm>=5000)base_pwm=5000;

                }
                if(burshless_control)
                {
                    brushless_duty += 10;

                }
                break;
            case REVERSE_ROTATION:
                if(brush_control)
                {
                    base_pwm -= 50;
                    if(base_pwm<=-5000)base_pwm=-5000;
                }
                if(burshless_control)
                {
                    brushless_duty -= 10;
                }

                break;
            case 2:
                is_speed_control = 0;
                brush_control = 0;
                burshless_control = 0;
                break;

            }
        }
    }
    list2_show(screen_update_flag);
    if(ui_index != E_LIST2)ips200_clear();
}

static void list2_show(uint8_t update_flag)
{
    ips200_show_string(55, 0, "--=<<Motor>>=--");
    if(update_flag == 1) ips200_clear();
    uint8_t list_len = sizeof(Option2) / sizeof(setting_LIST); //列表最大长度
    for(uint8_t i = 0 ; i < list_len; i++)
    {
        ips200_show_string(20, 20 + i * (Option2[i].height + interval), Option2[i].str);
    }
    ips200_show_char(0, 20 + ui_select * (Option2[0].height + interval), '>');

    char bursh_sp[10];
    char burshless_sp[10];
    sprintf(bursh_sp, "%d    ", base_pwm);
    sprintf(burshless_sp, "%d    ", brushless_duty);
    ips200_show_string(130, 19, bursh_sp);
    ips200_show_string(180, 39, burshless_sp);
}
/*
 * 子菜单3显示与处理部分
 */
void list3_proc(KEY_MSG* msg)
{
    uint8_t screen_update_flag = 0;
    if(msg->updated_flag != 0)
    {
        msg->updated_flag = 0;
        switch(msg->key_id)
        {
        case 1:
            if(ui_select + 1 < sizeof(Option1) / sizeof(setting_LIST)) ui_select++;
            screen_update_flag = 1;
            break;
        case -1:
            if(ui_select > 0) ui_select--;
            screen_update_flag = 1;
            break;
        case 2:
            switch(ui_select)
            {
            case 0:ui_index = E_HOME; break;
            case 1:break;
            case 2:break;
            case 3:break;
            case 4:break;
            case 5:break;
            case 6:break;
            case 7:break;
            }
//            ips200_clear();
            ui_select = 0;
        }
    }
    list3_show(screen_update_flag);
    if(ui_index != E_LIST3)ips200_clear();
}

static void list3_show(uint8_t update_flag)
{
    ips200_show_string(55, 0, "--=<<Camera>>=--");
    if(update_flag == 1) ips200_clear();
    uint8_t list_len = sizeof(Option1) / sizeof(setting_LIST); //列表最大长度
    for(uint8_t i = 0 ; i < list_len; i++)
    {
        ips200_show_string(20, 20 + i * (Option1[i].height + interval), Option1[i].str);
    }
    ips200_show_char(0, 20 + ui_select * (Option1[0].height + interval), '>');

    //image_show();

    ips200_show_binary_image(0,40,(const uint8 *)binary_image,IMAGE_WIDTH,IMAGE_HEIGHT,80,60);

//    ips200_show_gray_image(0, 40, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    char str1[30];
    char str2[30];
    char str3[30];
    char str4[30];
    char str5[30];
    char str6[30];
    char str7[30];
    char str8[30];
    char str9[30];

    switch(get_road_type())
    {
        case BEND:
            sprintf(str4, "road:");sprintf(str5, "BEND         ");
            break;
        case LONG_STRAIGHT:
            sprintf(str4, "road:");sprintf(str5, "LONG_STRAIGHT");
            break;
        case CIRCLE_FORECAST:
            sprintf(str4, "road:");sprintf(str5, "CIRCL_FORCST");
            break;
        case CIRCLE_READY_ENTRY:
            sprintf(str4, "road:");sprintf(str5, "CIRCL_RDY_ET");
            break;
        case CIRCLE_ENTRY:
            sprintf(str4, "road:");sprintf(str5, "CIRCL_ENTRY ");
            break;
        case CIRCLE_IN:
            sprintf(str4, "road: ");sprintf(str5, "CIRCLE_IN  ");
            break;
        case CIRCLE_READY_EXIT:
            sprintf(str4, "road:");sprintf(str5, "CIRCLE_R_EXT");
            break;
        case CIRCLE_EXIT:
            sprintf(str4, "road:");sprintf(str5, "CIRCLE_EXIT ");
            break;
        case BLOCK_IN:
            sprintf(str4, "road: ");sprintf(str5, "BLOCK_IN    ");
            break;
        case FINISH:
            sprintf(str4, "road: ");sprintf(str5, "FINISH      ");
            break;
        case FINISH_STOP:
            sprintf(str4, "road: ");sprintf(str5, "FINISH_STOP ");
            break;
        default:
            break;
    }

    switch(ele_lock)
    {
        case _BEND:
            sprintf(str6, "lock:");sprintf(str7, "_BEND ");
            break;
        case _OUT_BOUND:
            sprintf(str6, "lock:");sprintf(str7, "_OUT_BOUND");
            break;
        case _CROSS:
            sprintf(str6, "lock:");sprintf(str7, "_CROSS ");
            break;
        case _CIRCLE:
            sprintf(str6, "lock:");sprintf(str7, "_CIRCLE ");
            break;
        case _BLOCK:
            sprintf(str6, "lock:");sprintf(str7, "_BLOCK ");
            break;
        case _FINISH:
            sprintf(str6, "lock:");sprintf(str7, "_FINISH ");
            break;
        default:
            break;
    }

    sprintf(str1,"Middle Line:%.2f     ",get_image_middle_line());
//    sprintf(str8,"Left Motor PWM:%d    ",left_duty);
//    sprintf(str9,"Right Motor PWM:%d   ",right_duty);
    sprintf(str2,"break1:%d    ",break_point[0]);
    sprintf(str3,"break2:%d    ",break_point[1]);
//    sprintf(str2,"leftspwm:%.2f     ",get_curvity());
//    sprintf(str3,"rightspwm:%d    ",rightside_duty);
    sprintf(str8,"left lose:%d         ",left_lose);
    sprintf(str9,"right lose:%d        ",right_lose);

    ips200_show_string(0, 180, str1);
    ips200_show_string(0, 200, str2);
    ips200_show_string(0, 220, str3);
    ips200_show_string(0, 240, str4);ips200_show_string(40, 240, str5);
    ips200_show_string(0, 260, str6);ips200_show_string(40, 260, str7);
    ips200_show_string(0, 280, str8);
    ips200_show_string(0, 300, str9);


}

/*
 * 子菜单4-PID显示与处理部分
 */
void list4_proc(KEY_MSG* msg)
{
    static uint8 is_pid_control = 0;
    static uint8 pid_flag = 0;
    uint8_t screen_update_flag = 0;

    if(msg->updated_flag != 0)
    {
        msg->updated_flag = 0;
        if(!is_pid_control)
        {
            switch(msg->key_id)
            {
            case FORDWARD_ROTATION:
                if(ui_select + 1 < sizeof(Option4) / sizeof(setting_LIST)) ui_select++;
                screen_update_flag = 1;
                break;
            case REVERSE_ROTATION:
                if(ui_select > 0) ui_select--;
                screen_update_flag = 1;
                break;
            case 2:
                switch(ui_select)
                {
                    case 0:is_pid_control = 1; pid_flag = 1; break;
                    case 1:is_pid_control = 1; pid_flag = 2; break;
                    case 2:is_pid_control = 1; pid_flag = 3; break;
                    case 3:is_pid_control = 1; pid_flag = 4; break;
                    case 4:is_pid_control = 1; pid_flag = 5; break;
                    case 5:is_pid_control = 1; pid_flag = 6; break;
                    case 6:is_pid_control = 1; pid_flag = 7; break;
                    case 7:is_pid_control = 1; pid_flag = 8; break;
                    case 8:is_pid_control = 1; pid_flag = 9; break;
                    case 9:is_pid_control = 1; pid_flag = 10; break;
                    case 10:is_pid_control = 1; pid_flag = 11; break;
                    case 11:is_pid_control = 1; pid_flag = 12; break;
                    case 12:flash_save_data(&fd);ui_index = E_HOME; ui_select = 0; break;
                }
                ips200_clear();
//                ui_select = 0;
            }
        }
        else {
            switch(msg->key_id)
            {
            case FORDWARD_ROTATION:
                switch (pid_flag)
                {
                    case 1: IMAGE_KP+=0.1;      if(IMAGE_KP >= PID_MAX) IMAGE_KP = PID_MAX;         break;
                    case 2: IMAGE_KI+=0.01;     if(IMAGE_KI >= PID_MAX) IMAGE_KI = PID_MAX;         break;
                    case 3: IMAGE_KD+=0.1;      if(IMAGE_KD >= PID_MAX) IMAGE_KD = PID_MAX;         break;
                    case 4: VELOCITY_KP+=0.1;   if(VELOCITY_KP >= PID_MAX) VELOCITY_KP = PID_MAX;   break;
                    case 5: VELOCITY_KI+=0.01;  if(VELOCITY_KI >= PID_MAX) VELOCITY_KI = PID_MAX;   break;
                    case 6: VELOCITY_KD+=0.1;   if(VELOCITY_KD >= PID_MAX) VELOCITY_KD = PID_MAX;   break;
                    case 7: RATE_KP+=0.1;       if(RATE_KP >= PID_MAX) RATE_KP = PID_MAX;           break;
                    case 8: RATE_KI+=0.01;      if(RATE_KI >= PID_MAX) RATE_KI = PID_MAX;           break;
                    case 9: RATE_KD+=0.1;       if(RATE_KD >= PID_MAX) RATE_KD = PID_MAX;           break;
                    case 10: ANGLE_KP+=0.1;      if(ANGLE_KP >= PID_MAX) ANGLE_KP = PID_MAX;        break;
                    case 11: ANGLE_KI+=0.01;     if(ANGLE_KI >= PID_MAX) ANGLE_KP = PID_MAX;        break;
                    case 12: ANGLE_KD+=0.1;      if(ANGLE_KD >= PID_MAX) ANGLE_KP = PID_MAX;        break;
                    default: break;
                }
                break;
            case REVERSE_ROTATION:
                switch (pid_flag)
                {
                    case 1: IMAGE_KP-=0.1;      if(IMAGE_KP <= PID_MIN) IMAGE_KP = PID_MIN;         break;
                    case 2: IMAGE_KI-=0.01;     if(IMAGE_KI <= PID_MIN) IMAGE_KP = PID_MIN;         break;
                    case 3: IMAGE_KD-=0.1;      if(IMAGE_KD <= PID_MIN) IMAGE_KP = PID_MIN;         break;
                    case 4: VELOCITY_KP-=0.1;   if(VELOCITY_KP <= PID_MIN) VELOCITY_KP = PID_MIN;   break;
                    case 5: VELOCITY_KI-=0.01;  if(VELOCITY_KI <= PID_MIN) VELOCITY_KI = PID_MIN;   break;
                    case 6: VELOCITY_KD-=0.1;   if(VELOCITY_KD <= PID_MIN) VELOCITY_KD = PID_MIN;   break;
                    case 7: RATE_KP-=0.1;       if(RATE_KP <= PID_MIN) RATE_KP = PID_MIN;           break;
                    case 8: RATE_KI-=0.01;      if(RATE_KI <= PID_MIN) RATE_KI = PID_MIN;           break;
                    case 9: RATE_KD-=0.1;       if(RATE_KD <= PID_MIN) RATE_KD = PID_MIN;           break;
                    case 10: ANGLE_KP-=0.1;      if(ANGLE_KP <= PID_MIN) ANGLE_KP = PID_MIN;        break;
                    case 11: ANGLE_KI-=0.01;     if(ANGLE_KI <= PID_MIN) ANGLE_KP = PID_MIN;        break;
                    case 12: ANGLE_KD-=0.1;      if(ANGLE_KD <= PID_MIN) ANGLE_KP = PID_MIN;        break;
                    default: break;
                }
                break;
            case 2:
                is_pid_control = 0;
                pid_flag = 0;
                break;

            }
        }
    }
    list4_show(screen_update_flag);
    if(ui_index != E_LIST4)ips200_clear();
}

static void list4_show(uint8_t update_flag)
{
    ips200_show_string(55, 0, "--=<<PID>>=--");
    if(update_flag == 1) ips200_clear();
    uint8_t list_len = sizeof(Option4) / sizeof(setting_LIST); //列表最大长度
    for(uint8_t i = 0 ; i < list_len; i++)
    {
        ips200_show_string(20, 20 + i * (Option4[i].height + interval), Option4[i].str);
    }
    ips200_show_char(0, 20 + ui_select * (Option4[0].height + interval), '>');

    ips200_show_float(180, 20, IMAGE_KP, 2, 1);
    ips200_show_float(180, 39, IMAGE_KI, 2, 2);
    ips200_show_float(180, 55, IMAGE_KD, 2, 1);

    ips200_show_float(180, 75, VELOCITY_KP, 2, 1);
    ips200_show_float(180, 90, VELOCITY_KI, 2, 2);
    ips200_show_float(180, 110, VELOCITY_KD, 2, 1);

    ips200_show_float(180, 130, RATE_KP, 2, 1);
    ips200_show_float(180, 150, RATE_KI, 2, 2);
    ips200_show_float(180, 170, RATE_KD, 2, 1);

    ips200_show_float(180, 190, ANGLE_KP, 2, 1);
    ips200_show_float(180, 210, ANGLE_KI, 2, 2);
    ips200_show_float(180, 230, ANGLE_KD, 2, 1);

}

/*
 * 子菜单5显示与处理部分
 */
void list5_proc(KEY_MSG* msg)
{

}

/*
 * 子菜单6显示与处理部分
 */
void list6_proc(KEY_MSG* msg)
{

}

/*
 * 子菜单7显示与处理部分
 */
void list7_proc(KEY_MSG* msg)
{

}

/*
 * 子菜单8显示与处理部分
 */
void list8_proc(KEY_MSG* msg)
{

}

