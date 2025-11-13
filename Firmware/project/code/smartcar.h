#ifndef _SMARTCAR_H
#define _SMARTCAR_H

#include "zf_common_headfile.h"

#define FLASH_SECTOR        ( 47 )
#define FLASH_PAGE          ( 0  )

typedef struct  SmartCar_Param                                      //智能车调整参数
{
    float velocity_kp;
    float velocity_ki;
    float velocity_kd;

    float image_kp;
    float image_ki;
    float image_kd;

    float rate_kp;
    float rate_ki;
    float rate_kd;

    float angle_kp;
    float angle_ki;
    float angle_kd;

    float K_changepwm;

    int circle_dir[2];
    float circle_exit_angle[2];
    int circle_num;

    float global_speed;
    int sw_value;

    int side_motor_flag;
    int flash_flag;

} SMARTCAR_PARAM;

typedef struct
{
    uint8 sector_num;
    uint8 * data_addr;
    uint32 data_size;
}flash_data;

extern SMARTCAR_PARAM smartcar_param;
extern flash_data fd;

void flash_load_data(flash_data *fd);
void flash_save_data(flash_data *fd);

int my_flash_init(void);

#endif
