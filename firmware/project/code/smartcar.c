#include "smartcar.h"

flash_data fd_zero;
flash_data fd;

SMARTCAR_PARAM smartcar_param ;
SMARTCAR_PARAM param_zero = {
    .image_kp = 20.0,
    .image_ki = 0.05,
    .image_kd = 10.0,

    .velocity_kp = 6.0,
    .velocity_ki = 20.0,//0.02
    .velocity_kd = 0.0,

    .rate_kp = 20.0,
    .rate_ki = 0.0,
    .rate_kd = 15.0,

    .angle_kp = 7.0,
    .angle_ki = 0.0,
    .angle_kd = 6.0,

    .K_changepwm = 10.0,

    .circle_dir  = {1,1},//0-×ó£¬1-ÓÒ
    .circle_exit_angle = {38,-38},
    .circle_num = 1,

    .global_speed = 1.0,
    .sw_value = 0,

    .side_motor_flag = 0,
    .flash_flag = 0x5A,

};

void flash_init_data(flash_data *fd, uint8 num, void *addr, uint32 size)
{
    fd->sector_num = num;
    fd->data_addr = (void *) addr;
    fd->data_size = size;

    fd_zero.sector_num = num;
    fd_zero.data_addr = (uint8 *)&param_zero;
    fd_zero.data_size = size;

    system_delay_ms(5);

    flash_load_data(fd);
}

uint32 p[256];
void flash_load_data(flash_data *fd)
{

    flash_read_page(FLASH_SECTOR, FLASH_PAGE, p, sizeof(smartcar_param));

    memcpy(fd->data_addr, p, fd->data_size);
    printf("load data ok\n\r");
}

void flash_save_data(flash_data *fd)
{
    uint32 len = fd->data_size;
    if(len > 1024)
        return;

    memcpy(p, fd->data_addr, fd->data_size);

    flash_erase_sector(FLASH_SECTOR, FLASH_PAGE);
    flash_write_page(FLASH_SECTOR, FLASH_PAGE, p, sizeof(smartcar_param));
    flash_load_data(fd);

}

void flash_reset_data(flash_data *fd)
{

    flash_save_data(&fd_zero);
    flash_load_data(fd);
}

int my_flash_init(void)
{
    flash_init_data(&fd, FLASH_SECTOR, &smartcar_param, sizeof(smartcar_param));
    if( 0x5A != smartcar_param.flash_flag )
    {
        flash_reset_data(&fd);
    }

    return 0;
}
