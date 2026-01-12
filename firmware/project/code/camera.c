#include "zf_common_headfile.h"

TaskHandle_t    camera_task_handler; //摄像头处理任务

void Camera_Task( void * pvParameters )
{
    while(1)
    {
//        if(mt9v03x_finish_flag)
//        {
//            analyze_image();
//            image_show();
//            mt9v03x_finish_flag = 0;
//        }
    }
}

