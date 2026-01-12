/*
 * FreeRTOS_task.c
 *
 *  Created on: 2024/1/24
 *      Author: Misaka
 */
#include "FreeRTOS_task.h"

TaskHandle_t    start_task_handler; //开始任务

TaskHandle_t    led_flicker_handler;//LED闪烁任务

TaskHandle_t    key_handler;//ec11扫描任务

TaskHandle_t    gui_render_task_handler;//GUI渲染任务

TaskHandle_t    sensor_task_handler;//传感器数据采集任务

float dat[4];
void FreeRTOS_Start(void)
{

    xTaskCreate(start_task, "start_task", START_TASK_STACK_SIZE, NULL, START_TASK_PRIO, &start_task_handler);
    vTaskStartScheduler();
}

void start_task( void * pvParameters )
{
    taskENTER_CRITICAL();
    xTaskCreate(Led_Flicker_Task, "led_flicker", LED_FLICKER_STACK_SIZE, NULL, LED_FLICKER_PRIO, &led_flicker_handler);
    xTaskCreate(Key_Task, "key_detect", KEY_STACK_SIZE, NULL, KEY_PRIO, &key_handler);
    xTaskCreate(Gui_Render_Task, "gui_render", GUI_RENDER_STACK_SIZE, NULL, GUI_RENDER_PRIO, &gui_render_task_handler);
    //xTaskCreate(Buzzer_Task, "buzzer", BUZZER_TASK_STACK_SIZE, NULL, BUZZER_TASK_PRIO, &buzzer_handler);
    xTaskCreate(Sensor_Task, "sensor_10ms", Sensor_STACK_SIZE, NULL, Sensor_PRIO, &sensor_task_handler);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

void Led_Flicker_Task( void * pvParameters )
{
//    BaseType_t val;
    while(1)
    {
//        gpio_toggle_level(B0);
//        Buzzer_Alarm(1, 200);
//        printf("/************任务堆栈历史剩余最小值****************/\r\n");
//        printf("任务名称\t优先级\t堆栈剩余\r\n");
//        val = uxTaskGetStackHighWaterMark(start_task_handler);
//        printf("start:          %d      %d\r\n",START_TASK_PRIO,(int)val);
//        val = uxTaskGetStackHighWaterMark(gui_render_task_handler);
//        printf("gui:            %d      %d\r\n",GUI_RENDER_PRIO,(int)val);
//        val = uxTaskGetStackHighWaterMark(sensor_task_handler);
//        printf("sensor:         %d      %d\r\n",Sensor_PRIO,(int)val);
//        val = uxTaskGetStackHighWaterMark(key_handler);
//        printf("key_detect:     %d      %d\r\n",KEY_PRIO,(int)val);
//        val = uxTaskGetStackHighWaterMark(led_flicker_handler);
//        printf("led_flicker:    %d      %d\r\n",LED_FLICKER_PRIO,(int)val);

        taskENTER_CRITICAL();
        if(mt9v03x_finish_flag)
        {
            analyze_image();
            mt9v03x_finish_flag = 0;
        }
        taskEXIT_CRITICAL();
        vTaskDelay(1);
    }
}

void Gui_Render_Task( void * pvParameters )
{
    while(1)
    {
        for (uint8_t i = 0 ; i < E_UI_MAX ; i++)
        {
            if (ui_index == ui_list[i].index)//如果当前索引等于UI表中的索引
            {
                if (ui_list[i].cb)//执行UI对应的回调函数
                {
                    taskENTER_CRITICAL();
                    ui_list[i].cb(&key_msg);
                    taskEXIT_CRITICAL();
                }
            }
        }

//        float gyro[4];
//        gyro[0] = imu.g[imu_z];
//        gyro[1] = smartcar_status.goal_PWM;
//        dat[2] = smartcar_status.goal_angular_velocity;
//        dat[2] = get_image_middle_line();
//        JustFloat_Send(gyro, 4);

//        dat[0] = get_speed();
//        gyro[1] = smartcar_status.goal_PWM;
//        gyro[2] = smartcar_status.goal_angular_velocity;
//        gyro[3] = get_image_middle_line();
//        JustFloat_Send(dat, 2);

        vTaskDelay(30);

    }
}

void Sensor_Task( void * pvParameters )
{
    while(1)
    {
        update_speed_and_distance();
        update_voltage();

        analyze_road();
        decision();

        motor_control();
        side_motor_control();

        vTaskDelay(10);
    }
}

void Key_Task( void * pvParameters )
{
    while(1)
    {
        EC11_Scan();
        imu_updata();
//        if(mt9v03x_finish_flag)
//        {
//            analyze_image();
//            mt9v03x_finish_flag = 0;
//        }
        smartcar_status.goal_PWM = PWM_decision(smartcar_status.goal_angular_velocity);

        vTaskDelay(1);
    }
}



