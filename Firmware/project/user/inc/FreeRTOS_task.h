/*
 * FreeRTOS_task.h
 *
 *  Created on: 2024/1/24
 *      Author: Misaka
 */

#ifndef FREERTOS_TASK_H_
#define FREERTOS_TASK_H_

#include "zf_common_headfile.h"

void FreeRTOS_Start(void);

/*
 *  Sensor_Task 任务 配置
 */
#define Sensor_PRIO            5
#define Sensor_STACK_SIZE      512
//extern TaskHandle_t     key_handler;

void Sensor_Task( void * pvParameters );

/*
 *  Key_Task 任务 配置
 */
#define KEY_PRIO            4
#define KEY_STACK_SIZE      2000
//extern TaskHandle_t     key_handler;

void Key_Task( void * pvParameters );

/*
 *  GUI_RENDER 任务 配置
 */
#define GUI_RENDER_PRIO             1
#define GUI_RENDER_STACK_SIZE       512
//extern TaskHandle_t     gui_render_task_handler;

void Gui_Render_Task( void * pvParameters );

/*
 *  LED_FLICKER 任务 配置
 */
#define LED_FLICKER_PRIO         6
#define LED_FLICKER_STACK_SIZE   2048
//extern TaskHandle_t     led_flicker_handler;

void Led_Flicker_Task( void * pvParameters );

/*
 *  START_TASK 任务 配置
 */
#define START_TASK_PRIO         0
#define START_TASK_STACK_SIZE   128

extern float dat[4];
void start_task( void * pvParameters );


#endif /* FREERTOS_TASK_H_ */
