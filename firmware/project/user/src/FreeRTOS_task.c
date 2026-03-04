/*
 * FreeRTOS_task.c
 *
 *  Created on: 2024/1/24
 *      Author: Misaka
 */
#include "FreeRTOS_task.h"

/* ─────────────────────────── 任务句柄 ─────────────────────────── */
TaskHandle_t control_task_handle;
TaskHandle_t perception_task_handle;
TaskHandle_t system_task_handle;
TaskHandle_t ui_task_handle;

float dat[4];

/*
 *  Control_Task — 1ms 硬实时控制环
 *  触发方式：TIM2 1kHz 任务通知
 *  软分频：
 *    tick_count % 1  == 0   → 1ms  姿态/PWM/电机
 *    tick_count % 10 == 0   → 10ms 图像分析-角度环
 *    tick_count % 100 == 0  → 100ms 速度环
 */
static void Control_Task(void *pvParameters)
{
    uint32_t ulNotifiedValue;
    uint32_t tick_count = 0;

// Control_Task 通知位
#define NOTIFY_CONTROL_TICK     (1UL << 0)

    for (;;)
    {
        /* 阻塞等待 TIM6 通知，无超时 */
        xTaskNotifyWait(0x00,                    /* 进入时不清除位 */
                        NOTIFY_CONTROL_TICK,     /* 退出时清除位   */
                        &ulNotifiedValue,
                        portMAX_DELAY);

        if (ulNotifiedValue & NOTIFY_CONTROL_TICK)
        {
            tick_count++;

            /* ── 1ms 环 ── */
            imu_update();           /* 读取 ICM20602 原始数据并融合  */
            EC11_Scan();            /* 扫描旋转编码器，更新菜单状态  */

            smartcar_status.goal_PWM = PWM_decision(smartcar_status.goal_angular_velocity);         /* 根据姿态计算 PWM 输出量  */
            motor_control();        /* 主驱动电机 PWM 输出  */


            /* ── 10ms 环 ── */
            if (tick_count % 10 == 0)
            {
                /* 图像分析，PID 角度闭环  */
                side_motor_control();   /* 横向辅助电机 PWM 输出 */
            }

            /* ── 100ms 环 ── */
            if (tick_count % 100 == 0)
            {
                update_speed_and_distance();       /* 读编码器，PID 速度闭环  */
            }

            if (tick_count >= 1000)
            {
                tick_count = 0; // 防止溢出
            }
        }
    }
}

/*
 *  Perception_Task — 图像处理与决策
 *  触发方式：摄像头场中断-任务通知
 */
static void Perception_Task(void *pvParameters)
{
    uint32_t ulNotifiedValue;

// Perception_Task 通知位
#define NOTIFY_PERCEPTION_FRAME (1UL << 0)
    for (;;)
    {
        /* 阻塞等待摄像头帧通知 */
        xTaskNotifyWait(0x00,
                        NOTIFY_PERCEPTION_FRAME,
                        &ulNotifiedValue,
                        portMAX_DELAY);

        if (ulNotifiedValue & NOTIFY_PERCEPTION_FRAME)
        {
            analyze_image();    /* 图像预处理（二值化、滤波、中线计算等）   */
            analyze_road();     /* 赛道元素识别   */
            decision();         /* 路径决策，输出转向量  */
        }
    }
}

/*
 *  System_Task — 系统状态监测
 *  触发方式：vTaskDelayUntil 100ms 周期
 */
static void System_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        update_voltage();   /* ADC 采样，更新 system_status */
    }
}

/*
 *  UI_Task — 人机界面 & 编码器扫描
  *     触发方式：vTaskDelayUntil 40ms 周期
 */
static void UI_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(40);

//    BaseType_t val;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);


//        GUI_Update();   /* 刷新 IPS200 显示内容  */
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

//        printf("/************任务堆栈历史剩余最小值****************/\r\n");
//        printf("任务名称\t优先级\t堆栈剩余\r\n");
//        val = uxTaskGetStackHighWaterMark(control_task_handle);
//        printf("Control:        %d      %d\r\n", PRIORITY_CONTROL, (int)val);
//        val = uxTaskGetStackHighWaterMark(perception_task_handle);
//        printf("Perception:     %d      %d\r\n", PRIORITY_PERCEPTION, (int)val);
//        val = uxTaskGetStackHighWaterMark(system_task_handle);
//        printf("System:         %d      %d\r\n", PRIORITY_SYSTEM, (int)val);
//        val = uxTaskGetStackHighWaterMark(ui_task_handle);
//        printf("UI:             %d      %d\r\n", PRIORITY_UI, (int)val);
    }
}

void FreeRTOS_Start(void)
{
    taskENTER_CRITICAL();
    xTaskCreate(Control_Task,
                "Control",
                STACK_CONTROL,
                NULL,
                PRIORITY_CONTROL,
                &control_task_handle);

    xTaskCreate(Perception_Task,
                "Perception",
                STACK_PERCEPTION,
                NULL,
                PRIORITY_PERCEPTION,
                &perception_task_handle);

    xTaskCreate(System_Task,
                "System",
                STACK_SYSTEM,
                NULL,
                PRIORITY_SYSTEM,
                &system_task_handle);

    xTaskCreate(UI_Task,
                "UI",
                STACK_UI,
                NULL,
                PRIORITY_UI,
                &ui_task_handle);

    /* 启动 TIM6 1ms 定时器中断  */
    pit_ms_init(TIM6_PIT, 1);
    interrupt_set_priority(TIM6_IRQn, 5);

    taskEXIT_CRITICAL();

    vTaskStartScheduler();
}

/*
 *  TIM6 1ms中断句柄
 */

void pit_handler (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

    if (control_task_handle != NULL)
    {
        xTaskNotifyFromISR(control_task_handle,
                           NOTIFY_CONTROL_TICK,
                           eSetBits,
                           &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 *  摄像头 DVP 场中断句柄
 */
void dvp_handler (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (perception_task_handle != NULL)
    {
        xTaskNotifyFromISR(perception_task_handle,
                           NOTIFY_PERCEPTION_FRAME,
                           eSetBits,
                           &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
