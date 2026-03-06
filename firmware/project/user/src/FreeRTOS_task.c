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

static QueueHandle_t perception_queue = NULL;

// 指针数组，通过索引访问
uint8_t (*frame_buffers[2])[MT9V03X_H][MT9V03X_W] = {
    &mt9v03x_image0,
    &mt9v03x_image1
};

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
    PerceptionResult_t perception_result = {0.0f};  /* 保存最近一次有效结果 */

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

            motor_control(smartcar_status.inner_target);        /* 主驱动电机 PWM 输出  */


            /* ── 10ms 环 ── */
            if (tick_count % 10 == 0)
            {

                PerceptionResult_t tmp;
                if (xQueueReceive(perception_queue, &tmp, 0) == pdTRUE)
                {
                    perception_result = tmp;    /* 有新帧，更新 */
                }
                /* 无新帧时 perception_result 保持上次值，控制连续不中断 */

                smartcar_status.inner_target = OuterLoop_Update(perception_result.image_error);/* 图像分析，PID 角度闭环  */
                side_motor_control(perception_result.image_error);   /* 横向辅助电机 PWM 输出 */

            }

            /* ── 100ms 环 ── */
            if (tick_count % 100 == 0)
            {

                update_speed_and_distance();                   /* 读编码器(注意：速度计算和运行周期有关)，PID 速度闭环  */
                smartcar_status.base_PWM = SpeedLoop_Update(); // 更新 base_throttle
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
    uint32_t frame_idx;

    for (;;)
    {
        /* 阻塞等待摄像头帧通知 */
        xTaskNotifyWait(0x00,
                        0xFFFFFFFF,
                        &frame_idx,
                        portMAX_DELAY);


        image_analyze(frame_buffers[frame_idx]);    /* 图像处理  */
        analyze_road();     /* 赛道元素识别   */
        decision();         /* 路径决策，输出转向量  */

        /* ── 三步全部完成，打包结果写入队列 ── */
        PerceptionResult_t result;
        result.image_error = IMG_ERROR;

        /* xQueueOverwrite 专用于深度为1的队列，满时覆盖不阻塞 */
        xQueueOverwrite(perception_queue, &result);

//        // 在每次执行后检查堆栈剩余
//        high_water_mark = uxTaskGetStackHighWaterMark(NULL);
//
//        // 如果剩余太少，打印警告
//        if (high_water_mark < 512) {  // 小于512字节时警告
//            printf("Warning: Perception stack low! Remaining: %lu words\r\n",
//                   high_water_mark);
//        }
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

        for (uint8_t i = 0; i < E_UI_MAX; i++)
        {
            if (ui_index == ui_list[i].index && ui_list[i].cb)
            {
                ui_list[i].cb(&key_msg);
            }
        }
//        printf("Dynamic threshold: %u\r\n", IMG_DYN_THR);
//        printf("task \t prio \t stack \r\n");
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

    /* ── 创建感知结果队列，深度 1，元素大小为 PerceptionResult_t ── */
    perception_queue = xQueueCreate(1, sizeof(PerceptionResult_t));

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

// 空闲任务钩子函数
void vApplicationIdleHook(void) {
//    gpio_toggle_level(C4);
}

/*
 *  摄像头 DVP 场中断句柄
 */
void dvp_handler (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    gpio_toggle_level(C4);
    static uint32_t frame_cnt = 0;

    // 切换DMA缓冲区
    DVP->DMA_BUF0 = (uint32_t)(frame_cnt % 2 ? mt9v03x_image1 : mt9v03x_image0);

    if (perception_task_handle != NULL)
    {
        xTaskNotifyFromISR(perception_task_handle,
                           frame_cnt % 2,
                           eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
    }

    frame_cnt++;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
