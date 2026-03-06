/*
 * FreeRTOS_task.h
 *
 *  Created on: 2024/1/24
 *      Author: Misaka
 */

#ifndef FREERTOS_TASK_H_
#define FREERTOS_TASK_H_

#include "zf_common_headfile.h"

/* ─────────────────────────── 任务优先级 ─────────────────────────── */
#define PRIORITY_CONTROL        12
#define PRIORITY_PERCEPTION     10
#define PRIORITY_SYSTEM          3
#define PRIORITY_UI              2

/* ─────────────────────────── 任务栈深度 ─────────────────────────── */
#define STACK_CONTROL           512
#define STACK_PERCEPTION        2048
#define STACK_SYSTEM            512
#define STACK_UI                1024

typedef struct
{
    float image_error;    /* 图像横向偏差，供角度环使用 */
} PerceptionResult_t;

void FreeRTOS_Start(void);
void ConfigureTimeForRunTimeStats(void);
#endif /* FREERTOS_TASK_H_ */
