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
#define STACK_CONTROL           256
#define STACK_PERCEPTION        2048
#define STACK_SYSTEM            128
#define STACK_UI                512

/* ─────────────────────────── 任务通知位掩码 ─────────────────────── */
// Control_Task 通知位
#define NOTIFY_CONTROL_TICK     (1UL << 0)

// Perception_Task 通知位
#define NOTIFY_PERCEPTION_FRAME (1UL << 0)

extern float dat[4];

void FreeRTOS_Start(void);

#endif /* FREERTOS_TASK_H_ */
