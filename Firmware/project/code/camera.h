#ifndef _CAMERA_H_
#define _CAMERA_H_

extern TaskHandle_t    camera_task_handler;

/*
 *  CAMERA »ŒŒÒ ≈‰÷√
 */
#define CAMERA_PRIO         10
#define CAMERA_STACK_SIZE   512

void Camera_Task( void * pvParameters );







#endif
