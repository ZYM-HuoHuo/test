#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H
#include "main.h"
void task_create(void);
void IMU_task(void *pvParameters);
void GIMBAL_task(void *pvParameters);
void DETCET_task(void *pvParameters);

#endif

