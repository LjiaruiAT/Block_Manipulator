#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"
#include "PID_old.h"
#include "math.h"
#include "motor.h"
#include "motorEx.h"
#include "usart.h"
#include "semphr.h"
void Task_Init(void);
extern TaskHandle_t Block_Manipulator_Handle;
#endif

