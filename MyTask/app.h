#ifndef _APP_H_
#define _APP_H_

#include "Task_Init.h"
extern TaskHandle_t Block_Manipulator_Handle;
extern Motor2006Ex_t manipulator[8];
extern float init_rad[8];
extern float climb_rad[8];
extern int16_t motorCurrentBuf_1[4];
extern int16_t motorCurrentBuf_2[4];
void Block_Manipulator(void *pvParameters);
void climb_down(void);
void climb_up(void);
void turn(void);
void turn_back(void);
#endif
