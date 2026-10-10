#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"
void motor_Init(void);
void Motor_SetPWM(uint8_t n, int8_t PWM);
#endif
