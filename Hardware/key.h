#ifndef __KEY_H
#include "stm32f10x.h"
#define __KEY_H

#define KEY_COUNT				4

#define KEY_1					0
#define KEY_2					1
#define KEY_3					2
#define KEY_4					3

#define KEY_HOLD				0x01
#define KEY_DOWN				0x02
#define KEY_UP					0x04
#define KEY_SINGLE				0x08
#define KEY_DOUBLE				0x10
#define KEY_LONG				0x20
#define KEY_REPEAT				0x40
//每个按键对应数组里的一个字节。这个字节的每一位代表一个事件
//按下，松开、单击、双击、长按、连发、保持等等
extern uint8_t Key_Flag[];

void Key_Init(void);
uint8_t Key_Check(uint8_t n, uint8_t Flag);
void Key_Clear(void);
void Key_Tick(void);

#endif
