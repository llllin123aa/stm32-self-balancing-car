#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H
#include "stm32f10x.h"
void BlueSerial_Init(void);
void BlueSerial_SendByte(uint16_t Byte);
void BlueSerial_SendString(char *Str);
void BlueSerial_SendNum(uint32_t Num,uint16_t Length);
void BlueSerial_SendArray(uint32_t *Num,uint16_t Length);
void BlueSerial_printf(USART_TypeDef *USARTx,const char *fmt,...);
extern char BlueSerial_RxData[20];//接收的数据
extern uint8_t USART2_ReceivedFlag;
uint8_t BlueSerial_GetRxFlag(void);
#endif
