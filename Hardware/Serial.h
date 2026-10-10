#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f10x.h"
void Serial_Init(void);
void Send_Byte(uint16_t Byte);
void Send_String(char *Str);
void Send_Num(uint32_t Num,uint16_t Length);
void Send_Array(uint32_t *Num,uint16_t Length);
void uart_printf(USART_TypeDef *USARTx,const char *fmt,...);
extern char Serial_RxData[20];//接收的数据
extern uint8_t USART1_ReceivedFlag;
uint8_t Serial_GetRxFlag(void);
#endif
