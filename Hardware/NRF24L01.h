#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"

/* Compatible with the original remote: fixed 4-byte payload. */
extern uint8_t NRF24L01_RxPacket[4];

void NRF24L01_Init(void);
uint8_t NRF24L01_Receive(void);

#endif
