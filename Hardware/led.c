#include "stm32f10x.h"
#include "led.h"
static uint8_t led_status =0;//只在本文件中使用，加static
#define LED_port GPIOC
#define LED_Pin GPIO_Pin_13
void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_TypeStruct1;
	GPIO_TypeStruct1.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_TypeStruct1.GPIO_Pin = LED_Pin;
	GPIO_TypeStruct1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_port,&GPIO_TypeStruct1);
}
void LED_ON(void)
{
	led_status = 1;
	GPIO_ResetBits(LED_port,LED_Pin);//低电平点亮
} 
void LED_OFF(void)
{
	led_status = 0;
	GPIO_SetBits(LED_port,LED_Pin);//高电平熄灭
}
void LED_Turn(void)
{
	if(led_status == 1)
		LED_OFF();
	else LED_ON();
}
