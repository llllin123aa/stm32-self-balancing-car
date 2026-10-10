#include "stm32f10x.h"                  // Device header
#include "Key.h"

#define KEY_PRESSED				1
#define KEY_UNPRESSED			0

#define KEY_TIME_DOUBLE			200//0为关闭
#define KEY_TIME_LONG			1000
#define KEY_TIME_REPEAT			100

uint8_t Key_Flag[KEY_COUNT];

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
@brief 读某一个按键此刻“是按着还是没按”，具体做法是，给它一个n(KEY1~KEY4)，
它去读对应引脚的电平，然后返回结果
@param n=0对应的是PB1的按键，n=1对应PB0，n=2对应PA5，n=3对应PA4
*/

uint8_t Key_GetState(uint8_t n)
{
	if (n == KEY_1)
	{
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)//PB1
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_2)
	{
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)//PB0
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_3)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)//PA5
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_4)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)//PA4
		{
			return KEY_PRESSED;
		}
	}
	return KEY_UNPRESSED;
}

uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	if (Key_Flag[n] & Flag)
	{
		if (Flag != KEY_HOLD)
		{
			Key_Flag[n] &= ~Flag;
		}
		return 1;
	}
	return 0;
}

void Key_Clear(void)
{
	uint8_t i;
	for (i = 0; i < KEY_COUNT; i ++)
	{
		Key_Flag[i] = 0;
	}
}
/**
	@brief 这个函数负责点亮小灯
*/
void Key_Tick(void)
{
	static uint8_t Count, i;
	static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
	static uint8_t S[KEY_COUNT];
	static uint16_t Time[KEY_COUNT];
	//函数一进来，先让所有的按键倒计时减1。因为Key_Tick()每1ms调用一次
	//所以Time[i]的单位就是毫秒。KEY_TIME_LONG = 1000就代表1000ms=1s
	for (i = 0; i < KEY_COUNT; i ++)
	{
		if (Time[i] > 0)
		{
			Time[i] --;
		}
	}
	//每20次才做一次的是————消抖采样
	Count ++;
	if (Count >= 20)
	{
		Count = 0;
		
		for (i = 0; i < KEY_COUNT; i ++)
		{
			PrevState[i] = CurrState[i];
			CurrState[i] = Key_GetState(i);
			
			if (CurrState[i] == KEY_PRESSED)//按键按下保持
			{
				Key_Flag[i] |= KEY_HOLD;//按键状态HOLD位置1
			}
			else
			{
				Key_Flag[i] &= ~KEY_HOLD;
			}
			
			if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
			{
				Key_Flag[i] |= KEY_DOWN;//按键按下
			}
			
			if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_UP;//按键弹起
			}
			//状态机，S=0为待命，检测按键是否按下，当按键按下,启动定时，状态变成S1
			if (S[i] == 0)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Time[i] = KEY_TIME_LONG;//启动1s定时
					S[i] = 1;
				}
			}
			else if (S[i] == 1)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					Time[i] = KEY_TIME_DOUBLE;//第一下在1s内松手了，检测是否有第二下，跳转到S2
					S[i] = 2;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;//第一下满1s还没松开
					Key_Flag[i] |= KEY_LONG;//判定为长按
					S[i] = 4;//判断长按的时间，实现是否为连发
				}
			}
			else if (S[i] == 2)
			{
				if (CurrState[i] == KEY_PRESSED)//按下第二次
				{
					Key_Flag[i] |= KEY_DOUBLE;//判定为双击
					S[i] = 3;//进入S3
				}
				else if (Time[i] == 0)//否则判定为单击
				{
					Key_Flag[i] |= KEY_SINGLE;
					S[i] = 0;
				}
			}
			else if (S[i] == 3)
			{
				if (CurrState[i] == KEY_UNPRESSED)//等待按键松开，松开后回到S0
				{
					S[i] = 0;
				}
			}
			else if (S[i] == 4)//长按之后的连发
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
				else if (Time[i] == 0)//100ms时间到达，且还处于按下的状态
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_REPEAT;
					S[i] = 4;
				}
			}
		}
	}
}
