#include "stm32f10x.h"                  // Device header
#include "PWM.h"
//PA0(TIN2-CH1)输出PWMA、PA1(TIM2-CH2)输出PWMB
//AIN1:PB12,AIN2:PB13;BIN1:PB14,BIN2:PB15
void motor_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//AIN1,AIN2控制方向引脚
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;//主动输出高低电平来控制电机转向
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	PWM_Init();
}

/**
  * 函    数：直流电机设置PWM
  * 参    数：n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * 参    数：PWM 要设置的PWM值，范围：-100~100（负数为反转）
  * 返 回 值：无
  */
void Motor_SetPWM(uint8_t n, int8_t PWM)
{
	if (n == 1)			//指定左电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12);	//PB12置高电平
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//PB13置低电平
			PWM_SetCompare1(PWM);				//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);	//PB12置低电平
			GPIO_SetBits(GPIOB, GPIO_Pin_13);	//PB13置高电平
			PWM_SetCompare1(-PWM);				//设置PWM占空比
		}
	}
	else if (n == 2)	//指定右电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_14);	//PB14置低电平
			GPIO_SetBits(GPIOB, GPIO_Pin_15);	//PB15置高电平
			PWM_SetCompare2(PWM);				//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_14);	//PB14置高电平
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);	//PB15置低电平
			PWM_SetCompare2(-PWM);				//设置PWM占空比
		}
	}
}