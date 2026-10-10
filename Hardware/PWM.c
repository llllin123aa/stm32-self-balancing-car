#include "stm32f10x.h"
//PWM频率为20KHz
void PWM_Init(void)
{
	//初始化时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=100-1;//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=36-1;//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//输出捕获初始化
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//先填默认值
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//高电平有效，正常输出
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;//Freq = 72MHz/(ARR+1)/(PSC+1),Pulse = CCR/(ARR+1)
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	//开启定时器
	TIM_Cmd(TIM2,ENABLE);
}
/**
	@函数：设置CCR
	@参数： Compare为CCR，范围从0~100
*/
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM2,Compare);
}
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2,Compare);
}
/*原先ARR = 20000,PSC = 72,周期为20ms，Freq=50Hz
// speed: -100 ~ 100，0=停转，正数正转，负数反转
void Set_speed(int16_t speed)
{
    // speed=0   → CCR=1500 停转
    // speed=100 → CCR=2500 最快正转
    // speed=-100→ CCR=500  最快反转
    uint16_t ccr = 1500 + speed * 10;
    TIM_SetCompare2(TIM2, ccr);
}

CCR < 1500  → 反转，越小转速越快
CCR = 1500  → 停转
CCR > 1500  → 正转，越大转速越快

对应脉冲宽度：
1.5ms = 停转
<1.5ms = 反转
>1.5ms = 正转
周期为20ms，72MHz/72=1MHz,即1s计数1M次，即1微秒计数一次
0.5ms即500微秒，即计数500次
1.5ms即1500微秒，即计数1500次
2.5ms即2500微秒，即计数2500次
*/
