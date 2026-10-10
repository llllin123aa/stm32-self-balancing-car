#include "stm32f10x.h"                  // Device header
#include "stdio.h"
#include "stdarg.h"//支持可变参数
char Serial_RxData[20]={0};//接收缓冲区的数据

uint8_t Received_index=0;//接收位置索引
uint8_t Serial_RxFlag = 0;
/*
功能：
1.发送字符
2.发送字符串
3.发送数字
4.发送长数字
5.发送数组
6.重定向，printf发送字符串和数字
7.可变参数，vsprintf发送字符串
7.中断接收数据
*/
void Serial_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//初始化GPIO,串口的TXD接在PA10(STM32的RX)，RXD接在了PA9(STM32的TX)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//输出引脚TX的配置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//串口初始化
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//不设置硬件流控
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;//同时初始化输入和输出引脚
	USART_InitStructure.USART_Parity=USART_Parity_No;//不设置校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//一位停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//8位数据位
	USART_Init(USART1,&USART_InitStructure);
	
	//开启中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开始接收非空中断
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	//开启串口
	USART_Cmd(USART1,ENABLE);
	
}

void Send_Byte(uint16_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);// 等待发送缓冲区空
}

void Send_String(char *Str)//字符串结束的标志是最后一位为空'\0'
{
	while(*Str != '\0')
	{
		Send_Byte(*Str);
		Str++;
	}
}
/**
	*@brief X的y次方
*/
uint32_t Num_Pow(uint16_t X,uint16_t y)
{
	uint32_t i=1;
	while(y--)
	{
		i*=X;
	}
	return i;
}
/**
	*@brief	发送长数字
	*@param Num:发送的数字，Length：发送数字的长度

*/
void Send_Num(uint32_t Num,uint16_t Length)
{
	while(Length)
	{
		Send_Byte(Num/Num_Pow(10,Length-1)%10 + '0');//Send_Byte期望发送的是ASCII字符(即数字字符'0'到'9')
		Length--;
	}
}
/**
	*@brief	发送数组
	*@param Num:数组名，即首地址，数组中存放的是十六进制数字
					Length：发送数组的长度
	*@condition 需要设置为Hex模式

*/
void Send_Array(uint32_t *Num,uint16_t Length)
{
	uint16_t i=0;
	for(i=0;i<Length;i++)
		Send_Byte(Num[i]);
}

/**
	*@brief	重定向，将输出目标从“屏幕”改成“串口”
	*@param ch:当前要发送的单个字符，printf每次传一个字符进来
					FILE *f:文件流指针，表示输出目标(stdout/串口等)，在stm32中用不到，忽略即可
	*@condition 需要设置为文本模式
*/
int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(uint16_t)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//发送缓冲区为空，发送成功
	return ch;
}
/**
	*@brief	使用可变参数发送字符和数字
	*@param USARTx:选择串口
		@arg USART1,USART2
				*fmt:格式化字符串，如"Num:%d"
				...:可变参数，数量不固定，对应fmt里面的%d %s %f等
	*@condition 需要设置为文本模式
*/
void uart_printf(USART_TypeDef *USARTx,const char *fmt,...)
{
	char buf[256];
	va_list agrs;//定义一个
	va_start(agrs,fmt);//让args指向fmt之后的第一个可变参数
	vsprintf(buf,fmt,agrs);//把fmt和可变参数格式化成字符串存入buf
	va_end(agrs);//清理args，结束可变参数的使用
	
	char *p = buf;
	while(*p)
	{
		USART_SendData(USARTx,(uint16_t)*p);
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
		p++;
		//也可以写成USART_SendData(USARTx,(uint16_t)*p++);
	}
}
/**
	@brief 判断串口接收标志位
	@param 无
*/
uint8_t Serial_GetRxFlag(void)
{
	if(Serial_RxFlag)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	else
		return 0;
			
}
/**
*@brief	中断服务函数，用于接收数据
	*@param USARTx:选择串口
		@arg USART1,USART2
				*fmt:格式化字符串，如"Num:%d"
				...:可变参数，数量不固定，对应fmt里面的%d %s %f等
	*@condition 需要设置为文本模式,目前仅接受一个字符，超过一个会被后面的覆盖
*/
void USART1_IRQHandler(void)
{
	uint8_t byte=0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)//接收非空
	{
		
		byte = USART_ReceiveData(USART1);
		if(byte == '\n')//以\n作为结束的标志,同时不能让\r污染输出内容
		{
			Serial_RxData[Received_index]= '\0';//发送的字符串需要以'\0'作为结尾，否则会乱码
			Received_index=0;//索引回到初始位置
			Serial_RxFlag = 1;//置接收标志位变量为1
		}
		else if(byte != '\r')
		{
			Serial_RxData[Received_index] = byte;
			Received_index++;
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//移到外层，每次都清除
	}
}
