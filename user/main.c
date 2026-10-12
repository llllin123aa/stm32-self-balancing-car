#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "Timer.h"
#include "MPU6050.h"
#include "motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "math.h"//正切函数
#include "PID.h"
#include "NRF24L01.h"
#include "string.h"//字符串匹配strcmp、字符串分割strtok
#include "stdlib.h"//
uint16_t count;
int16_t AZ,AY,AX,GX,GY,GZ;
uint16_t TimeCount;
float AngleGyro;//陀螺仪得到的y角度
float AngleAcc;//加速度计得到的俯仰角
float AngleFinal;
#define ALPHA 0.98 //陀螺仪的权重系数
uint8_t RunFlag;//运行状态标志位
int16_t LeftPWM,RightPWM;//左轮PWM，右轮PWM
int16_t AvePWM,DifPWM;//平均PWM和差分PWM
char *Tag,*Name,*Value;//数据包的三个部分
float LeftSpeed,RightSpeed;//左轮速度，右轮速度
float AveSpeed,DifSpeed;//平均速度，差分速度
PID_t AnglePID = {
	.Kp=5,
	.Ki=0.1,
	.Kd=5,
	
	.OutMax = 100,//限制PWM范围
	.OutMin = -100,
	.OutOffset = 3,
	.ErrorIntMax = 600,
	.ErrorIntMin = -600,
};
PID_t SpeedPID = {
	.Kp=2,
	.Ki=0.05,
	.Kd=0,
	
	.OutMax = 20,
	.OutMin = -20,//限制倾角范围
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};
PID_t TurnPID = {
	.Kp=4,
	.Ki=3,
	.Kd=0,
	
	.OutMax = 50,
	.OutMin = -50,//限制倾角范围
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};
int8_t RemoteLV,RemoteRH;

int main()
{
	OLED_Init();
	Key_Init();
	LED_Init();
	Timer_Init();//调用该函数一定要写中断服务函数，否则程序会卡死
	MPU6050_Init();
	motor_Init();
	Encoder_Init();
	Serial_Init();
	uart_printf(USART1,"%s","注意：发送时需要以回车作为结尾");
	BlueSerial_Init();
	NRF24L01_Init();
	//BlueSerial_printf(USART2,"Test:%+5d",AZ);//蓝牙串口测试
	while(1)
	{
		static uint8_t plot_cnt = 0;
		if(++plot_cnt >= 5){ plot_cnt = 0;
				BlueSerial_printf(USART2,"[plot,%.2f,%.2f]", AnglePID.Target, AnglePID.Actual);
		}
		if(RunFlag){LED_ON();}else{LED_OFF();}
		if(NRF24L01_Receive())
		{
			RemoteLV = (int8_t)NRF24L01_RxPacket[0];
			RemoteRH = (int8_t)NRF24L01_RxPacket[1];
			SpeedPID.Target = RemoteLV / 50.0f;
			TurnPID.Target = RemoteRH / 25.0f;
		}
		if(Key_Check(0,KEY_SINGLE))
		{
			if(RunFlag ==0)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag=1;
			}
			else{RunFlag=0;}
		}
		else if(Key_Check(1,KEY_SINGLE))
		{
			//OLED_Clear();
			BlueSerial_printf(USART2,"Run:%d",      RunFlag);
			BlueSerial_printf(USART2,"Agy:%+06.1f", AngleGyro);    // 看是否超 ±50
			BlueSerial_printf(USART2,"Act:%+06.2f", AnglePID.Actual);
			BlueSerial_printf(USART2,"Out:%+06.1f", AnglePID.Out);
			BlueSerial_printf(USART2,"Acc:%+06.1f", AngleAcc);
		}
		

		
		//蓝牙串口滑杆功能设置PID参数
		/**/
		if(BlueSerial_GetRxFlag())//接收字符串
		{
			BlueSerial_printf(USART2,"%s",BlueSerial_RxData);
			//字符串拆分strtok
			char *Tag = strtok(BlueSerial_RxData,",");//返回的是当前字段的开头地址，自动记录下次扫描的位置
			if(strcmp(Tag,"key")==0)//Tag是指针，代表整个字符，而*Tag只是'k'
			{
				char *Name = strtok(NULL,",");//提取数据2，定义为按键名称
				char *Value = strtok(NULL,",");//提取数据3，定义为按键数值
				
			}
			else if(strcmp(Tag,"slider")==0)//滑杆
			{
				char *Name = strtok(NULL,",");//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL,",");//提取数据3，定义为滑杆数值
				if(strcmp(Name,"AngleKp")==0)//摇杆AngleKp,调节Kp参数
				{
					AnglePID.Kp = atof(Value);
				}
				else if(strcmp(Name,"AngleKi")==0)//摇杆AngleKi,调节Ki参数
				{
					AnglePID.Ki = atof(Value);
				}
				else if(strcmp(Name,"AngleKd")==0)//摇杆AngleKd,调节Kd参数
				{
					AnglePID.Kd = atof(Value);
				}
				else if(strcmp(Name,"SpeedKp")==0)//摇杆SpeedKi,调节Kp参数
				{
					SpeedPID.Kp = atof(Value);
				}
				else if(strcmp(Name,"SpeedKi")==0)//摇杆SpeedKd,调节Ki参数
				{
					SpeedPID.Ki = atof(Value);
				}
				else if(strcmp(Name,"SpeedKd")==0)//摇杆SpeedKd,调节Kd参数
				{
					SpeedPID.Kd = atof(Value);
				}
				else if(strcmp(Name,"TurnKp")==0)//摇杆TurnKi,调节Kp参数
				{
					TurnPID.Kp = atof(Value);
				}
				else if(strcmp(Name,"TurnKi")==0)//摇杆TurnKd,调节Ki参数
				{
					TurnPID.Ki = atof(Value);
				}
				else if(strcmp(Name,"TurnKd")==0)//摇杆TurnKd,调节Kd参数
				{
					TurnPID.Kd = atof(Value);
				}
				else if(strcmp(Name,"OffSet")==0)//摇杆TurnKd,调节Kd参数
				{
					AnglePID.OutOffset = atof(Value);
				}
			}
			else if(strcmp(Tag,"joystick")==0)//摇杆
			{
				int16_t LH = atoi(strtok(NULL,","));//atoi为字符串转整型,（左摇杆垂直）：前进/后退。往上推为正，往下为负。
				int16_t LV = atoi(strtok(NULL,","));
				int16_t RH = atoi(strtok(NULL,","));//RH（右摇杆水平）：左转/右转。往右推为正，往左为负。
				int16_t RV = atoi(strtok(NULL,","));
				SpeedPID.Target = LV/50.0;//目标速度控制在-2~2之间
				TurnPID.Target = RH/25.0;//差分速度控制在-4到4
				
			}
				
		}
		OLED_Clear();
		OLED_Printf(0,0,OLED_6X8,"   Angle");
		OLED_Printf(0,8,OLED_6X8,"P:%+05.2f",AnglePID.Kp);
		OLED_Printf(0,16,OLED_6X8,"I:%+05.2f",AnglePID.Ki);
		OLED_Printf(0,24,OLED_6X8,"D:%+05.2f",AnglePID.Kd);
		OLED_Printf(0,32,OLED_6X8,"T:%+05.2f",AnglePID.Target);
		OLED_Printf(0,40,OLED_6X8,"A:%+05.2f",AngleFinal);
		OLED_Printf(0,48,OLED_6X8,"O:%+05.2f",AnglePID.Out);
		OLED_Printf(0,56,OLED_6X8,"Offset:%+05.2f",AnglePID.OutOffset);
		
		OLED_Printf(50,0,OLED_6X8,"   Speed");
		OLED_Printf(50,8,OLED_6X8,"P:%+05.2f",SpeedPID.Kp);
		OLED_Printf(50,16,OLED_6X8,"I:%+05.2f",SpeedPID.Ki);
		OLED_Printf(50,24,OLED_6X8,"D:%+05.2f",SpeedPID.Kd);
		OLED_Printf(50,32,OLED_6X8,"D:%+05.2f",SpeedPID.Target);
		OLED_Printf(50,40,OLED_6X8,"A:%+05.2f",AveSpeed);
		OLED_Printf(50,48,OLED_6X8,"O:%+05.2f",SpeedPID.Out);

		OLED_Printf(88,0,OLED_6X8,"   Turn");
		OLED_Printf(88,8,OLED_6X8,"P:%+05.2f",TurnPID.Kp);
		OLED_Printf(88,16,OLED_6X8,"I:%+05.2f",TurnPID.Ki);
		OLED_Printf(88,24,OLED_6X8,"D:%+05.2f",TurnPID.Kd);
		OLED_Printf(88,32,OLED_6X8,"D:%+05.2f",TurnPID.Target);
		OLED_Printf(88,40,OLED_6X8,"A:%+05.2f",DifSpeed);
		OLED_Printf(88,48,OLED_6X8,"O:%+05.2f",TurnPID.Out);
		OLED_Update();
		//BlueSerial_printf(USART2,"[plot,%.2f,%.2f]",SpeedPID.Target,SpeedPID.Actual);
	}
 
}
/*定时器测试程序*/
void TIM1_UP_IRQHandler(void)//1ms
{
	static uint16_t Count0=0;//PID调控周期设置为10ms
	static uint16_t Count1=0;//读取编码器周期设置为50ms
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		Count0++;
		if(Count0==10)
		{
			Count0=0;
			
			MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);//加速度计和陀螺仪数据
			GY-=5;//抑制零漂
			AngleAcc = -atan2(AX*1.0,(AZ*1.0))*180/3.1415962;//atan2的范围是-180~180
			AngleAcc+=2.5;
			//先积分，后融合
			AngleGyro = AngleFinal +GY/32768.0 *2000 * 0.01;//量程为±2000，需要进行缩放,因为定时器是10ms，积分出来的单位是度/s
			AngleFinal = ALPHA * AngleGyro + (1.0f - ALPHA) * AngleAcc;
			if(AngleGyro>50|| AngleGyro<-50){RunFlag = 0;}//超过这个范围，小车无法保持平衡
			if(RunFlag)//只有开启了RunFlag才进行PID控制
			{
				//PID第一步：计算当前最新的俯仰角
				AnglePID.Actual =  AngleFinal;
				
				//PID第二步：根据目标倾角计算输出到电机的平均PWM(假定目标角度为0)
				PID_Update(&AnglePID);//得到输出值
				AvePWM = -AnglePID.Out;
				
				//变量转换
				LeftPWM = AvePWM + DifPWM/2;
				RightPWM = AvePWM - DifPWM/2;
				//输出限幅
				if(LeftPWM>100){LeftPWM = 100;}else if(LeftPWM <-100){LeftPWM = -100;}
				if(RightPWM>100){RightPWM = 100;}else if(RightPWM <-100){RightPWM = -100;}
				
				Motor_SetPWM(1,LeftPWM);
				Motor_SetPWM(2,RightPWM);
			}
			else
				{
					Motor_SetPWM(1,0);//左电机PWM设置为0
					Motor_SetPWM(2,0);//右电机PWM设置为0
				}
			
		}
		Count1++;
		if(Count1>=50)
		{
			Count1=0;
			LeftSpeed = Encoder_Get(1)/44.0/0.05/9.27666;//获取增量值,并计算每秒转速
			RightSpeed = Encoder_Get(2)/44.0/0.05/9.27666;//44是编码器极数，0.05是从50ms转化为s,9.27666减速比
			AveSpeed = (LeftSpeed + RightSpeed)/2;
			DifSpeed = LeftSpeed - RightSpeed;
			
			if(RunFlag)
			{
				//PID第一步：获取实际速度值
				SpeedPID.Actual = AveSpeed;
				//PID第二步：根据目标速度与实际速度的差值，转化为角度
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;//外环输出赋值给内环输入
				
				//TurnPID：第一步获取实际值
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
			
		}
		Key_Tick();//检测按键
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除中断标志位
	}
}
