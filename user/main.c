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
#include "Kalman.h"
#include "LQR.h"
#include "string.h"//字符串匹配strcmp、字符串分割strtok
#include "stdlib.h"//

/* 固件和参数版本随诊断日志输出，保证后续对比测试可以追溯。 */
#define ATTITUDE_FIRMWARE_VERSION  "R003-KALMAN-1.0"
#define KALMAN_PARAMETER_VERSION   "KALMAN-P1"
#define ATTITUDE_UPDATE_DT_S       0.010f
#define MPU6050_ACCEL_LSB_PER_G    2048.0f  /* MPU6050 已配置为 ±16g。 */
#define MPU6050_GYRO_LSB_PER_DPS   16.384f  /* MPU6050 已配置为 ±2000 degree/s。 */
#define ATTITUDE_SAFETY_LIMIT_DEG  50.0f
#define CALIBRATION_SAMPLE_COUNT   200U    /* 200 * 10 ms = 2 s 静止标定。 */
#define DEG_TO_RAD                 0.01745329252f
#define LQR_ENCODER_COUNTS_PER_REV 44.0f
#define LQR_GEAR_RATIO             9.27666f
/* 实测轮半径后替换；零值会使 LQR 配置保持不可用，杜绝未标定直接上车。 */
#define LQR_WHEEL_RADIUS_M         0.0f
#define LQR_REMOTE_SPEED_SCALE_MPS 0.01f

typedef enum
{
	FILTER_MODE_COMPLEMENTARY = 0,
	FILTER_MODE_KALMAN = 1
} FilterMode_t;

typedef enum
{
	CONTROL_MODE_PID = 0,
	CONTROL_MODE_LQR = 1
} ControlMode_t;

/* 蓝牙日志中的标定状态码：0 表示允许按键启动，其余值均保持电机锁定。 */
typedef enum
{
	CALIBRATION_OK = 0,
	CALIBRATION_IMU_ID_ERROR = 1,
	CALIBRATION_NOT_STILL = 2,
	CALIBRATION_FILTER_ERROR = 3
} CalibrationStatus_t;

uint16_t count;
int16_t AZ,AY,AX,GX,GY,GZ;
uint16_t TimeCount;
float AngleGyro;//陀螺仪得到的y角度
float AngleAcc;//加速度计得到的俯仰角
float AngleFinal;
float AngleComplementary;              /* 保留的互补滤波输出，仅用于回退和对比。 */
float AngleKalman;                     /* 卡尔曼滤波输出，默认作为 AngleFinal 来源。 */
float PitchReferenceOffsetDeg;          /* 标定时的机械平衡角；后续所有俯仰角以它为零点。 */
#define ALPHA 0.98 //陀螺仪的权重系数
uint8_t RunFlag;//运行状态标志位
volatile uint8_t ImuFault = 0;         /* 置位后拒绝启动，避免传感器异常时驱动电机。 */
volatile uint8_t FilterReady = 0;      /* 完成静止标定并成功复位滤波器后才允许启动。 */
volatile CalibrationStatus_t CalibrationStatus = CALIBRATION_NOT_STILL;
volatile uint32_t SystemTimeMs = 0;    /* TIM1 每次 1 ms 更新中断递增一次的调度时钟。 */
volatile uint32_t AttitudeTimeMs = 0;  /* 仅在 10 ms 姿态更新点累加的日志时间戳。 */
FilterMode_t FilterMode = FILTER_MODE_KALMAN; /* R-003 验收要求：默认采用卡尔曼。 */
ControlMode_t ControlMode = CONTROL_MODE_PID; /* LQR 完整验收前默认保留 PID。 */
KalmanFilter PitchKalman;
KalmanDiagnostic PitchKalmanDiagnostic;
LQRController LongitudinalLQR;
/*
 * 禁止用 PID 参数代替 LQR 增益。请由离线辨识/求解产物替换 k、版本号并将
 * validated 置为 1；轮径也必须同步填写，LQR 才允许切换。
 */
const LQRConfig LongitudinalLQRConfig = {
	.k = {0.0f, 0.0f, 0.0f, 0.0f},
	.pwm_limit = 20.0f,
	.pwm_slew_rate = 200.0f,
	.max_speed_mps = 0.30f,
	.max_accel_mps2 = 0.30f,
	.version = 0U,
	.validated = 0U
};

/*
 * 初始参数适合 10 ms 控制周期。实际装车后，应通过需求文档要求的静止/动态日志
 * 调整这些参数；不要再用 ALPHA 同时承担模型误差和测量噪声两种职责。
 */
const KalmanConfig PitchKalmanConfig = {
	.q_angle = 0.001f,
	.q_bias = 0.003f,
	.r_measure = 0.50f,
	.r_min = 0.30f,
	.r_max = 100.0f,
	.r_scale = 40.0f,
};
int16_t LeftPWM,RightPWM;//左轮PWM，右轮PWM
int16_t AvePWM,DifPWM;//平均PWM和差分PWM
char *Tag,*Name,*Value;//数据包的三个部分
float LeftSpeed,RightSpeed;//左轮速度，右轮速度
float AveSpeed,DifSpeed;//平均速度，差分速度
float LQRPositionM, LQRVelocityMps, LQRRequestedSpeedMps;
int32_t LeftEncoderAccum, RightEncoderAccum;
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

/* 将原始加速度计计数转换成 g 后计算俯仰观测角，符号与原互补滤波保持一致。 */
static float Attitude_AccelAngleDeg(int16_t acc_x, int16_t acc_z)
{
	return -atan2((float)acc_x, (float)acc_z) * 180.0f / 3.1415926f;
}

/* 三轴加速度模长用于判断加速度计是否仍主要测得重力。 */
static float Attitude_AccelMagnitudeG(int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
	float x_g = (float)acc_x / MPU6050_ACCEL_LSB_PER_G;
	float y_g = (float)acc_y / MPU6050_ACCEL_LSB_PER_G;
	float z_g = (float)acc_z / MPU6050_ACCEL_LSB_PER_G;
	return sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
}

/*
 * 上电标定必须在定时器中断启动前执行，因此它不会和中断中的 I2C 读取抢占同一条
 * 总线。标定期间 RunFlag 保持为 0，且显式将两个 PWM 归零。
 */
static uint8_t Attitude_Calibrate(void)
{
	uint16_t sample;
	float angle_sum = 0.0f;
	float gyro_sum = 0.0f;

	Motor_SetPWM(1, 0);
	Motor_SetPWM(2, 0);
	FilterReady = 0;
	CalibrationStatus = CALIBRATION_NOT_STILL;
	if (MPU6050_GetID() != 0x68)
	{
		CalibrationStatus = CALIBRATION_IMU_ID_ERROR;
		return 0; /* WHO_AM_I 不正确，禁止进入控制状态。 */
	}

	for (sample = 0; sample < CALIBRATION_SAMPLE_COUNT; sample++)
	{
		float accel_g;
		float gyro_y_dps;
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		accel_g = Attitude_AccelMagnitudeG(AX, AY, AZ);
		gyro_y_dps = (float)GY / MPU6050_GYRO_LSB_PER_DPS;

		/* 静止标定时过大的运动或加速度都使本次标定无效。 */
		if ((accel_g < 0.85f) || (accel_g > 1.15f) ||
			(gyro_y_dps < -10.0f) || (gyro_y_dps > 10.0f))
		{
			CalibrationStatus = CALIBRATION_NOT_STILL;
			return 0;
		}
		angle_sum += Attitude_AccelAngleDeg(AX, AZ);
		gyro_sum += gyro_y_dps;
		Delay_ms(10);
	}

	/*
	 * 三级 PID 的平衡目标固定为 0°。因此把“上电静止且已摆正的小车姿态”
	 * 定义为机械零位，避免 MPU 安装方向造成原始俯仰角接近 ±90° 时被安全阈值
	 * 误判为已经倾倒。上电标定时必须将车体扶在真实平衡姿态。
	 */
	PitchReferenceOffsetDeg = angle_sum / (float)CALIBRATION_SAMPLE_COUNT;
	AngleAcc = 0.0f;
	AngleComplementary = 0.0f;
	AngleGyro = 0.0f;
	Kalman_Reset(&PitchKalman, &PitchKalmanConfig, 0.0f,
			gyro_sum / (float)CALIBRATION_SAMPLE_COUNT);
	Kalman_GetDiagnostic(&PitchKalman, &PitchKalmanDiagnostic);
	AngleKalman = PitchKalmanDiagnostic.angle;
	AngleFinal = AngleKalman;
	FilterReady = 1;
	CalibrationStatus = CALIBRATION_OK;
	return 1;
}

/* 诊断日志仅在主循环中发送，绝不在定时器中断内等待串口发送完成。 */
static void Attitude_LogIfDue(void)
{
	static uint32_t last_log_time_ms = 0;
	uint32_t now_ms = AttitudeTimeMs;
	/* 9600 波特率发送一帧完整诊断文本约需数百毫秒；2 Hz 避免主循环被日志占满。 */
	if ((now_ms - last_log_time_ms) < 500U)
	{
		return;
	}
	last_log_time_ms = now_ms;
	Kalman_GetDiagnostic(&PitchKalman, &PitchKalmanDiagnostic);
	BlueSerial_printf(USART2,
		/*
		 * 启动故障定位帧刻意限制为短文本：256 字节栈缓冲和 9600 波特率下均可安全
		 * 发送。字段已足够判断标定状态、卡尔曼失败码和基本滤波输出。
		 */
		"[imu,t=%lu,dt=%.3f,kerr=%d,ready=%d,fault=%d,cal=%d,comp=%.2f,kal=%.2f,bias=%.3f,R=%.2f]",
		now_ms, PitchKalmanDiagnostic.dt_s, PitchKalmanDiagnostic.update_error,
		FilterReady, ImuFault, CalibrationStatus, AngleComplementary, AngleKalman,
		PitchKalmanDiagnostic.bias, PitchKalmanDiagnostic.r_adaptive);
	/* 此处不发送增益、原始 IMU 和 PWM，避免长帧影响控制与蓝牙稳定性。 */
}

int main()
{
	OLED_Init();
	Key_Init();
	LED_Init();
	MPU6050_Init();
	motor_Init();
	Encoder_Init();
	Serial_Init();
	BlueSerial_Init();
	NRF24L01_Init();
	LQR_Init(&LongitudinalLQR, &LongitudinalLQRConfig);
	/* 先让电机保持禁用状态完成标定，再开启会访问 IMU 的定时器中断。 */
	if (!Attitude_Calibrate())
	{
		ImuFault = 1;
		uart_printf(USART1, "IMU calibration failed; motor locked.\r\n");
		BlueSerial_printf(USART2, "[status,calibration=fail,code=%d]", CalibrationStatus);
	}
	else
	{
		uart_printf(USART1, "%s %s calibrated.\r\n", ATTITUDE_FIRMWARE_VERSION,
			KALMAN_PARAMETER_VERSION);
		BlueSerial_printf(USART2, "[status,calibration=ok,zero=%.2f]", PitchReferenceOffsetDeg);
	}
	Timer_Init();//调用该函数一定要写中断服务函数，否则程序会卡死
	uart_printf(USART1,"%s","注意：发送时需要以回车作为结尾");
	//BlueSerial_printf(USART2,"Test:%+5d",AZ);//蓝牙串口测试
	while(1)
	{
		static uint8_t plot_cnt = 0;
		if(++plot_cnt >= 5){ plot_cnt = 0;
				BlueSerial_printf(USART2,"[plot,%.2f,%.2f]", AnglePID.Target, AnglePID.Actual);
		}
		Attitude_LogIfDue();
		if(RunFlag){LED_ON();}else{LED_OFF();}
		if(NRF24L01_Receive())
		{
			RemoteLV = (int8_t)NRF24L01_RxPacket[0];
			RemoteRH = (int8_t)NRF24L01_RxPacket[1];
			SpeedPID.Target = RemoteLV / 50.0f;
			LQRRequestedSpeedMps = RemoteLV * LQR_REMOTE_SPEED_SCALE_MPS;
			TurnPID.Target = RemoteRH / 25.0f;
		}
		/* PB1 按下沿立即启停；原 KEY_SINGLE 需松开后再等待 200 ms，现场易被误判为无响应。 */
		if(Key_Check(0,KEY_DOWN))
		{
			if((RunFlag == 0) && FilterReady && !ImuFault)
			{
				if ((ControlMode == CONTROL_MODE_LQR) &&
					(!LQR_IsConfigUsable(&LongitudinalLQR) || (LQR_WHEEL_RADIUS_M <= 0.0f)))
				{
					BlueSerial_printf(USART2,"[lqr,enable=denied,reason=config]");
					continue;
				}
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				LQR_Reset(&LongitudinalLQR, LQRPositionM);
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
				else if(strcmp(Name,"FilterMode")==0)//0：互补滤波回退，1：默认卡尔曼滤波
				{
					uint8_t requested_mode = (uint8_t)atoi(Value);
					/* 未完成标定时拒绝切换到卡尔曼，保证切换不会绕过安全前置条件。 */
					if ((requested_mode == FILTER_MODE_COMPLEMENTARY) ||
						((requested_mode == FILTER_MODE_KALMAN) && FilterReady))
					{
						FilterMode = (FilterMode_t)requested_mode;
					}
				}
				else if(strcmp(Name,"ControlMode")==0)//0：PID，1：LQR；只允许停车且零速度时切换
				{
					uint8_t requested_mode = (uint8_t)atoi(Value);
					if ((RunFlag == 0) && (LQRRequestedSpeedMps > -0.001f) &&
						(LQRRequestedSpeedMps < 0.001f))
					{
						if ((requested_mode == CONTROL_MODE_PID) ||
							((requested_mode == CONTROL_MODE_LQR) && LQR_IsConfigUsable(&LongitudinalLQR) &&
							 (LQR_WHEEL_RADIUS_M > 0.0f)))
						{
							ControlMode = (ControlMode_t)requested_mode;
							LQR_Reset(&LongitudinalLQR, LQRPositionM);
						}
					}
				}
			}
			else if(strcmp(Tag,"joystick")==0)//摇杆
			{
				int16_t LH = atoi(strtok(NULL,","));//atoi为字符串转整型,（左摇杆垂直）：前进/后退。往上推为正，往下为负。
				int16_t LV = atoi(strtok(NULL,","));
				int16_t RH = atoi(strtok(NULL,","));//RH（右摇杆水平）：左转/右转。往右推为正，往左为负。
				int16_t RV = atoi(strtok(NULL,","));
				SpeedPID.Target = LV/50.0;//目标速度控制在-2~2之间
				LQRRequestedSpeedMps = LV * LQR_REMOTE_SPEED_SCALE_MPS;
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
		/* 启动失败时先看 F/E/C：F=滤波器就绪，E=IMU 故障，C=标定状态。T 必须递增。 */
		OLED_Printf(0,56,OLED_6X8,"T%lu R%dF%dE%dC%d", SystemTimeMs / 1000U,
			RunFlag, FilterReady, ImuFault, CalibrationStatus);
		
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
	static uint32_t LastAttitudeTimeMs = 0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/* 使用 1 ms 调度时间戳计算本次滤波的真实软件调度间隔，而非硬编码 dt。 */
		SystemTimeMs++;
		Count0++;
		if(Count0==10)
		{
			float gyro_y_dps;
			float accel_magnitude_g;
			float actual_dt_s;
			float wheel_m_per_count;
			int16_t left_encoder_delta;
			int16_t right_encoder_delta;
			Count0=0;
			AttitudeTimeMs = SystemTimeMs;
			actual_dt_s = (float)(AttitudeTimeMs - LastAttitudeTimeMs) / 1000.0f;
			LastAttitudeTimeMs = AttitudeTimeMs;
			/* 编码器在 100 Hz 读取；累计量仍提供 PID 的 50 ms 速度窗口。 */
			left_encoder_delta = Encoder_Get(1);
			right_encoder_delta = Encoder_Get(2);
			LeftEncoderAccum += left_encoder_delta;
			RightEncoderAccum += right_encoder_delta;
			wheel_m_per_count = 6.283185307f * LQR_WHEEL_RADIUS_M /
				(LQR_ENCODER_COUNTS_PER_REV * LQR_GEAR_RATIO);
			LQRPositionM += ((float)left_encoder_delta + (float)right_encoder_delta) *
				0.5f * wheel_m_per_count;
			LQRVelocityMps = ((float)left_encoder_delta + (float)right_encoder_delta) *
				0.5f * wheel_m_per_count / actual_dt_s;
			MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);//加速度计和陀螺仪数据
			gyro_y_dps = (float)GY / MPU6050_GYRO_LSB_PER_DPS;
			/* 从原始传感器角中减去上电标定的机械零位，平衡姿态即为 0°。 */
			AngleAcc = Attitude_AccelAngleDeg(AX, AZ) - PitchReferenceOffsetDeg;
			accel_magnitude_g = Attitude_AccelMagnitudeG(AX, AY, AZ);

			/*
			 * 保留原互补滤波作为可对比、可回退的独立路径。这里不再硬编码 GY-=5，
			 * 因为零偏应由卡尔曼状态 b 在线估计；互补路径只用于测试基线。
			 */
			AngleGyro = AngleComplementary + gyro_y_dps * ATTITUDE_UPDATE_DT_S;
			AngleComplementary = ALPHA * AngleGyro + (1.0f - ALPHA) * AngleAcc;

			/*
			 * 标定未完成时绝不能调用 Kalman_Update()：滤波器尚未 Kalman_Reset，
			 * 调用只会得到“未初始化”并覆盖真正的标定失败原因（ID 或静止检查）。
			 */
			if (FilterReady)
			{
				/* 卡尔曼路径使用实际调度周期以及已换算的 degree、degree/s、g。 */
				if (!Kalman_Update(&PitchKalman, gyro_y_dps, AngleAcc,
					accel_magnitude_g, actual_dt_s))
				{
					/* 计算非法（NaN/Inf、协方差异常等）即锁定电机，等待重新上电标定。 */
					FilterReady = 0;
					ImuFault = 1;
					CalibrationStatus = CALIBRATION_FILTER_ERROR;
					RunFlag = 0;
				}
			}
			Kalman_GetDiagnostic(&PitchKalman, &PitchKalmanDiagnostic);
			AngleKalman = PitchKalmanDiagnostic.angle;

			/* FilterMode 默认为卡尔曼；互补模式只保留给基线测试或故障排查。 */
			if (FilterMode == FILTER_MODE_KALMAN)
			{
				AngleFinal = AngleKalman;
			}
			else
			{
				AngleFinal = AngleComplementary;
			}

			/* 姿态超安全阈值时，在当前（至多 10 ms）控制周期内关闭电机。 */
			if (!FilterReady || ImuFault || (AngleFinal > ATTITUDE_SAFETY_LIMIT_DEG) ||
				(AngleFinal < -ATTITUDE_SAFETY_LIMIT_DEG))
			{
				RunFlag = 0;
			}
			if(RunFlag)
			{
				if (ControlMode == CONTROL_MODE_LQR)
				{
					LQRState state;
					float lqr_pwm;
					state.theta_rad = AngleKalman * DEG_TO_RAD;
					state.theta_rate_rad_s = PitchKalmanDiagnostic.rate * DEG_TO_RAD;
					state.position_m = LQRPositionM;
					state.velocity_mps = LQRVelocityMps;
					if (LQR_Update(&LongitudinalLQR, &state, LQRRequestedSpeedMps,
						actual_dt_s, &lqr_pwm))
					{
						AvePWM = (int16_t)lqr_pwm;
					}
					else
					{
						/* 非法 dt、状态或配置时同周期锁定。 */
						RunFlag = 0;
					}
				}
				else
				{
					AnglePID.Actual = AngleFinal;
					PID_Update(&AnglePID);
					AvePWM = -AnglePID.Out;
				}

				if (RunFlag)
				{
					LeftPWM = AvePWM + DifPWM/2;
					RightPWM = AvePWM - DifPWM/2;
					if(LeftPWM>100){LeftPWM = 100;}else if(LeftPWM <-100){LeftPWM = -100;}
					if(RightPWM>100){RightPWM = 100;}else if(RightPWM <-100){RightPWM = -100;}
					Motor_SetPWM(1,LeftPWM);
					Motor_SetPWM(2,RightPWM);
				}
				else
				{
					Motor_SetPWM(1,0);
					Motor_SetPWM(2,0);
				}
			}
			else
			{
				Motor_SetPWM(1,0);
				Motor_SetPWM(2,0);
			}
			
		}
		Count1++;
		if(Count1>=50)
		{
			Count1=0;
			/* 10 ms 时已读取并清零硬件计数器，这里使用累计量保持 PID 原有 50 ms 窗口。 */
			LeftSpeed = LeftEncoderAccum/44.0f/0.05f/9.27666f;
			RightSpeed = RightEncoderAccum/44.0f/0.05f/9.27666f;
			LeftEncoderAccum = 0;
			RightEncoderAccum = 0;
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
