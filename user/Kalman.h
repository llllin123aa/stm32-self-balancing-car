#ifndef __KALMAN_H
#define __KALMAN_H

#include "stm32f10x.h"

/*
 * 二阶卡尔曼滤波器的可调参数。
 * 所有角度均以 degree 为单位，角速度以 degree/s 为单位；因此 R、Q 的量纲
 * 分别与角度平方和状态传播误差有关，调参时不应混用弧度。
 */
typedef struct
{
	float q_angle;      /* 角度模型过程噪声：越大越快向加速度计观测收敛。 */
	float q_bias;       /* 陀螺零偏随机游走噪声：越大越快跟踪零偏变化。 */
	float r_measure;    /* 加速度计静止时的基础角度测量噪声。 */
	float r_min;        /* 自适应 R 的下限，防止过度相信一次观测。 */
	float r_max;        /* 自适应 R 的上限，仍保留最低限度的观测校正。 */
	float r_scale;      /* |加速度模长 - 1g| 到附加 R 的比例系数。 */
} KalmanConfig;

/* 供串口记录和在线调参判断使用的滤波器输出快照。 */
typedef struct
{
	float angle;        /* 估计的俯仰角（degree）。 */
	float bias;         /* 估计的陀螺仪 Y 轴零偏（degree/s）。 */
	float rate;         /* 已去偏的 Y 轴角速度（degree/s）。 */
	float r_adaptive;   /* 本次更新实际采用的测量噪声 R。 */
	float gain_angle;   /* 角度状态对应的卡尔曼增益 K0。 */
	float gain_bias;    /* 零偏状态对应的卡尔曼增益 K1。 */
	float innovation;   /* 加速度计观测角减预测角的残差（degree）。 */
	float dt_s;         /* 最近一次请求更新的控制周期（s），用于定位调度异常。 */
	uint8_t update_error; /* 最近一次更新的结果，见 KalmanUpdateError_t。 */
} KalmanDiagnostic;

/* 失败码必须保存在诊断数据中；安全锁定后仍可通过蓝牙确定根因。 */
typedef enum
{
	KALMAN_UPDATE_OK = 0,
	KALMAN_UPDATE_NOT_INITIALIZED = 1,
	KALMAN_UPDATE_INPUT_INVALID = 2,
	KALMAN_UPDATE_DT_INVALID = 3,
	KALMAN_UPDATE_COVARIANCE_INVALID = 4,
	KALMAN_UPDATE_OUTPUT_INVALID = 5
} KalmanUpdateError_t;

typedef struct
{
	KalmanConfig config;
	KalmanDiagnostic diagnostic;
	float p00;
	float p01;
	float p10;
	float p11;
	uint8_t initialized;
} KalmanFilter;

void Kalman_Reset(KalmanFilter *filter, const KalmanConfig *config,
				  float initial_angle, float initial_bias);
uint8_t Kalman_Update(KalmanFilter *filter, float gyro_rate_dps,
					  float accel_angle_deg, float accel_magnitude_g, float dt_s);
uint8_t Kalman_IsReady(const KalmanFilter *filter);
void Kalman_GetDiagnostic(const KalmanFilter *filter, KalmanDiagnostic *diagnostic);

#endif
