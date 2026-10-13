#ifndef __LQR_H
#define __LQR_H

#include "stm32f10x.h"

/*
 * 离散 LQR 的所有输入和增益均使用 SI 单位：
 * x = [theta(rad), theta_rate(rad/s), position(m), velocity(m/s)]。
 * 正方向必须在整车标定后与编码器、电机及 IMU 的实际方向保持一致。
 */
typedef struct
{
	float k[4];              /* u = -k * (x - x_ref)，u 为归一化驱动命令。 */
	float pwm_limit;         /* 平均 PWM 最大绝对值，单位为电机 PWM 计数。 */
	float pwm_slew_rate;     /* 平均 PWM 最大变化率，单位 PWM/s。 */
	float max_speed_mps;     /* 速度参考限幅，m/s。 */
	float max_accel_mps2;    /* 速度参考最大加速度，m/s^2。 */
	uint32_t version;        /* 与离线 A/B/Q/R/K 产物一致的版本号。 */
	uint8_t validated;       /* 仅在离线模型和台架方向测试完成后置 1。 */
} LQRConfig;

typedef struct
{
	float theta_rad;
	float theta_rate_rad_s;
	float position_m;
	float velocity_mps;
} LQRState;

typedef enum
{
	LQR_UPDATE_OK = 0,
	LQR_UPDATE_NOT_INITIALIZED = 1,
	LQR_UPDATE_CONFIG_INVALID = 2,
	LQR_UPDATE_INPUT_INVALID = 3,
	LQR_UPDATE_DT_INVALID = 4
} LQRUpdateError_t;

typedef struct
{
	LQRConfig config;
	float position_ref_m;
	float velocity_ref_mps;
	float last_pwm;
	float raw_u;             /* 限幅前的归一化输出，供日志和调参使用。 */
	float limited_u;         /* 限幅后的归一化输出。 */
	uint8_t saturated;
	uint8_t initialized;
	uint8_t update_error;
} LQRController;

void LQR_Init(LQRController *controller, const LQRConfig *config);
void LQR_Reset(LQRController *controller, float current_position_m);
uint8_t LQR_IsConfigUsable(const LQRController *controller);
uint8_t LQR_Update(LQRController *controller, const LQRState *state,
				   float requested_speed_mps, float dt_s, float *average_pwm);

#endif
