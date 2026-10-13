#include "Kalman.h"

/*
 * Cortex-M3 没有硬件浮点单元。这里不用依赖特定 C 库的 isfinite()，而以
 * NaN 自比较和合理的工程数值范围拦截溢出/非法输入，防止异常值进入电机控制。
 */
static uint8_t Kalman_IsFinite(float value)
{
	return (value == value) && (value > -100000.0f) && (value < 100000.0f);
}

static float Kalman_Clamp(float value, float min_value, float max_value)
{
	if (value < min_value)
	{
		return min_value;
	}
	if (value > max_value)
	{
		return max_value;
	}
	return value;
}

/*
 * 以静止标定得到的初始姿态和零偏建立滤波器。初始协方差给为 1，表示刚启动时
 * 两个状态均存在有限不确定度；随后卡尔曼增益会按观测质量自动收敛。
 */
void Kalman_Reset(KalmanFilter *filter, const KalmanConfig *config,
				  float initial_angle, float initial_bias)
{
	filter->config = *config;
	filter->diagnostic.angle = initial_angle;
	filter->diagnostic.bias = initial_bias;
	filter->diagnostic.rate = 0.0f;
	filter->diagnostic.r_adaptive = config->r_measure;
	filter->diagnostic.gain_angle = 0.0f;
	filter->diagnostic.gain_bias = 0.0f;
	filter->diagnostic.innovation = 0.0f;
	filter->diagnostic.dt_s = 0.0f;
	filter->diagnostic.update_error = KALMAN_UPDATE_OK;
	filter->p00 = 1.0f;
	filter->p01 = 0.0f;
	filter->p10 = 0.0f;
	filter->p11 = 1.0f;
	filter->initialized = 1;
}

/*
 * 完成一次“预测 + 校正”。状态为 [angle, bias]^T：
 *   angle(k) = angle(k-1) + (gyro_y - bias) * dt
 * 加速度模长偏离 1g 时，车辆很可能处于加速、刹车或受冲击状态；此时增大 R，
 * 使滤波器暂时主要依赖陀螺仪预测，而非受线加速度污染的倾角观测。
 */
uint8_t Kalman_Update(KalmanFilter *filter, float gyro_rate_dps,
					  float accel_angle_deg, float accel_magnitude_g, float dt_s)
{
	float p00;
	float p01;
	float p10;
	float p11;
	float residual_g;
	float innovation_variance;

	filter->diagnostic.dt_s = dt_s;
	if (filter->initialized == 0)
	{
		filter->diagnostic.update_error = KALMAN_UPDATE_NOT_INITIALIZED;
		return 0;
	}
	if (!Kalman_IsFinite(gyro_rate_dps) || !Kalman_IsFinite(accel_angle_deg) ||
		!Kalman_IsFinite(accel_magnitude_g))
	{
		filter->diagnostic.update_error = KALMAN_UPDATE_INPUT_INVALID;
		return 0;
	}
	if ((dt_s < 0.001f) || (dt_s > 0.100f))
	{
		filter->diagnostic.update_error = KALMAN_UPDATE_DT_INVALID;
		return 0;
	}

	/* 预测步骤：先用已估计的零偏修正陀螺仪，再积分更新角度。 */
	filter->diagnostic.rate = gyro_rate_dps - filter->diagnostic.bias;
	filter->diagnostic.angle += filter->diagnostic.rate * dt_s;

	/* 传播 2x2 误差协方差，并向对角线注入对应的过程噪声。 */
	p00 = filter->p00 + dt_s * (dt_s * filter->p11 - filter->p01 - filter->p10 + filter->config.q_angle);
	p01 = filter->p01 - dt_s * filter->p11;
	p10 = filter->p10 - dt_s * filter->p11;
	p11 = filter->p11 + filter->config.q_bias * dt_s;
	filter->p00 = p00;
	filter->p01 = p01;
	filter->p10 = p10;
	filter->p11 = p11;

	/* 根据 |a|-1g 自适应放大 R；绝对值手工计算以避免额外 libm 调用。 */
	residual_g = accel_magnitude_g - 1.0f;
	if (residual_g < 0.0f)
	{
		residual_g = -residual_g;
	}
	filter->diagnostic.r_adaptive = Kalman_Clamp(filter->config.r_measure +
			filter->config.r_scale * residual_g, filter->config.r_min, filter->config.r_max);

	innovation_variance = filter->p00 + filter->diagnostic.r_adaptive;
	if (!Kalman_IsFinite(innovation_variance) || (innovation_variance <= 0.000001f))
	{
		filter->diagnostic.update_error = KALMAN_UPDATE_COVARIANCE_INVALID;
		return 0;
	}

	/* 校正步骤：观测矩阵 H=[1, 0]，因此只直接观测角度状态。 */
	filter->diagnostic.innovation = accel_angle_deg - filter->diagnostic.angle;
	filter->diagnostic.gain_angle = filter->p00 / innovation_variance;
	filter->diagnostic.gain_bias = filter->p10 / innovation_variance;
	filter->diagnostic.angle += filter->diagnostic.gain_angle * filter->diagnostic.innovation;
	filter->diagnostic.bias += filter->diagnostic.gain_bias * filter->diagnostic.innovation;

	/* 以更新前的 P 保存量完成 P=(I-KH)P，避免就地覆盖造成计算错误。 */
	p00 = filter->p00;
	p01 = filter->p01;
	filter->p00 -= filter->diagnostic.gain_angle * p00;
	filter->p01 -= filter->diagnostic.gain_angle * p01;
	filter->p10 -= filter->diagnostic.gain_bias * p00;
	filter->p11 -= filter->diagnostic.gain_bias * p01;

	if (!Kalman_IsFinite(filter->diagnostic.angle) || !Kalman_IsFinite(filter->diagnostic.bias) ||
		!Kalman_IsFinite(filter->diagnostic.rate))
	{
		filter->initialized = 0;
		filter->diagnostic.update_error = KALMAN_UPDATE_OUTPUT_INVALID;
		return 0;
	}
	filter->diagnostic.update_error = KALMAN_UPDATE_OK;
	return 1;
}

uint8_t Kalman_IsReady(const KalmanFilter *filter)
{
	return filter->initialized;
}

void Kalman_GetDiagnostic(const KalmanFilter *filter, KalmanDiagnostic *diagnostic)
{
	*diagnostic = filter->diagnostic;
}
