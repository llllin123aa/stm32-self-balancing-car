#include "LQR.h"

/* 不依赖 Cortex-M3 工具链中不一致的 isfinite() 实现。 */
static uint8_t LQR_IsFinite(float value)
{
	return (value == value) && (value > -100000.0f) && (value < 100000.0f);
}

static float LQR_Clamp(float value, float min_value, float max_value)
{
	if (value < min_value) return min_value;
	if (value > max_value) return max_value;
	return value;
}

void LQR_Init(LQRController *controller, const LQRConfig *config)
{
	controller->config = *config;
	controller->position_ref_m = 0.0f;
	controller->velocity_ref_mps = 0.0f;
	controller->last_pwm = 0.0f;
	controller->raw_u = 0.0f;
	controller->limited_u = 0.0f;
	controller->saturated = 0;
	controller->initialized = 1;
	controller->update_error = LQR_UPDATE_OK;
}

void LQR_Reset(LQRController *controller, float current_position_m)
{
	/* 停车/切换模式时消除历史位置误差，避免重新使能时出现驱动脉冲。 */
	controller->position_ref_m = current_position_m;
	controller->velocity_ref_mps = 0.0f;
	controller->last_pwm = 0.0f;
	controller->raw_u = 0.0f;
	controller->limited_u = 0.0f;
	controller->saturated = 0;
	controller->update_error = LQR_UPDATE_OK;
}

uint8_t LQR_IsConfigUsable(const LQRController *controller)
{
	const LQRConfig *c = &controller->config;
	uint8_t i;
	if ((controller->initialized == 0) || (c->validated == 0) ||
		!LQR_IsFinite(c->pwm_limit) || !LQR_IsFinite(c->pwm_slew_rate) ||
		!LQR_IsFinite(c->max_speed_mps) || !LQR_IsFinite(c->max_accel_mps2) ||
		(c->pwm_limit <= 0.0f) || (c->pwm_slew_rate <= 0.0f) ||
		(c->max_speed_mps <= 0.0f) || (c->max_accel_mps2 <= 0.0f))
	{
		return 0;
	}
	for (i = 0; i < 4; i++)
	{
		if (!LQR_IsFinite(c->k[i])) return 0;
	}
	return 1;
}

uint8_t LQR_Update(LQRController *controller, const LQRState *state,
				   float requested_speed_mps, float dt_s, float *average_pwm)
{
	float speed_step;
	float error_theta;
	float error_rate;
	float error_position;
	float error_velocity;
	float pwm_target;
	float pwm_step;

	*average_pwm = 0.0f;
	if (!LQR_IsConfigUsable(controller))
	{
		controller->update_error = LQR_UPDATE_CONFIG_INVALID;
		return 0;
	}
	if (!LQR_IsFinite(state->theta_rad) || !LQR_IsFinite(state->theta_rate_rad_s) ||
		!LQR_IsFinite(state->position_m) || !LQR_IsFinite(state->velocity_mps) ||
		!LQR_IsFinite(requested_speed_mps))
	{
		controller->update_error = LQR_UPDATE_INPUT_INVALID;
		return 0;
	}
	if ((dt_s < 0.009f) || (dt_s > 0.011f))
	{
		controller->update_error = LQR_UPDATE_DT_INVALID;
		return 0;
	}

	/* 先限加速度得到 v_ref，再积分位置参考；没有隐藏积分控制项。 */
	requested_speed_mps = LQR_Clamp(requested_speed_mps,
		-controller->config.max_speed_mps, controller->config.max_speed_mps);
	speed_step = controller->config.max_accel_mps2 * dt_s;
	controller->velocity_ref_mps += LQR_Clamp(requested_speed_mps - controller->velocity_ref_mps,
		-speed_step, speed_step);
	controller->position_ref_m += controller->velocity_ref_mps * dt_s;

	error_theta = state->theta_rad;
	error_rate = state->theta_rate_rad_s;
	error_position = state->position_m - controller->position_ref_m;
	error_velocity = state->velocity_mps - controller->velocity_ref_mps;
	controller->raw_u = -(controller->config.k[0] * error_theta +
		controller->config.k[1] * error_rate + controller->config.k[2] * error_position +
		controller->config.k[3] * error_velocity);
	controller->limited_u = LQR_Clamp(controller->raw_u, -1.0f, 1.0f);
	pwm_target = controller->limited_u * controller->config.pwm_limit;
	pwm_step = controller->config.pwm_slew_rate * dt_s;
	controller->last_pwm += LQR_Clamp(pwm_target - controller->last_pwm, -pwm_step, pwm_step);
	controller->saturated = (controller->raw_u != controller->limited_u) ? 1 : 0;
	controller->update_error = LQR_UPDATE_OK;
	*average_pwm = controller->last_pwm;
	return 1;
}
