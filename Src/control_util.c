/*
 * speed_control.c
 */

#include "control_util.h"

//#if (MOTOR_TYPE == MOTOR_FAULHABER)
//float p_spd_integral_max_abs = 27.0f;  //Dang dung
//float p_spd_u_rate_max = 0.35f;
//#else
float p_spd_integral_max_abs = 30.5f;
float p_spd_u_rate_max = 0.45f;
//#endif

float w_speed_integral;
static rate_limit_t rlim_spdctrl;
static float w_speed_error, w_pid_error_raw;
static float f_lpf_speed_error = 0.85f;

float pid_controller(const float feedback, const float setpoint, pid_para_t pid_parameters, const float dt) {
	static float pre_speed_error;
	w_pid_error_raw = feedback - setpoint;
	//w_speed_error = LPF(w_pid_error_raw, pre_speed_error, f_lpf_speed_error, dt);
	w_speed_error = w_pid_error_raw;
	w_speed_integral = saturatef(w_speed_integral + dt * pid_parameters.ki * w_speed_error,
												-p_spd_integral_max_abs,
												p_spd_integral_max_abs);
	float u = pid_parameters.kp * w_speed_error + w_speed_integral + pid_parameters.kd * (w_speed_error - pre_speed_error) / dt;
	
	u = rate_limit(&rlim_spdctrl, u, -p_spd_u_rate_max, p_spd_u_rate_max);
	pre_speed_error = w_speed_error;

	return u;
}

void spdctrl_reset(void) {
  w_speed_integral = 0.0f;
}

int spdctrl_init(void) {
  rate_limit_reset(&rlim_spdctrl, 0.0f);
  spdctrl_reset();
  return 0;
}
