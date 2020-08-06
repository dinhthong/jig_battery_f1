/*
 * speed_control.h
 *
 */

#ifndef CONTROL_UTIL_H_
#define CONTROL_UTIL_H_

#include "global_define.h"
#include "utils.h"
float pid_controller(const float speed, const float speed_setpoint, pid_para_t pid_parameters, const float dt);
void spdctrl_reset(void);
int spdctrl_init(void);

#endif /* SPEED_CONTROL_H_ */
