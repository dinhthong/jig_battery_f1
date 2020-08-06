/*
 * 
 */

#ifndef CHARGER_SYSTEM_H_
#define CHARGER_SYSTEM_H_

#include "stm32f1xx_hal.h"
#define BUCK_DAC_MAX 1900
#define BUCK_DAC_MIN 900
void Initial_System_Timer(void);
void one_ms_tim3_init(void);
void set_pid_ctrl_para(void);
void system_state_step(void);
#endif
