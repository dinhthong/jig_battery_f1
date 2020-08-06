/*
 * modes.h
 */

#ifndef MODES_H_
#define MODES_H_

#include "stm32f1xx_hal.h"

typedef enum {
  INACTIVE = 0u,
  RUNNING,
  MANUAL_STEP,
  OPEN_LOOP,
	BOARD_CALIBRATION,
	CHECK_BATT
} modes_mode_t;

int modes_init(void);
void modes_step(uint32_t period_ms);
modes_mode_t modes_current_mode(void);
//void GUI_request_mode(uint8_t mode);
void set_mode_inactive(void);
uint32_t get_time_offset(void);

#endif /* MODES_H_ */
