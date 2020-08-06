/*
 * board.h
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "stm32f1xx_hal.h"

#define V_REF 2.959f

/*
	These values are very good to measure real voltage.
*/

#define R_BUCK_DOWN 978.0f
//#define R_BUCK_UP 3257.0f
#define R_BUCK_UP 3260.0f

#define R_BATT_DOWN 1009.0f
#define R_BATT_UP 3270.0f

#define BUCK_OUT_SIZE 20
#define BATT_OUT_SIZE 20
#define ADS_OUT_SIZE 150
#define ADC_CURRENT_SIZE 200

//float board_adc_to_buck_volt(const uint16_t adc_buck_reading);
//float get_battery_voltage(const uint16_t adc_battery_reading);
//float convert_reading_to_charge_current(const uint16_t adc_batt_current);
//void set_buck_dac_control(uint16_t buck_dac_val);
void board_step(const uint32_t period_ms);
float get_charge_current(void);
float get_battery_voltage(void);
float convert_reading_to_charge_current(const uint16_t adc_acs712_reading, const uint16_t adc_acs712_offset);
float board_adc_to_battery_volt(const uint16_t adc_battery_reading);
void calculate_battery_volt_while_charing(void);
float get_battery_volt_while_charing(void);
#endif /* BOARD_H_ */
