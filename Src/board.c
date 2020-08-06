#include "board.h"
#include "utils.h"
#include "global_define.h"

#define ACS_SENSITIVITY 0.185f;

/*
	Correct acs712 5A current sensor, need testing to find the best formula for current measurement.
		+ Linear regression
		+ Calibration (auto, manual, calib through software,...)
*/
float adc_get_voltage_raw(const uint16_t adc_val) {
	return (float)adc_val*V_REF/4095.0f;
}

float convert_reading_to_charge_current(const uint16_t adc_acs712_reading, const uint16_t adc_acs712_offset) {
		float ads_current_v;
		ads_current_v = adc_acs712_reading - adc_acs712_offset;
		//float acs712_voltage = adc_get_voltage_raw(adc_batt_current)-(V_REF/2.0f + adc_get_voltage_raw(acs712_raw_offset));
		//float acs712_voltage;
		ads_current_v = adc_get_voltage_raw(ads_current_v);
		//return acs712_voltage/ACS_SENSITIVITY;
	//	return (float)1000.0f*ads_current_v*0.002745f;
	return (float)1000.0f*2.09f*ads_current_v/ACS_SENSITIVITY;
}


static float board_adc_to_buck_volt(const uint16_t adc_buck_reading) {
	return adc_get_voltage_raw(adc_buck_reading)*(R_BUCK_UP+R_BUCK_DOWN)/R_BUCK_DOWN;
}

float board_adc_to_battery_volt(const uint16_t adc_battery_reading) {
	return adc_get_voltage_raw(adc_battery_reading)*(R_BATT_UP+R_BATT_DOWN)/R_BATT_DOWN;
}

//extern DAC_HandleTypeDef hdac;
static uint16_t w_actual_buck_dac;
//void set_buck_dac_control(uint16_t buck_dac_val) {
//		w_actual_buck_dac = buck_dac_val;
//}

extern uint16_t adcVal[3];

static float m_buck_output_voltage, m_battery_voltage, m_charge_current, adc_current_avr;
static float buck_output_buffer[BUCK_OUT_SIZE+1], battery_volt_buffer[BATT_OUT_SIZE+1], adc_current_buff[ADC_CURRENT_SIZE+1];

/*
	Update sensor readings
*/
uint8_t pre_batt_present, batt_present;
uint8_t batt_detect;

//extern float acs_adc_offset;
#include "modes.h"
uint8_t read_battery_volt;
extern float acs_adc_offset;
void board_step(const uint32_t period_ms) {
		(void) period_ms;
		static uint8_t buck_out_v_cnt, batt_v_cnt, ads_out_cnt, adc_current_cnt;
		m_buck_output_voltage = average_filter(buck_output_buffer, BUCK_OUT_SIZE, board_adc_to_buck_volt(adcVal[0]), &buck_out_v_cnt);
		m_battery_voltage = average_filter(battery_volt_buffer, BATT_OUT_SIZE, board_adc_to_battery_volt(adcVal[1]), &batt_v_cnt);
		adc_current_avr = average_filter(adc_current_buff, ADC_CURRENT_SIZE, adcVal[2], &adc_current_cnt);
		m_charge_current = convert_reading_to_charge_current(adc_current_avr, acs_adc_offset);
	
		/*
			Check battery state.
			Only check when BJT switch is off
		*/
			
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==0) {
		if (m_battery_voltage > 4.0f) {
			batt_present = 1;
		}
		else {
			batt_present = 0;
		}
		if (batt_present != pre_batt_present) {
			if (batt_present) {
				batt_detect = 1;
				pre_batt_present = 1;
			}
			if (!batt_present) {
				batt_detect = 0;
				pre_batt_present = 0;
			}
		}
		}
}
/*
	return current in miliamperes
*/
float get_charge_current(void) {
	 return m_charge_current;
}

float get_battery_voltage(void) {
	 return m_battery_voltage;
}
//extern uint8_t battery_read_while_charging_flag;
float w_dma_battery_voltage_while_charing_adc, w_battery_voltage_while_charging;
static recursive_mean_t rmean_dma_battery_voltage_while_charing_adc;

float get_battery_volt_while_charing(void) {
	return w_battery_voltage_while_charging;
}