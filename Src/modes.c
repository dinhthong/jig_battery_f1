/*
 * modes.c
 */
#include "global_define.h"
#include "utils.h"
#include "modes.h"
#include "board.h"
#include "charger_system.h"
#define SOFT_RUN 1

static uint8_t manual_button;

modes_mode_t system_state_mode = INACTIVE;

int modes_init(void)
{
  system_state_mode = INACTIVE;

  return 0;
}

uint32_t time_offset = 0;

/*
check current 'mode' variable and perform corresponding function.
*/
extern uint8_t batt_detect;
float w_initial_battery_voltage;
//extern float voltage_pid_integral;
#define ACS_CALIBRATION_SIZE 500
//static uint8_t i;
float acs_adc_offset=2091.0f;
extern uint16_t adcVal[3];
static recursive_mean_t rmean_adc_acs712;
extern float w_battery_voltage_while_charging;
void modes_step(uint32_t period_ms)
{
		
		const uint32_t button_hold_time_ms = 1200u;
		static uint32_t delay_ms = 0u;
		switch (system_state_mode) {
    /*
      Check button to switch from INACTIVE to ACTIVE mode.
			Or turn the 'manual_button' = 1 by software
    */
    case INACTIVE:
      if (manual_button) {
        delay_ms += period_ms;
        if (delay_ms >= button_hold_time_ms) {
					system_state_mode = BOARD_CALIBRATION;
					delay_ms = 0u;
        }
      } else {
        delay_ms = 0u;
      }
      break;
      case BOARD_CALIBRATION:
				/*
						Calib the current sensor offset
				*/
				DIS_MOSFET;
				acs_adc_offset = recursive_mean_add(&rmean_adc_acs712, (float)adcVal[2]);
				if (recursive_mean_samples(&rmean_adc_acs712) >= 100) {
					system_state_mode = CHECK_BATT;
				}
				break;
		case CHECK_BATT:
			/*
				read get_battery_voltage
				wait until detect any voltage >2.0v
				if the voltage is in chargable range, switch to RUNNNING = CHARGING state
				always show current status
				or else we can put the detect battery as a periodic task, if the user press charge button but battery is not present or 
				in a bad state -> output a message, abort the charge request.
			*/
			// if battery present
			if (batt_detect) {
				// get battery's initial voltage for control algorithm
				w_initial_battery_voltage = get_battery_voltage();
				// start running charging algorithm
				system_state_mode = RUNNING;
				//voltage_pid_integral = 1400.0f;
				EN_MOSFET;
				/**/
				w_battery_voltage_while_charging = w_initial_battery_voltage;
				set_pid_ctrl_para();
			}
			else {
				system_state_mode = INACTIVE;
				// force manual var to zero	
				manual_button = 0;
			}
      break;
				
					
    case RUNNING:

			//charge_battery_control();
			
      break;

    case MANUAL_STEP:
      if (manual_button) {
        delay_ms += period_ms;
        if (delay_ms >= button_hold_time_ms) {
          system_state_mode = OPEN_LOOP;
        }
      } else {
        delay_ms = 0u;
      }
      break;

    default:
      break; 
  }
}

modes_mode_t modes_current_mode(void)
{
  return system_state_mode;
}

void set_mode_inactive(void) {
    system_state_mode = INACTIVE;
}

uint32_t get_time_offset(void){
	return time_offset;
}
