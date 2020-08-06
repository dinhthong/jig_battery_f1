/*
 */
#include "global_define.h"
#include "charger_system.h"
#include "utils.h"
#include "board.h"
#include "modes.h"
#include "main.h"
#define CHARGE_CONTROL_SIZE 10
#define STOP_CHARGING_CURRENT 200.0f

//uint16_t set_charge_dac_batt(uint16_t pre_control_dac, uint8_t up_down);
//static void system_state_step(void);
void charge_battery_control(const uint32_t period_ms);
static float charge_current_control(float error, float dt);
static float charge_voltage_control(float error, float dt);

//TIM_HandleTypeDef htim3;

/*
Timer 5 microsecond counter
*/

void Initial_System_Timer(void)
{
//	RCC->APB1ENR |= 0x0008;	
//	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
//	TIM5->CR2 = 0x0000;
//	TIM5->CNT = 0x0000;
//	TIM5->ARR = 0xFFFFFFFF;
//	TIM5->PSC = 84 - 1;	//1us
//	TIM5->EGR = 0x0001;
//	TIM5->CR1 |= 0x0001; // Enable
}

void one_ms_tim3_init(void)
{
//	__TIM3_CLK_ENABLE();
//	TIM_ClockConfigTypeDef sClockSourceConfig;
//	TIM_MasterConfigTypeDef sMasterConfig;

//	htim3.Instance = TIM3;
//	htim3.Init.Prescaler = ((SystemCoreClock/2)/1000000)-1;
//	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim3.Init.Period = 1000-1;
//	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//	{
//	//	Error_Handler();
//	}

//	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//	{
//	//	 Error_Handler();
//	}

//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//	{
//	//	Error_Handler();
//	}
//	HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
//	HAL_NVIC_EnableIRQ(TIM3_IRQn);
//	HAL_TIM_Base_Start_IT(&htim3);
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance==htim3.Instance) {
//			system_state_step();
//	}
//}
#define BATTERY_READ_PERIOD 30000
uint8_t manual_reset;
uint8_t battery_read_while_charging_flag;
void system_state_step(void) {
	const uint32_t modes_period_ms    = 8;
//	const uint32_t inverter_period_ms = 4;
	const uint32_t board_period_ms    = 2;
	const uint32_t control_period_ms  = 4;
	static uint16_t tim3_int_count, battery_read_while_charging_cnt;
	static uint8_t task_ticker;

	if (manual_reset) {
		NVIC_SystemReset();
	}
	if (0u == (task_ticker % board_period_ms)) {
			board_step(board_period_ms);
	}
	
	if ((task_ticker % modes_period_ms) == 0) {
			modes_step(modes_period_ms);
	}
	/*
	
	*/
	if ((task_ticker % control_period_ms) == 0) {
			charge_battery_control(control_period_ms);
	}
	
	task_ticker++;
	
	/*
		Blink red led 
	*/
	tim3_int_count++;
	if (tim3_int_count >= 250) {
		tim3_int_count = 0;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	}
	
	if (modes_current_mode() == RUNNING) {
		battery_read_while_charging_cnt++;
		if (battery_read_while_charging_cnt >= BATTERY_READ_PERIOD) {
			battery_read_while_charging_cnt = 0;
			DIS_MOSFET;
			battery_read_while_charging_flag = 1;
		}
	}
}

uint8_t cc_cv_mode;
#define CHARGE_VOLT_SP_MIN 6.89f 
#define CHARGE_VOLT_SP_MAX 8.55f
static float _charge_current_sp, _charge_voltage_sp = 7.7f;
extern float m_buck_output_voltage, m_battery_voltage, m_charge_current;
/*
	Initial DAC value, buck_out = 8.4V
*/
//static uint16_t actual_dac = 1345;

static float w_feedback_current, w_feedback_battery_voltage;
static float w_feedback_current_buffer, w_feedback_battery_voltage_buffer;
extern float w_initial_battery_voltage;
static pid_para_t charge_current_pid, charge_voltage_pid;

void set_pid_ctrl_para(void) {
	charge_current_pid.kp = 0.6f;
	charge_current_pid.ki = 0.0f;
	charge_current_pid.kd = 0.0f;
	charge_voltage_pid.kp = 2.5f;
	charge_voltage_pid.ki = 36.8f;
	charge_voltage_pid.kd = 0.0f;
}
static float w_charge_current_pid_output, w_charge_voltage_error, w_charge_current_error;
static float control_dt, w_control_dac_out;
#define BATTERY_MEASURE_FREQUENCY 10
float w_charge_voltage_pid_out;
void charge_battery_control(const uint32_t period_ms) {
		static uint8_t control_cnt;
		if (modes_current_mode()==RUNNING) {
			/*
				second stage filter.
			*/
			w_feedback_current_buffer+=get_charge_current();
			w_feedback_battery_voltage_buffer+=get_battery_voltage();
			control_cnt++;
			if (control_cnt >= CHARGE_CONTROL_SIZE) {
					control_cnt = 0;
					w_feedback_current = w_feedback_current_buffer/CHARGE_CONTROL_SIZE;
					w_feedback_battery_voltage = w_feedback_battery_voltage_buffer/CHARGE_CONTROL_SIZE;
					float charge_current_error;
					static uint32_t pre_control_cnt;
					
					switch (cc_cv_mode) {
					case 0:
						/*
							At constant current mode.
							Change batt_voltage accordingly to error current, thus output control command to DAC channel.
							If the maximum charging voltage is reached but setpoint current won't change, that's maximum charging current.
							If the actual battery is stable at 8.4V, switch to constant voltage mode
						*/
						w_charge_current_error = _charge_current_sp - w_feedback_current;
						
//						control_dt = (MICROS()-pre_control_cnt)*0.000001f;
//						pre_control_cnt = MICROS();
						w_charge_current_pid_output = charge_current_control(w_charge_current_error, control_dt);
						_charge_voltage_sp = saturatef(_charge_voltage_sp, CHARGE_VOLT_SP_MIN, CHARGE_VOLT_SP_MAX);
						/*
								@todo: add rate limit for battery's charging setpoint.
					
						*/
						w_charge_voltage_error = _charge_voltage_sp - w_feedback_battery_voltage;
						w_charge_voltage_error = -w_charge_voltage_error;
						
						w_charge_voltage_pid_out = charge_voltage_control(w_charge_voltage_error, control_dt);
						
						w_control_dac_out = saturatef(w_charge_voltage_pid_out + 1450.0f, BUCK_DAC_MIN, BUCK_DAC_MAX);

						/*
								pid current_error to voltage setpoint
						*/

						write_DAC_1((uint16_t)w_control_dac_out);
//						if (get_battery_volt_while_charing() <= 4.395f) {
//							/*
//								Battery is removed
//							*/
//							cc_cv_mode = 2;
//						}
//						if (get_battery_volt_while_charing() >= 8.395f) {
//							_charge_voltage_sp = 8.45f;
//							cc_cv_mode = 1;
//						}
						
						break;
					case 1:
						/*
							Control voltage to stable at 8.4V, if the charge current is equal or lesser than 0.1C, stop charging process.
						*/
//						control_dt = (MICROS()-pre_control_cnt)*0.000001f;
			//			pre_control_cnt = MICROS();
						w_charge_voltage_error = _charge_voltage_sp - w_feedback_battery_voltage;
						w_charge_voltage_error = -w_charge_voltage_error;
						w_charge_voltage_pid_out = charge_voltage_control(w_charge_voltage_error, control_dt);
						w_control_dac_out = saturatef(w_charge_voltage_pid_out + 1450.0f, BUCK_DAC_MIN, BUCK_DAC_MAX);
						write_DAC_1((uint16_t)w_control_dac_out);
						if (w_feedback_current <= STOP_CHARGING_CURRENT) {
							cc_cv_mode = 2;
						}
						break;
					case 2:
						/*
							Battery is regarded as fully charged.
							
						*/
						cc_cv_mode = 0;
						DIS_MOSFET;
						set_mode_inactive();
						break;
					default:
						break;
				}

				w_feedback_current_buffer = 0.0f;
				w_feedback_battery_voltage_buffer = 0.0f;
			}
	}
}

/*
	allow setting dac if that value won't make the output higher than 8.4 volt
	up volt -> increase batt volt -> decrease DAC val
	@return: new dac value to output DAC
*/
//uint16_t actual_lm_dac;

//uint16_t set_charge_dac_batt(uint16_t pre_control_dac, uint8_t up_down) {
//	uint16_t temp_new_dac;

//	if (get_battery_voltage()>=8.5f) {
//		up_down = 1;
//	}
//	if (get_battery_voltage()<=6.4f) {
//		up_down = 0;
//	}
//	// increase volt
//	if (up_down == 0) {
//			/*
//				decrease dac output to 1 unit
//			*/
//			temp_new_dac =pre_control_dac-1;
//		
//	}
//	else {
//		// lower limit
//			temp_new_dac = pre_control_dac+1;
//	}

//	/*
//	Saturate output 
//	*/
//	return saturate_u16(temp_new_dac, BUCK_DAC_MIN, BUCK_DAC_MAX);
//}
#define CHARGE_CURRENT_I_MAX 1.5f
static float current_pid_integral;
float charge_current_control(float error, float dt) {
	static float pre_error;
	#if 1
	float temp;
	if (dt>1.0f) {
		dt = 0.04f;
	}		
	current_pid_integral+=charge_current_pid.ki*error*dt;
	current_pid_integral = saturatef(current_pid_integral, -CHARGE_CURRENT_I_MAX, CHARGE_CURRENT_I_MAX);
	temp = charge_current_pid.kp*error + current_pid_integral + charge_current_pid.kp*(error-pre_error)/dt;
	pre_error = error;
	return temp;
	#else
	temp = 0.0f;
	#endif
	
}
#define CHARGE_CONTROL_I_MAX 400.0f
static float voltage_pid_integral;
static float charge_voltage_control(float error, float dt) {
	static float pre_error;
	#if 1
	float temp;
	if (dt>1.0f) {
		dt = 0.04f;
	}		
	voltage_pid_integral += charge_voltage_pid.ki*error*dt;
	voltage_pid_integral = saturatef(voltage_pid_integral, -CHARGE_CONTROL_I_MAX, CHARGE_CONTROL_I_MAX);
	temp = charge_voltage_pid.kp*error + voltage_pid_integral + charge_voltage_pid.kp*(error-pre_error)/dt;
	pre_error = error;
	return temp;
	#else
	return 0;
	#endif
}
