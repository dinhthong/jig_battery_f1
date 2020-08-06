/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#define EN_MOSFET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
#define DIS_MOSFET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

#define MANUAL 0
#define SQRT3			( 1.732050807568877f )

#define MICROS() TIM5->CNT
typedef enum 
{
	CHARGE,
	DISCHARGE,
	NONE
} State;
typedef struct
{
	float battery_V_1,battery_V_2,battery_V_3,battery_V_4,battery_V_5;
	float I_out_1,I_out_2,I_out_3,I_out_4,I_out_5;
}Measure_value;
typedef struct
{
	float Buck1,Buck2,Buck3,Buck4,Buck5;
	float Vgs1,Vgs2,Vgs3,Vgs4,Vgs5;
} Control_value;
