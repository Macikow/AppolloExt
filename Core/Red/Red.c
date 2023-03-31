/*
 * Red.c
 *
 *  Created on: Mar 27, 2023
 *      Author: MaciejKowalczyk
 */

#include "main.h"
#include "Red.h"

typedef uint16_t analog_value_t;


red_ouput_test_activ_t red_output_active;

typedef enum
{
	reg_error,
	reg_ack

}register_status_t;

typedef enum output_state_e
{
	turned_off,
	turned_on
}output_state_t;


struct red_inputs
{
	analog_value_t red1_loop;
	analog_value_t red2_loop;
	analog_value_t red3_loop;
}red_inputs_s;

struct red_outputs
{
	output_state_t red_1_power_ctrl;
	output_state_t red_reset_ctrl;
	output_state_t red_2_power_ctrl;
	output_state_t red_3_power_ctrl;
}red_outputs_s;


register_status_t update_registes(void)
{
	if(red_outputs_s.red_1_power_ctrl == turned_on) RED_1_POWER_CTRL_GPIO_Port->ODR |= RED_1_POWER_CTRL_Pin;
		else RED_1_POWER_CTRL_GPIO_Port->ODR &= ~RED_1_POWER_CTRL_Pin;

	if(red_outputs_s.red_2_power_ctrl == turned_on) RED_2_POWER_CTRL_GPIO_Port->ODR |= RED_2_POWER_CTRL_Pin;
		else RED_1_POWER_CTRL_GPIO_Port->ODR &= ~RED_2_POWER_CTRL_Pin;

	if(red_outputs_s.red_3_power_ctrl == turned_on) RED_3_POWER_CTRL_GPIO_Port->ODR |= RED_3_POWER_CTRL_Pin;
		else RED_1_POWER_CTRL_GPIO_Port->ODR &= ~RED_3_POWER_CTRL_Pin;

	if(red_outputs_s.red_reset_ctrl == turned_on) RED_RESET_CTRL_GPIO_Port->ODR |= RED_RESET_CTRL_Pin;
		else RED_1_POWER_CTRL_GPIO_Port->ODR &= ~RED_RESET_CTRL_Pin;

	// check if ouput is corrected setup

	if( (red_outputs_s.red_1_power_ctrl == turned_on) != 	( (RED_1_POWER_CTRL_GPIO_Port->ODR & RED_1_POWER_CTRL_Pin) > 0) )	return reg_error;
	if( (red_outputs_s.red_2_power_ctrl == turned_on) != 	( (RED_2_POWER_CTRL_GPIO_Port->ODR & RED_2_POWER_CTRL_Pin) > 0) )	return reg_error;
	if( (red_outputs_s.red_3_power_ctrl == turned_on) != 	( (RED_3_POWER_CTRL_GPIO_Port->ODR & RED_3_POWER_CTRL_Pin) > 0) )	return reg_error;
	if( (red_outputs_s.red_reset_ctrl == turned_on) 	!= 	( (RED_RESET_CTRL_GPIO_Port->ODR & RED_RESET_CTRL_Pin) > 0) )		return reg_error;


	return reg_ack;
}

/*
 * function is called from main with timer event - each 1s
 *
 */
void red_output_test_handler()
{
	static uint8_t red_ouput_status_cnt = 0; // counter is incremented each time that fuc is called

	if(red_output_active == active)
	{
		switch(red_ouput_status_cnt)
		{
		case 0:
			red_outputs_s.red_1_power_ctrl = turned_on;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_off;
			red_outputs_s.red_reset_ctrl = turned_off;
			red_ouput_status_cnt++;
			break;
		case 1:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_on;
			red_outputs_s.red_3_power_ctrl = turned_off;
			red_outputs_s.red_reset_ctrl = turned_off;
			red_ouput_status_cnt++;
			break;
		case 2:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_on;
			red_outputs_s.red_reset_ctrl = turned_off;
			red_ouput_status_cnt++;
			break;
		case 3:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_off;
			red_outputs_s.red_reset_ctrl = turned_on;
			red_ouput_status_cnt++;
			break;
		case 4:
			red_outputs_s.red_1_power_ctrl = turned_on;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_on;
			red_outputs_s.red_reset_ctrl = turned_off;
			red_ouput_status_cnt++;
			break;
		case 5:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_on;
			red_outputs_s.red_3_power_ctrl = turned_off;
			red_outputs_s.red_reset_ctrl = turned_on;
			red_ouput_status_cnt++;
			break;
		case 6:
			red_outputs_s.red_1_power_ctrl = turned_on;
			red_outputs_s.red_2_power_ctrl = turned_on;
			red_outputs_s.red_3_power_ctrl = turned_off;
			red_outputs_s.red_reset_ctrl = turned_off;
			red_ouput_status_cnt++;
			break;
		case 7:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_on;
			red_outputs_s.red_reset_ctrl = turned_on;
			red_ouput_status_cnt++;
			break;
		case 8:
			red_outputs_s.red_1_power_ctrl = turned_off;
			red_outputs_s.red_2_power_ctrl = turned_off;
			red_outputs_s.red_3_power_ctrl = turned_on;
			red_outputs_s.red_reset_ctrl = turned_on;
			red_ouput_status_cnt=0;
			break;
		}
	}


	if( update_registes() == reg_error)
	{
		//TODO printf on terminal we have an error
		while (1);
	}

}
