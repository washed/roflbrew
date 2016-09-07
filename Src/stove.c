/*
 * stove.c
 *
 *  Created on: 19.05.2016
 *      Author: washed
 */

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "stove.h"

#ifdef no
STOVE stove0;

const uint32_t stove_powersteps[STOVE_POWERSTEP_COUNT] =
{
		300,
		600,
		800,
		1000,
		1200,
		1300,
		1500,
		1600,
		1800,
		2000
};

void initStove( STOVE* stove_handle )
{
	stove_handle->current_powerstep = STOVE_STARTING_POWERSTEP;
	stove_handle->requested_powerstep = STOVE_STARTING_POWERSTEP;
	stove_handle->power_enabled = 0;
	stove_handle->powermode_enabled = 0;
	stove_handle->wait_time = 0;
	stove_handle->next_op = STOVE_OP_NOP;

	stove_handle->gpio_bank[STOVE_POWER_PIN_INDEX] = STOVE_POWER_PIN_BANK;
	stove_handle->gpio_pin[STOVE_POWER_PIN_INDEX] = STOVE_POWER_PIN;

	stove_handle->gpio_bank[STOVE_POWERMODE_PIN_INDEX] = STOVE_POWERMODE_PIN_BANK;
	stove_handle->gpio_pin[STOVE_POWERMODE_PIN_INDEX] = STOVE_POWERMODE_PIN;

	stove_handle->gpio_bank[STOVE_INCPOWER_PIN_INDEX] = STOVE_INCPOWER_PIN_BANK;
	stove_handle->gpio_pin[STOVE_INCPOWER_PIN_INDEX] = STOVE_INCPOWER_PIN;

	stove_handle->gpio_bank[STOVE_DECPOWER_PIN_INDEX] = STOVE_DECPOWER_PIN_BANK;
	stove_handle->gpio_pin[STOVE_DECPOWER_PIN_INDEX] = STOVE_DECPOWER_PIN;
}

void handleStoveOPs( STOVE* stove_handle )
{
	switch ( stove_handle->next_op )
	{

	case STOVE_OP_PIN0_LOW_STOVE_ON:
		HAL_GPIO_WritePin( STOVE0_GPIO_Port, STOVE0_Pin, GPIO_PIN_RESET );
		stove_handle->power_enabled = 1;
		stove_handle->next_op = STOVE_OP_NOP;
		stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
		break;
	case STOVE_OP_PIN0_LOW_STOVE_OFF:
		HAL_GPIO_WritePin( STOVE0_GPIO_Port, STOVE0_Pin, GPIO_PIN_RESET );
		initStove( stove_handle );
		stove_handle->power_enabled = 0;
		stove_handle->next_op = STOVE_OP_NOP;
		stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
		break;
	case STOVE_OP_PIN0_HIGH_STOVE_ON:
		break;
	case STOVE_OP_PIN0_HIGH_STOVE_OFF:
		break;
	case STOVE_OP_PIN1_LOW:
		HAL_GPIO_WritePin( STOVE1_GPIO_Port, STOVE1_Pin, GPIO_PIN_RESET );
		stove_handle->powermode_enabled = 1;
		stove_handle->next_op = STOVE_OP_NOP;
		stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
		break;
	case STOVE_OP_PIN1_HIGH:
		break;
	case STOVE_OP_PIN2_LOW:
		HAL_GPIO_WritePin( STOVE2_GPIO_Port, STOVE2_Pin, GPIO_PIN_RESET );
		stove_handle->current_powerstep--;
		if ( stove_handle->current_powerstep < 0 )
			stove_handle->current_powerstep = 0;
		stove_handle->next_op = STOVE_OP_NOP;
		stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
		break;
	case STOVE_OP_PIN2_HIGH:
		break;
	case STOVE_OP_PIN3_LOW:
		HAL_GPIO_WritePin( STOVE3_GPIO_Port, STOVE3_Pin, GPIO_PIN_RESET );
		stove_handle->current_powerstep++;
		if ( stove_handle->current_powerstep > STOVE_POWERSTEP_COUNT )
			stove_handle->current_powerstep = STOVE_POWERSTEP_COUNT;
		stove_handle->next_op = STOVE_OP_NOP;
		stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
		break;
	default:
	case STOVE_OP_NOP:
		break;
	}

	stove_handle->handle_op = 0;
}

void switchStoveOn( STOVE* stove_handle )
{
	// ON
	if ( (!stove_handle->power_enabled) && (stove_handle->handle_op == 0) && (stove_handle->next_op == STOVE_OP_NOP) && (stove_handle->wait_time == 0) )
	{
		HAL_GPIO_WritePin( STOVE0_GPIO_Port, STOVE0_Pin, GPIO_PIN_SET );
		stove_handle->next_op = STOVE_OP_PIN0_LOW_STOVE_ON;
		stove_handle->wait_time = STOVE_BUTTON_PRESS_TIME;
	}
}

void switchStoveOff( STOVE* stove_handle )
{
	// OFF
	if ( (stove_handle->power_enabled) && (stove_handle->handle_op == 0) && (stove_handle->next_op == STOVE_OP_NOP) && (stove_handle->wait_time == 0) )
	{
		HAL_GPIO_WritePin( stove_handle->gpio_bank[STOVE_POWER_PIN_INDEX], STOVE0_Pin, GPIO_PIN_SET );
		stove_handle->next_op = STOVE_OP_PIN0_LOW_STOVE_OFF;
		stove_handle->wait_time = STOVE_BUTTON_PRESS_TIME;
	}
}

void switchStovePowerMode( STOVE* stove_handle )
{
	// POWER
	if ( (stove_handle->power_enabled) && (!stove_handle->powermode_enabled) && (stove_handle->handle_op == 0) && (stove_handle->next_op == STOVE_OP_NOP) && (stove_handle->wait_time == 0) )
	{
		HAL_GPIO_WritePin( STOVE1_GPIO_Port, STOVE1_Pin, GPIO_PIN_SET );
		stove_handle->next_op = STOVE_OP_PIN1_LOW;
		stove_handle->wait_time = STOVE_BUTTON_PRESS_TIME;
	}
}

void increaseStovePower( STOVE* stove_handle )
{
	//Powerup
	if ( (stove_handle->powermode_enabled) && (stove_handle->power_enabled) && (stove_handle->handle_op == 0) && (stove_handle->next_op == STOVE_OP_NOP) && (stove_handle->wait_time == 0) )
	{
		HAL_GPIO_WritePin( STOVE3_GPIO_Port, STOVE3_Pin, GPIO_PIN_SET );
		stove_handle->next_op = STOVE_OP_PIN3_LOW;
		stove_handle->wait_time = STOVE_BUTTON_PRESS_TIME;
	}
}

void decreaseStovePower( STOVE* stove_handle )
{
	//Powerdown
	if ( (stove_handle->powermode_enabled) && (stove_handle->power_enabled) && (stove_handle->handle_op == 0) && (stove_handle->next_op == STOVE_OP_NOP) && (stove_handle->wait_time == 0) )
	{
		HAL_GPIO_WritePin( STOVE2_GPIO_Port, STOVE2_Pin, GPIO_PIN_SET );
		stove_handle->next_op = STOVE_OP_PIN2_LOW;
		stove_handle->wait_time = STOVE_BUTTON_PRESS_TIME;
	}
}

void setStovePower( STOVE* stove_handle )
{
	if ( stove_handle->current_powerstep == stove_handle->requested_powerstep )
		return;

	if ( (stove_handle->requested_powerstep < 0) || (stove_handle->requested_powerstep > STOVE_POWERSTEP_COUNT) )
		return;


	if ( stove_handle->current_powerstep > stove_handle->requested_powerstep )
		decreaseStovePower( stove_handle );
	else if ( stove_handle->current_powerstep < stove_handle->requested_powerstep )
		increaseStovePower( stove_handle );

}
#endif
