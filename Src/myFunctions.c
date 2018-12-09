/*
 * myFunctions.c
 *
 *  Created on: 2.11.2018
 *      Author: Santtu
 */
#ifndef FUNC_C
#define FUNC_C

#include "variables.h"
uint8_t channel = 0;
uint8_t channel_checker = 0;

/*
 *
 *
 *  */
void Output_Write(uint8_t channel, uint8_t state)
{
	switch(channel)
	{
		case 1:
			HAL_GPIO_WritePin(O1_GPIO_Port, O1_Pin, state);
		break;
		case 2:
			HAL_GPIO_WritePin(O2_GPIO_Port, O2_Pin, state);
		break;
		case 3:
			HAL_GPIO_WritePin(O3_GPIO_Port, O3_Pin, state);
		break;
		case 4:
			HAL_GPIO_WritePin(O4_GPIO_Port, O4_Pin, state);
		break;
		case 5:
			HAL_GPIO_WritePin(O5_GPIO_Port, O5_Pin, state);
		break;
		case 6:
			HAL_GPIO_WritePin(O6_GPIO_Port, O6_Pin, state);
		break;
		case 7:
			HAL_GPIO_WritePin(O7_GPIO_Port, O7_Pin, state);
		break;
		default:
			// error message
		break;

	}
}

void Output_Toggle(uint8_t channel)
{
	switch(channel)
	{
		case 1:
			HAL_GPIO_TogglePin(O1_GPIO_Port, O1_Pin);
		break;
		case 2:
			HAL_GPIO_TogglePin(O2_GPIO_Port, O2_Pin);
		break;
		case 3:
			HAL_GPIO_TogglePin(O3_GPIO_Port, O3_Pin);
		break;
		case 4:
			HAL_GPIO_TogglePin(O4_GPIO_Port, O4_Pin);
		break;
		case 5:
			HAL_GPIO_TogglePin(O5_GPIO_Port, O5_Pin);
		break;
		case 6:
			HAL_GPIO_TogglePin(O6_GPIO_Port, O6_Pin);
		break;
		case 7:
			HAL_GPIO_TogglePin(O7_GPIO_Port, O7_Pin);
		break;
		default:
			// error message
		break;

	}
}


int PressedButton()
{
	uint8_t temp_channel;
	temp_channel = Input();
	// tarkista nappi/napit painettuna >300ms
	while(temp_channel == Input() && tick.Value < 500)
	{
		tick.Stop = HAL_GetTick();
		tick.Value = tick.Stop - tick.Start;
	}
	if(tick.Value > 490)
	{
		tick.Value = 0;
		return temp_channel;
	}
	else
	{
		tick.Value = 0;
		CDC_Transmit_FS("Press button longer than 500ms\n",31);
		return 0;
	}

}

// Switch case for channels
void doStuff(uint8_t input_code)
{
	switch(input_code)
	{
		case 1:

		break;
		case 2:

		break;
		case 3:

		break;
		case 4:

		break;
		case 5:

		break;
		case 6:

		break;
		case 7:

		break;
		case 8:

		break;
		case 9:

		break;
		case 10:

		break;
		case 11:

		break;
		case 12:

		break;
	}
}

int Input()
{

		channel = 0;
		if(HAL_GPIO_ReadPin(I1_GPIO_Port, I1_Pin))
		{
			if(HAL_GPIO_ReadPin(I1_GPIO_Port, I1_Pin))
				channel += 1;

		}
		if(HAL_GPIO_ReadPin(I2_GPIO_Port, I2_Pin))
		{
			if(HAL_GPIO_ReadPin(I2_GPIO_Port, I2_Pin))
				channel += 2;

		}
		if(HAL_GPIO_ReadPin(I3_GPIO_Port, I3_Pin))
		{
			if(HAL_GPIO_ReadPin(I3_GPIO_Port, I3_Pin))
				channel += 4;

		}
		if(HAL_GPIO_ReadPin(I4_GPIO_Port, I4_Pin))
		{
			if(HAL_GPIO_ReadPin(I4_GPIO_Port, I4_Pin))
				channel += 8;

		}
		if(HAL_GPIO_ReadPin(I5_GPIO_Port, I5_Pin))
		{
			if(HAL_GPIO_ReadPin(I5_GPIO_Port, I5_Pin))
				channel += 16;

		}
		if(HAL_GPIO_ReadPin(I6_GPIO_Port, I6_Pin))
		{
			if(HAL_GPIO_ReadPin(I6_GPIO_Port, I6_Pin))
				channel += 32;

		}
		if(HAL_GPIO_ReadPin(I7_GPIO_Port, I7_Pin))
		{
			if(HAL_GPIO_ReadPin(I7_GPIO_Port, I7_Pin))
				channel += 64;
		}

	return channel;
}


/*  Functions to control outputs */

// timer 1ms steps
void timer(uint16_t time)
{
	tick.Value = 0;
	tick.Start = HAL_GetTick();					// returns at the moment time
	while(tick.Value < time)
	{
		tick.Stop = HAL_GetTick();
		tick.Value = tick.Stop - tick.Start;	// calculate milliseconds
	}
}

// Pulse 0-65000ms
void Pulse(uint8_t channel, uint16_t pulse_time)
{
	tick.Value = 0;
	Output_Write(channel,1);
	timer(pulse_time);
	Output_Write(channel,0);
}


#endif
