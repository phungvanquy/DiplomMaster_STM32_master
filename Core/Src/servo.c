/*
 * servo.c
 *
 *  Created on: Apr 13, 2022
 *      Author: Phung Van Quy
 */

#include "servo.h"
#define MIN_PULSE_WIDTH	544
#define MAX_PULSE_WIDTH 2400
uint32_t Map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Servo_Write(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t angle)
{
	uint32_t ccr = Map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	switch(Channel)
	{
		case TIM_CHANNEL_1:
			htim->Instance->CCR1 = ccr;
			break;
		case TIM_CHANNEL_2:
			htim->Instance->CCR2 = ccr;
			break;
		case TIM_CHANNEL_3:
			htim->Instance->CCR3 = ccr;
			break;
		case TIM_CHANNEL_4:
			htim->Instance->CCR4 = ccr;
			break;
	}
	HAL_TIM_PWM_Start(htim, Channel);
}

