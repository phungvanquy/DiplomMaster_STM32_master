/*
 * servo.h
 *
 *  Created on: Apr 13, 2022
 *      Author: Phung Van Quy
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"

void Servo_Write(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t angle);

#endif /* INC_SERVO_H_ */
