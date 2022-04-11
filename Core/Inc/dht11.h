/*
 * dht11.h
 *
 *  Created on: Apr 11, 2022
 *      Author: Phung Van Quy
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#define DHT11_PORT			GPIOB
#define DHT11_PIN			GPIO_PIN_0

#include "main.h"
#include "stm32f4xx_hal.h"

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypeDef;


void delay_us(uint32_t us);

void delay_ms(uint32_t ms);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void DHT11_Start(void);

uint8_t DHT11_Check_Response(void);

uint8_t DHT11_Read(void);

void DHT11_GetData(DHT_DataTypeDef *DHT_Data);

#endif /* INC_DHT11_H_ */
