/*
 * dht11.c
 *
 *  Created on: Apr 11, 2022
 *      Author: Phung Van Quy
 */

#include "dht11.h"

DHT_DataTypeDef DHT_Data;

extern TIM_HandleTypeDef htim10;

void delay_us(uint32_t us)
{
	__HAL_TIM_SetCounter(&htim10, 0);
	HAL_TIM_Base_Start(&htim10);
	while(__HAL_TIM_GetCounter(&htim10) < us);
	HAL_TIM_Base_Stop(&htim10);
}
void delay_ms(uint32_t ms)
{
	uint32_t i;
	for(i=0; i<ms; i++)
	{
		delay_us(1000);
	}
}


//volatile float Temperature = 0;
//volatile float Humidity = 0;

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void)
{
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	delay_us(20);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void)
{
	uint8_t Response = 0;
	delay_us(40);
	if(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
	{
		delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
		{
			Response = 1;
		}
		else
		{
			Response = 0;
		}
	}
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));

	return Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i,j;
	for(j=0; j<8; j++)
	{
		while(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
		delay_us(40);
		if(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
		{
			i &= ~(1<<(7-j));			// write 0
		}
		else
		{
			i |= (1<<(7-j));			// if the pin is high, write 1
			while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));		// wait for the pin to go low
		}
	}
	return i;
}

void DHT11_GetData(DHT_DataTypeDef *DHT_Data)
{
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint16_t SUM, RH, TEMP;

	DHT11_Start();
	if(DHT11_Check_Response())
	{
		Rh_byte1 = DHT11_Read();
		Rh_byte2 = DHT11_Read();
		Temp_byte1 = DHT11_Read();
		Temp_byte2 = DHT11_Read();
		SUM = DHT11_Read();
	}

	if(SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
	{
		DHT_Data->Temperature = Temp_byte1;
		DHT_Data->Humidity = Rh_byte1;
	}
}
