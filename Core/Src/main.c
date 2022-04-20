/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "MFRC522.h"
#include "dht11.h"
#include "servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// type of data to send to server
#define Sensor_DATA 5
#define CardID_DATA 6

// type of command received from server
#define SCAN_CARDID 1
#define TOGGLE_LED  2
#define SENSOR_MEASURING 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for CmdParsing */
osThreadId_t CmdParsingHandle;
const osThreadAttr_t CmdParsing_attributes = {
  .name = "CmdParsing",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ToggleLed */
osThreadId_t ToggleLedHandle;
const osThreadAttr_t ToggleLed_attributes = {
  .name = "ToggleLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ScanCard */
osThreadId_t ScanCardHandle;
const osThreadAttr_t ScanCard_attributes = {
  .name = "ScanCard",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorMeasuring */
osThreadId_t SensorMeasuringHandle;
const osThreadAttr_t SensorMeasuring_attributes = {
  .name = "SensorMeasuring",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendCardID */
osThreadId_t SendCardIDHandle;
const osThreadAttr_t SendCardID_attributes = {
  .name = "SendCardID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendSensorData */
osThreadId_t SendSensorDataHandle;
const osThreadAttr_t SendSensorData_attributes = {
  .name = "SendSensorData",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoControl */
osThreadId_t ServoControlHandle;
const osThreadAttr_t ServoControl_attributes = {
  .name = "ServoControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for cardID_Queue */
osMessageQueueId_t cardID_QueueHandle;
const osMessageQueueAttr_t cardID_Queue_attributes = {
  .name = "cardID_Queue"
};
/* Definitions for sensorData_Queue */
osMessageQueueId_t sensorData_QueueHandle;
const osMessageQueueAttr_t sensorData_Queue_attributes = {
  .name = "sensorData_Queue"
};
/* Definitions for mutex_uart2 */
osMutexId_t mutex_uart2Handle;
const osMutexAttr_t mutex_uart2_attributes = {
  .name = "mutex_uart2"
};
/* Definitions for mutex_uart6 */
osMutexId_t mutex_uart6Handle;
const osMutexAttr_t mutex_uart6_attributes = {
  .name = "mutex_uart6"
};
/* Definitions for scanCardAvailable */
osEventFlagsId_t scanCardAvailableHandle;
const osEventFlagsAttr_t scanCardAvailable_attributes = {
  .name = "scanCardAvailable"
};
/* Definitions for parsingCMDAvailable */
osEventFlagsId_t parsingCMDAvailableHandle;
const osEventFlagsAttr_t parsingCMDAvailable_attributes = {
  .name = "parsingCMDAvailable"
};
/* Definitions for servoControlIsAvailable */
osEventFlagsId_t servoControlIsAvailableHandle;
const osEventFlagsAttr_t servoControlIsAvailable_attributes = {
  .name = "servoControlIsAvailable"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM3_Init(void);
void CmdParsing_Task(void *argument);
void ToggleLed_Task(void *argument);
void ScanCard_Task(void *argument);
void SensorMeasuring_Task(void *argument);
void SendCardID_Task(void *argument);
void SendSensorData_Task(void *argument);
void ServoControl_task(void *argument);

/* USER CODE BEGIN PFP */
void sendDataToServer(volatile WareHouse_t* wareHouse, uint8_t typeOfData);
void scanCardIdHandle(uint8_t wareHouseId);
void toggleLEDHanlde(uint8_t wareHouseId, uint8_t ledId, uint8_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//**** Data to working with FRID ****
extern uchar senddata[];
extern uchar keyA[6];
extern uchar keyA_new[6];
extern uchar keyB[6] ;
extern uchar keyB_new[6];

//**** Data of ware house ****
volatile WareHouse_t wareHouse_1, wareHouse_2, wareHouse_3;


volatile char uartReceivedData[16];

volatile uint8_t receivedDataFromServer[8]={0};

volatile uint8_t sensorMeasuringIsEnable = 1;

volatile uint8_t cmdToSend[8]={0}; 		// this is used for send CMD to Slave

volatile DHT_DataTypeDef DHT_Data;	// Data to store temperature and humidity

//**** RTOS object ****
osMutexId_t mutex_id;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	wareHouse_1.index = 1;
	wareHouse_2.index = 2;
	wareHouse_3.index = 3;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HAL_UART_Transmit(&huart2, (uchar*) "\n\rSerial Connected...\n\r", 23, 5000);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutex_uart2 */
  mutex_uart2Handle = osMutexNew(&mutex_uart2_attributes);

  /* creation of mutex_uart6 */
  mutex_uart6Handle = osMutexNew(&mutex_uart6_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of cardID_Queue */
  cardID_QueueHandle = osMessageQueueNew (1, sizeof(WareHouse_t), &cardID_Queue_attributes);

  /* creation of sensorData_Queue */
  sensorData_QueueHandle = osMessageQueueNew (3, sizeof(WareHouse_t), &sensorData_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CmdParsing */
  CmdParsingHandle = osThreadNew(CmdParsing_Task, NULL, &CmdParsing_attributes);

  /* creation of ToggleLed */
  ToggleLedHandle = osThreadNew(ToggleLed_Task, NULL, &ToggleLed_attributes);

  /* creation of ScanCard */
  ScanCardHandle = osThreadNew(ScanCard_Task, NULL, &ScanCard_attributes);

  /* creation of SensorMeasuring */
  SensorMeasuringHandle = osThreadNew(SensorMeasuring_Task, NULL, &SensorMeasuring_attributes);

  /* creation of SendCardID */
  SendCardIDHandle = osThreadNew(SendCardID_Task, NULL, &SendCardID_attributes);

  /* creation of SendSensorData */
  SendSensorDataHandle = osThreadNew(SendSensorData_Task, NULL, &SendSensorData_attributes);

  /* creation of ServoControl */
  ServoControlHandle = osThreadNew(ServoControl_task, NULL, &ServoControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of scanCardAvailable */
  scanCardAvailableHandle = osEventFlagsNew(&scanCardAvailable_attributes);

  /* creation of parsingCMDAvailable */
  parsingCMDAvailableHandle = osEventFlagsNew(&parsingCMDAvailable_attributes);

  /* creation of servoControlIsAvailable */
  servoControlIsAvailableHandle = osEventFlagsNew(&servoControlIsAvailable_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_SS3_Pin|SPI_SS2_Pin|LED_YELLOW_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_SS_GPIO_Port, SPI3_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_SS3_Pin SPI_SS2_Pin SPI3_SS_Pin LED_YELLOW_Pin
                           LED_GREEN_Pin */
  GPIO_InitStruct.Pin = SPI_SS3_Pin|SPI_SS2_Pin|SPI3_SS_Pin|LED_YELLOW_Pin
                          |LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6){
		osEventFlagsSet(parsingCMDAvailableHandle, 0x00000001U);
		HAL_UART_Receive_IT(&huart6, (uint8_t*)receivedDataFromServer, 8);
	}

//	if(huart == &huart1){
//		for(int i = 0; i<16; i++){
//			uchar temp[3];
//			sprintf((char*)temp, (const char*)"%02x ", (char)wareHouse_2.idOfScannedCard[i]);
//			HAL_UART_Transmit(&huart2, temp, 3, HAL_MAX_DELAY);
//		}
//	}
}


void sendDataToServer(volatile WareHouse_t* wareHouse, uint8_t typeOfData){

	uchar dataToSend[18]={0};

	switch(typeOfData){
		case Sensor_DATA:
			dataToSend[0] = wareHouse->temperature;
			dataToSend[1] = wareHouse->humidity;
			dataToSend[17] = wareHouse->index;
			break;
		case CardID_DATA:
			for(int i = 0; i<16; i++){
				dataToSend[i]=wareHouse->idOfScannedCard[i];
			}
			dataToSend[17] = wareHouse->index;
			break;
	}

	dataToSend[16] = typeOfData;


	HAL_UART_Transmit(&huart6, dataToSend, 18, HAL_MAX_DELAY);

}



void scanCardIdHandle(uint8_t wareHouseId){
	switch(wareHouseId){
	case 1:
		while(1){
			if( ScanCardAndGetDataFromBlock(0, wareHouse_1.idOfScannedCard, keyA) == MI_OK){
					for(int i = 0; i<16; i++){
						uchar temp[3];
						sprintf((char*)temp, (const char*)"%02x ", (char)wareHouse_1.idOfScannedCard[i]);
						HAL_UART_Transmit(&huart2, temp, 3, HAL_MAX_DELAY);
					}
					HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);

					// Send cardID to the cardID_Queue
					osMessageQueuePut(cardID_QueueHandle, &wareHouse_1, 0U, osWaitForever);

					return;
				}
		}
		break;
	case 2:
		// Send Cmd to warehouse2 (stm32 slave) to update value of warehouse2
		cmdToSend[0] = SCAN_CARDID;

		HAL_UART_Transmit(&huart1, cmdToSend, 8, HAL_MAX_DELAY);
		osDelay(10);
		HAL_UART_Receive(&huart1, wareHouse_2.idOfScannedCard, 16, HAL_MAX_DELAY);

		for(int i = 0; i<16; i++){
			uchar temp[3];
			sprintf((char*)temp, (const char*)"%02x ", (char)wareHouse_2.idOfScannedCard[i]);
			HAL_UART_Transmit(&huart2, temp, 3, HAL_MAX_DELAY);
		}

		osMessageQueuePut(cardID_QueueHandle, &wareHouse_2, 0U, osWaitForever);
		break;
	default:
		break;
	}
}

void toggleLEDHanlde(uint8_t wareHouseId, uint8_t ledId, uint8_t state){

	switch(wareHouseId){
		case 1:
			if(ledId == 1){
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, state);
			}else if(ledId == 2){
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, state);
			}else if(ledId == 3){
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, state);
				osEventFlagsSet(servoControlIsAvailableHandle, 0x00000001U);
			}

			break;
		case 2:
			// Send cmd to warehouse2 to set state of led
			cmdToSend[0] = TOGGLE_LED;
			cmdToSend[1] =  ledId;
			cmdToSend[2] =  state;
			HAL_UART_Transmit(&huart1, cmdToSend, 8, 1000);
		default:
			break;
		}
}

/**********************************************************************************************************/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CmdParsing_Task */
/**
  * @brief  Function implementing the CmdParsing thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CmdParsing_Task */
void CmdParsing_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_UART_Receive_IT(&huart6, (uint8_t*)receivedDataFromServer, 8);
  /* Infinite loop */
  for(;;)
  {
	// Wait for flag to parsing comming CMD
	osEventFlagsWait(parsingCMDAvailableHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);

	// Parsing CMD
	switch(receivedDataFromServer[0]){
	case SCAN_CARDID:
		osEventFlagsSet(scanCardAvailableHandle, 0x00000001U);
		break;
	case TOGGLE_LED:
		toggleLEDHanlde(receivedDataFromServer[1], receivedDataFromServer[2], receivedDataFromServer[3]);
		break;
	case SENSOR_MEASURING:
			//dosomething
		sensorMeasuringIsEnable = receivedDataFromServer[7];
		break;
	}

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ToggleLed_Task */
/**
* @brief Function implementing the ToggleLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ToggleLed_Task */
void ToggleLed_Task(void *argument)
{
  /* USER CODE BEGIN ToggleLed_Task */

  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END ToggleLed_Task */
}

/* USER CODE BEGIN Header_ScanCard_Task */
/**
* @brief Function implementing the ScanCard thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScanCard_Task */
void ScanCard_Task(void *argument)
{
  /* USER CODE BEGIN ScanCard_Task */

  /* Infinite loop */
  for(;;)
  {
	// Wait for scan flag
	osEventFlagsWait(scanCardAvailableHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);

	// Mutex protect for uart2 start
//	osMutexAcquire(mutex_uart2Handle, osWaitForever);

	// Scan card id in ware house (id = receivedDataFromServer[1]) and send data to queue to SendCardID_Task
	scanCardIdHandle(receivedDataFromServer[1]);

	// Mutex protect for uart2 end
//	osMutexRelease(mutex_uart2Handle);

    osDelay(1);
  }
  /* USER CODE END ScanCard_Task */
}

/* USER CODE BEGIN Header_SensorMeasuring_Task */
/**
* @brief Function implementing the SensorMeasuring thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorMeasuring_Task */
void SensorMeasuring_Task(void *argument)
{
  /* USER CODE BEGIN SensorMeasuring_Task */
  /* Infinite loop */
	HAL_TIM_Base_Start(&htim10);
  for(;;)
  {
	  if(sensorMeasuringIsEnable){
		DHT11_GetData(&DHT_Data);
		wareHouse_1.humidity = DHT_Data.Humidity;
		wareHouse_1.temperature = DHT_Data.Temperature;
		osMessageQueuePut(sensorData_QueueHandle, &wareHouse_1, NULL, osWaitForever);
	  }

	  osDelay(1000);

	  if(sensorMeasuringIsEnable){
		//Send Cmd to warehouse2 to get humidity and temperature
		cmdToSend[0] = SENSOR_MEASURING;
		HAL_UART_Transmit(&huart1, cmdToSend, 8, HAL_MAX_DELAY);
		HAL_UART_Receive(&huart1, &wareHouse_2.humidity, 1, 1000);
		HAL_UART_Receive(&huart1, &wareHouse_2.temperature, 1, 1000);
		osMessageQueuePut(sensorData_QueueHandle, &wareHouse_2, NULL, osWaitForever);
	  }
	  osDelay(1000);


  }
  /* USER CODE END SensorMeasuring_Task */
}

/* USER CODE BEGIN Header_SendCardID_Task */
/**
* @brief Function implementing the SendCardID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendCardID_Task */
void SendCardID_Task(void *argument)
{
  /* USER CODE BEGIN SendCardID_Task */
	WareHouse_t wareHouse;
  /* Infinite loop */
  for(;;)
  {
	// wait for data from queue
	osMessageQueueGet(cardID_QueueHandle, &wareHouse, NULL, osWaitForever);

	// Mutex protect for uart6 (connect to Esp) start
	osMutexAcquire(mutex_uart6Handle, osWaitForever);

	// send Data to ESP8266 -> Sever
	sendDataToServer(&wareHouse, CardID_DATA);

	// Mutex protect for uart6 end
	osMutexRelease(mutex_uart6Handle);

    osDelay(1);
  }
  /* USER CODE END SendCardID_Task */
}

/* USER CODE BEGIN Header_SendSensorData_Task */
/**
* @brief Function implementing the SendSensorData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendSensorData_Task */
void SendSensorData_Task(void *argument)
{
  /* USER CODE BEGIN SendSensorData_Task */
	WareHouse_t wareHouse;

  /* Infinite loop */
  for(;;)
  {
	// wait for data from queue
	osMessageQueueGet(sensorData_QueueHandle, &wareHouse, NULL, osWaitForever);

	// Mutex protect for uart6 (connect to Esp) start
	osMutexAcquire(mutex_uart6Handle, osWaitForever);

	// send Data to ESP8266 -> Sever
	sendDataToServer(&wareHouse, Sensor_DATA);

	// Mutex protect for uart6 end
	osMutexRelease(mutex_uart6Handle);

    osDelay(1);
  }
  /* USER CODE END SendSensorData_Task */
}

/* USER CODE BEGIN Header_ServoControl_task */
/**
* @brief Function implementing the ServoControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoControl_task */
void ServoControl_task(void *argument)
{
  /* USER CODE BEGIN ServoControl_task */
	Servo_Write(&htim3, TIM_CHANNEL_1, 90);
	int isOpen = 0;
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(servoControlIsAvailableHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
	if(isOpen == 0){
		Servo_Write(&htim3, TIM_CHANNEL_1, 0);
		isOpen = 1;
	}else {
		Servo_Write(&htim3, TIM_CHANNEL_1, 90);
		isOpen = 0;
	}

	osDelay(1);
  }
  /* USER CODE END ServoControl_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
