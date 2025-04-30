/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	LEFT,
	RIGHT
} SideState;

typedef enum{
	LEFT_BURST,
	LEFT_GAP,
	RIGHT_BURST,
	RIGHT_GAP,
	START_BURST,
	START_GAP,
	STOP_BURST,
	STOP_GAP
} TX_STATE;

typedef struct{
	uint16_t burstLen;
	uint16_t gapLen;
} SignalData;

typedef struct{
	uint8_t head;
	uint8_t tail;
	SignalData items[64];
} Queue;

typedef struct{
	uint16_t burstPeriod;
	uint16_t gapPeriod;
	uint16_t burstLower;
	uint16_t burstUpper;
	uint16_t gapLower;
	uint16_t gapUpper;
	uint8_t symbol;
} SymbolTiming;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile bool receivedStartSymbol = false;

// ISR variables
static volatile uint8_t triggerCount = 0;
static volatile uint16_t lastTriggerTime = 0;
static volatile SignalData rxSignal;
// RX data variables
static volatile uint8_t rxDataBuffer[280];
static volatile uint16_t rxDataIndex;
// TX data variables
static uint8_t txDataBuffer[280];
static uint8_t txDataLength = 0;
static volatile uint8_t txDataByte = 0;
static volatile SymbolTiming txCurrentSymbol;
static volatile uint8_t txDataIndex = 0;
static volatile TX_STATE txStatus;
// Data processing variables
static volatile Queue signalBuffer;
static SideState currentDataSide = LEFT;
static uint8_t currentByte = 0;

                   //  BURST PERIOD  GAP PERIOD  MIN BURST  MAX BURST   MIN GAP  MAX GAP   SYMBOL
const SymbolTiming startSymbol = { 521,       174,       504,        591,       141,     207,     254};
const SymbolTiming stopSymbol =  { 521,       174,       504,        591,       141,     207,     255};

const SymbolTiming symbolTable[16] = {
//  BURST PERIOD  GAP PERIOD  MIN BURST  MAX BURST   MIN GAP  MAX GAP   SYMBOL
		{ 104,       174,       87,         174,       141,     207,     0x0 },
		{ 104,       243,       87,         174,       210,     276,     0x1 },
		{ 104,       313,       87,         174,       280,     345,     0x2 },
		{ 104,       382,       87,         174,       349,     415,     0x3 },
		{ 104,       452,       87,         174,       418,     484,     0x4 },
		{ 104,       521,       87,         174,       488,     554,     0x5 },
		{ 208,       174,       191,        278,       141,     207,     0x6 },
		{ 208,       243,       191,        278,       210,     276,     0x7 },
		{ 208,       313,       191,        278,       280,     345,     0x8 },
		{ 208,       382,       191,        278,       349,     415,     0x9 },
		{ 208,       452,       191,        278,       418,     484,     0xA },
		{ 313,       174,       295,        382,       141,     207,     0xB },
		{ 313,       243,       295,        382,       210,     276,     0xC },
		{ 313,       313,       295,        382,       280,     345,     0xD },
		{ 417,       174,       399,        486,       141,     207,     0xE },
		{ 417,       243,       399,        486,       210,     276,     0xF },
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static int8_t map_symbol(SignalData);
static void send_to_queue(SignalData);
static void process_optical_data();
static void owc_transmit(uint8_t*, uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  process_optical_data();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1462 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 731 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OWC_RX_Pin */
  GPIO_InitStruct.Pin = OWC_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OWC_RX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void owc_transmit(uint8_t* data, uint8_t size){

	memcpy(txDataBuffer, data, size);
	txDataLength = size;
	txStatus = START_BURST;
	__HAL_TIM_SET_AUTORELOAD(&htim4, 100);
	HAL_TIM_Base_Start_IT(&htim4);

}

static void process_optical_data(){

	while(signalBuffer.head != signalBuffer.tail){

		SignalData currentSignal = signalBuffer.items[signalBuffer.tail];
		int8_t dataSymbol = map_symbol(currentSignal);

		uint8_t nextindex = (signalBuffer.tail + 1) % 64;

		printf("Burst %d: %d \n",signalBuffer.tail, currentSignal.burstLen);
		printf("Gap %d: %d \n",signalBuffer.tail, currentSignal.gapLen);

		if(currentDataSide == LEFT){
			currentByte = dataSymbol << 4;
			currentDataSide = RIGHT;
		}
		else{
			currentByte |= dataSymbol;
			rxDataBuffer[rxDataIndex] = currentByte;
			rxDataIndex++;

			currentByte = 0;
			currentDataSide = LEFT;
		}

		signalBuffer.tail = nextindex;

	}
}

static int8_t map_symbol(SignalData signal){

	for(int i = 0; i < 16; i++){

		SymbolTiming current = symbolTable[i];

		if( (signal.burstLen >= current.burstLower && signal.burstLen <= current.burstUpper)  &&   (signal.gapLen >= current.gapLower && signal.gapLen <= current.gapUpper) ){

			return current.symbol;

		}

	}

	return -1;

}

static void send_to_queue(SignalData newSignal){
	uint8_t nextindex = (signalBuffer.head + 1) % 64;

	signalBuffer.items[signalBuffer.head] = newSignal;
	signalBuffer.head = nextindex;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	uint16_t timeStamp =  __HAL_TIM_GET_COUNTER(&htim3);
	uint16_t duration = timeStamp - lastTriggerTime;
	lastTriggerTime = timeStamp;
	triggerCount++;

    if(triggerCount == 2){
		rxSignal.burstLen = duration;
	}
	else if(triggerCount > 2){
		rxSignal.gapLen = duration;
		send_to_queue(rxSignal);
		triggerCount = 1;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	switch(txStatus){

	case START_BURST:

		__HAL_TIM_SET_AUTORELOAD(&htim4, startSymbol.burstPeriod);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		txStatus = START_GAP;

		break;
	case START_GAP:

		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_AUTORELOAD(&htim4, startSymbol.gapPeriod);
		__HAL_TIM_SET_COUNTER(&htim4, 0);

		txCurrentSymbol = symbolTable[txDataBuffer[txDataIndex] >> 4];
		txStatus = LEFT_BURST;

		break;

	case LEFT_BURST:

		__HAL_TIM_SET_AUTORELOAD(&htim4, txCurrentSymbol.burstPeriod);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		txStatus = LEFT_GAP;

		break;
	case LEFT_GAP:

		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_AUTORELOAD(&htim4, txCurrentSymbol.gapPeriod);
		__HAL_TIM_SET_COUNTER(&htim4, 0);

		txCurrentSymbol = symbolTable[txDataBuffer[txDataIndex] & 0xF];
		txStatus = RIGHT_BURST;

		break;
	case RIGHT_BURST:

		__HAL_TIM_SET_AUTORELOAD(&htim4, txCurrentSymbol.burstPeriod);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		txStatus = RIGHT_GAP;

		break;
	case RIGHT_GAP:

		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_AUTORELOAD(&htim4, txCurrentSymbol.gapPeriod);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		txDataIndex++;

		if(txDataIndex >= txDataLength){
			txStatus = STOP_BURST;
			return;
		}

		txCurrentSymbol = symbolTable[txDataBuffer[txDataIndex] >> 4];
		txStatus = LEFT_BURST;

		break;
	case STOP_BURST:

		__HAL_TIM_SET_AUTORELOAD(&htim4, stopSymbol.burstPeriod);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		txStatus = STOP_GAP;

		break;
	case STOP_GAP:

		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim4);
		memset(txDataBuffer, 0, 280);
		txStatus = START_BURST;

		break;

	}
}



int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
   ITM_SendChar(*ptr++);
  }
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
