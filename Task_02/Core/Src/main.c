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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Page 1138 - HAL Reference - Callback Registration
//#define USE_HAL_TIM_REGISTER_CALLBACK 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Uncomment this to test timeout_blocking functionality
//#define BLOCKING
// Uncomment this to test timeout_nonblocking functionality
#define NON_BLOCKING

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t callback_flag = 0;
uint8_t timer_flag = 0;
uint8_t led_switcher = 0;
enum switcher{
	RED,
	GREEN,
	BLUE,
	OFF
};
//void *callback;
// Definition can be overridden to use different return types or parameters
#ifndef CALLBACK
#define CALLBACK
typedef void (*CB)(void);
CB callback=NULL;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void _tim_timeout_blocking(uint16_t ms);
void _tim_timeout_nonblocking_with_callback(unsigned int ms, void (*cb)(void));
void my_callback(void);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  // Deactivate LEDs from the beginning.
  HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin|RGB_GREEN_Pin|RGB_BLUE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef BLOCKING
	  _tim_timeout_blocking(1000);
#endif

#ifdef NON_BLOCKING
	  _tim_timeout_nonblocking_with_callback(1000, my_callback);
//	   Activate sleep mode (because otherwise nonblocking would be resetting itself
	  HAL_SuspendTick(); // System-Ticks act as Interrupts and need to be deactivated
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	  HAL_ResumeTick(); // Reactivate system-ticks
	  if(callback_flag) {
		  switch(led_switcher) {
		  case RED:
			  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_RESET);
			  break;
		  case GREEN:
			  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_RESET);
			  break;
		  case BLUE:
			  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_RESET);
			  break;
		  case OFF:
			  break;
		  }
		  callback_flag = 0; // Reset callback flag
	  }
#endif

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 32000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65536-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUTTON_Pin|RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_Pin RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin|RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
#ifdef NON_BLOCKING
	if(htim == &htim16) {
		// Stop the timer
		HAL_TIM_Base_Stop_IT(&htim16);
		// I couldn't figure out how to do this without a global variable
		// I've read the section about Callback Registration but I had no idea how to implement it
		// In general user callbacks can be realized using Callback Registration (look it up in HAL reference)
		(*callback)();
	}
#endif
#ifdef BLOCKING
	if (htim == &htim16) {
		// Only when interrupt is used - but I couldn't make it work
		// Deactivate LED in here, when insanity hits...
//		HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

		// Stop the Timer Interrupt
		HAL_TIM_Base_Stop_IT(&htim16);
		// Toggle LED when testing how the program should be run
		HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
	}
#endif
}
void _tim_timeout_blocking(uint16_t ms) {
	// Set autoreload to specified milliseconds
	// This assumes that the *.ioc configuration is set to have 1ms per tick
	__HAL_TIM_SET_AUTORELOAD(&htim16, ms);
	// Set counter to 0 for sanity (redundant - just making sure)
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	// Event generation register => Update generation (Reinitialize counter & update registers)
	TIM16->EGR = 1;
	// Status register => Reset Update Interrupt Flag (Because it was set by EGR - line above)
	// Barebone: TIM16->SR &= 0
	__HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);

	// Barebone Variante
	// TIM16->DIER |= 1; // Timer aktivieren
	// TIM16->CR1 |= 1; // Start counter by setting Counter-Enable-Bit CEN

	// Variante 1 - Using just timer as counter without sleep mode and interrupts
//	HAL_TIM_Base_Start(&htim16);
	// Also could use Interrupts for activating LED or doing something else.
	// Wait for timer to reach target. (Instead of Sleep mode)
//	while(1) {
//		if(__HAL_TIM_GET_COUNTER(&htim16) >= ms) break;
//	}

	// Variante 2 - Using interrupts with sleep mode
	HAL_TIM_Base_Start_IT(&htim16);
	// Alternative to using the HAL function
//	__HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
	// Enter sleep mode until Interrupt wakes up MCU
	HAL_SuspendTick(); // System-Ticks act as Interrupts and need to be deactivated
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick(); // Reactivate system-ticks
	// Exits sleep mode on interrupt
}
void _tim_timeout_nonblocking_with_callback(unsigned int ms, CB cb) {
	callback = cb; // Set global function pointer variable to callback
	// Activate flag for switching LEDs in main-loop
	// This flag is also used to differentiate between blocking and nonblocking
	callback_flag = 1;
	// Set autoreload to specified milliseconds
	// This assumes that the configuration is set to have 1ms per tick
	__HAL_TIM_SET_AUTORELOAD(&htim16, ms);
	// Set counter to 0 for sanity (redundant - just making sure)
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	// Event generation register => Update generation (Reinitialize counter & update registers)
	TIM16->EGR = 1;
	// Status register => Reset Update Interrupt Flag (Because it was set by EGR - line above)
	// Barebone: TIM16->SR &= 0
	__HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim16);
}
void my_callback(void) {
	// Switch to next led
	led_switcher++;
	// Reset to 0 on last mode
	if(led_switcher >= 4) {
		led_switcher = 0;
	}
	// Disable all LEDs, so next one can turn on.
	HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin|RGB_GREEN_Pin|RGB_BLUE_Pin, GPIO_PIN_SET);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
