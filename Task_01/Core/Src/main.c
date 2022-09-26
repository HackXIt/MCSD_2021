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
void blinky(float brightness, GPIO_TypeDef * Port, uint16_t Pin);
int buttoncheck(GPIO_TypeDef * Port, uint16_t Pin);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BRIGHTNESS_1 0.1 // 10 %
#define BRIGHTNESS_2 0.4 // 40 %
#define BRIGHTNESS_3 1.0 // 100 %
#define PERIOD 20 // 20 milliseconds == 50 Hz
#define DEBOUNCE 20 // 20 milliseconds

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum switcher {
	RED,
	GREEN,
	BLUE
};
enum brightness {
	LEVEL_1,
	LEVEL_2,
	LEVEL_3
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  int counter = 0;
  uint8_t stop_flag = 0;
  uint8_t next_flag = 0;
  uint8_t led_switcher = RED; // Start with RGB_RED
  uint8_t brightness_switcher = LEVEL_1; // Start with BRIGHTNESS_1
  GPIO_TypeDef * current_port;
  uint16_t current_pin;
  float current_brightness;
  // Turn LEDs off, because they are ON due to inverted logic.
  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET); // Board LED on == Program running
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Set current active LED
	  switch(led_switcher) {
	  case RED:
		  current_port = RGB_RED_GPIO_Port;
		  current_pin = RGB_RED_Pin;
		  break;
	  case GREEN:
		  current_port = RGB_GREEN_GPIO_Port;
		  current_pin = RGB_GREEN_Pin;
		  break;
	  case BLUE:
		  current_port = RGB_BLUE_GPIO_Port;
		  current_pin = RGB_BLUE_Pin;
		  break;
	  }
	  // Set current active brightness
	  switch(brightness_switcher) {
	  case LEVEL_1:
		  current_brightness = BRIGHTNESS_1;
		  break;
	  case LEVEL_2:
		  current_brightness = BRIGHTNESS_2;
		  break;
	  case LEVEL_3:
		  current_brightness = BRIGHTNESS_3;
		  break;
	  }
	  blinky(current_brightness, current_port, current_pin);
	  if(!buttoncheck(BUTTON_GPIO_Port, BUTTON_Pin)) {
		  next_flag = 1; // Set flag to switch to next brightness / LED
		  while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) {
			  counter++;
			  // Turn LED / Program OFF after approximately 1 second
			  // 50 * 0.02s = 1s
			  if(counter >= 50) {
				  stop_flag = 1;
				  next_flag = 0;
				  brightness_switcher = LEVEL_1;
				  led_switcher = RED;
				  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);
				  // Board LED OFF == Program stops
				  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
				  break;
			  }
			  blinky(current_brightness, current_port, current_pin); // Acts as de-bounce
		  }
		  counter = 0; // Reset counter when exiting HOLD loop
	  }
	  if(next_flag) {
		  brightness_switcher++;
		  if(brightness_switcher >= 3) { // Switch to next color when last brightness is reached
			  brightness_switcher = LEVEL_1;
			  led_switcher++;
			  if(led_switcher >= 3) { // Reset to 1st color when this was the last.
				  led_switcher = RED;
			  }
		  }
		  next_flag = 0;
	  }
	  // Endless loop for halting the program
	  while(stop_flag) {
		  if(!buttoncheck(BUTTON_GPIO_Port, BUTTON_Pin)) {
			  // Start loop as long as button is pressed
			  while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) {
				  counter++;
				  // If counter reaches 50, approximately 1 second has passed
				  // Resume the program.
				  if(counter >= 50) {
					  stop_flag = 0;
					  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
					  break;
				  }
				  HAL_Delay(DEBOUNCE);
			  }
			  counter = 0; // Reset counter when exiting HOLD loop
		  }
	  }
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
  HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
  GPIO_InitStruct.Pin = RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void blinky(float duty_cycle, GPIO_TypeDef * Port, uint16_t Pin) {
	// Period = 20ms -> 50 Hertz
	// on_time -> 10%, 50% or 100% => 20 * 0.1 | 0.5 | 1.0
	// off_time -> Period - on_time
	// duty_cycle == brightness
	int on_time = (int)(PERIOD * duty_cycle);
	int off_time = PERIOD - on_time;
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET); // LED on
	HAL_Delay(on_time);
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET); // LED off
	HAL_Delay(off_time);
}
int buttoncheck(GPIO_TypeDef * Port, uint16_t Pin) {
	uint8_t button_value = HAL_GPIO_ReadPin(Port, Pin);
	// Only de-bounce if button was actually pressed.
	if(!button_value) {
		HAL_Delay(DEBOUNCE);
	}
	return button_value;
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
