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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PERIOD 20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void duty_cycle(int on_time, GPIO_TypeDef * Port, uint16_t Pin);
void blinky(float brightness, int blink_time, GPIO_TypeDef * Port, uint16_t Pin);
int buttonCheck(int debounce, GPIO_TypeDef * Port, uint16_t Pin);
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
  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);
  int switcher = 0;
  uint16_t currentPin;
  GPIO_TypeDef * currentPort;
//  float brightness = 0.0;
//  int on_time = 1;
  int button_pressed = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Switch through the currently active color
	  switch(switcher) {
	  case 0:
		  currentPort = RGB_RED_GPIO_Port;
		  currentPin = RGB_RED_Pin;
		  break;
	  case 1:
		  currentPort = RGB_GREEN_GPIO_Port;
		  currentPin = RGB_GREEN_Pin;
		  break;
	  case 2:
		  currentPort = RGB_BLUE_GPIO_Port;
		  currentPin = RGB_BLUE_Pin;
		  break;
	  }

	  /* BRIGHTNESS-VERSION Begin */
//	  // This is the version where a button press changes brightness
//	  // LED is switched automatically
//	  while(brightness < 1.0) {
//		  // Increase brightness by 1 step with every button press
//		  button_pressed = buttonCheck(5, BUTTON_GPIO_Port, BUTTON_Pin);
//		  if(!button_pressed) {
//			  brightness += 0.1;
//			  on_time = (int) (PERIOD * brightness);
//			  // Wait quarter of a second if brightness changed
//			  // Otherwise it won't be possible to cycle through each level
//			  HAL_Delay(250);
//		  }
//		  duty_cycle(on_time, currentPort, currentPin); // Execute 1 cycle with given on_time
//	  }
//	  // Reset brightness & on_time
//	  brightness = 0.0;
//	  on_time = 1;
	  /* BRIGHTNESS-VERSION End */

	  /* BLINK-VERSION Begin */
	  // This is the version where a button press changes color
	  // The LED blinks for 300ms in total
	  // LED brightness is changed automatically
	  while(button_pressed) {
		  for(float i = 0.0; i < 1.0; i+=0.1) {
			  // I chose 150 milliseconds per ON/OFF because it looked nice
			  // It was a sweet-spot between Button still being recognized quickly
			  // and blinking not being annoyingly fast
			  blinky(i, 150, currentPort, currentPin);
			  button_pressed = buttonCheck(5, BUTTON_GPIO_Port, BUTTON_Pin);
			  if(!button_pressed) {
				  break;
			  }
		  }
	  }
	  // Reset button_pressed to loop next LED
	  button_pressed = 1;
	  /* BLINK-VERSION End */

	  // Turn off all LEDs before switching to the next (won't be visible without delay)
	  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_SET);

	  // Switch to next RGB-Color
	  switcher++;
	  if(switcher >= 3) {
		  switcher = 0;
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
  GPIO_InitStruct.Pin = RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void duty_cycle(int on_time, GPIO_TypeDef * Port, uint16_t Pin) {
	// on_time is amount of steps that the LED is on in one period
	// a period is equal to 20 microseconds, which is equal to 50 Hertz
	// By stepping through from 1 to 20, I increase the on-time in one period
	// Effectively increasing the brightness of the LED.

	// This is software Pulse-Width-Modulation
	// IT WON'T be able to replicate REAL PWM
	// Also there might be better ways to do this
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET); // LED ON
	HAL_Delay(on_time);
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET); // LED OFF
	HAL_Delay(PERIOD - on_time);
}
void blinky(float brightness, int blink_time, GPIO_TypeDef * Port, uint16_t Pin) {
	// Brightness is a value from 0.0 to 1.0 => 0% to 100%
	// I use a
	int on_time = (int) (PERIOD * brightness);
	// Calculate amount of cycles to stay ON
	int cycles = blink_time / PERIOD;
	for(int i = 0; i < cycles; i++) {
		duty_cycle(on_time, Port, Pin);
	}
	// Leave LED OFF for the same amount of time
	HAL_Delay(blink_time);
}
int buttonCheck(int debounce, GPIO_TypeDef * Port, uint16_t Pin) {
	// Read the current value from the Button
	int value = HAL_GPIO_ReadPin(Port, Pin);
	// Debounce the button = Wait a bit before returning
	HAL_Delay(debounce);
	return value;
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
