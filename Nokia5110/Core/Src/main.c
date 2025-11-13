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

#include "n3310.h"
#include "picture.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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

	N3310Struct Display;

	Display.DC_Port = LCD_DC_GPIO_Port;
	Display.DC_Pin = LCD_DC_Pin;

	Display.CE_Port = LCD_CE_GPIO_Port;
	Display.CE_Pin = LCD_CE_Pin;

	Display.RST_Port = LCD_RST_GPIO_Port;
	Display.RST_Pin = LCD_RST_Pin;

	Display.SDIN_Port = LCD_SDIN_GPIO_Port;
	Display.SDIN_Pin = LCD_SDIN_Pin;

	Display.SCLK_Port = GPIOA;
	Display.SCLK_Pin = LCD_SCLK_Pin;


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init(&Display);
  LCD_Clear(&Display);

  // Draw Pixels
  for (uint8_t x=0; x< LCD_X_RES; x+=4) {
	  for (uint8_t y=0; y< LCD_Y_RES; y+=4) {
		  LCD_DrawPixel(&Display, x, y, PIXEL_ON);
	  }
  }
  LCD_Update(&Display);
  HAL_Delay(3000);


  // Draw Lines
  LCD_Clear(&Display);
  LCD_DrawLine(&Display, 0, 0, LCD_X_RES-1, LCD_Y_RES-1, PIXEL_ON);
  LCD_DrawLine(&Display, 0, LCD_Y_RES-1, LCD_X_RES-1, 0, PIXEL_ON);
  LCD_Update(&Display);
  HAL_Delay(3000);

  // Draw Circles
  LCD_Clear(&Display);
  LCD_DrawCircle(&Display, LCD_X_RES/2, LCD_Y_RES/2, 10, PIXEL_ON);
  LCD_DrawCircle(&Display, LCD_X_RES/2, LCD_Y_RES/2, 15, PIXEL_ON);
  LCD_DrawCircle(&Display, LCD_X_RES/2, LCD_Y_RES/2, 20, PIXEL_ON);
  LCD_Update(&Display);
  HAL_Delay(3000);

  // Draw Rectangles
  LCD_Clear(&Display);
  LCD_DrawRect(&Display, 10,10,LCD_X_RES-10,LCD_Y_RES-10, PIXEL_ON);
  LCD_DrawRect(&Display, 0,0,24,24, PIXEL_ON);
  LCD_DrawRect(&Display, LCD_X_RES-24,LCD_Y_RES-24,LCD_X_RES-1,LCD_Y_RES-1, PIXEL_ON);
  LCD_Update(&Display);
  HAL_Delay(3000);

  //Print Text
  uint8_t buf[] = " Hello!";
  LCD_Clear(&Display);
  LCD_SetTextPos(&Display, 0, 0);
  LCD_Print(&Display, buf, FONT_1X); // Font Size 1
  LCD_Update(&Display);
  //HAL_Delay(3000);

  LCD_SetTextPos(&Display, 0, 2);
  LCD_Print(&Display, buf, FONT_2X); // Font Size 2
  LCD_Update(&Display);
  HAL_Delay(3000);

  //Print Numbers
  uint8_t bufNumbers[] = "3.45";
  LCD_Clear(&Display);
  LCD_SetTextPos(&Display, 0, 1);
  LCD_Print(&Display, bufNumbers, FONT_2X); // Font Size 2
  LCD_Update(&Display);
  HAL_Delay(3000);

  LCD_Clear(&Display);
  LCD_SetTextPos(&Display, 0, 2);
  LCD_Print(&Display, bufNumbers, FONT_4X); // Font Size 4
  LCD_Update(&Display);
  HAL_Delay(3000);


  //Menu
  uint8_t items[4][8]= {" Item 1\0", " Item 2\0", " Item 3\0", " Item 4\0"} ;

  for (uint8_t item=0; item<4; item++) {
	  LCD_Clear(&Display);

	  for (uint8_t i=0; i<4; i++) {
		  LCD_SetTextPos(&Display, 0, i);
		  LCD_Print(&Display, items[i], FONT_1X);
	  }

	  LCD_SetTextPos(&Display, 0, item);
	  LCD_Chr(&Display, '>', FONT_1X);
	  LCD_IvertLine(&Display, item);

	  LCD_Update(&Display);
	  HAL_Delay(1000);
  }


  //Draw Image
  LCD_Image(&Display, Picture);
  LCD_Update(&Display);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin|LCD_SDIN_Pin
                          |LCD_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_CE_Pin LCD_RST_Pin LCD_SDIN_Pin
                           LCD_SCLK_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin|LCD_SDIN_Pin
                          |LCD_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
