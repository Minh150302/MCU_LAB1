/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(int counter,
		GPIO_TypeDef* a_seg_GPIO_Port, uint16_t a_seg_Pin,
		GPIO_TypeDef* b_seg_GPIO_Port, uint16_t b_seg_Pin,
		GPIO_TypeDef* c_seg_GPIO_Port, uint16_t c_seg_Pin,
		GPIO_TypeDef* d_seg_GPIO_Port, uint16_t d_seg_Pin,
		GPIO_TypeDef* e_seg_GPIO_Port, uint16_t e_seg_Pin,
		GPIO_TypeDef* f_seg_GPIO_Port, uint16_t f_seg_Pin,
		GPIO_TypeDef* g_seg_GPIO_Port, uint16_t g_seg_Pin)
{
	switch(counter){
		case 0:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, RESET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, SET);
			break;
		case 1:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, SET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, SET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, SET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, SET);
			break;
		case 2:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, SET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, RESET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, SET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, SET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, SET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, SET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, SET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, SET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, RESET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, SET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, SET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, SET);
			break;
		case 8:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, RESET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		case 9:
			HAL_GPIO_WritePin(a_seg_GPIO_Port, a_seg_Pin, RESET);
			HAL_GPIO_WritePin(b_seg_GPIO_Port, b_seg_Pin, RESET);
			HAL_GPIO_WritePin(c_seg_GPIO_Port, c_seg_Pin, RESET);
			HAL_GPIO_WritePin(d_seg_GPIO_Port, d_seg_Pin, RESET);
			HAL_GPIO_WritePin(e_seg_GPIO_Port, e_seg_Pin, SET);
			HAL_GPIO_WritePin(f_seg_GPIO_Port, f_seg_Pin, RESET);
			HAL_GPIO_WritePin(g_seg_GPIO_Port, g_seg_Pin, RESET);
			break;
		default:
			break;
	}
}

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  while (1)
  {
	  if(counter >= 10) counter = 0;
	  display7SEG(counter++,
			  A_seg_GPIO_Port, A_seg_Pin,
			  B_seg_GPIO_Port, B_seg_Pin,
			  C_seg_GPIO_Port, C_seg_Pin,
			  D_seg_GPIO_Port, D_seg_Pin,
			  E_seg_GPIO_Port, E_seg_Pin,
			  F_seg_GPIO_Port, F_seg_Pin,
			  G_seg_GPIO_Port, G_seg_Pin);

	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  HAL_GPIO_WritePin(GPIOA, LED_RED_V_Pin|LED_YELLOW_V_Pin|LED_GREEN_V_Pin|LED_RED_H_Pin
                          |LED_YELLOW_H_Pin|LED_GREEN_H_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_seg_Pin|B_seg_Pin|C_seg_Pin|D_seg_Pin
                          |E_seg_Pin|F_seg_Pin|G_seg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_V_Pin LED_YELLOW_V_Pin LED_GREEN_V_Pin LED_RED_H_Pin
                           LED_YELLOW_H_Pin LED_GREEN_H_Pin */
  GPIO_InitStruct.Pin = LED_RED_V_Pin|LED_YELLOW_V_Pin|LED_GREEN_V_Pin|LED_RED_H_Pin
                          |LED_YELLOW_H_Pin|LED_GREEN_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_seg_Pin B_seg_Pin C_seg_Pin D_seg_Pin
                           E_seg_Pin F_seg_Pin G_seg_Pin */
  GPIO_InitStruct.Pin = A_seg_Pin|B_seg_Pin|C_seg_Pin|D_seg_Pin
                          |E_seg_Pin|F_seg_Pin|G_seg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
