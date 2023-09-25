/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_V_Pin GPIO_PIN_2
#define LED_RED_V_GPIO_Port GPIOA
#define LED_YELLOW_V_Pin GPIO_PIN_3
#define LED_YELLOW_V_GPIO_Port GPIOA
#define LED_GREEN_V_Pin GPIO_PIN_4
#define LED_GREEN_V_GPIO_Port GPIOA
#define LED_RED_H_Pin GPIO_PIN_5
#define LED_RED_H_GPIO_Port GPIOA
#define LED_YELLOW_H_Pin GPIO_PIN_6
#define LED_YELLOW_H_GPIO_Port GPIOA
#define LED_GREEN_H_Pin GPIO_PIN_7
#define LED_GREEN_H_GPIO_Port GPIOA
#define A_seg_Pin GPIO_PIN_0
#define A_seg_GPIO_Port GPIOB
#define B_seg_Pin GPIO_PIN_1
#define B_seg_GPIO_Port GPIOB
#define C_seg_Pin GPIO_PIN_2
#define C_seg_GPIO_Port GPIOB
#define D_seg2_Pin GPIO_PIN_10
#define D_seg2_GPIO_Port GPIOB
#define E_seg2_Pin GPIO_PIN_11
#define E_seg2_GPIO_Port GPIOB
#define F_seg2_Pin GPIO_PIN_12
#define F_seg2_GPIO_Port GPIOB
#define G_seg2_Pin GPIO_PIN_13
#define G_seg2_GPIO_Port GPIOB
#define D_seg_Pin GPIO_PIN_3
#define D_seg_GPIO_Port GPIOB
#define E_seg_Pin GPIO_PIN_4
#define E_seg_GPIO_Port GPIOB
#define F_seg_Pin GPIO_PIN_5
#define F_seg_GPIO_Port GPIOB
#define G_seg_Pin GPIO_PIN_6
#define G_seg_GPIO_Port GPIOB
#define A_seg2_Pin GPIO_PIN_7
#define A_seg2_GPIO_Port GPIOB
#define B_seg2_Pin GPIO_PIN_8
#define B_seg2_GPIO_Port GPIOB
#define C_seg2_Pin GPIO_PIN_9
#define C_seg2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
