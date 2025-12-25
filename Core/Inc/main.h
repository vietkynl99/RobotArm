/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SOFT_I2C_SCL_Pin GPIO_PIN_13
#define SOFT_I2C_SCL_GPIO_Port GPIOC
#define M5_E1_Pin GPIO_PIN_14
#define M5_E1_GPIO_Port GPIOC
#define M5_E1_EXTI_IRQn EXTI15_10_IRQn
#define M6_E1_Pin GPIO_PIN_15
#define M6_E1_GPIO_Port GPIOC
#define M6_E1_EXTI_IRQn EXTI15_10_IRQn
#define IO_EXPANDER_INT_Pin GPIO_PIN_4
#define IO_EXPANDER_INT_GPIO_Port GPIOA
#define IO_EXPANDER_INT_EXTI_IRQn EXTI4_IRQn
#define M2_E1_Pin GPIO_PIN_10
#define M2_E1_GPIO_Port GPIOB
#define M2_E1_EXTI_IRQn EXTI15_10_IRQn
#define M3_E1_Pin GPIO_PIN_11
#define M3_E1_GPIO_Port GPIOB
#define M3_E1_EXTI_IRQn EXTI15_10_IRQn
#define M4_E1_Pin GPIO_PIN_12
#define M4_E1_GPIO_Port GPIOB
#define M4_E1_EXTI_IRQn EXTI15_10_IRQn
#define M1_E2_Pin GPIO_PIN_13
#define M1_E2_GPIO_Port GPIOB
#define M2_E2_Pin GPIO_PIN_14
#define M2_E2_GPIO_Port GPIOB
#define M3_E2_Pin GPIO_PIN_15
#define M3_E2_GPIO_Port GPIOB
#define M4_E2_Pin GPIO_PIN_12
#define M4_E2_GPIO_Port GPIOA
#define M5_E2_Pin GPIO_PIN_15
#define M5_E2_GPIO_Port GPIOA
#define M6_E2_Pin GPIO_PIN_3
#define M6_E2_GPIO_Port GPIOB
#define M1_E1_Pin GPIO_PIN_8
#define M1_E1_GPIO_Port GPIOB
#define M1_E1_EXTI_IRQn EXTI9_5_IRQn
#define SOFT_I2C_SDA_Pin GPIO_PIN_9
#define SOFT_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
