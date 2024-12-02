/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define PWM1__Pin GPIO_PIN_5
#define PWM1__GPIO_Port GPIOE
#define PWM1_E6_Pin GPIO_PIN_6
#define PWM1_E6_GPIO_Port GPIOE
#define PWM2__Pin GPIO_PIN_2
#define PWM2__GPIO_Port GPIOA
#define PWM2_A3_Pin GPIO_PIN_3
#define PWM2_A3_GPIO_Port GPIOA
#define En2_A_Pin GPIO_PIN_14
#define En2_A_GPIO_Port GPIOE
#define En2_A_EXTI_IRQn EXTI15_10_IRQn
#define En2_B_Pin GPIO_PIN_15
#define En2_B_GPIO_Port GPIOE
#define PWM3__Pin GPIO_PIN_14
#define PWM3__GPIO_Port GPIOB
#define PWM3_B15_Pin GPIO_PIN_15
#define PWM3_B15_GPIO_Port GPIOB
#define En3_A_Pin GPIO_PIN_12
#define En3_A_GPIO_Port GPIOD
#define En3_A_EXTI_IRQn EXTI15_10_IRQn
#define En3_B_Pin GPIO_PIN_13
#define En3_B_GPIO_Port GPIOD
#define En4_B_Pin GPIO_PIN_9
#define En4_B_GPIO_Port GPIOG
#define En4_A_Pin GPIO_PIN_10
#define En4_A_GPIO_Port GPIOG
#define En4_A_EXTI_IRQn EXTI15_10_IRQn
#define PWM4__Pin GPIO_PIN_4
#define PWM4__GPIO_Port GPIOB
#define PWM4_B5_Pin GPIO_PIN_5
#define PWM4_B5_GPIO_Port GPIOB
#define En1_A_Pin GPIO_PIN_0
#define En1_A_GPIO_Port GPIOE
#define En1_A_EXTI_IRQn EXTI0_IRQn
#define En1_B_Pin GPIO_PIN_1
#define En1_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
