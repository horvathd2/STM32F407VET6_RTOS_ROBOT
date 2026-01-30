/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include <stdlib.h>
#include <stdio.h>
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
void delay_us(TIM_HandleTypeDef *htim_delay, uint32_t us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_MOTOR1_Pin GPIO_PIN_3
#define ADC_MOTOR1_GPIO_Port GPIOA
#define MOT1_PWM_TIM1CH3_Pin GPIO_PIN_13
#define MOT1_PWM_TIM1CH3_GPIO_Port GPIOE
#define MOT1_PWM_TIM1CH4_Pin GPIO_PIN_14
#define MOT1_PWM_TIM1CH4_GPIO_Port GPIOE
#define MOT1_ENC_TIM3CH1_Pin GPIO_PIN_6
#define MOT1_ENC_TIM3CH1_GPIO_Port GPIOC
#define MOT1_ENC_TIM3CH2_Pin GPIO_PIN_7
#define MOT1_ENC_TIM3CH2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
