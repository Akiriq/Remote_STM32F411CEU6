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
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOC
#define BP_Pin GPIO_PIN_0
#define BP_GPIO_Port GPIOA
#define ADC1_SPAR_Pin GPIO_PIN_1
#define ADC1_SPAR_GPIO_Port GPIOA
#define ADC1_SLIDE_Pin GPIO_PIN_2
#define ADC1_SLIDE_GPIO_Port GPIOA
#define ADC1_JOY_X_Pin GPIO_PIN_3
#define ADC1_JOY_X_GPIO_Port GPIOA
#define ADC1_JOY_Y_Pin GPIO_PIN_4
#define ADC1_JOY_Y_GPIO_Port GPIOA
#define ADC1_POT_A_Pin GPIO_PIN_5
#define ADC1_POT_A_GPIO_Port GPIOA
#define ADC1_POT_B_Pin GPIO_PIN_6
#define ADC1_POT_B_GPIO_Port GPIOA
#define BP_G_Pin GPIO_PIN_7
#define BP_G_GPIO_Port GPIOA
#define BP_M_Pin GPIO_PIN_0
#define BP_M_GPIO_Port GPIOB
#define BP_D_Pin GPIO_PIN_1
#define BP_D_GPIO_Port GPIOB
#define BP_SEL_Pin GPIO_PIN_2
#define BP_SEL_GPIO_Port GPIOB
#define BP_JOY_Pin GPIO_PIN_12
#define BP_JOY_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_13
#define LED_4_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_14
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_8
#define LED_1_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_15
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_7
#define NRF_IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
