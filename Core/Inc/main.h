/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_encoder.h"
#include "LCD1602.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	float Target_Counts;
	float RPM;
	float Rise_time;
	float Fall_time;
	float Offset_Counts;
} User_inputs;

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
#define ENCODER_A_Pin GPIO_PIN_0
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_1
#define ENCODER_B_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOA
#define BT_SET_Pin GPIO_PIN_6
#define BT_SET_GPIO_Port GPIOA
#define BT_SET_EXTI_IRQn EXTI9_5_IRQn
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOA
#define BT_CONFIG_Pin GPIO_PIN_0
#define BT_CONFIG_GPIO_Port GPIOB
#define BT_CONFIG_EXTI_IRQn EXTI0_IRQn
#define PWM_MOTOR_Pin GPIO_PIN_8
#define PWM_MOTOR_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_11
#define D7_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_12
#define D6_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define BT_DEC_Pin GPIO_PIN_4
#define BT_DEC_GPIO_Port GPIOB
#define BT_DEC_EXTI_IRQn EXTI4_IRQn
#define EN_Pin GPIO_PIN_5
#define EN_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_6
#define RW_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
