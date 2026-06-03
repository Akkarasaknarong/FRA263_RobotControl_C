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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <arm_math.h>
#include <stdint.h>
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define BUTTON_RESET_Pin GPIO_PIN_0
#define BUTTON_RESET_GPIO_Port GPIOC
#define PROX_Pin GPIO_PIN_2
#define PROX_GPIO_Port GPIOC
#define E_STOP_Pin GPIO_PIN_3
#define E_STOP_GPIO_Port GPIOC
#define RELAY_MODE_Pin GPIO_PIN_1
#define RELAY_MODE_GPIO_Port GPIOA
#define RELAY_READY_Pin GPIO_PIN_4
#define RELAY_READY_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Reed_SW_Close_Pin GPIO_PIN_10
#define Reed_SW_Close_GPIO_Port GPIOB
#define JS_Rotate_Right_Pin GPIO_PIN_11
#define JS_Rotate_Right_GPIO_Port GPIOB
#define JS_RESET_Pin GPIO_PIN_12
#define JS_RESET_GPIO_Port GPIOB
#define JS_EMERGENCY_Pin GPIO_PIN_13
#define JS_EMERGENCY_GPIO_Port GPIOB
#define JS_Updown_Pin GPIO_PIN_14
#define JS_Updown_GPIO_Port GPIOB
#define JS_OpenClose_Pin GPIO_PIN_15
#define JS_OpenClose_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_7
#define DIR_GPIO_Port GPIOC
#define TEST_Pin GPIO_PIN_8
#define TEST_GPIO_Port GPIOC
#define JS_HOME_Pin GPIO_PIN_11
#define JS_HOME_GPIO_Port GPIOA
#define JS_Rotate_Left_Pin GPIO_PIN_12
#define JS_Rotate_Left_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Gripper_Open_Pin GPIO_PIN_10
#define Gripper_Open_GPIO_Port GPIOC
#define Gripper_Close_Pin GPIO_PIN_11
#define Gripper_Close_GPIO_Port GPIOC
#define Gripper_Down_Pin GPIO_PIN_12
#define Gripper_Down_GPIO_Port GPIOC
#define Gripper_Up_Pin GPIO_PIN_2
#define Gripper_Up_GPIO_Port GPIOD
#define Reed_SW_Up_Pin GPIO_PIN_4
#define Reed_SW_Up_GPIO_Port GPIOB
#define Reed_SW_Down_Pin GPIO_PIN_5
#define Reed_SW_Down_GPIO_Port GPIOB
#define MODE_SELECTOR_Pin GPIO_PIN_7
#define MODE_SELECTOR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
