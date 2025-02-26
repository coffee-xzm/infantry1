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
#define DIR3_Pin GPIO_PIN_13
#define DIR3_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_14
#define DIR4_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_15
#define DIR1_GPIO_Port GPIOC
#define ECHO_Pin GPIO_PIN_1
#define ECHO_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_2
#define TRIG_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_11
#define RX_GPIO_Port GPIOB
#define PWM_SNAIL_Pin GPIO_PIN_9
#define PWM_SNAIL_GPIO_Port GPIOA
#define DIR5_Pin GPIO_PIN_10
#define DIR5_GPIO_Port GPIOA
#define PWM5_Pin GPIO_PIN_11
#define PWM5_GPIO_Port GPIOA
#define SERVO_DIR_Pin GPIO_PIN_12
#define SERVO_DIR_GPIO_Port GPIOA
#define CHANNELA_Pin GPIO_PIN_15
#define CHANNELA_GPIO_Port GPIOA
#define CHANNELB_Pin GPIO_PIN_3
#define CHANNELB_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_4
#define SERVO_PWM_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_5
#define DIR2_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_7
#define PWM1_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_8
#define PWM4_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_9
#define PWM3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
