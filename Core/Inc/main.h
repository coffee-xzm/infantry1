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
#include "dvc_serialplot.h"
#include "alg_pid.h"
#include "drv_uart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern Class_Serialplot serialplot;
/* 接收缓冲区声明 */
extern uint8_t rx_buffer[256];

/* 电机速度变量声明 */
extern uint16_t speed;         // 电机1速度
// extern uint16_t speed2;         // 电机2速度
// extern uint16_t speed3;         // 电机3速度
// extern uint16_t speed4;         // 电机4速度
extern uint16_t speed5;         // 电机5速度
extern uint16_t speed_snail;    // 小蜗轮电机速度

/* 电机方向变量声明 */
extern uint8_t dirs;
extern GPIO_PinState dir1;      // 电机1方向
extern GPIO_PinState dir2;      // 电机2方向
extern GPIO_PinState dir3;      // 电机3方向
extern GPIO_PinState dir4;      // 电机4方向
// extern GPIO_PinState dir5;      // 电机5方向

/* 舵机控制变量声明 */
extern uint8_t angle_servo;    // 舵机PWM值

/* PID控制器实例声明 */
extern Class_PID pid;           // PID控制器实例
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
