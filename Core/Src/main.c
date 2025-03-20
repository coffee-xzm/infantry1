/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "drv_uart.h"
#include "dvc_serialplot.h"
#include "hcsr04.h"
#include "alg_pid.h"
#include "drv_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Class_Serialplot serialplot;
Class_PID pid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[256];
/* 电机速度变量初始化 */
uint16_t speed = 0;    // 电机1速度
// uint16_t speed2 = 0;    // 电机2速度
// uint16_t speed3 = 0;    // 电机3速度
// uint16_t speed4 = 0;    // 电机4速度
uint16_t speed5 = 0;    // 电机5速度
uint16_t speed_snail = 0;   // snail电机速度

/* 电机方向变量初始化 */
uint8_t dirs;
GPIO_PinState dir1 = GPIO_PIN_RESET;    // 电机1方向
GPIO_PinState dir2 = GPIO_PIN_RESET;    // 电机2方向
GPIO_PinState dir3 = GPIO_PIN_RESET;    // 电机3方向
GPIO_PinState dir4 = GPIO_PIN_RESET;    // 电机4方向
// GPIO_PinState dir5 = GPIO_PIN_RESET;    // 电机5方向

/* 舵机控制变量初始化 */
uint8_t angle_servo = 0;               // 舵机PWM值 256份
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
  //if receive data from serial plot
};

// uint8_t jdy_init_data[] = "AT+BAUD4";
// void jdy31_init()
// {
//   UART_Send_Data(&huart3, jdy_init_data, sizeof(jdy_init_data) - 1);
// }

void tim_start()
{
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
};

void Motor_Control()
    {
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,speed);
        HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,dir1);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,speed);
        HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,dir2);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,speed);
        HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin,dir3);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,speed);
        HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin,dir4);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,speed5);
        HAL_GPIO_WritePin(DIR5_GPIO_Port,DIR5_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,speed_snail);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pid.Get_Out());
        // HAL_GPIO_WritePin(SERVO_DIR_GPIO_Port,SERVO_DIR_Pin,angle_servo);
        pid.TIM_Adjust_PeriodElapsedCallback();
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
  pid.TIM_Adjust_PeriodElapsedCallback();
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //Uart_Init(&huart3, rx_buffer, 256, Serialplot_Call_Back);

  char** command;
  serialplot.Init(&huart3,6,command,Serialplot_Data_Type_INT32,0xAB);

  pid.Init(1,1,1);

	Hcsr04Init(&htim3,TIM_CHANNEL_4);
	
  //PID_set();
  // jdy31_init();

  tim_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t da[] = {0,1,0,1};
    //UART_Send_Data(&huart3,da,4);
    Motor_Control();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @description: 定时器输出捕获中断
 * @param {TIM_HandleTypeDef} *htim
 * @return {*}
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)    //捕获回调函数
{
  Hcsr04TimIcIsr(htim);
  serialplot.Set_Data(Hcsr04Info.distance);
  serialplot.TIM_Add_PeriodElapsedCallback();
}
 
/**
 * @description: 定时器溢出中断
 * @param {*}
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)    //在中断回调函数中添加用户代码
{
  Hcsr04TimOverflowIsr(htim);
  if(htim->Instance == TIM1)
  {
    // 触发新的HC-SR04测量
    Hcsr04Start();
  }
}
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
