/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "APP/robot.h"
#include "robot_task.h"

#include "APP/graphic_draw.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |        :::::::::       :::::::::::       ::::::::       ::::    :::       ::::::::::       ::::::::::       :::::::::  | |  
  | |       :+:    :+:          :+:          :+:    :+:      :+:+:   :+:       :+:              :+:              :+:    :+:  | |  
  | |      +:+    +:+          +:+          +:+    +:+      :+:+:+  +:+       +:+              +:+              +:+    +:+   | |  
  | |     +#++:++#+           +#+          +#+    +:+      +#+ +:+ +#+       +#++:++#         +#++:++#         +#++:++#:     | |  
  | |    +#+                 +#+          +#+    +#+      +#+  +#+#+#       +#+              +#+              +#+    +#+     | |  
  | |   #+#                 #+#          #+#    #+#      #+#   #+#+#       #+#              #+#              #+#    #+#      | |  
  | |  ###             ###########       ########       ###    ####       ##########       ##########       ###    ###       | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |        :::::::::       :::::::::::       ::::::::       ::::    :::       ::::::::::       ::::::::::       :::::::::  | |  
  | |       :+:    :+:          :+:          :+:    :+:      :+:+:   :+:       :+:              :+:              :+:    :+:  | |  
  | |      +:+    +:+          +:+          +:+    +:+      :+:+:+  +:+       +:+              +:+              +:+    +:+   | |  
  | |     +#++:++#+           +#+          +#+    +:+      +#+ +:+ +#+       +#++:++#         +#++:++#         +#++:++#:     | |  
  | |    +#+                 +#+          +#+    +#+      +#+  +#+#+#       +#+              +#+              +#+    +#+     | |  
  | |   #+#                 #+#          #+#    #+#      #+#   #+#+#       #+#              #+#              #+#    #+#      | |  
  | |  ###             ###########       ########       ###    ####       ##########       ##########       ###    ###       | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
  | |        :::::::::       :::::::::::       ::::::::       ::::    :::       ::::::::::       ::::::::::       :::::::::  | |  
  | |       :+:    :+:          :+:          :+:    :+:      :+:+:   :+:       :+:              :+:              :+:    :+:  | |  
  | |      +:+    +:+          +:+          +:+    +:+      :+:+:+  +:+       +:+              +:+              +:+    +:+   | |  
  | |     +#++:++#+           +#+          +#+    +:+      +#+ +:+ +#+       +#++:++#         +#++:++#         +#++:++#:     | |  
  | |    +#+                 +#+          +#+    +#+      +#+  +#+#+#       +#+              +#+              +#+    +#+     | |  
  | |   #+#                 #+#          #+#    #+#      #+#   #+#+#       #+#              #+#              #+#    #+#      | |  
  | |  ###             ###########       ########       ###    ####       ##########       ##########       ###    ###       | |  
  | |                                                                                                                        | |  
  | |                                                                                                                        | |  
__| |________________________________________________________________________________________________________________________| |__
__   ________________________________________________________________________________________________________________________   __
  | |                                                                                                                        | |  
*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	disable_all_it();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	#if 0
	      :::::::::       :::::::::::       ::::::::       ::::    :::       ::::::::::       ::::::::::       :::::::::
     :+:    :+:          :+:          :+:    :+:      :+:+:   :+:       :+:              :+:              :+:    :+:
    +:+    +:+          +:+          +:+    +:+      :+:+:+  +:+       +:+              +:+              +:+    +:+ 
   +#++:++#+           +#+          +#+    +:+      +#+ +:+ +#+       +#++:++#         +#++:++#         +#++:++#:   
  +#+                 +#+          +#+    +#+      +#+  +#+#+#       +#+              +#+              +#+    +#+   
 #+#                 #+#          #+#    #+#      #+#   #+#+#       #+#              #+#              #+#    #+#    
###             ###########       ########       ###    ####       ##########       ##########       ###    ###  	
	#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /*Be sure to place MX_DMA_Init() before MX_USARTx_UART_Init()*/

  __HAL_TIM_DISABLE_IT(&M_TIM_HANDLE,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&H_TIM_HANDLE,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&IMU_TIM_HANDLE,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&DETECT_TIM_HANDLE,TIM_IT_UPDATE);

  /* Timer Configuration */
  M_TIM_HANDLE.Instance->ARR = 1e4f / M_TIM_FREQ - 1; // 200hz
  M_TIM_HANDLE.Instance->PSC = 8400 - 1;
  H_TIM_HANDLE.Instance->ARR = 1e4f / H_TIM_FREQ - 1; // 500hz
  H_TIM_HANDLE.Instance->PSC = 8400 - 1;
  IMU_TIM_HANDLE.Instance->ARR = 1e4f / IMU_TIM_FREQ - 1; // 1250hz
  IMU_TIM_HANDLE.Instance->PSC = 16800 - 1;
  DETECT_TIM_HANDLE.Instance->ARR = 1e4f / DETECT_TIM_FREQ - 1; // 10hz
  DETECT_TIM_HANDLE.Instance->PSC = 8400 - 1;

  MAGAZINE_TIM_HANDLE.Instance->ARR = 1e4f / MAGAZINE_TIM_FREQ - 1; // 25hz
  MAGAZINE_TIM_HANDLE.Instance->PSC = 8400 - 1;
  BEEP_TIM_HANDLE.Instance->ARR = 1e5f / BEEP_TIM_FREQ - 1; // 400hz
  BEEP_TIM_HANDLE.Instance->PSC = 420 - 1;
//  WS2812_TIM_HANDLE.Instance->ARR = 84e6f / WS2812_TIM_FREQ - 1; // 40 0000hz
//  WS2812_TIM_HANDLE.Instance->PSC = 1 - 1;

  /*robot init*/
  robot_init();

  /*disable imu heat ctrl*/
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  TIM10->CCR1 = 0;
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);   

  /*enable beep*/
 __HAL_TIM_SetCompare(&BEEP_TIM_HANDLE, BEEP_TIM_CHANNAL, 0);
 HAL_TIM_PWM_Start(&BEEP_TIM_HANDLE,BEEP_TIM_CHANNAL);

  /*enable magazine*/
  __HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, 0);
  HAL_TIM_PWM_Start(&MAGAZINE_TIM_HANDLE,MAGAZINE_TIM_CHANNAL);

  #if OS_ENABLE
  __HAL_TIM_DISABLE_IT(&M_TIM_HANDLE,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&H_TIM_HANDLE,TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(&IMU_TIM_HANDLE,TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
  __HAL_TIM_DISABLE_IT(&DETECT_TIM_HANDLE,TIM_IT_UPDATE);

  task_create();
  #else
	// __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
  HAL_TIM_Base_Start_IT(&M_TIM_HANDLE);
  HAL_TIM_Base_Start_IT(&H_TIM_HANDLE);
  HAL_TIM_Base_Start_IT(&IMU_TIM_HANDLE);
  HAL_TIM_Base_Start_IT(&DETECT_TIM_HANDLE);
  #endif  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	#if OS_ENABLE == 0
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		robot_main_loop();
  }
	#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	#if OS_ENABLE == 0
	if (htim == &htim3) {  // 10Hz
   diag_slow_time_cycle();
   RGB_loop(10,0.5);
	}
	if (htim == &htim9) {  // 1250Hz
   //Test0_Point_Start();
   robot_imu_tim_cycle();
   //Test0_Point_Stop();
	}
	if (htim == &htim6){  // 200Hz
   robot_m_tim_cycle();
	}
	if (htim == &htim7) {  // 1000Hz
   //Test1_Point_Start();
   robot_h_tim_cycle();
   //Test1_Point_Stop();
	}
	#endif
  /* USER CODE END Callback 1 */
}

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
