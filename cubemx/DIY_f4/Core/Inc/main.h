/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOA
#define Serial_TX_Pin GPIO_PIN_2
#define Serial_TX_GPIO_Port GPIOA
#define Serial_RX_Pin GPIO_PIN_3
#define Serial_RX_GPIO_Port GPIOA
#define Left_A_Pin GPIO_PIN_6
#define Left_A_GPIO_Port GPIOA
#define Left_B_Pin GPIO_PIN_7
#define Left_B_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOB
#define Right_A_Pin GPIO_PIN_12
#define Right_A_GPIO_Port GPIOD
#define Right_B_Pin GPIO_PIN_13
#define Right_B_GPIO_Port GPIOD
#define MPU_TX_Pin GPIO_PIN_9
#define MPU_TX_GPIO_Port GPIOA
#define MPU_RX_Pin GPIO_PIN_10
#define MPU_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define VAL_MAX		50				//最大速度	单位： cm/s
#define MOTOR_ARR	1000			
#define EC_ARR	65536.7			
#define VAL_MAX_PWM 430			//pwm限幅输出
#define ENCODER_TIME 0.010	//读取编码器时间	单位：s
#define EC_1	1400					//电机转一圈编码器增量 1400
#define PI		3.1415926			
#define D		  4.3						//轮子直径		单位	cm
#define C_1		PI*D					//轮子的周长	单位  cm


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
