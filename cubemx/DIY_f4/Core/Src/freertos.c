/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay.h"
#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"
#include "tim.h"
#include "usart.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

atk_ms901m_gyro_data_t gyro_dat;                   /* ���������� */
atk_ms901m_accelerometer_data_t accelerometer_dat; /* ���ٶȼ����� */
atk_ms901m_quaternion_data_t quaternion_dat;			 /* ��Ԫ�� */
atk_ms901m_attitude_data_t attitude_dat;					 /* ��̬�� */

Motor_Stat Left_Motor;	/*��������*/
Motor_Stat Right_Motor;	/*��������*/

extern Pid local_pid;//λ��
extern Pid incremental_pid;//����




/* USER CODE END Variables */
/* Definitions for MPU_TASK */
osThreadId_t MPU_TASKHandle;
const osThreadAttr_t MPU_TASK_attributes = {
  .name = "MPU_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Read_MPU(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  
	/* start timers, add new ones, ... */	
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MPU_TASK */
  MPU_TASKHandle = osThreadNew(Read_MPU, NULL, &MPU_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Read_MPU */

/**
* @brief ��MPU����
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_MPU */
void Read_MPU(void *argument)
{
  /* USER CODE BEGIN Read_MPU */
	/*�״ε��ȲŻᱻִ��*/
		
	/*�״ε��ȲŻᱻִ��*/
	/* Infinite loop */
  
	for(;;)
  {
		uint8_t ret = atk_ms901m_init();
		if(ret != 0)osDelay(100);
		atk_ms901m_get_attitude(&attitude_dat,100);		
		osDelay(100);
	}
  
  /* USER CODE END Read_MPU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

