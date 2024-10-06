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
atk_ms901m_gyro_data_t gyro_dat;                   /* 陀螺仪数据 */
atk_ms901m_accelerometer_data_t accelerometer_dat; /* 加速度计数据 */
atk_ms901m_quaternion_data_t quaternion_dat;			 /* 四元数 */
atk_ms901m_attitude_data_t attitude_dat;					 /* 姿态角 */

Motor_Stat Left_Motor;	/*左轮数据*/
Motor_Stat Right_Motor;	/*右轮数据*/

extern Pid local_pid;//位置
extern Pid incremental_pid;//增量

/* USER CODE END Variables */
osThreadId MPU_TASKHandle;
osThreadId SERIAL_TASKHandle;
osThreadId TEST_TASKHandle;
osThreadId PID_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Read_mpu(void const * argument);
void Send_serial(void const * argument);
void test(void const * argument);
void PID_Control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of MPU_TASK */
  osThreadDef(MPU_TASK, Read_mpu, osPriorityNormal, 0, 128);
  MPU_TASKHandle = osThreadCreate(osThread(MPU_TASK), NULL);

  /* definition and creation of SERIAL_TASK */
  osThreadDef(SERIAL_TASK, Send_serial, osPriorityIdle, 0, 128);
  SERIAL_TASKHandle = osThreadCreate(osThread(SERIAL_TASK), NULL);

  /* definition and creation of TEST_TASK */
  osThreadDef(TEST_TASK, test, osPriorityIdle, 0, 128);
  TEST_TASKHandle = osThreadCreate(osThread(TEST_TASK), NULL);

  /* definition and creation of PID_TASK */
  osThreadDef(PID_TASK, PID_Control, osPriorityHigh, 0, 128);
  PID_TASKHandle = osThreadCreate(osThread(PID_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Read_mpu */
/**
  * @brief  Function implementing the MPU_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Read_mpu */
void Read_mpu(void const * argument)
{
  /* USER CODE BEGIN Read_mpu */
  /* Infinite loop */
  for(;;)
  {
		uint8_t ret = atk_ms901m_init();
		if(ret != 0)osDelay(100);
		atk_ms901m_get_attitude(&attitude_dat,100);		
		osDelay(100);
  }
  /* USER CODE END Read_mpu */
}

/* USER CODE BEGIN Header_Send_serial */
/**
* @brief Function implementing the SERIAL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_serial */
void Send_serial(void const * argument)
{
  /* USER CODE BEGIN Send_serial */
  /* Infinite loop */
  for(;;)
  {
		osDelay(100);

  }
  /* USER CODE END Send_serial */
}

/* USER CODE BEGIN Header_test */
/**
* @brief Function implementing the TEST_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test */
void test(void const * argument)
{
  /* USER CODE BEGIN test */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		
		osDelay(1000);
  }
  /* USER CODE END test */
}

/* USER CODE BEGIN Header_PID_Control */
/**
* @brief Function implementing the PID_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_Control */
void PID_Control(void const * argument)
{
  /* USER CODE BEGIN PID_Control */
  /* Infinite loop */
  for(;;)
  {
    Motor_Set_Dis(5,20);
		osDelay(20);
  }
  /* USER CODE END PID_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

