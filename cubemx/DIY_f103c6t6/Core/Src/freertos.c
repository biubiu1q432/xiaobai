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
#include "pid.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Motor_Stat Left_Motor;	/*左轮数据*/
Motor_Stat Right_Motor;	/*右轮数据*/

/*pid*/
extern Pid incremental_pid;
extern Pid local_pid;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//测试
float temp1_pwm;
float temp2_pwm;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ENCODER_TASK */
osTimerId_t ENCODER_TASKHandle;
const osTimerAttr_t ENCODER_TASK_attributes = {
  .name = "ENCODER_TASK"
};
/* Definitions for PID_TASK */
osTimerId_t PID_TASKHandle;
const osTimerAttr_t PID_TASK_attributes = {
  .name = "PID_TASK"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ENCODER_(void *argument);
void PID(void *argument);

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

  /* Create the timer(s) */
  /* creation of ENCODER_TASK */
  ENCODER_TASKHandle = osTimerNew(ENCODER_, osTimerPeriodic, NULL, &ENCODER_TASK_attributes);

  /* creation of PID_TASK */
  PID_TASKHandle = osTimerNew(PID, osTimerPeriodic, NULL, &PID_TASK_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	
	//编码器控制频率设定 单位 ms
	osTimerStart(ENCODER_TASKHandle,ENCODER_TIME*1000);
	//PID控制频率设定 单位 ms
	osTimerStart(PID_TASKHandle,PID_TIME*1000);
	
	
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* ENCODER_ function */
void ENCODER_(void *argument)
{
  /* USER CODE BEGIN ENCODER_ */
	/*左侧*/
	float left_distance = 0;
	//正反
	int left_tmp_ec = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	if(left_tmp_ec > 60000)	left_tmp_ec = (left_tmp_ec - 65536.3);

	Left_Motor.EncodeCount = left_tmp_ec;
	left_distance = (float)(Left_Motor.EncodeCount/EC_1)*C_1;
	Left_Motor.Distance += left_distance;
	Left_Motor.Motorspeed = left_distance/ENCODER_TIME;
	
	printf("%.0f  					%.2f  					%.2f  \n",Left_Motor.EncodeCount,left_distance,Left_Motor.Motorspeed);

	
  /* USER CODE END ENCODER_ */
}

/* PID function */
void PID(void *argument)
{
  /* USER CODE BEGIN PID */
//	temp2_pwm = PID_realize(&local_pid,Left_Motor.Distance);
//	incremental_pid.target_val = temp2_pwm;
//	temp1_pwm = Incremental_PID(&incremental_pid,Left_Motor.Motorspeed);
//	Motor_Set(temp1_pwm,0);
  
  /* USER CODE END PID */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

