/* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * File Name          : freertos.c
//   * Description        : Code for freertos applications
//   ******************************************************************************
//   * @attention
//   *
//   * Copyright (c) 2024 STMicroelectronics.
//   * All rights reserved.
//   *
//   * This software is licensed under terms that can be found in the LICENSE file
//   * in the root directory of this software component.
//   * If no LICENSE file comes with this software, it is provided AS-IS.
//   *
//   ******************************************************************************
//   */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
/* Definitions for control_loop */
osThreadId_t control_loopHandle;
const osThreadAttr_t control_loop_attributes = {
  .name = "control_loop",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for logger */
osThreadId_t loggerHandle;
const osThreadAttr_t logger_attributes = {
  .name = "logger",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for logger_handle */
osThreadId_t logger_handleHandle;
const osThreadAttr_t logger_handle_attributes = {
  .name = "logger_handle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void start_control_loop(void *argument);
void logger_init(void *argument);

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
//   /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//   /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
//   /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
//   /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of control_loop */
  control_loopHandle = osThreadNew(start_control_loop, NULL, &control_loop_attributes);

  /* creation of logger */
  loggerHandle = osThreadNew(logger_init, NULL, &logger_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//   /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
//   /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_start_control_loop */
// /**
//   * @brief  Function implementing the control_loop thread.
//   * @param  argument: Not used
//   * @retval None
//   */
/* USER CODE END Header_start_control_loop */
void start_control_loop(void *argument)
{
  /* USER CODE BEGIN start_control_loop */
//   /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, 100);
    HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
    // osDelay(200);
  }
  /* USER CODE END start_control_loop */
}

/* USER CODE BEGIN Header_logger_init */
// /**
// * @brief Function implementing the logger thread.
// * @param argument: Not used
// * @retval None
// */
/* USER CODE END Header_logger_init */
void logger_init(void *argument)
{
  /* USER CODE BEGIN logger_init */
//   /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, 200);    
    HAL_GPIO_TogglePin(GPIOB, LD2_Pin);

  }
  /* USER CODE END logger_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

