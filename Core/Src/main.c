/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actuator.h"
#include "sbus.h"
#include "imu.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// actuator configuration and pins

// Left Front Motor - PD12
actuator_config_t left_front_motor = {
  .timer = &htim4,
  .channel = TIM_CHANNEL_1,
  .max_pulse_width = 2,
  .min_pulse_width = 1,
};

// Right Front Motor - PD13
actuator_config_t right_front_motor = {
  .timer = &htim4,
  .channel = TIM_CHANNEL_2,
  .max_pulse_width = 2,
  .min_pulse_width = 1,
};

// Left Back Motor - PD14
actuator_config_t left_back_motor = {
  .timer = &htim4,
  .channel = TIM_CHANNEL_3,
  .max_pulse_width = 2,
  .min_pulse_width = 1,
};

// Right Back Motor - PD15
actuator_config_t right_back_motor = {
  .timer = &htim4, 
  .channel = TIM_CHANNEL_4,
  .max_pulse_width = 2,
  .min_pulse_width = 1,
};



// input channels configuration

channel_info_t teleop_commands = {
  .channels = {},
  .throttle = 0,
  .roll = 0,
  .pitch = 0,
  .yaw = 0,
  .frame_lost = false,
  .failsafe_activated = false,
};

// IMU Configuration
MPU6050_t imu = {
  .Accel_X_OFFSET = 0,
  .Accel_Y_OFFSET = 0,
  .Accel_Z_OFFSET = 0,
  .Ax = 0,
  .Ay = 0,
  .Az = 0,
  .Gyro_X_OFFSET = 0,
  .Gyro_Y_OFFSET = 0,
  .Gyro_Z_OFFSET = 0,
  .Gx = 0,
  .Gy = 0,
  .Gz = 0,
  .sensorRawGx = 15,
  .updatePeriod = 0,
  .temperature = 0,
  .accelRoll = 0,
  .accelPitch = 0,
  .gyroRoll = 0,
  .gyroPitch = 0,
  .alpha = 0,
  .filteredRoll = 0,
  .filteredPitch = 0,  
};

// Roll and Pitch Controller

pid_controller_t roll_rate_control = {
  .curr_state = 0,
  .output = 0,
  .setpoint = 0,
  .kp = 0,
  .ki = 0,
  .kd = 0,
  .output_max = 0,
  .output_min = 0,
  .error_accumulation = 0,
  .prev_error = 0,
  .control_loop_period = 0.0050,
};

pid_controller_t pitch_rate_control = {
  .curr_state = 0,
  .output = 0,
  .setpoint = 0,
  .kp = 0,
  .ki = 0,
  .kd = 0,
  .output_max = 0,
  .output_min = 0,
  .error_accumulation = 0,
  .prev_error = 0,
  .control_loop_period = 0.0050,
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
volatile bool control_loop_deadline = false;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {  
  if (htim == &htim3) {
    control_loop_deadline = true;
  }
}


// callback for incoming UART messages - check for packet header and read more bytes as required
// Defining intermediate buffer to be populated and parsed

volatile uint8_t header_bytes[25] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart -> Instance == USART3) {
    if (header_bytes[0] == HEADER_BYTE) {
      update_channels(&teleop_commands, header_bytes);
    }
  }
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize timer interrupts

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim5);
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);

  // Initialize UART RX Interrupts
  pwm_init(&left_front_motor);
  pwm_init(&right_front_motor);
  pwm_init(&left_back_motor);
  pwm_init(&right_back_motor);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  typedef enum {UNCALIBRATED, DISARMED, ARMED} quadcopter_state_t;

  quadcopter_state_t current_state = UNCALIBRATED;

  /* USER CODE BEGIN PFP */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  /* USER CODE END PFP */
  // Kick off receiving new packets

  HAL_UART_Receive_IT(&huart3, header_bytes, 25);
  while (1)
  {
    HAL_UART_Receive_IT(&huart3, header_bytes, 25);
    /*
    This is the superloop during which the following occurs:
    1.) All peripherals, sensor drivers, etc. are initialized
    2.) A state machine holding and managing the different states of the 
        quadcopter is setup and tracks when conditions to switch states occur

    The following is a list of states for the quadcopter

    UNCALIBRATED: 
    State when the quadcopter is just powered on and the ESC calibration isn't complete, IMU, other sensors
    are still being initialized

    DISARMED:
    Once the calibration and other initialization is complete, the drone is able to fly but is disarmed. 
    This is fixed by a specific arming pattern (likely a switch to read from or a specific sequence of
    RC stick movements)

    ARMED:
    Drone has been armed and is ready to fly. At this stage, rotors start spinning at a slow speed and 
    corresponding stick movements will initiate flight

    The drone can switch from ARMED to DISARMED via the flick of the arm switch, in case of a situation
    where the pilot deems it to be unfit. Furthermore, it will be able to put itself in "DISARMED" state
    by switching the ARM switch back.
    

    */

    // Print all of the channel values:
    // for (int i = 0; i < 4; i++) {
    //   printf("Channel %d: %d\n", i, teleop_commands.channels[i]);
    // }

    // Update the IMU readings, take most recent RC commands, and run control loop

    switch(current_state) {
      case UNCALIBRATED:
        printf("Calibrating\n");
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
        // Run calibration steps and advance state to DISARMED
        // vajra_calibrate();

        // Initialize IMU
        uint8_t check_dev_val = MPU6050_Init(&hi2c1, &imu);
        printf("Checkdev: %u\n", check_dev_val);
        // if (MPU6050_Init(&hi2c1, &imu)) {
        //   printf("IMU Initialized succesfully\n");
        // } else {
        //   printf("IMU Not initialized succesfully\n");
        // }
        printf("HOES MAD FR\n");
        current_state = ARMED;
        break;
      case DISARMED:
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);

        // When the quadcopter is disarmed, it can only advance to an armed state if the appropriate switch is flicked
        if (teleop_commands.arm_switch_status) {
          
          current_state = ARMED;
        } else {
          // Continue in disarmed state 
        }
        break;
      case ARMED:
        HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
        // state estimation and control loop goes ahead based on flag set in 400 Hz timer interrupt
        if (control_loop_deadline) {
          // Update the IMU readings, take most recent RC commands, and run control loop
          MPU6050_Update_All(&hi2c1, &imu);
          printf("Roll Degrees: %f\n", imu.Gx);
          printf("Pitch Degrees: %f\n", imu.Gy);
          // Update rollm pitch and yaw rate setpoints with min/max being 45 deg/seconds
          // TODO: Setpoint mapper function defined in sbus.h

          // Update setpoint on all three axes controllers
          // update_controls(&imu, &teleop_commands);

          // Output mixer - take outputs of three controllers, and apply on each motor 
          // depending on position and direction
          // TODO: Develop function to write to all four RC outputs at once
        }
        // TODO: Check if disarm switch/channel is flicked, and if so, switch back into DISARMED state
        if (!teleop_commands.arm_switch_status) {
          // current_state = DISARMED;
        }
        break;
    }

    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
