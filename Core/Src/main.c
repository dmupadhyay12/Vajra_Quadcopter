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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actuator.h"
#include "rc_input.h"
#include "sbus.h"

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

// Throttle channel is Channel 1 - Pin PA6
input_channel_t throttle_channel = {
  .current_pulse_width = 0,
  .max_pulse_width = 3200,
  .min_pulse_width = 0,
};

// Roll channel is Channel 2 - Pin PC7
input_channel_t roll_channel = {
  .current_pulse_width = 0,
  .max_pulse_width = 3200,
  .min_pulse_width = 0,
};

// Yaw channel is Channel 3 - Pin PC8
input_channel_t yaw_channel = {
  .current_pulse_width = 0,
  .max_pulse_width = 3200,
  .min_pulse_width = 0,
};

// Pitch channel is Channel 4 - Pin PB1
input_channel_t pitch_channel = {
  .current_pulse_width = 0,
  .max_pulse_width = 3200,
  .min_pulse_width = 0,
};

channel_info_t teleop_commands = {
  .channels = {},
  .throttle = 0,
  .roll = 0,
  .pitch = 0,
  .yaw = 0,
  .frame_lost = false,
  .failsafe_activated = false,
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {  
  // TODO: Implement system utime
}


// callback for incoming UART messages - check for packet header and read more bytes as required
// Defining intermediate buffer to be populated and parsed

volatile uint8_t header_bytes[25] = {0};
volatile bool conversion_pending = false;
volatile first_byte_detected = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  first_byte_detected = true;
  if (huart -> Instance == USART3) {
    if (header_bytes[0] == HEADER_BYTE) {
      update_channels(&teleop_commands, header_bytes);
    }
    // if (!conversion_pending) {
    //   // check if the byte is the packet header
    //   printf("Header: %x\n", header_bytes[0]);
    //   if (header_bytes[0] == HEADER_BYTE) {
    //     // printf("HEADER BYTE DETECTED!\n");
    //     header_bytes[0] = 0; // reset first byte 
    //     conversion_pending = 1;
    //     HAL_UART_Receive_IT(&huart, header_bytes + 1, 20);
    //   }
    // } else {
    //   // process the bytes and reset flag
    //   printf("Read all 25 bytes!");
    //   update_channels(&teleop_commands, header_bytes);
    // } 
  }
}

volatile bool capture_detected = 0;
volatile int channel_during_capture = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

  static volatile bool rising_edge_captured_channel1 = false;
  static volatile uint16_t rising_edge_value_channel1 = 0;
  static volatile uint16_t falling_edge_value_channel1 = 0;

  static volatile bool rising_edge_captured_channel2 = false;
  static volatile uint16_t rising_edge_value_channel2 = 0;
  static volatile uint16_t falling_edge_value_channel2 = 0;

  static volatile bool rising_edge_captured_channel3 = false;
  static volatile uint16_t rising_edge_value_channel3 = 0;
  static volatile uint16_t falling_edge_value_channel3 = 0;

  static volatile bool rising_edge_captured_channel4 = false;
  static volatile uint16_t rising_edge_value_channel4 = 0;
  static volatile uint16_t falling_edge_value_channel4 = 0;

  channel_during_capture = htim -> Channel;
  if (channel_during_capture == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (!rising_edge_captured_channel1) {
      rising_edge_value_channel1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
      rising_edge_captured_channel1 = true;
    } else {
      capture_detected = true;
      falling_edge_value_channel1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

      // update the actuator channel percentage value
      update_pulse_width(&throttle_channel, rising_edge_value_channel1, falling_edge_value_channel1);
      rising_edge_captured_channel1 = false;
    }
  } 
  else if (channel_during_capture == HAL_TIM_ACTIVE_CHANNEL_2) {
    if (!rising_edge_captured_channel2) {
      rising_edge_value_channel2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
      rising_edge_captured_channel2 = true;
    } else {
      // capture_detected = true;
      falling_edge_value_channel2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

      // update the actuator channel percentage value
      update_pulse_width(&roll_channel, rising_edge_value_channel2, falling_edge_value_channel2);
      rising_edge_captured_channel2 = false;
    }
  } 
  else if (channel_during_capture == HAL_TIM_ACTIVE_CHANNEL_3) {
    if (!rising_edge_captured_channel3) {
      rising_edge_value_channel1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
      rising_edge_captured_channel3 = true;
    } else {
      falling_edge_value_channel3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

      // update the actuator channel percentage value
      update_pulse_width(&yaw_channel, rising_edge_value_channel3, falling_edge_value_channel3);
      rising_edge_captured_channel3 = false;
    }
  } 
  else if (channel_during_capture == HAL_TIM_ACTIVE_CHANNEL_4) {
    if (!rising_edge_captured_channel4) {
      rising_edge_value_channel4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
      rising_edge_captured_channel4 = true;
    } else {
      falling_edge_value_channel4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);

      // update the actuator channel percentage value
      update_pulse_width(&pitch_channel, rising_edge_value_channel4, falling_edge_value_channel4);
      rising_edge_captured_channel4 = false;
    }
  } else {
    // Channel irrelevant to operation of flight software
  }
}

float get_input_percentage(input_channel_t* channel) {
  return (float) (channel -> current_pulse_width - channel -> min_pulse_width) / (float)(channel -> max_pulse_width - channel -> min_pulse_width);
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
  UART_HandleTypeDef huart;

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

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim5);
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);

  // Start input capture interrupts
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

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

    // Update RC control state

    pwm_update_percentage(&left_front_motor, 25);
    pwm_update_percentage(&right_front_motor, 50);
    pwm_update_percentage(&left_back_motor, 75);
    pwm_update_percentage(&right_back_motor, 100);

    float roll_input = get_channel_percentage(&roll_channel);
    float yaw_input = get_channel_percentage(&yaw_channel);
    float throttle_input = get_channel_percentage(&throttle_channel);
    float pitch_input = get_channel_percentage(&pitch_channel);

    // Print all of the channel values:
    // for (int i = 0; i < 4; i++) {
    //   printf("Channel %d: %d\n", i, teleop_commands.channels[i]);
    // }

    switch(current_state) {
      case UNCALIBRATED:
        // Run calibration steps and advance state to DISARMED
        // vajra_calibrate();
        current_state = DISARMED;
        break;
      case DISARMED:
        // When the quadcopter is disarmed, it can only advance to an armed state if both sticks
        // are in the bottom left of the box. 
        // This occurs when all four channels are less than 10% 
        if ((yaw_input < 10.0) && (roll_input < 10.0) && (throttle_input < 10.0) && (pitch_input < 10.0)) {
          current_state = ARMED;
          // TODO: Set LEDs or some form of audio-visual feedback indicating that the quadcopter is armed
        } else {
          // Continue in disarmed state 
        }
        break;
      case ARMED:
        // state estimation and control loop goes ahead - in each timer interrupt iteration you must 
        // create a 
        // TODO: Check if disarm switch/channel is flicked, and if so, switch back into DISARMED state
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
