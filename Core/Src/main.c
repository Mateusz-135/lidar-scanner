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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TFLuna.h"
#include "Servo.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// #####  #####   ###   #####  #  #####     #   #   ###   ####   #   ###   ####   #      #####  #####
// #        #    #   #    #    #  #         #   #  #   #  #   #  #  #   #  #   #  #      #      #
// #####    #    #####    #    #  #         #   #  #####  ####   #  #####  ####   #      #####  #####
//     #    #    #   #    #    #  #          # #   #   #  #  #   #  #   #  #   #  #      #          #
// #####    #    #   #    #    #  #####       #    #   #  #   #  #  #   #  ####   #####  #####  #####

static uint8_t scan_in_progress = 0;

static float increment_yaw = 1.0f;
static float increment_pitch = 1.0f;
static uint16_t change_pitch_direction_count = 0;
static long int expected_sample_count = 0;
static long int sample_count = 0;

static uint16_t sensor_frequency = 500/5;

static int counter = 0;

static uint8_t received_byte[1];
#define SEND_DATA_SIZE 14
static uint8_t data14byte[SEND_DATA_SIZE];
static uint8_t read_step_info_flag = 0;

#define LUNA_DATA_SIZE  8
static uint8_t tfluna_data_frame[LUNA_DATA_SIZE];

static int16_t pitch = 0;
static int16_t yaw = 0;

static Servo YawServo;
static Servo PitchServo;
static TFLuna sensor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void serialWriteMeasurement14Byte_IT(UART_HandleTypeDef *huart);
void executeScan(Servo *yawServo, Servo *pitchServo, TFLuna *sensor, GPIO_PinState *stop);
void homePosition(Servo *YawServo, Servo *PitchServo);
void flushBuffer(UART_HandleTypeDef *huart);
void executeScanTopBottom(Servo *YawServo, Servo *PitchServo, TFLuna *sensor);
void finishScanning(Servo *YawServo, Servo *PitchServo, TFLuna *sensor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// #   #   ###   ####   #####     #  #   #  #####  #####  ####   ####   #   #  ####   #####  #####
// #   #  #   #  #   #    #       #  ##  #    #    #      #   #  #   #  #   #  #   #    #    #
// #   #  #####  ####     #       #  # # #    #    #####  ####   ####   #   #  ####     #    #####
// #   #  #   #  #  #     #       #  #  ##    #    #      #  #   #  #   #   #  #        #        #
//  ###   #   #  #   #    #       #  #   #    #    #####  #   #  #   #   ###   #        #    #####

typedef enum int8_t{
	UNKNOWN = 0x60,
	STOP_ALL = 0x61,
	START_SCANNING = 0x62,
	OK = 0x63,
	FRAME_BEGIN = 0x64,
	FRAME_END = 0x65,
	SCAN_FINISHED = 0x66,
	STEP_INFO = 0x67
} MessageCode;

// Interrupt after finished transmission (using HAL_UART_Transmit_IT())
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) // tf luna
	{
	}
	else if(huart == &huart2) // usb
	{
		if(sample_count == expected_sample_count && scan_in_progress == 1)
		{
			finishScanning(&YawServo, &PitchServo, &sensor);
			return;
		}

		if(scan_in_progress == 1)
		{
			HAL_UART_Receive_IT(&huart1, tfluna_data_frame, LUNA_DATA_SIZE);
			return;
		}

		HAL_UART_Receive_IT(&huart2, received_byte, 1);

	}
}

// Interrupt after receiving data (using HAL_UART_Recieve_IT())
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) // tf luna
	{
		if(counter++ % 2 == 0)
		{
			sample_count++;

			serialWriteMeasurement14Byte_IT(&huart2);

			if(sample_count > 1)
			{
				if(sample_count % change_pitch_direction_count == 0)
				{
					servoMoveBy(&YawServo, increment_yaw);
				}
				if(sample_count % change_pitch_direction_count == 1)
				{
					increment_pitch = -increment_pitch;
				}
			}
			servoMoveBy(&PitchServo, increment_pitch);
		}
		else
			HAL_UART_Receive_IT(&huart1, tfluna_data_frame, LUNA_DATA_SIZE);
	}
	else if(huart == &huart2) // usb
	{
		if(read_step_info_flag == 1)
		{
			read_step_info_flag = 0;
			increment_yaw = (float)(*received_byte) / 10.0;
			increment_pitch = (float)(*received_byte) / 10.0;
			HAL_UART_Receive_IT(&huart2, received_byte, 1);
			return;
		}

		switch(*received_byte)
		{
		case STOP_ALL:
			homePosition(&YawServo, &PitchServo);
			HAL_UART_Receive_IT(&huart2, received_byte, 1);
			break;
		case START_SCANNING:
			executeScanTopBottom(&YawServo, &PitchServo, &sensor);
			break;
		case STEP_INFO:
			read_step_info_flag = 1;
			HAL_UART_Receive_IT(&huart2, received_byte, 1);
			break;
		default:
			HAL_UART_Receive_IT(&huart2, received_byte, 1);
		}
		*received_byte = UNKNOWN;

	}
}

// #####  #   #  #   #  #####  #####  #  #####  #   #  #####
// #      #   #  ##  #  #        #    #  #   #  ##  #  #
// #####  #   #  # # #  #        #    #  #   #  # # #  #####
// #      #   #  #  ##  #        #    #  #   #  #  ##      #
// #       ###   #   #  #####    #    #  #####  #   #  #####

void executeScanTopBottom(Servo *YawServo, Servo *PitchServo, TFLuna *sensor)
{
	scan_in_progress = 1;

	if(increment_pitch < 0.0)
		increment_pitch = -increment_pitch;

	counter = 0;
	sample_count = 0;

	float pitch_range = PitchServo->max_angle_allowed_by_user - PitchServo->min_angle_allowed_by_user;
	float yaw_range = YawServo->max_angle_allowed_by_user - YawServo->min_angle_allowed_by_user;

	change_pitch_direction_count = (pitch_range / increment_pitch) + 1;
	expected_sample_count = (((pitch_range / increment_pitch) + 1) * ((yaw_range / increment_yaw) + 1));

	homePosition(YawServo, PitchServo);

	TFLunaEnableOutput(sensor, 1);

	HAL_UART_Receive_IT(sensor->uart_handler, tfluna_data_frame, LUNA_DATA_SIZE);
}

void serialWriteMeasurement14Byte_IT(UART_HandleTypeDef *huart)
{
	pitch = (int16_t)(10.0f * PitchServo.position);
	yaw = (int16_t)(10.0f * YawServo.position);

	data14byte[0] = (int8_t)FRAME_BEGIN;
	data14byte[1] = tfluna_data_frame[0];
	data14byte[2] = tfluna_data_frame[1];
	data14byte[3] = (uint8_t)pitch;
	data14byte[4] = (uint8_t)(pitch >> 8);
	data14byte[5] = (uint8_t)yaw;
	data14byte[6] = (uint8_t)(yaw >> 8);
	data14byte[7] = tfluna_data_frame[2];
	data14byte[8] = tfluna_data_frame[3];
	data14byte[9] = tfluna_data_frame[4];
	data14byte[10] = tfluna_data_frame[5];
	data14byte[11] = tfluna_data_frame[6];
	data14byte[12] = tfluna_data_frame[7];
	data14byte[13] = (int8_t)FRAME_END;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data14byte, SEND_DATA_SIZE);
}

void finishScanning(Servo *YawServo, Servo *PitchServo, TFLuna *sensor)
{
	TFLunaEnableOutput(sensor, 0);
	homePosition(YawServo, PitchServo);
	scan_in_progress = 0;
	sample_count = 0;

	data14byte[0] = (uint8_t)SCAN_FINISHED;
	data14byte[1] = (uint8_t)SCAN_FINISHED;
	data14byte[2] = (uint8_t)SCAN_FINISHED;
	data14byte[3] = (uint8_t)SCAN_FINISHED;
	data14byte[4] = (uint8_t)SCAN_FINISHED;
	data14byte[5] = (uint8_t)SCAN_FINISHED;
	data14byte[6] = (uint8_t)SCAN_FINISHED;
	data14byte[7] = (uint8_t)SCAN_FINISHED;
	data14byte[8] = (uint8_t)SCAN_FINISHED;
	data14byte[9] = (uint8_t)SCAN_FINISHED;
	data14byte[10] = (uint8_t)SCAN_FINISHED;
	data14byte[11] = (uint8_t)SCAN_FINISHED;
	data14byte[12] = (uint8_t)SCAN_FINISHED;
	data14byte[13] = (uint8_t)SCAN_FINISHED;

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)data14byte, SEND_DATA_SIZE);
}

void homePosition(Servo *YawServo, Servo *PitchServo)
{
	float homeYaw = YawServo->min_angle_allowed_by_user, homePitch = PitchServo->min_angle_allowed_by_user;

	servoMoveTo(PitchServo, homePitch);
	servoMoveTo(YawServo, homeYaw);
}

void flushBuffer(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive(sensor.uart_handler, tfluna_data_frame, 200, 10);
}

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // #  #   #  #  #####  #   ###   #      #  #####   ###   #####  #  #####  #   #
  // #  ##  #  #    #    #  #   #  #      #  #      #   #    #    #  #   #  ##  #
  // #  # # #  #    #    #  #####  #      #  #####  #####    #    #  #   #  # # #
  // #  #  ##  #    #    #  #   #  #      #      #  #   #    #    #  #   #  #  ##
  // #  #   #  #    #    #  #   #  #####  #  #####  #   #    #    #  #####  #   #

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  init(&YawServo, &htim3, TIM_CHANNEL_2);
  YawServo.min_pulse_width_microseconds = 600 /* + offset */;
  YawServo.max_pulse_width_microseconds = 2570 /* + offset */;
  YawServo.min_angle_possible = -90.0;
  YawServo.max_angle_possible = 90.0;
  YawServo.min_angle_allowed_by_user = -90.0;
  YawServo.max_angle_allowed_by_user = 90.0;

  init(&PitchServo, &htim3, TIM_CHANNEL_1);
  PitchServo.min_pulse_width_microseconds = 500 + 10/* + offset */;
  PitchServo.max_pulse_width_microseconds = 2600 + 10 /* + offset */;
  PitchServo.min_angle_possible = -30.0;
  PitchServo.max_angle_possible = 150.0;
  PitchServo.min_angle_allowed_by_user = -30.0;
  PitchServo.max_angle_allowed_by_user = 90.0;

  sensor.uart_handler = &huart1;
  TFLunaEnableOutput(&sensor, 0);
  TFLunaSetOutputFormat(&sensor, CM_8BYTE);
  TFLunaSetFrequency(&sensor, sensor_frequency);

  flushBuffer(sensor.uart_handler);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // #   #   ###   #  #   #     #   #  #   #  #  #      #####     #      #####  #####  ####
  // ## ##  #   #  #  ##  #     #   #  #   #  #  #      #         #      #   #  #   #  #   #
  // # # #  #####  #  # # #     # # #  #####  #  #      #####     #      #   #  #   #  ####
  // #   #  #   #  #  #  ##     # # #  #   #  #  #      #         #      #   #  #   #  #
  // #   #  #   #  #  #   #      # #   #   #  #  #####  #####     #####  #####  #####  #

  HAL_UART_Receive_IT(&huart2, received_byte, 1);

  while (1)
  {
	  if(scan_in_progress == 0)
		  homePosition(&YawServo, &PitchServo);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  } // while(1)

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
