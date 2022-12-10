/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// UART INPUTS
uint8_t user_input[1]; // where the user input will be stored

// UART OUTPUTS
uint8_t boot_message[] = " READY"; //start up message
uint8_t stopSensor_message[] = " sensors stop"; //start up message
uint8_t resumeSensor_message[] = " sensors resumed"; //start up message
uint8_t checkpoint_message[] = " point"; //start up message
uint8_t checkpoint1_message[] = " fir"; //start up message
uint8_t checkpoint2_message[] = " sec"; //start up message
uint8_t error_message[] = " CONFIRM BYTE IS WRONG. reset and try again"; //
uint8_t final_message[] = "";  //" GOOD JOB ROBOT ";

//set up the constants value
float Kp = 2;
//float Ki = 1;
//float Kd = 1.5;

int P, I, D;
//int lastError = 0;
//int errors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//int error_sum = 0;
int autoMode = 0;
int baseMotorSpeed = 37;
int outerSensorError = 80;
int innerSesnorError = 12;

int checkpoint = 0;
int numcheckpoint = 0;
int path = 0;

int timer1;
int timer2;
int timer3;
int timer4;
int stopSensor = 0;
int sensorB9 = 1;
int motorSpeedMultiplier = 10;

int leftTurn = 0;

int tmp1 = 5000;
int tmp2 = 0;

//int last_end = 0;	// 0 -> Left, 1 -> Right
//int last_idle = 0; // how long since last sensor read

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void motorControl(int pos_left, int pos_right) {
	if (pos_left < 0) { // check if negative value. negative value means move in opposite direction
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
				-motorSpeedMultiplier * pos_left); // 10 * input for left motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // 10 * input for left motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
				motorSpeedMultiplier * pos_left);
	}

	if (pos_right < 0) { //check if negative value. negative value means move in opposite direction
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				-motorSpeedMultiplier * pos_right); // 10 * input for right motor
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // 10 * input for right motor
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				motorSpeedMultiplier * pos_right);
	}
}

void turnPush() {
	timer3 = HAL_GetTick();
	while (HAL_GetTick() - timer3 < 800) {
		if (HAL_GetTick() - timer3 <= 400) {
			motorControl(0, 85);
		}
		if (HAL_GetTick() - timer3 > 400 && HAL_GetTick() - timer3 <= 800) {
			motorControl(0, -85);
		}
		if (HAL_GetTick() - timer3 > 800) {
			break;
		}

	}
	checkpoint = 4;
}

int sensorRead(void) { // reads the sensors
	int error = 0;

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) {

		stopSensor = 1;
		timer1 = HAL_GetTick();

		if (checkpoint == 0) { //at the 1st checkpoint go left (90 degree turn after B)
			if (path == 0) {

				while (HAL_GetTick() - timer1 < 325) { //rotate left motor at -40% and right motor at 60%
					motorControl(-35, 50);
				}
				checkpoint = 1;
				stopSensor = 0;
			} else if (path == 1) {
				stopSensor = 1;
				timer1 = HAL_GetTick();
				while (HAL_GetTick() - timer1 < 325) { //rotate left motor at 60% and right motor at -40%
					motorControl(50, -35);
				}
				checkpoint = 3;
				timer4 = HAL_GetTick();
				stopSensor = 0;

			}
		} else if (checkpoint == 1) { //at the 2nd checkpoint go left (45 degree turn after B)

			while (HAL_GetTick() - timer1 < 150) { //rotate left motor at -40% and right motor at 60%
				motorControl(-35, 50);
			}
			checkpoint = 2;
			stopSensor = 0;

		} else if (checkpoint == 2) { //at the 3rd checkpoint go right (90 degree turn after B)

			/*if (path == 1) {
			 while (HAL_GetTick() - timer1 < 350) { //rotate left motor at 60% and right motor at -40%
			 motorControl(60, -40);
			 }
			 } else if (path == 0) {*/
			timer1 = HAL_GetTick();
			while (HAL_GetTick() - timer1 < 125) { //rotate left motor at 60% and right motor at -40%
				motorControl(-35, 55);
			}
//			}
			checkpoint = 3;
			timer4 = HAL_GetTick();
			stopSensor = 0;
			baseMotorSpeed = 80;
		}

	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET
			&& stopSensor == 0) {
		if (checkpoint != 4) {
			error += outerSensorError;
		}
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET
			&& stopSensor == 0) {
		error += innerSesnorError;
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET
			&& stopSensor == 0) {
		error += -innerSesnorError;
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET
			&& stopSensor == 0) {

		if (checkpoint == 3 && HAL_GetTick() - timer4 > 3000) {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			turnPush();

		} else if (checkpoint != 4) {
			error += -outerSensorError;
		}
	}

	return error;
}

/*void pastErrors(int error) { // shifts the values of errors array by one then add the newest error to the first errors value -> errors[0]
 for (int i = 9; i > 0; i--) {
 errors[i] = errors[i - 1];
 }
 errors[0] = error;
 }

 int sumErrors(int num_of_past_errors) { // returns the sum of a custom number of past errors
 int sum = 0;
 for (int i = 0; i < num_of_past_errors; i++) {
 sum += errors[i];
 }
 return sum;
 }*/

void PIDControl(void) {
	int error = sensorRead();
//	pastErrors(error);

	P = error;
//	I = sumErrors(5);
//	D = lastError - error;
//	lastError = error;

//	int motorspeed = P * Kp + I * Ki + D * Kd;
	int motorspeed = P * Kp;
	int motorspeedl = baseMotorSpeed + motorspeed;
	int motorspeedr = baseMotorSpeed - motorspeed;

	if (motorspeedl > 100)
		motorspeedl = 100;
	else if (motorspeedl < -100)
		motorspeed = -100;
	if (motorspeedr > 100)
		motorspeedr = 100;
	else if (motorspeedr < -100)
		motorspeedr = -100;

	motorControl(motorspeedl, motorspeedr);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1) { // callback for when we receive an input

	int input = (int) user_input[0];

	if (input == 70) {
		motorControl(65, 65);
	} else if (input == 66) {
		motorControl(-65, -65);
	} else if (input == 76) {
		motorControl(-65, 65);
	} else if (input == 82) {
		motorControl(65, -65);
	} else if (input == 71) {
		motorControl(40, 100);
	} else if (input == 73) {
		motorControl(100, 40);
	} else if (input == 72) {
		motorControl(-40, -100);
	} else if (input == 77) {
		motorControl(-40, -100);
	} else if (input == 83) {
		motorControl(0, 0);
	} else if (input == 86) {
		motorControl(-100, 100);
	}

	else
// if not one of the above send error message
		HAL_UART_Transmit_IT(huart1, error_message, sizeof(error_message));

	HAL_UART_Transmit_IT(huart1, user_input, 2);
	HAL_UART_Receive_IT(huart1, user_input, 2);

}

int main(void) {
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
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
//		l293d pins
//	A3 -> enable 1-2
//	A0 -> enable 3-4
//  A2 -> input 1
//	A1 -> input 2
//	A6 -> input 3
//	A7 -> input 4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // enable 1-2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // enable 3-4

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Initialise the timer 2 channel 2 for first motor (input 2)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Initialise the timer 2 channel 3 for first motor (input 1)

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Initialise the timer 3 channel 1 for second motor (input 3)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Initialise the timer 3 channel 2 for second motor (input 4)

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Initialise the timer 3 channel 2 for servo motor
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 25); //2.5% (25/1000)*100

	HAL_UART_Transmit_IT(&huart1, boot_message, sizeof(boot_message)); // sends READY on start-up
	HAL_UART_Receive_IT(&huart1, user_input, 2);

	int buttonB1_debounce = 0;
	int buttonB11_debounce = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (autoMode == 1) {
			/*if (HAL_GetTick() - timer2 < 3750 && path == 0) {
				motorSpeedMultiplier = 25;
				innerSesnorError = 6;
			} else {
				motorSpeedMultiplier = 10;
				innerSesnorError = 12;
			}*/
			PIDControl();
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET
				&& buttonB1_debounce == 0) {

			buttonB1_debounce = 1;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET
				&& buttonB1_debounce == 1) {
			if (autoMode == 1) {
				autoMode = 0;
				motorControl(0, 0);
				checkpoint = 0;
			} else if (autoMode == 0) {
				autoMode = 1;
				timer2 = HAL_GetTick();
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			}
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			buttonB1_debounce = 0;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET
				&& buttonB11_debounce == 0) {

			buttonB11_debounce = 1;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET
				&& buttonB11_debounce == 1) {
			if (path == 0) {
				path = 1;
			} else if (path == 1) {
				path = 0;
			}
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			buttonB11_debounce = 0;
		}

		//blinks the on-board led at 2Hz
		if (HAL_GetTick() - timer1 >= 250) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			timer1 = HAL_GetTick();
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* USER CODE END 3 */
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1440;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1440;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1440;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB11 PB6 PB7
	 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
