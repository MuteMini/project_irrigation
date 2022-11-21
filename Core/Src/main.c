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

//Primary Functions
uint16_t Request_Moisture_Data( void );
void Average_Moisture_Data( double*, char, uint16_t );
void Adjustor_Change( const uint16_t, char *, const char );
void Request_Moisture_Threshold();
void Moisture_Level_Vs_Threshold();
void Open_Motor();

//Helper Functions
void Set_LED_Pin( char, GPIO_PinState );
char Increment_Timer( char );

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ADD_LED(a) ((a < 4) ? 1 : 0)
#define MINUS_LED(a) ((a > 0) ? -1 : 0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* 	Maximum available total moisture value to calculate moisture percentage
 	Calculated by (Max Volt / Nominal Reference) * 4092
 	Pin out Max Voltage, through testing, is about 2.2 volts
 	Nominal Reference Voltage is around 2.0~2.5 */
const short MAX_MOISTURE = 1800;

//	Calculates and stores how much time has passed
uint16_t time_passed = 0;
//	Holds a value between [0, 240): unit is 0.5s
short time_count = 0;

// 	Represents button presses using boolean values.
char b_left_on = 0;
char b_right_on = 0;
// 	Represents what LED should be on for moisture sensor.
char led_light = 2;

// 	Used to calculate average in O(1) memory
char average_size = 0;
double soil_moisture = 0;

// 	A boolean-type value to test if the MCU is polling the moisture sensor
char is_polling = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** @function	Request_Moisture_Data
 *
 * 	@brief	Uses ADC driver to poll and return the value of the ADC IN9 pin
 *
 * 	@return ADC IN9 pin input
 */
uint16_t Request_Moisture_Data()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1);
}

/** @function	Average_Moisture_Data
 *
 * 	@brief	Updates the average parameter given it's current size and new value
 * 			without storing into an array
 *
 *  @param[in/out]	average 	Average value being manipulated
 *	@param[in]		size		Number of variables currently averaged
 *	@param[in]		newVal		New value to be added to average
 *
 *	@retval None
 */
void Average_Moisture_Data( double *average, char size, uint16_t newVal )
{
	(*average) += (newVal - (*average)) / size;
}

/** @function	Adjustor_Change
 *
 * 	@brief	Generalizes the code to turn on and off LED values based on button
 * 			inputs by passing certain variables through pointers.
 *
 *  @param[in]		BUTTON_PIN 	Holds the GPIO pin of the button
 *	@param[in/out]	b_on		Used to only activate code on button's first press
 *	@param[in]		increase	Determines if led_light is increased or deceased.
 *
 *	@retval None
 */
void Adjustor_Change( const uint16_t BUTTON_PIN, char *b_on, const char increase ) 
{
	//	If button is being pressed for the first time,
	if( !HAL_GPIO_ReadPin( GPIOB, BUTTON_PIN ) && !(*b_on) )
	{
		//	Set button as being pressed
		*b_on = 1;

		//	Reset current pin
		Set_LED_Pin( led_light, GPIO_PIN_RESET );

		//	Change led_light according to increase variable
		led_light += (increase ? ADD_LED( led_light ) : MINUS_LED( led_light ));

		//	Sets new led pin on
		Set_LED_Pin( led_light, GPIO_PIN_SET );
	}
	//	If button is released, set b_on as being off.
	else if( HAL_GPIO_ReadPin( GPIOB, BUTTON_PIN ) )
	{
		*b_on = 0;
	}
}

void Request_Moisture_Threshold()
{

}

void Moisture_Level_Vs_Threshold()
{

}

void Set_Motor( char openClose )
{

}

/** @function	Set_LED_Pin
 *
 * 	@brief	Given a value from [0, 4], sets the representing LED pin
 * 			to the given pin state.
 *
 *  @param	led_pin 	Holds a numerical representation of LED pins
 *	@param	state		The new state of the led_pin
 *
 *	@retval None
 */
void Set_LED_Pin( const char led_pin, GPIO_PinState state )
{
	//	Throws error if value (led_pin) is not in correct range
	assert_param( led_pin >= 0 && led_pin <= 4 );

	switch( led_pin ) {
		case 0:
			HAL_GPIO_WritePin( GPIOC, GPIO_PIN_7, state );
			break;
		case 1:
			HAL_GPIO_WritePin( GPIOC, GPIO_PIN_6, state );
			break;
		case 2:
			HAL_GPIO_WritePin( GPIOC, GPIO_PIN_5, state );
			break;
		case 3:
			HAL_GPIO_WritePin( GPIOC, GPIO_PIN_4, state );
			break;
		case 4:
			HAL_GPIO_WritePin( GPIOC, GPIO_PIN_3, state );
			break;
	}
}

/** @function	Increment_Timer
 *
 * 	@brief	Uses TIM to determine when to increment the timer_count
 * 			variable every 0.5 seconds.
 *
 *  @param	poll 	Used to flicker on-board LED when polling occurs
 *
 *	@retval 1 if 0.5 seconds have passed, 0 if it hasn't.
 */
char Increment_Timer( char poll ){
	if( __HAL_TIM_GET_COUNTER( &htim2 ) - time_passed >= 5000)
	{
		if( poll )
		{
			HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_5 );
		}

		//	Resets time_passed if it goes beyond the TIM's full period
		time_passed = ( __HAL_TIM_GET_COUNTER( &htim2 ) == htim2.Init.Period ) ?
								0 : __HAL_TIM_GET_COUNTER( &htim2 );
		return 1;
	}
	return 0;
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //	Set default light to be set
  Set_LED_Pin( led_light, GPIO_PIN_SET );

  //	Timer start
  HAL_TIM_Base_Start( &htim2 );

  //	Start PWM signal creation
  HAL_TIM_PWM_Start( &htim5, TIM_CHANNEL_2 );
  //__HAL_TIM_SET_COMPARE( &htim5, TIM_CHANNEL_2, 500 );

  //	Get current time
  time_passed = __HAL_TIM_GET_COUNTER(&htim2);

  //htim5.Instance -> CCR1 = 50;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //TESTING CODE

	  //prescaler: 168-1 => 84 000 000 / 168 = 500 000
	  //frequency: 10000 => 500 000 / 10000 = 50hz
	  // [500, 2500]
	  __HAL_TIM_SET_COMPARE( &htim5, TIM_CHANNEL_2, 300 );
	  HAL_Delay(1000);
	  __HAL_TIM_SET_COMPARE( &htim5, TIM_CHANNEL_2, 1500 );
	  HAL_Delay(1000);



	  //	Checks if buttons are being pressed and changes on LED pin
	  Adjustor_Change( GPIO_PIN_4, &b_left_on, 0 );
	  Adjustor_Change( GPIO_PIN_5, &b_right_on, 1 );
    
	  //	If enough time has passed (1 second), toggle LED and add a second to the timer variable
	  time_count += Increment_Timer( is_polling );

	  //	time_count [0, 60) => activate valve IF moisture is below threshold
	  //	time_count [60, 120) => nothing happens
	  //	time_count [120, 180) => poll moisture sensor
	  if( time_count < 60 )
	  {
		  //	Activate valve if moisture is below threshold
		  Moisture_Level_Vs_Threshold();
	  }
	  else if ( time_count < 120 )
	  {
		  //do nothing
	  }
	  else if ( time_count < 180 )
	  {
		  //	Initalizes polling process
		  if ( time_count == 120 )
		  {
			  is_polling = 1;
			  soil_moisture = 0;
			  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_0, GPIO_PIN_SET );
		  }

		  //	Calculates running moisture average
		  uint16_t cur_moist = Request_Moisture_Data();
		  Average_Moisture_Data( &soil_moisture, time_count-120, cur_moist );

//		  printf("%d  ", (int)soil_moisture);
//		  printf("%d\n", cur_moist);
	  }
	  else
	  {
		  //	Resets timer and stops polling process.
		  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_0, GPIO_PIN_RESET );
		  is_polling = 0;
		  time_count = 0;
	  }

	  HAL_Delay(1);
    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 168-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int Idx;
	for(Idx = 0; Idx<len; Idx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
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
