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
#include "Red.h"
#include "Green.h"
#include "Blue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_CHANNELS_NO 6
#define ADC_ARG_FROM_5 5

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ADC_DATA 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */



global_varible_t global;
volatile uint8_t global_cnt = 0;
volatile uint8_t adc_mem = 0;

uint16_t ADC_Data_table[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rgb_let_toggle()
{
	if(global_cnt  < TIME_MAX_STATES/3 )
	{
		LED_BLUE_GPIO_Port->ODR ^= LED_BLUE_Pin;
		LED_RED_GPIO_Port->ODR &= ~LED_RED_Pin;
		LED_GREEN_GPIO_Port->ODR &= ~LED_GREEN_Pin;

	}
	else if(global_cnt  < 2*TIME_MAX_STATES/3 )
	{
		LED_BLUE_GPIO_Port->ODR &= ~LED_BLUE_Pin;
		LED_RED_GPIO_Port->ODR ^= LED_RED_Pin;
		LED_GREEN_GPIO_Port->ODR &= ~LED_GREEN_Pin;
	}
	else if(global_cnt  > 2*TIME_MAX_STATES/3 )
	{
		LED_BLUE_GPIO_Port->ODR &= ~LED_BLUE_Pin;
		LED_RED_GPIO_Port->ODR &= ~LED_RED_Pin;
		LED_GREEN_GPIO_Port->ODR ^= LED_GREEN_Pin;
	}
}


void ADC_Enable(void)
{
	ADC1->CR2 |= ADC_CR2_ADON;

	uint32_t delay= 10000;
	while(delay--);
}

void ADC_start(void)
{
	ADC1->SR = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}


void DMA_ADC_init(void)
{
//	Enable clocks
	RCC->APB2ENR |= (1 << 8);  //Enable ADC1
	RCC->AHB1ENR |= (1 << 0);  //Enable GPIO

// Set prescaler

	ADC->CCR |= (1 << 16); //PLCK devide by 4

// Set scan mode
	ADC1->CR1 |= ADC_CR1_SCAN; // Enable scan mode
	ADC1->CR1 &= ~(1 << 24 ) ; //
	ADC1->CR1 &= ~(1 << 25 ) ; // Set resolution for 12bits

// Set Continues conversation EOC, and Data Aligment

	//ADC1->CR2 |= ADC_CR2_CONT;	// contiunes conversion mode
	ADC1->CR2 |= ADC_CR2_EOCS; //  EOC after each conversion
	ADC1->CR2 &= ~(1 << 11); // Data Aligment Right

//Scannig order
	ADC1->SQR3 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 |= (0 << 0); 	// channel 0 1st
	ADC1->SQR3 |= (1 << 5); 	// channel 1 2th
	ADC1->SQR3 |= (3 << 10); 	// channel 3 3th
	ADC1->SQR3 |= (4 << 15); 	// channel 4 4th
	ADC1->SQR3 |= (5 << 20); 	// channel 5 5th
	ADC1->SQR3 |= (6 << 25); 	// channel 6 6th
	ADC1->SQR2 |= (7 << 0); 	// channel 7 7th
	ADC1->SQR2 |= (8 << 5); 	// channel 8 8th

	// Sum = 7

//Set sapling time for channels
	ADC1->SMPR2 =0;
	ADC1->SMPR2 |= (4 << 0 );	//1
	ADC1->SMPR2 |= (4 << 3 ); 	//2
	ADC1->SMPR2 |= (4 << 6 );	//3
	ADC1->SMPR2 |= (4 << 9 );	//4
	ADC1->SMPR2 |= (4 << 12 );	//5
	ADC1->SMPR2 |= (4 << 15 );	//6
	ADC1->SMPR2 |= (4 << 18 );	//7
	ADC1->SMPR2 |= (4 << 21 );	//8

// Set the regular channels sequence lenth
	 ADC1->SQR1 |= (7 << 20); //7 channels for convertion

// Set the respective GPIO pins int teh analog mode
	 GPIOA->MODER |= (3 << (2*0) );	//0 - 1
	 GPIOA->MODER |= (3 << (2*1) );	//1 - 2
	 GPIOA->MODER |= (3 << (2*3) );	//3 - 3
	 GPIOA->MODER |= (3 << (2*4) );	//4 - 4
	 GPIOA->MODER |= (3 << (2*5) );	//5 - 5
	 GPIOA->MODER |= (3 << (2*6) );	//6 - 6
	 GPIOA->MODER |= (3 << (2*7) );	//7 - 7
	 GPIOB->MODER |= (3 << (2*0) );	//7 - 7

// Enable ADC for DMA

	 ADC1->CR2 |= ADC_CR2_DMA; // Enable DMA mode
	 ADC1->CR2 &= ~ADC_CR2_CONT; // Enable Contounes Request

	 ADC_Enable();

// Enable DMA Clock
	 RCC->AHB1ENR |= (1 << 22 ); // Enable RCC for DMA

// DMA channel 2 stream 0 configuration
	 DMA2_Stream0->CR &= ~DMA_SxCR_EN; // Disable DMA for configuration

// memory data size
	 DMA2_Stream0->CR &= ~(1 << 14);
	 DMA2_Stream0->CR |= (1 << 13); // half-word 16bit

// perith  data size
	 DMA2_Stream0->CR &= ~(1 << 12);
	 DMA2_Stream0->CR |= (1 << 11); // half-word 16bit

// memory increment mode
	 DMA2_Stream0->CR |= DMA_SxCR_MINC; // Enable memory increment mode

// perith increment mode
	 DMA2_Stream0->CR &= ~DMA_SxCR_PINC; // Disable peripherial increment mode

// Data transfer direction
	 DMA2_Stream0->CR &= ~(1 << 7); // Peripherial to memory
	 DMA2_Stream0->CR &= ~(1 << 6); // Peripherial to memory

// DMA flow control
	 DMA2_Stream0->CR &= ~DMA_SxCR_PFCTRL; // DMA flow control enable

//	Circular mode
	 DMA2_Stream0->CR |= DMA_SxCR_CIRC; // Circular mode enabled

// Channel set
	 DMA2_Stream0->CR &= ~(1 << 25);
	 DMA2_Stream0->CR &= ~(1 << 26);
	 DMA2_Stream0->CR &= ~(1 << 27);
//
	 DMA2_Stream0->CR &= ~(1 << 19);
	 DMA2_Stream0->CR &= ~(1 << 18);

// Number of data to transfer
	 DMA2_Stream0->NDTR = 8;
// Peripherial Address set
	 DMA2_Stream0->PAR =(uint32_t *) &ADC1->DR;

// Memory address set
	 DMA2_Stream0->M0AR =(uint32_t *) ADC_Data_table;

// Set interrupt
	 DMA2_Stream0->CR |= DMA_SxCR_TCIE;

	 HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	 HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

// Enable DMA
	 DMA2_Stream0->CR |= DMA_SxCR_EN;

}

void adc_measure_handler(const uint8_t counter_value)
{
	static uint8_t temp_bit_mem;
	if(temp_bit_mem == counter_value) return;
	temp_bit_mem = counter_value;

	ADC_start();


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
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  DMA_ADC_init();

  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Data_table, MAX_ADC_DATA);
 //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  //HAL_ADC_Start(&hadc1);
  while (1)
  {
	  adc_measure_handler(global_cnt);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  TIM3->DIER |= TIM_DIER_UIE;
  TIM3->CR1 |= TIM_CR1_CEN;
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GREEN_1_IO_1_CTRL_Pin|GREEN_1_I_2_CTRL_Pin|GREEN_1_I_1_CTRL_Pin|RED_1_POWER_CTRL_Pin
                          |RED_RESET_CTRL_Pin|RED_3_POWER_CTRL_Pin|RED_2_POWER_CTRL_Pin|BLUE1_POWER_CTRL_Pin
                          |LED_Pin|D0_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CBD_GREEN_LOCK_CTRL_Pin|GPIO_PIN_8|BLUE_2_POWER_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRY_OUT_1_CTRL_Pin|DRY_OUT_2_CTRL_Pin|GREEN_READER_CTRL_Pin|BLUE_1_OUT_2_CTRL_Pin
                          |BLUE_2_OUT_1_CTRL_Pin|BLUE_2_OUT_2_CTRL_Pin|LED_RED_Pin|LED_GREEN_Pin
                          |LED_BLUE_Pin|GREEN_READER_CTRLB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_1_OUT_CTRL_GPIO_Port, BLUE_1_OUT_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_1_IO_1_READ_Pin GREEN_1_I_3_CTRL_Pin BLUE_2_INPUT_Pin */
  GPIO_InitStruct.Pin = GREEN_1_IO_1_READ_Pin|GREEN_1_I_3_CTRL_Pin|BLUE_2_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_1_IO_1_CTRL_Pin GREEN_1_I_2_CTRL_Pin GREEN_1_I_1_CTRL_Pin RED_1_POWER_CTRL_Pin
                           RED_RESET_CTRL_Pin RED_3_POWER_CTRL_Pin RED_2_POWER_CTRL_Pin BLUE1_POWER_CTRL_Pin
                           LED_Pin D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = GREEN_1_IO_1_CTRL_Pin|GREEN_1_I_2_CTRL_Pin|GREEN_1_I_1_CTRL_Pin|RED_1_POWER_CTRL_Pin
                          |RED_RESET_CTRL_Pin|RED_3_POWER_CTRL_Pin|RED_2_POWER_CTRL_Pin|BLUE1_POWER_CTRL_Pin
                          |LED_Pin|D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CBD_GREEN_LOCK_CTRL_Pin PA8 BLUE_2_POWER_CTRL_Pin */
  GPIO_InitStruct.Pin = CBD_GREEN_LOCK_CTRL_Pin|GPIO_PIN_8|BLUE_2_POWER_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRY_OUT_1_CTRL_Pin DRY_OUT_2_CTRL_Pin GREEN_READER_CTRL_Pin BLUE_1_OUT_2_CTRL_Pin
                           BLUE_2_OUT_1_CTRL_Pin BLUE_2_OUT_2_CTRL_Pin LED_RED_Pin LED_GREEN_Pin
                           LED_BLUE_Pin GREEN_READER_CTRLB9_Pin */
  GPIO_InitStruct.Pin = DRY_OUT_1_CTRL_Pin|DRY_OUT_2_CTRL_Pin|GREEN_READER_CTRL_Pin|BLUE_1_OUT_2_CTRL_Pin
                          |BLUE_2_OUT_1_CTRL_Pin|BLUE_2_OUT_2_CTRL_Pin|LED_RED_Pin|LED_GREEN_Pin
                          |LED_BLUE_Pin|GREEN_READER_CTRLB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_1_OUT_1_INPUT_Pin PB14 PB15 */
  GPIO_InitStruct.Pin = BLUE_1_OUT_1_INPUT_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : REV_BIT_Pin */
  GPIO_InitStruct.Pin = REV_BIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(REV_BIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_1_OUT_CTRL_Pin */
  GPIO_InitStruct.Pin = BLUE_1_OUT_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_1_OUT_CTRL_GPIO_Port, &GPIO_InitStruct);

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
