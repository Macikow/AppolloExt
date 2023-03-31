/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TIME_MAX_STATES 25
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GREEN_1_IO_1_READ_Pin GPIO_PIN_13
#define GREEN_1_IO_1_READ_GPIO_Port GPIOC
#define GREEN_1_IO_1_CTRL_Pin GPIO_PIN_14
#define GREEN_1_IO_1_CTRL_GPIO_Port GPIOC
#define GREEN_1_I_3_CTRL_Pin GPIO_PIN_15
#define GREEN_1_I_3_CTRL_GPIO_Port GPIOC
#define GREEN_1_I_2_CTRL_Pin GPIO_PIN_0
#define GREEN_1_I_2_CTRL_GPIO_Port GPIOC
#define GREEN_1_I_1_CTRL_Pin GPIO_PIN_1
#define GREEN_1_I_1_CTRL_GPIO_Port GPIOC
#define RED_1_POWER_CTRL_Pin GPIO_PIN_2
#define RED_1_POWER_CTRL_GPIO_Port GPIOC
#define RED_RESET_CTRL_Pin GPIO_PIN_3
#define RED_RESET_CTRL_GPIO_Port GPIOC
#define P12_RGB_I_Pin GPIO_PIN_0
#define P12_RGB_I_GPIO_Port GPIOA
#define P24_RGB_I_Pin GPIO_PIN_1
#define P24_RGB_I_GPIO_Port GPIOA
#define CBD_GREEN_LOCK_CTRL_Pin GPIO_PIN_2
#define CBD_GREEN_LOCK_CTRL_GPIO_Port GPIOA
#define VOLT_CTRL_U_Pin GPIO_PIN_3
#define VOLT_CTRL_U_GPIO_Port GPIOA
#define RED_1_LOOP_I_Pin GPIO_PIN_4
#define RED_1_LOOP_I_GPIO_Port GPIOA
#define RED_2_LOOP_I_Pin GPIO_PIN_5
#define RED_2_LOOP_I_GPIO_Port GPIOA
#define RED_3_LOOP_I_Pin GPIO_PIN_6
#define RED_3_LOOP_I_GPIO_Port GPIOA
#define P12_U_Pin GPIO_PIN_7
#define P12_U_GPIO_Port GPIOA
#define RED_3_POWER_CTRL_Pin GPIO_PIN_4
#define RED_3_POWER_CTRL_GPIO_Port GPIOC
#define RED_2_POWER_CTRL_Pin GPIO_PIN_5
#define RED_2_POWER_CTRL_GPIO_Port GPIOC
#define ANALOG_REV_Pin GPIO_PIN_0
#define ANALOG_REV_GPIO_Port GPIOB
#define DRY_OUT_1_CTRL_Pin GPIO_PIN_1
#define DRY_OUT_1_CTRL_GPIO_Port GPIOB
#define DRY_OUT_2_CTRL_Pin GPIO_PIN_10
#define DRY_OUT_2_CTRL_GPIO_Port GPIOB
#define GREEN_READER_CTRL_Pin GPIO_PIN_12
#define GREEN_READER_CTRL_GPIO_Port GPIOB
#define BLUE_1_OUT_1_INPUT_Pin GPIO_PIN_13
#define BLUE_1_OUT_1_INPUT_GPIO_Port GPIOB
#define GREEN_1_RX_Pin GPIO_PIN_6
#define GREEN_1_RX_GPIO_Port GPIOC
#define GREEN_1_TX_Pin GPIO_PIN_7
#define GREEN_1_TX_GPIO_Port GPIOC
#define BLUE_2_INPUT_Pin GPIO_PIN_8
#define BLUE_2_INPUT_GPIO_Port GPIOC
#define BLUE1_POWER_CTRL_Pin GPIO_PIN_9
#define BLUE1_POWER_CTRL_GPIO_Port GPIOC
#define BLUE_2_POWER_CTRL_Pin GPIO_PIN_10
#define BLUE_2_POWER_CTRL_GPIO_Port GPIOA
#define REV_BIT_Pin GPIO_PIN_15
#define REV_BIT_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_11
#define D0_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_12
#define D1_GPIO_Port GPIOC
#define BLUE_1_OUT_CTRL_Pin GPIO_PIN_2
#define BLUE_1_OUT_CTRL_GPIO_Port GPIOD
#define BLUE_1_OUT_2_CTRL_Pin GPIO_PIN_3
#define BLUE_1_OUT_2_CTRL_GPIO_Port GPIOB
#define BLUE_2_OUT_1_CTRL_Pin GPIO_PIN_4
#define BLUE_2_OUT_1_CTRL_GPIO_Port GPIOB
#define BLUE_2_OUT_2_CTRL_Pin GPIO_PIN_5
#define BLUE_2_OUT_2_CTRL_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_8
#define LED_BLUE_GPIO_Port GPIOB
#define GREEN_READER_CTRLB9_Pin GPIO_PIN_9
#define GREEN_READER_CTRLB9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef struct {
	uint8_t event_counter;
}global_varible_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
