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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* ESC MODE SELECTION
 *
 * uncomment this if needed --> Make sure only one of them is uncommented at same time*/
#define ESC_TEST_ENABLE
//#define ESC_PROGRAMMING_MODE_ENABLE

/* TIMER CUSTOMISATION
 * define the range for esc, should be between 1 and 2 ms pulse at 50Hz
 * */
#define ESC_THROTTLE_RANGE_MAX 6000
#define ESC_THROTTLE_RANGE_MIN 3000
#define TIMER_CHANNEL TIM_CHANNEL_4

/* ESC PROGRAMING MODE SETUP
 *
 *
 */
#define ESC_SETTINGS_BRAKE ESC_BRAKE_ENABLE		/* PARAM: ESC_BRAKE_ENABLE | ESC_BRAKE_DISABLE */

#define ESC_SETTINGS_BATTERY ESC_BATTERY_LIPO		/* PARAM: ESC_BATTERY_LIPO | ESC_BATTERY_NIMH */

#define ESC_SETTINGS_CUTOFF_MODE ESC_SOFT_CUTOFF /* PARAM: ESC_SOFT_CUTOFF | ESC_HARD_CUTOFF*/









typedef enum {
	ESC_BRAKE_ENABLE,ESC_BRAKE_DISABLE
}ESC_BRAKE_MODE;

typedef enum {
	ESC_BATTERY_LIPO,ESC_BATTERY_NIMH
}ESC_BATTERY_TYPE;

typedef enum {
	ESC_SOFT_CUTOFF,ESC_HARD_CUTOFF
}ESC_CUTTOFF_MODE;



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD1_Pin GPIO_PIN_9
#define LD1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
