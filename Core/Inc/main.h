/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRACKDET_Pin GPIO_PIN_2
#define TRACKDET_GPIO_Port GPIOE
#define RIGHTMKR_Pin GPIO_PIN_3
#define RIGHTMKR_GPIO_Port GPIOE
#define LEFTMKR_Pin GPIO_PIN_4
#define LEFTMKR_GPIO_Port GPIOE
#define SLOW_Pin GPIO_PIN_0
#define SLOW_GPIO_Port GPIOF
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOF
#define HOME_Pin GPIO_PIN_2
#define HOME_GPIO_Port GPIOF
#define MISC4_Pin GPIO_PIN_3
#define MISC4_GPIO_Port GPIOF
#define ESTOP_Pin GPIO_PIN_4
#define ESTOP_GPIO_Port GPIOF
#define BUMPER_Pin GPIO_PIN_5
#define BUMPER_GPIO_Port GPIOF
#define LEFTSEL_Pin GPIO_PIN_8
#define LEFTSEL_GPIO_Port GPIOF
#define RIGHTSEL_Pin GPIO_PIN_9
#define RIGHTSEL_GPIO_Port GPIOF
#define DIRSEL_Pin GPIO_PIN_10
#define DIRSEL_GPIO_Port GPIOF
#define LED_GRN_Pin GPIO_PIN_0
#define LED_GRN_GPIO_Port GPIOB
#define AREA1_Pin GPIO_PIN_12
#define AREA1_GPIO_Port GPIOF
#define AREA2_Pin GPIO_PIN_13
#define AREA2_GPIO_Port GPIOF
#define AREA3_Pin GPIO_PIN_14
#define AREA3_GPIO_Port GPIOF
#define AREA4_Pin GPIO_PIN_15
#define AREA4_GPIO_Port GPIOF
#define M0_Pin GPIO_PIN_0
#define M0_GPIO_Port GPIOG
#define M1_Pin GPIO_PIN_1
#define M1_GPIO_Port GPIOG
#define OBS_SEL_Pin GPIO_PIN_11
#define OBS_SEL_GPIO_Port GPIOE
#define OBS1_Pin GPIO_PIN_12
#define OBS1_GPIO_Port GPIOE
#define OBS2_Pin GPIO_PIN_13
#define OBS2_GPIO_Port GPIOE
#define OBS3_Pin GPIO_PIN_14
#define OBS3_GPIO_Port GPIOE
#define READY_Pin GPIO_PIN_15
#define READY_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define RLY1_Pin GPIO_PIN_8
#define RLY1_GPIO_Port GPIOD
#define RLY2_Pin GPIO_PIN_9
#define RLY2_GPIO_Port GPIOD
#define RLY3_Pin GPIO_PIN_10
#define RLY3_GPIO_Port GPIOD
#define RLY4_Pin GPIO_PIN_11
#define RLY4_GPIO_Port GPIOD
#define RLY5_Pin GPIO_PIN_14
#define RLY5_GPIO_Port GPIOD
#define RLY6_Pin GPIO_PIN_15
#define RLY6_GPIO_Port GPIOD
#define SD_Pin GPIO_PIN_2
#define SD_GPIO_Port GPIOG
#define F1_Pin GPIO_PIN_3
#define F1_GPIO_Port GPIOG
#define F2_Pin GPIO_PIN_4
#define F2_GPIO_Port GPIOG
#define F3_Pin GPIO_PIN_5
#define F3_GPIO_Port GPIOG
#define F4_Pin GPIO_PIN_6
#define F4_GPIO_Port GPIOG
#define F5_Pin GPIO_PIN_7
#define F5_GPIO_Port GPIOG
#define ONLED_Pin GPIO_PIN_8
#define ONLED_GPIO_Port GPIOG
#define ALARM_Pin GPIO_PIN_3
#define ALARM_GPIO_Port GPIOD
#define SDLED_Pin GPIO_PIN_9
#define SDLED_GPIO_Port GPIOG
#define RSTLED_Pin GPIO_PIN_10
#define RSTLED_GPIO_Port GPIOG
#define F1LED_Pin GPIO_PIN_11
#define F1LED_GPIO_Port GPIOG
#define F2LED_Pin GPIO_PIN_12
#define F2LED_GPIO_Port GPIOG
#define F3LED_Pin GPIO_PIN_13
#define F3LED_GPIO_Port GPIOG
#define F4LED_Pin GPIO_PIN_14
#define F4LED_GPIO_Port GPIOG
#define PCON_Pin GPIO_PIN_4
#define PCON_GPIO_Port GPIOB
#define CHGDET_Pin GPIO_PIN_5
#define CHGDET_GPIO_Port GPIOB
#define LED_ORG_Pin GPIO_PIN_7
#define LED_ORG_GPIO_Port GPIOB
#define WREN_Pin GPIO_PIN_8
#define WREN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
