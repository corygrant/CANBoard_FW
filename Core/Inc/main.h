/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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
#define AI1_Pin GPIO_PIN_0
#define AI1_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_1
#define AI2_GPIO_Port GPIOA
#define AI3_Pin GPIO_PIN_2
#define AI3_GPIO_Port GPIOA
#define AI4_Pin GPIO_PIN_3
#define AI4_GPIO_Port GPIOA
#define AI5_Pin GPIO_PIN_4
#define AI5_GPIO_Port GPIOA
#define DI1_Pin GPIO_PIN_5
#define DI1_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_6
#define DI2_GPIO_Port GPIOA
#define DI3_Pin GPIO_PIN_7
#define DI3_GPIO_Port GPIOA
#define DI4_Pin GPIO_PIN_0
#define DI4_GPIO_Port GPIOB
#define DI5_Pin GPIO_PIN_1
#define DI5_GPIO_Port GPIOB
#define DI6_Pin GPIO_PIN_8
#define DI6_GPIO_Port GPIOA
#define DI7_Pin GPIO_PIN_9
#define DI7_GPIO_Port GPIOA
#define DI8_Pin GPIO_PIN_10
#define DI8_GPIO_Port GPIOA
#define DO1_Pin GPIO_PIN_15
#define DO1_GPIO_Port GPIOA
#define DO2_Pin GPIO_PIN_3
#define DO2_GPIO_Port GPIOB
#define DO3_Pin GPIO_PIN_4
#define DO3_GPIO_Port GPIOB
#define DO4_Pin GPIO_PIN_5
#define DO4_GPIO_Port GPIOB
#define CAN_ID_2_Pin GPIO_PIN_6
#define CAN_ID_2_GPIO_Port GPIOB
#define CAN_ID_1_Pin GPIO_PIN_7
#define CAN_ID_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define BUILD 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
