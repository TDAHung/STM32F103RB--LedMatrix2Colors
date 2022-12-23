/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define EN_ROW7_Pin GPIO_PIN_15
#define EN_ROW7_GPIO_Port GPIOC
#define EN_ROW8_Pin GPIO_PIN_3
#define EN_ROW8_GPIO_Port GPIOC
#define EN_ROW1_Pin GPIO_PIN_7
#define EN_ROW1_GPIO_Port GPIOA
#define EN_ROW3_Pin GPIO_PIN_13
#define EN_ROW3_GPIO_Port GPIOB
#define EN_ROW4_Pin GPIO_PIN_10
#define EN_ROW4_GPIO_Port GPIOA
#define EN_ROW6_Pin GPIO_PIN_15
#define EN_ROW6_GPIO_Port GPIOA
#define EN_ROW5_Pin GPIO_PIN_12
#define EN_ROW5_GPIO_Port GPIOC
#define EN_ROW2_Pin GPIO_PIN_6
#define EN_ROW2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define EN_ROW1_Port GPIOA
#define EN_ROW2_Port GPIOB
#define EN_ROW3_Port GPIOB
#define EN_ROW4_Port GPIOA
#define EN_ROW5_Port GPIOC
#define EN_ROW6_Port GPIOA
#define EN_ROW7_Port GPIOC
#define EN_ROW8_Port GPIOC

#define Green5_Pin GPIO_PIN_10
#define Red5_Pin GPIO_PIN_11
#define Green6_Pin GPIO_PIN_13
#define Red6_Pin GPIO_PIN_14
#define Green7_Pin GPIO_PIN_13
#define Red7_Pin GPIO_PIN_14
#define Green8_Pin GPIO_PIN_1
#define Red8_Pin GPIO_PIN_2
#define Green1_Pin GPIO_PIN_5
#define Red1_Pin GPIO_PIN_6
#define Green2_Pin GPIO_PIN_2
#define Red2_Pin GPIO_PIN_1
#define Green3_Pin GPIO_PIN_15
#define Red3_Pin GPIO_PIN_14
#define Green4_Pin GPIO_PIN_3
#define Red4_Pin GPIO_PIN_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
