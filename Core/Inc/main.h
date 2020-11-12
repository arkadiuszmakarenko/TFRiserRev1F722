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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INTSIG8_Pin GPIO_PIN_13
#define INTSIG8_GPIO_Port GPIOC
#define INTSIG8_EXTI_IRQn EXTI15_10_IRQn
#define RW_Pin GPIO_PIN_14
#define RW_GPIO_Port GPIOC
#define FIRE1_Pin GPIO_PIN_0
#define FIRE1_GPIO_Port GPIOC
#define KBD_DATA_Pin GPIO_PIN_3
#define KBD_DATA_GPIO_Port GPIOC
#define FIRE0_Pin GPIO_PIN_3
#define FIRE0_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_4
#define A0_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_0
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_2
#define D2_GPIO_Port GPIOB
#define KBD_CLOCK_Pin GPIO_PIN_11
#define KBD_CLOCK_GPIO_Port GPIOB
#define INTSIG2_Pin GPIO_PIN_12
#define INTSIG2_GPIO_Port GPIOB
#define INTSIG2_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_6
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_7
#define A2_GPIO_Port GPIOC
#define INTSIG4_Pin GPIO_PIN_8
#define INTSIG4_GPIO_Port GPIOC
#define INTSIG5_Pin GPIO_PIN_9
#define INTSIG5_GPIO_Port GPIOC
#define A4_Pin GPIO_PIN_10
#define A4_GPIO_Port GPIOA
#define INTSIG3_Pin GPIO_PIN_15
#define INTSIG3_GPIO_Port GPIOA
#define INTSIG6_Pin GPIO_PIN_10
#define INTSIG6_GPIO_Port GPIOC
#define INTSIG7_Pin GPIO_PIN_11
#define INTSIG7_GPIO_Port GPIOC
#define D3_Pin GPIO_PIN_3
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOB
#define INTSIG1_Pin GPIO_PIN_9
#define INTSIG1_GPIO_Port GPIOB
#define INTSIG1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

typedef struct {
	uint8_t enable;
	uint8_t index;
	uint16_t buttons_data;
} gamepad_buttons_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
