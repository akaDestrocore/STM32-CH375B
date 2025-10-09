/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define CUSTOM_HID_EPIN_SIZE        0x08U
#define CUSTOM_HID_EPOUT_SIZE       0x04U
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define user_btn_Pin GPIO_PIN_0
#define user_btn_GPIO_Port GPIOA

#define CH375A_INT_PIN GPIO_PIN_14
#define CH375A_INT_PORT GPIOE
#define CH375B_INT_PIN GPIO_PIN_15
#define CH375B_INT_PORT GPIOE

#define green_led_Pin GPIO_PIN_12
#define green_led_GPIO_Port GPIOD
#define orange_led_Pin GPIO_PIN_13
#define orange_led_GPIO_Port GPIOD
#define red_led_Pin GPIO_PIN_14
#define red_led_GPIO_Port GPIOD
#define blue_led_Pin GPIO_PIN_15
#define blue_led_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
// CH375 CONFIG
#define CH375_WORK_BAUDRATE     115200
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_MODULE_NUM        2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
