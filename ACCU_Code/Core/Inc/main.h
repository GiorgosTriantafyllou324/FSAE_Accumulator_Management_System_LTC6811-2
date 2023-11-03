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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC_STATE_Pin GPIO_PIN_14
#define PC_STATE_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define TEMP_DCDC_Pin GPIO_PIN_1
#define TEMP_DCDC_GPIO_Port GPIOA
#define StonPatoSouGiorgo_Pin GPIO_PIN_2
#define StonPatoSouGiorgo_GPIO_Port GPIOA
#define HUMIDITY_Pin GPIO_PIN_3
#define HUMIDITY_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SD_Detect_Pin GPIO_PIN_4
#define SD_Detect_GPIO_Port GPIOC
#define VS_OVER_60V_Pin GPIO_PIN_5
#define VS_OVER_60V_GPIO_Port GPIOC
#define SPARE_DIG_Pin GPIO_PIN_1
#define SPARE_DIG_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB
#define IMD_PWM_Pin GPIO_PIN_7
#define IMD_PWM_GPIO_Port GPIOC
#define FANS_OUT_Pin GPIO_PIN_8
#define FANS_OUT_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_9
#define VBUS_SENSE_GPIO_Port GPIOA
#define USB_ENABLE_Pin GPIO_PIN_10
#define USB_ENABLE_GPIO_Port GPIOA
#define IMD_OK_Pin GPIO_PIN_15
#define IMD_OK_GPIO_Port GPIOA
#define AMS_OK_Pin GPIO_PIN_10
#define AMS_OK_GPIO_Port GPIOC
#define AIR_P_DRIVER_Pin GPIO_PIN_12
#define AIR_P_DRIVER_GPIO_Port GPIOC
#define AIR_P_Supp_Pin GPIO_PIN_3
#define AIR_P_Supp_GPIO_Port GPIOB
#define AIR_M_State_Pin GPIO_PIN_4
#define AIR_M_State_GPIO_Port GPIOB
#define AIR_M_Supp_Pin GPIO_PIN_5
#define AIR_M_Supp_GPIO_Port GPIOB
#define AIR_P_State_Pin GPIO_PIN_6
#define AIR_P_State_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
