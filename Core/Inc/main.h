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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

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
#define U3RX_BOOT_BT_Pin GPIO_PIN_8
#define U3RX_BOOT_BT_GPIO_Port GPIOB
#define VSTIM_PWM_Pin GPIO_PIN_6
#define VSTIM_PWM_GPIO_Port GPIOB
#define SWO_ST_Pin GPIO_PIN_3
#define SWO_ST_GPIO_Port GPIOB
#define STIM_LS18_Pin GPIO_PIN_6
#define STIM_LS18_GPIO_Port GPIOD
#define STIM_LS11_Pin GPIO_PIN_5
#define STIM_LS11_GPIO_Port GPIOD
#define STIM_LS15_Pin GPIO_PIN_4
#define STIM_LS15_GPIO_Port GPIOD
#define STIM_LS7_Pin GPIO_PIN_1
#define STIM_LS7_GPIO_Port GPIOD
#define STIM_LS10_Pin GPIO_PIN_7
#define STIM_LS10_GPIO_Port GPIOD
#define STIM_LS8_Pin GPIO_PIN_2
#define STIM_LS8_GPIO_Port GPIOD
#define STIM_LS14_Pin GPIO_PIN_0
#define STIM_LS14_GPIO_Port GPIOD
#define SWCLK_ST_Pin GPIO_PIN_14
#define SWCLK_ST_GPIO_Port GPIOA
#define STIM_LS12_Pin GPIO_PIN_3
#define STIM_LS12_GPIO_Port GPIOD
#define ISTIM_DACena_Pin GPIO_PIN_11
#define ISTIM_DACena_GPIO_Port GPIOA
#define EN_5V5_Pin GPIO_PIN_15
#define EN_5V5_GPIO_Port GPIOC
#define SWDIO_ST_Pin GPIO_PIN_13
#define SWDIO_ST_GPIO_Port GPIOA
#define MEAS_SWEen5V_Pin GPIO_PIN_9
#define MEAS_SWEen5V_GPIO_Port GPIOF
#define STM_nLED_Pin GPIO_PIN_2
#define STM_nLED_GPIO_Port GPIOC
#define STIM_HS_OE_Pin GPIO_PIN_0
#define STIM_HS_OE_GPIO_Port GPIOC
#define nRST_ST_Pin GPIO_PIN_10
#define nRST_ST_GPIO_Port GPIOG
#define STIM_LS16_Pin GPIO_PIN_14
#define STIM_LS16_GPIO_Port GPIOD
#define ISTIM_LSadc_Pin GPIO_PIN_1
#define ISTIM_LSadc_GPIO_Port GPIOA
#define STIM_LS21_Pin GPIO_PIN_12
#define STIM_LS21_GPIO_Port GPIOE
#define STIM_LS2_Pin GPIO_PIN_10
#define STIM_LS2_GPIO_Port GPIOD
#define STIM_LS13_Pin GPIO_PIN_9
#define STIM_LS13_GPIO_Port GPIOD
#define STIM_LS17_Pin GPIO_PIN_13
#define STIM_LS17_GPIO_Port GPIOD
#define STIM_LS9_Pin GPIO_PIN_15
#define STIM_LS9_GPIO_Port GPIOD
#define ISTIM_HSadc_Pin GPIO_PIN_2
#define ISTIM_HSadc_GPIO_Port GPIOA
#define ISTIM_DACuc_Pin GPIO_PIN_4
#define ISTIM_DACuc_GPIO_Port GPIOA
#define STIM_HS_LOAD_Pin GPIO_PIN_3
#define STIM_HS_LOAD_GPIO_Port GPIOA
#define STIM_LS22_Pin GPIO_PIN_8
#define STIM_LS22_GPIO_Port GPIOE
#define STIM_LS20_Pin GPIO_PIN_9
#define STIM_LS20_GPIO_Port GPIOE
#define STIM_LS4_Pin GPIO_PIN_15
#define STIM_LS4_GPIO_Port GPIOE
#define MISO2_BT_Pin GPIO_PIN_14
#define MISO2_BT_GPIO_Port GPIOB
#define STIM_LS24_Pin GPIO_PIN_11
#define STIM_LS24_GPIO_Port GPIOD
#define STIM_HS_SCK_Pin GPIO_PIN_5
#define STIM_HS_SCK_GPIO_Port GPIOA
#define VBIAS_LEADOFF_Pin GPIO_PIN_6
#define VBIAS_LEADOFF_GPIO_Port GPIOA
#define STIM_HS_DIS_Pin GPIO_PIN_5
#define STIM_HS_DIS_GPIO_Port GPIOC
#define STIM_LS3_Pin GPIO_PIN_11
#define STIM_LS3_GPIO_Port GPIOE
#define STIM_LS5_Pin GPIO_PIN_14
#define STIM_LS5_GPIO_Port GPIOE
#define U3TX_BT_Pin GPIO_PIN_10
#define U3TX_BT_GPIO_Port GPIOB
#define SCK2_BT_Pin GPIO_PIN_13
#define SCK2_BT_GPIO_Port GPIOB
#define STIM_LS6_Pin GPIO_PIN_12
#define STIM_LS6_GPIO_Port GPIOD
#define STIM_HS_MOSI_Pin GPIO_PIN_7
#define STIM_HS_MOSI_GPIO_Port GPIOA
#define STIM_HS_ENA_Pin GPIO_PIN_4
#define STIM_HS_ENA_GPIO_Port GPIOC
#define STIM_LS23_Pin GPIO_PIN_10
#define STIM_LS23_GPIO_Port GPIOE
#define STIM_LS1_Pin GPIO_PIN_13
#define STIM_LS1_GPIO_Port GPIOE
#define NSS2_BT_Pin GPIO_PIN_12
#define NSS2_BT_GPIO_Port GPIOB
#define MOSI2_BT_Pin GPIO_PIN_15
#define MOSI2_BT_GPIO_Port GPIOB
#define STIM_LS19_Pin GPIO_PIN_8
#define STIM_LS19_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
