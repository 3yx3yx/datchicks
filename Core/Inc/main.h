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
void delayUs (uint32_t micros);
uint8_t getModuleId(void);
void sendByteSPI (uint8_t byte)	;
void displayFloat (float value);
void hx711clkPulse (void);
uint32_t getValueHX711 (uint8_t input, uint8_t gain);
float getDS18B20 (void);
 void luxSensInit (void);
  float getluxSens (void);
	 float getTempNTC(uint16_t adcTherm);
void adcSeq (void); 
void adcSum (void);
void adcAverage (void);
float getVoltage(uint16_t adc);
float getAHT20 (void)	;
float getUVIndex ();
void clearEXTIs (void);
void enableExtis (void);
void timerTask (void);
void lcMeterTask (void);
void rj45_connectors_recognise (void);
uint32_t hx711Average (uint8_t input, uint8_t gain);
void Hx711Task (void);
float getCOppm(uint16_t adc);
float getOxygenPercent(uint16_t adc);
float getResistance(void);
float getVoltageCurrent(void);
float getNitratSensor();
void spirograph (void);
void averageAdc_for_N_msec (uint16_t msec);
void waiting_animation(void);
void dacWrite(uint16_t data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define relayPin GPIO_PIN_15
#define relayPort GPIOB
#define clk_Pin GPIO_PIN_3
#define clk_GPIO_Port GPIOA
#define data1_Pin GPIO_PIN_4
#define data1_GPIO_Port GPIOA
#define data2_Pin GPIO_PIN_5
#define data2_GPIO_Port GPIOA
#define data3_Pin GPIO_PIN_6
#define data3_GPIO_Port GPIOA
#define exti1_Pin GPIO_PIN_7
#define exti1_GPIO_Port GPIOA
#define exti1_EXTI_IRQn EXTI9_5_IRQn
#define exti2_Pin GPIO_PIN_0
#define exti2_GPIO_Port GPIOB
#define exti2_EXTI_IRQn EXTI0_IRQn
#define exti3_Pin GPIO_PIN_1
#define exti3_GPIO_Port GPIOB
#define exti3_EXTI_IRQn EXTI1_IRQn
#define exti4_Pin GPIO_PIN_11
#define exti4_GPIO_Port GPIOB
#define exti4_EXTI_IRQn EXTI15_10_IRQn
#define id1_Pin GPIO_PIN_9
#define id1_GPIO_Port GPIOA
#define id2_Pin GPIO_PIN_10
#define id2_GPIO_Port GPIOA
#define id3_Pin GPIO_PIN_15
#define id3_GPIO_Port GPIOA
#define clk_soft_Pin GPIO_PIN_3
#define clk_soft_GPIO_Port GPIOB
#define mosi_soft_Pin GPIO_PIN_5
#define mosi_soft_GPIO_Port GPIOB
#define id4_Pin GPIO_PIN_8
#define id4_GPIO_Port GPIOB
#define id5_Pin GPIO_PIN_9
#define id5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
