/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "driver.h"
#include "gpio.h"
#include "I2C.h"

#include "lora.h"
#include "spi.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define LORA_PORT GPIOD
#define LORA_CS GPIO_ODR_OD11

#define FLASH_PORT GPIOE
#define FLASH_CS GPIO_ODR_OD11
#define FLASH_PAGE_SIZE 256
#define FLASH_PAGE_COUNT 65536

#define LOAD_CELL_PORT GPIOA
#define LOAD_CELL_CS GPIO_ODR_OD2

#define LOAD_CELL_PORT GPIOA
#define LOAD_CELL_CS GPIO_ODR_OD2

#define TRANSDUCER_PORT GPIOG
#define TRANSDUCER_CS GPIO_ODR_OD4

#define USART_INTERFACE USART6 //double check this
//#define USART_PIN
#define USB_BAUD 921600
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
//Ignition Arming GPIO


//SMD LEDs DEFINITIONS
#define LED_1_PWR GPIO_ODR_OD12
#define LED_2_PWR GPIO_ODR_OD11
#define LED_3_PWR GPIO_ODR_OD10
#define LED_4_PWR GPIO_ODR_OD9
#define LED_5_PWR GPIO_ODR_OD8
#define LED_6_PWR GPIO_ODR_OD9


//LED GPIO DEFINITIONS
#define PWR_LED GPIO_ODR_OD0
#define LOCAL_LED GPIO_ODR_OD1
#define REMOTE_LED GPIO_ODR_OD7
#define TRANSDUCER_LED GPIO_ODR_OD8
#define N2O_LED GPIO_ODR_OD5
#define O2_LED GPIO_ODR_OD6

//GSE FUNCTION RELATED GPIO DEFINITIONS
#define ACTIVATE_SW GPIO_ODR_OD15
#define LOCAL_CONTROL_SW GPIO_ODR_OD14
#define DUMP_SW GPIO_ODR_OD1
#define N2O_ISO_SW GPIO_ODR_OD13
#define O2_ISO_SW GPIO_ODR_OD12
#define IGNITION_SW GPIO_ODR_OD11
#define N2O_DEADMAN_SW GPIO_ODR_OD0
#define O2_DEADMAN_SW GPIO_ODR_OD5
#define IGNITE_DEADMAN_SW GPIO_ODR_OD4

//IGNITION COIL
#define IGNITION1_OP GPIO_ODR_OD14
#define IGNITION1_ARM GPIO_ODR_OD15
#define IGNITION2_OP GPIO_ODR_OD2
#define IGNITION2_ARM GPIO_ODR_OD3

#define IGNITION1_OP_OFF (0x01<<14)
#define IGNITION1_ARM_OFF (0x01<<15)
#define IGNITION2_OP_OFF (0x01<<2)
#define IGNITION2_ARM_OFF (0x01<<3)

//relays

#define CH1_Operate GPIO_ODR_OD13
#define CH1_Arm GPIO_ODR_OD14
#define CH1_Cont GPIO_ODR_OD11

#define CH2_Operate GPIO_ODR_OD10
#define CH2_Arm GPIO_ODR_OD12
#define CH2_Cont GPIO_ODR_OD15

#define CH3_Operate GPIO_ODR_OD11
#define CH3_Arm GPIO_ODR_OD12
#define CH3_Cont GPIO_ODR_OD8

#define CH4_Operate GPIO_ODR_OD9
#define CH4_Arm GPIO_ODR_OD10
#define CH4_Cont GPIO_ODR_OD13

//state for remote access
#define MANUAL_PURGE 0x01 << 7
#define O2_FILL_ACTIVATE 0x01 << 6
#define SWITCH_SELECTOR 0x01 << 5
#define N2O_FILL_ACTIVATE 0x01 << 4
#define IGNITION_FIRE 0x01 << 3
#define IGNITION_SELECTED 0x01 << 2
#define GAS_FILLED_SELECTED 0x01 << 1
#define SYSTEM_ACTIVATED 0x01

void RX_Receive(void);
//#define LORA_CS GPIO_ODR_OD11
#define LORA_DIO0 GPIO_ODR_OD7 //DIO0 GPIO




typedef enum{
	ERROR_NONE = 0,
	ERROR_INVALID_PACKET_ID,
	ERROR_INVALID_PACKET_DATA,
	ERROR_RX_FAILED,
	ERROR_TX_FAILED,
	ERROR_SYSTEM_STATE_FAILED
}ErrorCode;

void transmit_packets_spam(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
