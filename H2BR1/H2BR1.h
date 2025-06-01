/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H2BR1.h
 Description   : Header file for module H2BR1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H2BR1_H
#define H2BR1_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H2BR1_MemoryMap.h"
#include "H2BR1_uart.h"
#include "H2BR1_gpio.h"
#include "H2BR1_dma.h"
#include "H2BR1_inputs.h"
#include "H2BR1_eeprom.h"
#include "MAX30100_reg_address.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H2BR1

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

/* SPO2 External Interrupt Pin */
#define SPO2_EXT_INT_PIN    GPIO_PIN_7
#define SPO2_EXT_INT_PORT   GPIOB

/* I2C Pin Definition */
#define SENSOR_I2C_SCL_PIN  GPIO_PIN_3
#define SENSOR_I2C_SDA_PIN  GPIO_PIN_4
#define SENSOR_I2C_PORT     GPIOB

#define I2C_HANDLER         &hi2c2

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_13

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS		 3

/* Streaming Parameters */
#define MIN_PERIOD_MS		     100
#define MAX_TIMEOUT_MS		     0xFFFFFFFF
#define STREAM_MODE_TO_PORT      1
#define STREAM_MODE_TO_TERMINAL  2

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H2BR1_OK =0,
	H2BR1_ERR_UNKNOWNMESSAGE,
	H2BR1_ERR_WRONGPARAMS,
	H2BR1_ERR_TERMINATED,
	H2BR1_ERROR =255
} Module_Status;

/* SPO2 Signal type */
typedef enum {
	HR = 0, SPO2,
} All_Data;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status HR_Sample(uint8_t *heartRate);
Module_Status SPO2_Sample(uint8_t *SPO2);
Module_Status FingerState(FINGER_STATE *fingerState);
Module_Status SampleReadFlag(uint8_t *sampleReadFlag);
Module_Status ResetSampleReadFlag(void);

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction);
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout) ;

#endif /* H2BR1_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
