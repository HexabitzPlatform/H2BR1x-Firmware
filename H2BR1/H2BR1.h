/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H2BR1.h
 Description   : Header file for module H2BR1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H2BR1_H
#define H2BR1_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H2BR1_MemoryMap.h"
#include "H2BR1_uart.h"
#include "H2BR1_gpio.h"
#include "H2BR1_dma.h"
#include "H2BR1_inputs.h"
#include "H2BR1_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H2BR1


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 0
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */
#define P1uart &huart6
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart6


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

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6


/* Module-specific Definitions */

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_7

#define NUM_MODULE_PARAMS		1

/* EXG Module GPIO Pinout */
#define SDN_EXG_Pin             GPIO_PIN_6
#define SDN_EXG_GPIO_Port       GPIOA
#define LODP_EXG_Pin            GPIO_PIN_7
#define LODP_EXG_GPIO_Port      GPIOA
#define LODN_EXG_Pin            GPIO_PIN_0
#define LODN_EXG_GPIO_Port      GPIOB

/* H2BR1 Module Special Timer */


/* H2BR1 Module Special ADC */


/* H2BR1 Module special parameters */


/* Module EEPROM Variables */
// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* EXG Module_Status Type Definition */
typedef enum {
	H2BR1_OK =0,
	H2BR1_ERR_UnknownMessage,
	H2BR1_ERR_WrongParams,
	H2BR1_ERROR =255
} Module_Status;


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |								  APIs							          ||
/* -----------------------------------------------------------------------
 */

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);


/* -----------------------------------------------------------------------
 |								Commands							      ||
/* -----------------------------------------------------------------------
 */


#endif /* H2BR1_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
