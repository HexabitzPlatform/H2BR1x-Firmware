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
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */
#define P1uart &huart6
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart4


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
#define _IND_LED_PORT			GPIOA
#define _IND_LED_PIN			GPIO_PIN_5

#define NUM_MODULE_PARAMS		1




/* Module Special I2C */

#define MCU_SCL_Pin                  GPIO_PIN_3
#define MCU_SCL_GPIO_Port            GPIOB
#define MCU_SDA_Pin                  GPIO_PIN_4
#define MCU_SDA_GPIO_Port            GPIOB
/* EXT Module GPIO Pinout */
#define MAX30100_INT_Pin             GPIO_PIN_1
#define MAX30100_INT_GPIO_Port       GPIOD


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

extern I2C_HandleTypeDef hi2c2;
//I2C_HandleTypeDef HANDLER_I2C_MAX30100;
#define HANDLER_I2C_MAX30100  hi2c2

#define MAX30100_I2C_ADDRESS         0xAE
#define MAX30100_SAMPLE_SIZE		 4
#define MAX30100_FIFO_SAMPLES_SIZE	 16
#define MAX30100_FIFO_DATA_SIZE		 64

/****************************Registers Address*****************************/
#define MAX30100_INTERRUPT_ADDR      0x00U
#define MAX30100_INTERRUPT_EN_ADDR   0x01U
#define MAX30100_FIFO_W_PTR_ADDR     0x02U
#define MAX30100_OVF_COUNTER_ADDR    0x03U
#define MAX30100_FIFO_R_PTR_ADDR     0x04U
#define MAX30100D_FIFO_DATA_ADDR     0x05U
#define MAX30100_MODE_CONF_ADDR      0x06U
#define MAX30100_SPO2_CONF_ADDR      0x07U
#define MAX30100_LED_CONF_ADDR       0x09U
#define MAX30100_TEMP_INTEGER_ADDR   0x16U
#define MAX30100_TEMP_FRACTION_ADDR  0x17U
#define MAX30100_REVISION_ADDR       0xfeU
#define MAX30100_PART_ADDR           0xffU

/*******************************************************************************
* Register      : Interrupt Status
* Address       : 0x00
* Permission    : R
*******************************************************************************/
#define	MAX30100_INTERRUPT_PWR_RDY_MASK		0x01U
#define	MAX30100_INTERRUPT_SPO2_RDY_MASK	0x10U
#define	MAX30100_INTERRUPT_HR_RDY_MASK		0x20U
#define	MAX30100_INTERRUPT_TEMP_RDY_MASK	0x40U
#define MAX30100_INTERRUPT_A_FULL_MASK		0x80U
#define	MAX30100_INTERRUPT_PWR_RDY_POSITION	     0
#define	MAX30100_INTERRUPT_SPO2_RDY_POSITION	 4
#define	MAX30100_INTERRUPT_HR_RDY_POSITION	     5
#define	MAX30100_INTERRUPT_TEMP_RDY_POSITION	 6
#define	MAX30100_INTERRUPT_A_FULL_POSITION	     7

/*******************************************************************************
* Register      : Interrupt Enable
* Address       : 0x01
* Permission    : R/W
*******************************************************************************/
#define	MAX30100_INTERRUPT_EN_SPO2_RDY_MASK	     0x10U
#define	MAX30100_INTERRUPT_EN_HR_RDY_MASK	     0x20U
#define	MAX30100_INTERRUPT_EN_TEMP_RDY_MASK	     0x40U
#define MAX30100_INTERRUPT_EN_A_FULL_MASK	     0x80U
#define	MAX30100_INTERRUPT_EN_SPO2_RDY_POSITION	 4
#define	MAX30100_INTERRUPT_EN_HR_RDY_POSITION	 5
#define	MAX30100_INTERRUPT_EN_TEMP_RDY_POSITION	 6
#define	MAX30100_INTERRUPT_EN_A_FULL_POSITION	 7

typedef enum
{
	INTERRUPT_EN_SPO2_RDY_DISABLED		= 0X00U,
	INTERRUPT_EN_SPO2_RDY_ENABLED		= 0X10U,
} INTERRUPT_EN_SPO2_RDY_BIT;

typedef enum
{
	INTERRUPT_EN_HR_RDY_DISABLED		= 0X00U,
	INTERRUPT_EN_HR_RDY_ENABLED		    = 0X20U,
} INTERRUPT_EN_HR_RDY_BIT;

typedef enum
{
	INTERRUPT_EN_TEMP_RDY_DISABLED		= 0X00U,
	INTERRUPT_EN_TEMP_RDY_ENABLED	    = 0X40U,
} INTERRUPT_EN_TEMP_RDY_BIT;

typedef enum
{
	INTERRUPT_EN_AFULL_DISABLED		= 0X00U,
	INTERRUPT_EN_AFULL_ENABLED	    = 0X80U,
} INTERRUPT_EN_A_FULL_BIT;

/*******************************************************************************
* Register      : Mode Configuration
* Address       : 0x06
* Permission    : R/W
*******************************************************************************/
#define	MAX30100_MODE_CONF_MODE_MASK	     0x07
#define	MAX30100_MODE_CONF_TEMP_EN_MASK	     0x08
#define	MAX30100_MODE_CONF_RESET_MASK	     0x40
#define	MAX30100_MODE_CONF_SHDN_MASK	     0x80

#define	MAX30100_MODE_CONF_MODE_POSITION	  0
#define	MAX30100_MODE_CONF_TEMP_EN_POSITION	  3
#define	MAX30100_MODE_CONF_RESET_POSITION	  6
#define	MAX30100_MODE_CONF_SHDN_POSITION	  7

/*******************************************************************************
* Register      : SPO2 Configuration
* Address       : 0x07
* Permission    : R/W
*******************************************************************************/
#define	MAX30100_SPO2_CONF_LED_PW_MASK	         0x03
#define	MAX30100_SPO2_CONF_SR_MASK	             0x1c
#define	MAX30100_SPO2_CONF_HI_RES_ENB_MASK	     0x40
#define	MAX30100_SPO2_CONF_LED_PW_POSITION	      0
#define	MAX30100_SPO2_CONF_SR_POSITION	          2
#define	MAX30100_SPO2_CONF_HI_RES_ENB_POSITION	  6

/*******************************************************************************
* Register      : LED Configuration
* Address       : 0x09
* Permission    : R/W
*******************************************************************************/
#define	MAX30100_LED_CONF_IR_PA_MASK	    0x0fU
#define	MAX30100_LED_CONF_RED_PA_MASK	    0xf0U
#define	MAX30100_LED_CONF_IR_PA_POSITION	0
#define	MAX30100_LED_CONF_RED_PA_POSITION	4

/*******************************************************************************/
/* Table 3. Mode Control Page 15*/
typedef enum
{
	UNUSED_MODE    = 0x00U,
	HR_MODE        = 0x02U,
	SPO2_MODE      = 0x03U,
}MAX30100_MODE;

/* Table 4. SPO2 Sample Rate Control Page 16*/
/*************************************************************
Table 8 and Table 9 Page 19
   SampleRate   PulseWidth_Max(us)   Resolution_Max(bit)
      50              1600               16
      100             1600               16
      167             800                15
      200             800                15
      400             400                14
      600             200                13
      800             200                13
      1000            200                13
****************************************************************/
typedef enum
{
	MAX30100_SPO2_SR_50 =0x00U,
	MAX30100_SPO2_SR_100,
	MAX30100_SPO2_SR_167,
	MAX30100_SPO2_SR_200,
	MAX30100_SPO2_SR_400,
	MAX30100_SPO2_SR_600,
	MAX30100_SPO2_SR_800,
	MAX30100_SPO2_SR_1000,
} MAX30100_SpO2_SR;

/* Table 5. Led Pulse Width Control Page 17*/
/**************************************************************
Table 8 and Table 9 Page 19
   PulseWidth(us)  SampleRate_Max   Resolution_Max(bit)
      200             1000              13
      400             1000              14
      800             200               15
      1600            100               16
***************************************************************/
typedef enum
{
    MAX30100_LED_PW_200 =0x00U,
    MAX30100_LED_PW_400,
    MAX30100_LED_PW_800,
    MAX30100_LED_PW_1600,
} MAX30100_LED_PW;

/* Table 6. Led Current Control Page 17*/
typedef enum
{
    MAX30100_LED_CURRENT_0P0 =0x00U,
    MAX30100_LED_CURRENT_4P4,
    MAX30100_LED_CURRENT_7P6,
    MAX30100_LED_CURRENT_11P0,
    MAX30100_LED_CURRENT_14P2,
    MAX30100_LED_CURRENT_17P4,
    MAX30100_LED_CURRENT_20P8,
    MAX30100_LED_CURRENT_24P0,
    MAX30100_LED_CURRENT_27P1,
    MAX30100_LED_CURRENT_30P6,
    MAX30100_LED_CURRENT_33P8,
    MAX30100_LED_CURRENT_37P0,
    MAX30100_LED_CURRENT_40P2,
    MAX30100_LED_CURRENT_43P6,
    MAX30100_LED_CURRENT_46P8,
    MAX30100_LED_CURRENT_50P0,
} MAX30100_LED_Current;

/**********************Oxymeter defines****************************/
#define FINGER_DETECTING_THRESHOLD     25000  // IR value received when finger is put
#define OXYMETER_MAX_BUFFER_SIZE       600  // 6 sec
#define CURRENT_MODIFYING_DURATION_MS  200
#define CURRENT_MODIFYING_THRESHOLD    7500
#define FILTERING_SAMPLES_SHIFT        150  //At this sample Out of Filtering is constants
#define WINDOW_FOR_DETECING_PEAK       10
#define MIN_HEART_PERIOD_SEC           0.46 // 130bpm
#define MAX_HEART_PERIOD_SEC           2   // 30bpm

typedef enum
{
	FINGER_STATE_NOT_DETECTED,
	FINGER_STATE_DETECTED,
}FINGER_STATE;

typedef enum
{
	CURRENT_BIAS_STATE_NOT_OK,
	CURRENT_BIAS_STATE_OK,
}CURRENT_BIAS_STATE;
/*****************************************************************/

typedef struct
{
	uint8_t interruptFlag;
	MAX30100_MODE		  mode;
	MAX30100_SpO2_SR 	  SPO2SampleRate;
	MAX30100_LED_PW       ledPulseWidth;
	MAX30100_LED_Current  redPa;
	MAX30100_LED_Current  irPa;
	uint8_t  interruptStatusReg;
	uint8_t  interruptEnableReg;
	uint8_t	 modeConfReg;
	uint8_t	 SPO2ConfReg;
	uint8_t	 ledConfReg;
	uint8_t	 fifoDataReg;
	uint16_t redSamples[MAX30100_FIFO_SAMPLES_SIZE];
	uint16_t irSamples[MAX30100_FIFO_SAMPLES_SIZE];
	uint8_t dataReadingFlag1;
	uint8_t dataReadingFlag2;
	float durationBetweenPeak1Peak2;
	float durationBetweenPeak2Peak3;
	float samplesBetweenPeak1Peak3;
	uint8_t	numOfPeaks;
	uint8_t	heartRate;
	uint8_t  SPO2;
	uint16_t bufferIndex;
	uint16_t irRawBuffer[OXYMETER_MAX_BUFFER_SIZE];
	uint16_t redRawBuffer[OXYMETER_MAX_BUFFER_SIZE];
	uint32_t timeLastBiasModified;
	FINGER_STATE fingerState;
	CURRENT_BIAS_STATE currentBiasState;
	uint32_t processStartTick;
	uint32_t processEndTick;
	uint32_t processTimeMs;
}MAX30100_s;

extern  MAX30100_s MaxStruct;

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

Module_Status Init_MAX30100(void);
Module_Status Plot_To_UART(uint8_t port, MAX30100_MODE mode);
Module_Status HR_Mode_Read_Buffer(uint16_t *irSampleBuffer);
Module_Status SPO2_Mode_Read_Buffer(uint16_t *redSampleBuffer, uint16_t *irSampleBuffer);
Module_Status Get_Finger_State(FINGER_STATE *fingerState);
Module_Status Get_HR(uint8_t *heartRate);
Module_Status Get_SPO2(uint8_t *SPO2);
Module_Status Reset_SampleRead_Flag();
Module_Status Get_SampleRead_Flag(uint8_t *sampleReadFlag);

/* -----------------------------------------------------------------------
 |								Commands							      ||
/* -----------------------------------------------------------------------
 */


#endif /* H2BR1_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
