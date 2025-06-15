/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H2BR1_inputs.h
 Description: Declares functions and macros for digital and analog inputs.
 Inputs: Supports buttons (momentary/on-off, NO/NC) and ADC channels (P2, P3).
 ADC: Defines pins, ports, and channels for voltage, temperature, and Vref readings.
*/

/* Includes ****************************************************************/
#include "BOS.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_adc_ex.h"
#include "string.h"
/*This module includes an ADC on port P2.*/
/* ADC Macro Definitions ***************************************************/
/* Port-ADC Definitions */
#define ADC_CH1_PIN   		GPIO_PIN_2
#define ADC_CH2_PIN   		GPIO_PIN_3
#define ADC_CH3_PIN   		0
#define ADC_CH4_PIN   		0
#define ADC12_PORT  		P2
#define ADC34_PORT			0
#define ADC12_GPIO_PORT  	GPIOA
#define ADC34_GPIO_PORT		0
#define ADC_CH1_USART   	USART2
#define ADC_CH2_USART   	USART2
#define ADC_CH3_USART   	0
#define ADC_CH4_USART   	0
#define ADC_CH1_CHANNEL   	ADC_CHANNEL_2
#define ADC_CH2_CHANNEL   	ADC_CHANNEL_3
#define ADC_CH3_CHANNEL   	0
#define ADC_CH4_CHANNEL   	0

/* Constant Macros */
#define VREF_CAL            ((uint16_t *)((uint32_t)0x1FFF75AA))
#define AVG_SLOPE           4.3
#define V25                 1.41

/***************************************************************************/
/* Exported Functions Prototypes *******************************************/
/***************************************************************************/
extern void ReadTempAndVref(float *temp,float *Vref);
extern BOS_Status ReadADCChannel(uint8_t Port,char *side,float *ADC_Value);
extern BOS_Status ADCSelectPort(uint8_t ADC_port);
extern BOS_Status GetReadPercentage(uint8_t port, char *side, float *precentageValue);
extern BOS_Status ADCDeinitChannel(uint8_t port);

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
