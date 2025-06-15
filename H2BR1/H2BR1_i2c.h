/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H2BR1_i2c.h
 Description: Header file for I2C configuration on H2BR1 module.
 Peripherals: Declares I2C2 handle and MAX30100 communication functions.
 Features: Prototypes for I2C initialization and MAX30100 read/write operations.
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "stm32g0xx_hal.h"

/* Exported Variables ******************************************************/
extern I2C_HandleTypeDef hi2c2;

/* Exported Functions ******************************************************/
extern void MX_I2C_Init(void);

extern Module_Status MAX30100_Write(uint8_t regAddr, uint8_t txData, uint32_t timeout);
extern Module_Status MAX30100_Read(uint8_t regAddr, uint8_t *pRxData, uint8_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /*__i2c_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
