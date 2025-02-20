/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0_i2c.c
 Description   : This file provides code for the configuration
 of the I2C instances.

 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

Module_Status MAX30100_Write(uint8_t regAddr, uint8_t txData, uint32_t timeout);
Module_Status MAX30100_Read(uint8_t regAddr, uint8_t *pRxData, uint8_t size, uint32_t timeout);

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
void MX_I2C2_Init(void) {
	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* Initialize I2C2 peripheral */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10B17DB5; // Normal mode (100 kHz)
	hi2c2.Init.OwnAddress1 = 0; // No specific address required for master mode
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual address mode
	hi2c2.Init.OwnAddress2 = 0; // Not used, set to 0
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK; // No mask for second address
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disable clock stretching
	HAL_I2C_Init(&hi2c2);

	/** Configure Analogue filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE); // Enable analog filter

	/** Configure Digital filter */
	HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0); // Digital filter set to 0 (disabled)

	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB3     ------> I2C2_SCL
    PB4     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB3     ------> I2C2_SCL
    PB4     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/**********************MAX30100 Interface Functions****************************/
Module_Status MAX30100_Write(uint8_t regAddr, uint8_t txData, uint32_t timeout)
{
	Module_Status Status = HAL_OK;
	uint8_t data[2] = {0};
	data[0] = regAddr;
	data[1] = txData;
	uint8_t size = 2;
//	taskENTER_CRITICAL();
//	HAL_I2C_Master_Transmit(&HANDLER_I2C_MAX30100, MAX30100_I2C_ADDRESS , data, size, timeout);
//	taskEXIT_CRITICAL();

	return Status;
}


Module_Status MAX30100_Read(uint8_t regAddr, uint8_t *pRxData, uint8_t size, uint32_t timeout)
{
	Module_Status Status = HAL_OK;
//	taskENTER_CRITICAL();
//	HAL_I2C_Master_Transmit(&HANDLER_I2C_MAX30100, MAX30100_I2C_ADDRESS, &regAddr, 1, timeout);
//	taskEXIT_CRITICAL();
//
//	taskENTER_CRITICAL();
//	HAL_I2C_Master_Receive(&HANDLER_I2C_MAX30100, MAX30100_I2C_ADDRESS,  pRxData, size, timeout);
//	taskEXIT_CRITICAL();

	return Status;
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
