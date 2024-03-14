/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : MAX17330_reg_address.h
 Description   :  Header file of register battery charger/gauge (MAX17330).

 */


/* Exported macros -----------------------------------------------------------*/
/* Battery Charger/Gauge I2C addresses */
#define I2C_6Ch_W_ADD				0x6C
#define I2C_6Ch_R_ADD				0x6D
#define I2C_16h_W_ADD				0x16
#define I2C_16h_R_ADD				0x17
#define FST_I2C_LMT_ADD				0xFF

/* Battery Charger/Gauge special macros */
#define SENSE_RES_VAL				0.01
#define SENSE_RES_REG_VAL			0x03E8
#define CAP_RESOL_VAL				0.000005/SENSE_RES_VAL
#define PERCENT_RESOL_VAL			256
#define VOLT_RESOL_VAL				0.000078125
#define CUR_RESOL_VAL				0.0000015625/SENSE_RES_VAL
#define TEMP_RESOL_VAL				256
#define POWER_RESOL_VAL				0.0008
#define RES_RESOL_VAL				4096
#define TIME_RESOL_VAL				5.625
#define THRMRES_BCONST_REF_VAL			34
#define THRMRES_CONFIG_REG_VAL		0x71E8
#define MANFCTR_NAME_SIZE			0x0C
#define DEVICE_NAME_SIZE			0x0A
#define MANFCTR_DATA_SIZE			0x1A
#define MANFCTR_INFO_SIZE			0x18
#define SERIAL_NUUM_SIZE			0x08
#define ID_BUF_SIZE					MANFCTR_NAME_SIZE + DEVICE_NAME_SIZE
#define BLOCK_TIME 					7500
#define UPDATE_TIME 				1300
#define RECALL_TIME 				5
#define NUM_1S_1					1
#define NUM_1S_2					3
#define NUM_1S_3					7
#define NUM_1S_4					15
#define NUM_1S_5					31
#define NUM_1S_6					63
#define NUM_1S_7					127
#define NUM_1S_8					255
#define ALRT_EN						4
#define CMD_STAT_ERR_CLEAR			4

/* Alert thresholds values */
#define MIN_VOLT_ALRT_THRE			0x64			/*  minimum alert voltage threshold */
#define MAX_VOLT_ALRT_THRE			0x64			/*  maximum alert voltage threshold */
#define MIN_TEMP_ALRT_THRE			0x10			/*  minimum alert temperature threshold */
#define MAX_TEMP_ALRT_THRE			0x64			/*  maximum alert temperature threshold */
#define MIN_SOC_ALRT_THRE			0x10			/*  minimum alert SOC threshold */
#define MAX_SOC_ALRT_THRE			0x64			/*  maximum alert SOC threshold */
#define MIN_CUR_ALRT_THRE			0x0A			/*  minimum alert current threshold */
#define MAX_CUR_ALRT_THRE			0x64			/*  maximum alert current threshold */

/*  Charger/Gauge registers addresses */
/* Charging status and configurations registers */
#define CHARGE_STATUS_REG_ADD		0x00A3
#define CHARGE_VOLTAGE_REG_ADD		0x002A
#define CHARGE_CURRENT_REG_ADD		0x0028

/* Protection Status and Configurations registers */
#define PROTECT_CONFIGS_REG_ADD		0x01D7
#define PROTECT_STATUS_REG_ADD		0x00D9
#define BATTERY_STATUS_REG_ADD		0x01A8
#define PROTECT_ALERT_REG_ADD		0x00AF

/* Status and Configurations registers */
#define SENSE_RES_REG_ADD			0x01CF			/* application sense resistor value register */
#define PACK_CONFIG_REG_ADD			0x01B5			/* pack configurations register */
#define THERM_CONFIG_REG_ADD		0x01CA			/* external thermistor configuration register */
#define NCONFIG_REG_ADD				0x01B0			/* configuration register */
#define CONFIG_REG_ADD				0x000B			/* configuration register */
#define CONFIG2_REG_ADD				0x00AB			/* configuration2 register */

/* Fuel gauge registers */
#define REP_SOC_REG_ADD				0x0006			/*  reported State Of Charge register */
#define REP_CAP_REG_ADD				0x0005			/*  reported capacity register */
#define FULL_CAP_REG_ADD			0x0010			/*  full capacity reported register */
#define TTE_REG_ADD					0x0011			/*  time to empty register */
#define TTF_REG_ADD					0x0020			/*  time to full register */
#define AGE_REG_ADD					0x0007			/*  age register */
#define CYCLES_REG_ADD				0x0017			/*  charge discharge cycles register */
#define NUM_CYCLES_REG_ADD			0x01A4			/*  number of cycles register */
#define INTERNAL_RES_REG_ADD		0x0014			/*  calculated internal resistance register */
#define VOLT_TEMP_REG_ADD			0x01AA			/* Voltage temperature register */
#define AVG_CAP_REG_ADD				0x001F			/*  average capacity register */
#define AVG_SOC_REG_ADD				0x000E			/*  average State Of Charge register */
#define AGE_FRCST_REG_ADD			0x00B9			/*  age forecast register */
#define NV_CONFIG0_REG_ADD			0x01B8			/* nNVCfg0 register */
#define NV_CONFIG2_REG_ADD			0x01BA			/* nNVCfg2 register */

/* Analog measurements registers */
#define CELL_VOLT_REG_ADD			0x001A			/*  cell voltage register */
#define VOLT_REG_ADD				0x00D7			/*  voltage register */
#define CURRENT_REG_ADD				0x001C			/*  current register */
#define TEMP_REG_ADD				0x001B			/*  temperature register */
#define DIE_TEMP_REG_ADD			0x001B			/*  die temperature register */
#define POWER_REG_ADD				0x00B1			/*  power register */

/* Alert thresholds registers */
#define VOLT_ALRT_THRE_REG_ADD		0x018C			/*  alert voltage thresholds register */
#define TEMP_ALRT_THRE_REG_ADD		0x018D			/*  alert temperature thresholds register */
#define SOC_ALRT_THRE_REG_ADD		0x018F			/*  alert SOC thresholds register */
#define CUR_ALRT_THRE_REG_ADD		0x018E			/*  alert current thresholds register */

/* IDs registers */
#define SERIAL_NUM_REG_ADD			0x011C			/*  charger/gauge serial number register */
#define MANFCTR_NAME_REG_ADD		0x0120			/*  charger/gauge manufacturer number register */
#define DEVICE_NAME_REG_ADD			0x0121			/*  charger/gauge device name register */
#define MANFCTR_DATA_REG_ADD		0x0123			/*  charger/gauge manufacturer data register */
#define MANFCTR_INFO_REG_ADD		0x0170			/*  charger/gauge manufacturer info register */

/* Commands registers */
#define CMD_REG_ADD					0x0060			/*  charger/gauge command register */
#define CMD_STAT_REG_ADD			0x0061			/*  charger/gauge command status register */
#define REM_UPDT_REG_ADD			0x01FD			/*  charger/gauge remaining updates register */
