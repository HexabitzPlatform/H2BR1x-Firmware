/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : MAX17330_reg_address.h
 Description   :  Header file of register battery charger/gauge (MAX17330).

 */


/* Exported macros -----------------------------------------------------------*/

//I2C_HandleTypeDef HANDLER_I2C_MAX30100;
//#define HANDLER_I2C_MAX30100  hi2c2

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
