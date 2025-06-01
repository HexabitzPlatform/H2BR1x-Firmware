/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H2BR1.c
 Description   : Source code for module H2BR1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H2BR1_inputs.h"
#include "H2BR1_i2c.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

All_Data PortFunction;
MAX30100_s MaxStruct;


TimerHandle_t xTimerStream = NULL;

/* Private Variables *******************************************************/
/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */


/* Global variables for sensor data used in ModuleParam */
uint8_t H2BR1_heartRate = 0;
uint8_t H2BR1_SPO2 = 0;
FINGER_STATE H2BR1_fingerState = 0;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={
    {.ParamPtr = &H2BR1_heartRate, .ParamFormat = FMT_UINT8, .ParamName = "heartrate"},
    {.ParamPtr = &H2BR1_SPO2, .ParamFormat = FMT_UINT8, .ParamName = "spo2"},
    {.ParamPtr = &H2BR1_fingerState, .ParamFormat = FMT_UINT8, .ParamName = "fingerstate"}
};

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char*,size_t);
typedef void (*SampleToBuffer)(float *buffer);


/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
Module_Status Init_MAX30100(void);

void MAX30100_Reset(void);
void MAX30100_Read_FIFO();
void MAX30100_Clear_FIFO(void);
void MAX30100_Set_Mode(MAX30100_MODE mode);
void MAX30100_Enable_Interrupt(INTERRUPT_EN_A_FULL_BIT aFull, INTERRUPT_EN_TEMP_RDY_BIT tempRdy,
		INTERRUPT_EN_HR_RDY_BIT hrRdy, INTERRUPT_EN_SPO2_RDY_BIT Spo2Rdy);

void MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW pulseWidth );
void MAX30100_Set_SpO2_SampleRate(MAX30100_SpO2_SR sampleRate);
void MAX30100_Set_Led_Current(MAX30100_LED_Current redPa, MAX30100_LED_Current irPa );

void Oxymeter_Detect_Finger();
void Oxymeter_Signal_Processing();
void Oxymeter_Add_Samples_To_Buffers();
void Oxymeter_Modify_Led_Current_Bias();

/* These two functions should be Used in external interrupt service routine */
void Read_Data_When_Interrupt(void);
void Oxymeter_Calculating_HR_SPO2(void);

/* Stream Functions */
void StreamTimeCallback(TimerHandle_t xTimerStream);

void SampleHRToString(char *cstring, size_t maxLen);
void SampleSPO2ToString(char *cstring, size_t maxLen);

Module_Status SampleToTerminal(uint8_t dstPort, All_Data mode);
Module_Status ExportStreanToPort (uint8_t module,uint8_t port,All_Data Sensor,uint32_t Numofsamples,uint32_t timeout);

static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples);
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function);
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
		bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_HR_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SPO2_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_FingerStateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE StreamSPO2Command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure ***************************************************/
/* CLI command structure : HR_Sample */
const CLI_Command_Definition_t CLI_HR_SampleCommandDefinition = {
	( const int8_t * ) "hrsample", /* The command string to type. */
	( const int8_t * ) "hrsample:\r\nTake one sample measurement to measure heart rate after 6 seconds from placing the hand on the sensor.\r\n\r\n",
	CLI_HR_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : SPO2_Sample */
const CLI_Command_Definition_t CLI_SPO2_SampleCommandDefinition = {
	( const int8_t * ) "spo2sample", /* The command string to type. */
	( const int8_t * ) "spo2sample:\r\nTake one sample measurement to measure oxygenation rate after 6 seconds from placing the hand on the sensor.\r\n\r\n",
	CLI_SPO2_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/***************************************************************************/
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [EMG]/[EEG]/[EOG]/[ECG] (Numofsamples ) (time in ms) [port] [module].\r\n\r\n",
	StreamSPO2Command,
	-1
};

/***************************************************************************/
/* CLI command structure : FingerState */
const CLI_Command_Definition_t CLI_FingerStateCommandDefinition = {
	( const int8_t * ) "fingerstate", /* The command string to type. */
	( const int8_t * ) "fingerstate:\r\nFeel the presence of a finger on or near the sensor.\r\n\r\n",
	CLI_FingerStateCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status StopeCliStreamFlag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the StopeCliStreamFlag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}
Module_Status Y ;
/***************************************************************************/
/* H2BR1 module initialization */
void Module_Peripheral_Init(void) {

//	 __HAL_RCC_GPIOB_CLK_ENABLE();
//	 __HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();

	SPO2GPIOInit();
	MX_I2C_Init();
	Y=Init_MAX30100();

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		}
	}

	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream = xTimerCreate("StreamTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*) 1, StreamTimeCallback);

}

/***************************************************************************/
/*  H2BR1 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H2BR1_OK;
	uint8_t fingerState = 0;
	uint8_t module = 0;

	switch (code) {

	case CODE_H2BR1_HR_SAMPLE:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], HR);
		break;

	case CODE_H2BR1_SPO2_SAMPLE:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], SPO2);
		break;

	default:
		result = H2BR1_ERR_UNKNOWNMESSAGE;
		break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_HR_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SPO2_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_FingerStateCommandDefinition);
}

/***************************************************************************/
/* This function is useful only for input (sensors) modules.
 * Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = BOS_OK;
	FINGER_STATE FingerTemp;
	uint8_t temp = 0;

	switch (paramIndex) {

	/* Sample Heart Rate */
	case 1:
		status = HR_Sample(&temp);
		if (status == BOS_OK)
			*value = (float) temp;
		break;

		/* Sample SPO2 */
	case 2:
		status = SPO2_Sample(&temp);
		if (status == BOS_OK)
			*value = (float) temp;
		break;

		/* Sample Finger State */
	case 3:
		status = FingerState(&FingerTemp);
		if (status == BOS_OK)
			*value = (float) FingerTemp;
		break;

		/* Invalid parameter index */
	default:
		status = BOS_ERR_WrongParam;
		break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* Callback function triggered by a timer to manage data streaming.
 * xTimerStream: Handle of the timer that triggered the callback.
 */
void StreamTimeCallback(TimerHandle_t xTimerStream) {

	uint32_t SampleCount = 0u;                   /* Total sample counter */

	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if (STREAM_MODE_TO_PORT == StreamMode) {
		if ((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)) {
			SampleToPort(PortModule, PortNumber, PortFunction);

		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}

	/* Stream mode to terminal: Export to terminal */
	else if (STREAM_MODE_TO_TERMINAL == StreamMode) {
		if ((SampleCount <= TerminalNumOfSamples)
				|| (0 == TerminalNumOfSamples)) {
			SampleToTerminal(TerminalPort, PortFunction);
		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}
}
/***************************************************************************/
/* Samples heart rate data into a buffer.
 * buffer: Pointer to the buffer where heart rate data will be stored.
 */
void SampleHRBuf(float *buffer) {
    uint8_t heartRate;
    HR_Sample(&heartRate);
    *buffer = (float)heartRate;
}

/***************************************************************************/
/* Samples SpO2 data into a buffer.
 * buffer: Pointer to the buffer where SpO2 data will be stored.
 */
void SampleSPO2Buf(float *buffer) {
    uint8_t spo2;
    SPO2_Sample(&spo2);
    *buffer = (float)spo2;
}
/***************************************************************************/
/* Streams sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SampleHRBuf, SampleSPO2Buf).
 */
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function) {
    Module_Status status = H2BR1_OK;
    uint16_t StreamIndex = 0;
    uint32_t period = timeout / Numofsamples;

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS)
        return H2BR1_ERR_WRONGPARAMS;

    stopStream = false;

    /* Stream data to buffer */
    while ((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
        float sample;
        function(&sample);
        buffer[StreamIndex] = sample;
        StreamIndex++;

        /* Delay for the specified period */
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Check if streaming should be stopped */
        if (stopStream) {
            status = H2BR1_ERR_TERMINATED;
            break;
        }
    }

    return status;
}
/***************************************************************************/
void MAX30100_Reset(void) {
	uint8_t modeConfReg = 0;

	modeConfReg = modeConfReg & MAX30100_MODE_CONF_RESET_MASK;
	MAX30100_Write(MAX30100_MODE_CONF_ADDR, modeConfReg, 100);

	// delay until completing reset process
	HAL_Delay(1);
	MaxStruct.modeConfReg = modeConfReg;
}

/***************************************************************************/
void MAX30100_Enable_Interrupt(INTERRUPT_EN_A_FULL_BIT aFull, INTERRUPT_EN_TEMP_RDY_BIT tempRdy, INTERRUPT_EN_HR_RDY_BIT hrRdy,
		INTERRUPT_EN_SPO2_RDY_BIT Spo2Rdy) {

	uint8_t interrEnReg = aFull | tempRdy | hrRdy | Spo2Rdy;
	MAX30100_Write(MAX30100_INTERRUPT_EN_ADDR, interrEnReg, 100);
	MaxStruct.interruptEnableReg = interrEnReg;
}

/***************************************************************************/
void MAX30100_Set_Mode(MAX30100_MODE mode) {
	MaxStruct.mode = mode;
	uint8_t modeConfReg = 0;

	MAX30100_Read(MAX30100_MODE_CONF_ADDR, &modeConfReg, 1, 100);
	modeConfReg = (modeConfReg & ~MAX30100_MODE_CONF_MODE_MASK) | (mode << MAX30100_MODE_CONF_MODE_POSITION);
	/*Enable Temp_En bit */
	if (mode == SPO2_MODE)
		modeConfReg |= MAX30100_MODE_CONF_TEMP_EN_MASK;
	/* Disable Temp_En bit */
	else
		modeConfReg &= ~MAX30100_MODE_CONF_TEMP_EN_MASK;

	MAX30100_Write(MAX30100_MODE_CONF_ADDR, modeConfReg, 100);
	MaxStruct.modeConfReg = modeConfReg;
	/* disable all interrupt bits */
	if (mode == UNUSED_MODE)
		MAX30100_Enable_Interrupt(INTERRUPT_EN_AFULL_DISABLED, INTERRUPT_EN_TEMP_RDY_DISABLED,
				INTERRUPT_EN_HR_RDY_DISABLED, INTERRUPT_EN_SPO2_RDY_DISABLED);
	/* enable A_Full interrupt bit */
	else
		MAX30100_Enable_Interrupt(INTERRUPT_EN_AFULL_ENABLED, INTERRUPT_EN_TEMP_RDY_DISABLED,
				INTERRUPT_EN_HR_RDY_DISABLED, INTERRUPT_EN_SPO2_RDY_DISABLED);
}

/***************************************************************************/
void MAX30100_Set_SpO2_SampleRate(MAX30100_SpO2_SR sampleRate) {
	uint8_t SPO2ConfReg = 0;
	MaxStruct.SPO2SampleRate = sampleRate;

	MAX30100_Read(MAX30100_SPO2_CONF_ADDR, &SPO2ConfReg, 1, 100);
	SPO2ConfReg = (SPO2ConfReg & ~MAX30100_SPO2_CONF_SR_MASK) | (sampleRate << MAX30100_SPO2_CONF_SR_POSITION);

	MAX30100_Write(MAX30100_SPO2_CONF_ADDR, SPO2ConfReg, 100);
	MaxStruct.SPO2ConfReg = SPO2ConfReg;
}

/***************************************************************************/
void MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW pulseWidth) {
	uint8_t SPO2ConfReg = 0;
	MaxStruct.ledPulseWidth = pulseWidth;

	MAX30100_Read(MAX30100_SPO2_CONF_ADDR, &SPO2ConfReg, 1, 100);
	SPO2ConfReg = (SPO2ConfReg & ~MAX30100_SPO2_CONF_LED_PW_MASK) | ((pulseWidth << MAX30100_SPO2_CONF_LED_PW_POSITION) & MAX30100_SPO2_CONF_LED_PW_MASK);

	MAX30100_Write(MAX30100_SPO2_CONF_ADDR, SPO2ConfReg, 100);
	MaxStruct.SPO2ConfReg = SPO2ConfReg;
}

/***************************************************************************/
void MAX30100_Set_Led_Current(MAX30100_LED_Current redPa, MAX30100_LED_Current irPa) {
	uint8_t ledConfReg = 0;
	MaxStruct.redPa = redPa;

	ledConfReg = (redPa << MAX30100_LED_CONF_RED_PA_POSITION) | (irPa << MAX30100_LED_CONF_IR_PA_POSITION);
	MAX30100_Write(MAX30100_LED_CONF_ADDR, ledConfReg, 100);
	MaxStruct.ledConfReg = ledConfReg;
}

/***************************************************************************/
void MAX30100_Read_FIFO() {
	uint8_t i = 0, j = 0;
	uint8_t fifoData[MAX30100_FIFO_DATA_SIZE] = { 0 };

	/* timeout =1000msec; reading 16 samples needs from 15-300msec */
	MAX30100_Read(MAX30100D_FIFO_DATA_ADDR, fifoData, MAX30100_FIFO_DATA_SIZE, 1000);

	for (i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++) {
		/* First read byte is MSB. Size of sample is 2 bytes */
		MaxStruct.irSamples[i] = (fifoData[j] << 8) | fifoData[j + 1];
		MaxStruct.redSamples[i] = (fifoData[j + 2] << 8) | fifoData[j + 3];
		j += 4;
	}
}

/***************************************************************************/
void MAX30100_Clear_FIFO(void) {

	MAX30100_Write(MAX30100_FIFO_W_PTR_ADDR, 0x00, 100);
	MAX30100_Write(MAX30100_OVF_COUNTER_ADDR, 0x00, 100);
	MAX30100_Write(MAX30100_FIFO_R_PTR_ADDR, 0x00, 100);
}

/***************************************************************************/
void Oxymeter_Add_Samples_To_Buffers() {
	uint16_t buffIndex = MaxStruct.bufferIndex; // last value of bufferIndex

	if (buffIndex >= OXYMETER_MAX_BUFFER_SIZE)
		buffIndex = 0;

	for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++) {
		if (buffIndex < OXYMETER_MAX_BUFFER_SIZE) {
			MaxStruct.irRawBuffer[buffIndex] = MaxStruct.irSamples[i];
			MaxStruct.redRawBuffer[buffIndex] = MaxStruct.redSamples[i];
			buffIndex++;
		}
	}
	MaxStruct.bufferIndex = buffIndex;
}

/***************************************************************************/
void Oxymeter_Modify_Led_Current_Bias() {
	uint8_t needModifying = 0;
	uint8_t redLedCurrentIndex = (uint8_t) (MaxStruct.redPa);
	uint32_t tLastModified = MaxStruct.timeLastBiasModified;

	if (MaxStruct.mode == SPO2_MODE) {

		if (HAL_GetTick() - tLastModified > CURRENT_MODIFYING_DURATION_MS) {
			if (MaxStruct.irSamples[0] - MaxStruct.redSamples[0] > CURRENT_MODIFYING_THRESHOLD && redLedCurrentIndex < MAX30100_LED_CURRENT_50P0) {
				++redLedCurrentIndex;
				needModifying = 1;
			}

			else if (MaxStruct.redSamples[0] - MaxStruct.irSamples[0] > CURRENT_MODIFYING_THRESHOLD && redLedCurrentIndex > MAX30100_LED_CURRENT_0P0) {
				--redLedCurrentIndex;
				needModifying = 1;
			} else
				MaxStruct.currentBiasState = CURRENT_BIAS_STATE_OK;

			if (needModifying == 1) {
				MaxStruct.currentBiasState = CURRENT_BIAS_STATE_NOT_OK;
				MAX30100_Set_Led_Current(redLedCurrentIndex, MAX30100_LED_CURRENT_50P0);
				MaxStruct.timeLastBiasModified = HAL_GetTick();
			}
		}
	}
}

/***************************************************************************/
void Oxymeter_Detect_Finger() {
	MaxStruct.fingerState = FINGER_STATE_NOT_DETECTED;

	for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++) {
		if (MaxStruct.irSamples[i] > FINGER_DETECTING_THRESHOLD)
			MaxStruct.fingerState = FINGER_STATE_DETECTED;
		else {
			MaxStruct.fingerState = FINGER_STATE_NOT_DETECTED;
			MaxStruct.bufferIndex = 0;
			MaxStruct.heartRate = 0;
			MaxStruct.SPO2 = 0;
			break;
		}
	}
}

/*-----------------------------------------------------------*/
void Oxymeter_Signal_Processing() {
	float ts = 0.009407; /* should be 0.01 but period of sample from IC not accurate (0.009407); fs=100 */
	MaxStruct.processStartTick = HAL_GetTick();

	/************Removing DC (HPF)*************/
	float irRemovedDC[OXYMETER_MAX_BUFFER_SIZE] = { 0 };
	float redRemovedDC[OXYMETER_MAX_BUFFER_SIZE] = { 0 };
	// HPF: 1st order, Fc=0.8Hz
	uint16_t previousInput = 0;
	uint16_t currentInput = 0;
	// HPF: 1st order, Fc=0.8Hz, fs=100
	for (uint16_t i = 1; i < OXYMETER_MAX_BUFFER_SIZE; i++) {
		previousInput = MaxStruct.irRawBuffer[i - 1];
		currentInput = MaxStruct.irRawBuffer[i];
		irRemovedDC[i] = 0.9755 * (float) currentInput
				- 0.9755 * (float) previousInput + 0.9510 * irRemovedDC[i - 1];
	}
	// HPF: 1st order, Fc=0.8Hz, fs=100
	for (uint16_t i = 1; i < OXYMETER_MAX_BUFFER_SIZE; i++) {
		previousInput = MaxStruct.redRawBuffer[i - 1];
		currentInput = MaxStruct.redRawBuffer[i];
		redRemovedDC[i] = 0.9755 * (float) currentInput
				- 0.9755 * (float) previousInput + 0.9510 * redRemovedDC[i - 1];
	}
	/************LPF Filtering************/
	float irFiltered[OXYMETER_MAX_BUFFER_SIZE] = { 0 };
	// HPF: 2nd order, Fc=2Hz, fs=100
	for (uint16_t i = 2; i < OXYMETER_MAX_BUFFER_SIZE; i++)
		irFiltered[i] = 0.0036 * irRemovedDC[i] + 0.0072 * irRemovedDC[i - 1]
				+ 0.0036 * irRemovedDC[i - 2] + 1.8227 * irFiltered[i - 1]
				- 0.8372 * irFiltered[i - 2];
	/************Detecting Peaks***********/
	uint8_t numOfPeaks = 0;
	uint16_t peakIndex[3] = { 0 };
	for (uint16_t i = (FILTERING_SAMPLES_SHIFT + WINDOW_FOR_DETECING_PEAK);
			i < (OXYMETER_MAX_BUFFER_SIZE - WINDOW_FOR_DETECING_PEAK); i++) {
		if (irFiltered[i] < irFiltered[i - 1]
				&& irFiltered[i] < irFiltered[i + 1]) // min peak
						{
			if (irFiltered[i] < irFiltered[i - WINDOW_FOR_DETECING_PEAK]
					&& irFiltered[i]
							< irFiltered[i + WINDOW_FOR_DETECING_PEAK]) {
				numOfPeaks++;
				peakIndex[numOfPeaks - 1] = i;
				if (numOfPeaks == 3)
					break;
			}
		}
	}
	MaxStruct.numOfPeaks = numOfPeaks;
	/*****Calculating Heart Rate (HR)******/
	uint16_t peakInd1 = 0, peakInd2 = 0, peakInd3 = 0;
	float durP1_P2 = 0, durP2_P3 = 0, samplesP1_P3 = 0;
	float period = 0;
	float HR = 0;
	if (numOfPeaks == 3) {
		peakInd1 = peakIndex[0];
		peakInd2 = peakIndex[1];
		peakInd3 = peakIndex[2];
		durP1_P2 = (peakInd2 - peakInd1) * ts;
		durP2_P3 = (peakInd3 - peakInd2) * ts;
		samplesP1_P3 = peakInd3 - peakInd1;
		MaxStruct.durationBetweenPeak1Peak2 = durP1_P2;
		MaxStruct.durationBetweenPeak2Peak3 = durP2_P3;
		MaxStruct.samplesBetweenPeak1Peak3 = samplesP1_P3;
		if (durP1_P2 > MIN_HEART_PERIOD_SEC && durP1_P2 < MAX_HEART_PERIOD_SEC) {
			if (durP1_P2 <= 1.2 * durP2_P3 && durP1_P2 >= 0.8 * durP2_P3) {
				period = (durP1_P2 + durP2_P3) / 2.0; // period = mean of (durP1_P2, durP2_P3)
				HR = 60.0 / period; // HR=60/T
				MaxStruct.heartRate = roundf(HR);
			}
		} else
			MaxStruct.heartRate = 0;
	} else
		MaxStruct.heartRate = 0;
	/**********Calculating SPO2***********/
	// Calibration array // SaO2 Look-up Table(http://www.ti.com/lit/an/slaa274b/slaa274b.pdf)
	uint8_t spO2LUT[43] = { 100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 98, 98,
			98, 98, 98, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95,
			95, 95, 95, 95, 94, 94, 94, 94, 94, 93, 93, 93, 93, 93 };
	float irACSqSum = 0, redACSqSum = 0;
	uint8_t SPO2Iindex = 0, ratio = 0, SPO2Value;

	if (MaxStruct.mode == SPO2_MODE) {
		if (HR != 0) {
			for (uint16_t i = peakInd1; i <= peakInd3; i++) {
				irACSqSum = irACSqSum + (irRemovedDC[i] * irRemovedDC[i]);
				redACSqSum = redACSqSum + (redRemovedDC[i] * redRemovedDC[i]);
			}
			ratio = roundf(100.0 * log(redACSqSum / samplesP1_P3) / log(irACSqSum / samplesP1_P3));
			if (ratio > 66)
				SPO2Iindex = ratio - 66;

			else if (ratio > 50)
				SPO2Iindex = ratio - 50;

			SPO2Value = spO2LUT[SPO2Iindex];
			if (SPO2Value >= 93 && SPO2Value <= 100)
				MaxStruct.SPO2 = SPO2Value;

			else
				MaxStruct.SPO2 = 0;

		} else
			MaxStruct.SPO2 = 0;
	}

	MaxStruct.processEndTick = HAL_GetTick();
	MaxStruct.processTimeMs = MaxStruct.processEndTick - MaxStruct.processStartTick;
}

/***************************************************************************/
/* This function should put within external interrupt function */
void Read_Data_When_Interrupt(void) {
	uint8_t interruptReg = 0;

	MAX30100_Read(MAX30100_INTERRUPT_ADDR, &interruptReg, 1, 100);

	/* if Samples FIFO Buffer is full (A_Full==1) */
	if ((interruptReg & MAX30100_INTERRUPT_A_FULL_MASK)
			>> MAX30100_INTERRUPT_A_FULL_POSITION) {
		MAX30100_Read_FIFO(MaxStruct);
		MaxStruct.dataReadingFlag1 = 1;
		MaxStruct.dataReadingFlag2 = 1;
	}
}

/***************************************************************************/
void Oxymeter_Calculating_HR_SPO2(void) {
	Oxymeter_Detect_Finger(MaxStruct);

	if (MaxStruct.fingerState == FINGER_STATE_DETECTED) {
		Oxymeter_Add_Samples_To_Buffers(MaxStruct);
		Oxymeter_Modify_Led_Current_Bias(MaxStruct);
	}

	if (MaxStruct.bufferIndex == OXYMETER_MAX_BUFFER_SIZE)  // buffer is full
		Oxymeter_Signal_Processing(MaxStruct);
}

/***************************************************************************/
/* initialize MAX30100 */
Module_Status Init_MAX30100(void) {
	uint8_t status = H2BR1_OK;
	uint8_t mode = SPO2_MODE;
	uint8_t interruptReg = 0;

	/* primary values of RedLed 33.8 mA and IrLed 50mA so that received light intensity
	 * (from Red and Ir) is nearly adjacent but it is different from finger to other.. */
	MAX30100_Set_Led_Current(MAX30100_LED_CURRENT_33P8, MAX30100_LED_CURRENT_50P0);
	MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW_1600);
	MAX30100_Set_SpO2_SampleRate(MAX30100_SPO2_SR_100);
	MAX30100_Set_Mode(mode);

	/* InterruptReg should be read firstly so that interrupt pin goes 'high'
	 * When max30100 is booted interrupt pin is 'low' */
	if (MAX30100_Read(MAX30100_INTERRUPT_ADDR, &interruptReg, 1, 100) == H2BR1_OK)
		status = H2BR1_OK;
	else
		status = H2BR1_ERR_WRONGPARAMS;

	return status;
}

/***************************************************************************/
/* Streams heart rate (HR) or oxygen saturation (SPO2) samples to the terminal.
 * dstPort: The port number used for data transmission.
 * mode: The mode of operation (HR_MODE for heart rate, SPO2_MODE for oxygenation).
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data mode) {
	Module_Status status = H2BR1_OK; /* Initialize operation status as success */
	char formattedString[20]; /* Buffer for formatted output string */

	/* Validate the port number */
	if (dstPort == 0)
		return H2BR1_ERR_WRONGPARAMS; /* Return error for invalid port */

	/* Process data based on the selected mode */
	switch (mode) {
	case HR:
		/* Check if new IR data is available */
		if (MaxStruct.dataReadingFlag1 == 1) {
			MaxStruct.dataReadingFlag1 = 0;
			/* Loop through IR samples and send them to the terminal */
			for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++) {
				snprintf(formattedString, sizeof(formattedString), "i%d\r\n", MaxStruct.irSamples[i]);
				HAL_UART_Transmit(GetUart(dstPort), (uint8_t*) formattedString, strlen(formattedString), 100);
			}
		}
		break;

	case SPO2:
		/* Check if new IR and RED data is available */
		if (MaxStruct.dataReadingFlag1 == 1) {
			MaxStruct.dataReadingFlag1 = 0;
			/* Loop through IR and RED samples and send them to the terminal */
			for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++) {
				snprintf(formattedString, sizeof(formattedString), "i%dr%d\r\n", MaxStruct.irSamples[i], MaxStruct.redSamples[i]);
				HAL_UART_Transmit(GetUart(dstPort), (uint8_t*) formattedString, strlen(formattedString), 100);
			}
		}
		break;

	default:
		status = H2BR1_ERR_WRONGPARAMS;
		break;
	}

	return status;
}

/***************************************************************************/
void SampleHRToString(char *cstring, size_t maxLen) {
	uint8_t heartRate;
	HR_Sample(&heartRate);
	snprintf(cstring, maxLen, "HeartRate: %d \r\n", heartRate);
}

/***************************************************************************/
void SampleSPO2ToString(char *cstring, size_t maxLen) {
	uint8_t SPO2;
	SPO2_Sample(&SPO2);
	snprintf(cstring, maxLen, "SPO2: %d \r\n", SPO2);
}

/***************************************************************************/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100;
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		/* Look for ENTER key to stop the stream */
		for (uint8_t chr = 1; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[pcPort - 1][chr] == '\r') {
				UARTRxBuf[pcPort - 1][chr] = 0;
				StopeCliStreamFlag = 1;
				return H2BR1_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H2BR1_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H2BR1_OK;
}

/***************************************************************************/
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function) {
	Module_Status status = H2BR1_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	long numTimes = timeout / period;

	if (period < MIN_PERIOD_MS)
		return H2BR1_ERR_WRONGPARAMS;

	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf[pcPort - 1][chr] == '\r') {
			UARTRxBuf[pcPort - 1][chr] = 0;
		}
	}

	if (1 == StopeCliStreamFlag) {
		StopeCliStreamFlag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}

	if (period > timeout)
		timeout = period;

	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(pcPort, (char*) pcOutputString, strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period, Numofsamples) != H2BR1_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");

	return status;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/***************************************************************************/
/* Sample Read Flag for IC MAX30100
 * sampleReadFlag: pointer to a buffer to store value it always gives 1
 */
Module_Status SampleReadFlag(uint8_t *sampleReadFlag) {
	uint8_t status = H2BR1_OK;

	*sampleReadFlag = MaxStruct.dataReadingFlag2;

	return status;
}

/***************************************************************************/
/* Reset Sample Read Flag for IC MAX30100 */
Module_Status ResetSampleReadFlag(void) {
	uint8_t status = H2BR1_OK;

	MaxStruct.dataReadingFlag2 = 0;

	return status;
}

/***************************************************************************/
/* read the presence of a finger on or near the sensor.
 * fingerState: pointer to a buffer to store value.
 */
Module_Status FingerState(FINGER_STATE *fingerState) {
	uint8_t status = H2BR1_OK;

	*fingerState = MaxStruct.fingerState;

	return status;
}

/***************************************************************************/
/* read heart rate after 6 seconds from placing the hand on the sensor.
 * heartRate: pointer to a buffer to store value
 */
Module_Status HR_Sample(uint8_t *heartRate) {
	uint8_t status = H2BR1_OK;

	*heartRate = MaxStruct.heartRate;

	return status;
}

/***************************************************************************/
/* read oxygenation rate after 6 seconds from placing the hand on the sensor.
 * SPO2: pointer to a buffer to store value
 */
Module_Status SPO2_Sample(uint8_t *SPO2) {
	uint8_t status = H2BR1_OK;

	*SPO2 = MaxStruct.SPO2;

	return status;
}

/***************************************************************************/
/*
 * @brief: Samples data from a heart rate (HR) or oxygen saturation (SPO2) sensor
 *         and exports it to a specified port or module.
 * @param dstModule: The module number to export data to.
 * @param dstPort: The port number to export data to.
 * @param dataFunction: The type of sensor (HR for heart rate, SPO2 for oxygen saturation).
 * @retval: Module status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction) {
    uint8_t hrValue = 0; /* Variable to store heart rate sample */
    uint8_t spo2Value = 0; /* Variable to store oxygen saturation sample */
    Module_Status status = H2BR1_OK; /* Initialize operation status as success */

    /* Validate the port and module ID */
    if (dstPort == 0 && dstModule == myID) {
        return H2BR1_ERR_WRONGPARAMS;
    }

    /* Process data based on the requested sensor type */
    switch (dataFunction) {
        case HR:
            /* Sample heart rate data */
            status = HR_Sample(&hrValue);

            /* If data is to be sent locally */
            if (dstModule == myID) {
                writePxITMutex(dstPort, (char*)&hrValue, sizeof(uint8_t), 10);
            } else {
                /* Send data to another module */
                MessageParams[0] = FMT_UINT8;                                   /* Data format: uint8 */
                MessageParams[1] = (status == H2BR1_OK) ? BOS_OK : BOS_ERROR;  /* Operation status */
                MessageParams[2] = 1;                                          /* Number of elements (hrValue) */
                MessageParams[3] = (uint8_t)(CODE_H2BR1_HR_SAMPLE >> 0);      /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H2BR1_HR_SAMPLE >> 8);      /* Command code MSB */
                MessageParams[5] = hrValue;                                   /* Heart rate value */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 5);
            }
            break;

        case SPO2:
            /* Sample oxygen saturation data */
            status = SPO2_Sample(&spo2Value);

            /* If data is to be sent locally */
            if (dstModule == myID) {
                writePxITMutex(dstPort, (char*)&spo2Value, sizeof(uint8_t), 10);
            } else {
                /* Send data to another module */
                MessageParams[0] = FMT_UINT8;                                   /* Data format: uint8 */
                MessageParams[1] = (status == H2BR1_OK) ? BOS_OK : BOS_ERROR;  /* Operation status */
                MessageParams[2] = 1;                                          /* Number of elements (spo2Value) */
                MessageParams[3] = (uint8_t)(CODE_H2BR1_SPO2_SAMPLE >> 0);    /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H2BR1_SPO2_SAMPLE >> 8);    /* Command code MSB */
                MessageParams[5] = spo2Value;                                 /* Oxygen saturation value */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 5);
            }
            break;

        default:
            status = H2BR1_ERR_WRONGPARAMS;
            break;
    }

    return status;
}

/***************************************************************************/
/* Streams data to the specified port and module with a given number of samples.
 * targetModule: The target module to which data will be streamed.
 * portNumber: The port number on the module.
 * portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * numOfSamples: The number of samples to stream.
 * streamTimeout: The interval (in milliseconds) between successive data transmissions.
 */
Module_Status StreamToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
	Module_Status Status = H2BR1_OK;
	uint32_t SamplePeriod = 0u;

	/* Check timer handle and timeout validity */
	if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H2BR1_ERROR;

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule = dstModule;
	PortNumber = dstPort;
	PortFunction = dataFunction;
	PortNumOfSamples = numOfSamples;
	/* Calculate the period from timeout and number of samples */
	SamplePeriod = streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if (xTimerIsTimerActive(xTimerStream)) {
		if (pdFAIL == xTimerStop(xTimerStream, 100))
			return H2BR1_ERROR;
	}

	/* Start the stream timer */
	if (pdFAIL == xTimerStart(xTimerStream, 100))
		return H2BR1_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100))
		return H2BR1_ERROR;

	return Status;
}

/***************************************************************************/
/* Streams data to the specified terminal port with a given number of samples.
 * targetPort: The port number on the terminal.
 * dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * numOfSamples: The number of samples to stream.
 * streamTimeout: The interval (in milliseconds) between successive data transmissions.
 */
Module_Status StreamToTerminal(uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
	Module_Status Status = H2BR1_OK;
	uint32_t SamplePeriod = 0u;
	uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */

	/* Check timer handle and timeout validity */
	if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H2BR1_ERROR;

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort = dstPort;
	PortFunction = dataFunction;
	TerminalNumOfSamples = numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod = streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if (xTimerIsTimerActive(xTimerStream)) {
		if (pdFAIL == xTimerStop(xTimerStream, 100))
			return H2BR1_ERROR;
	}

	/* Start the stream timer */
	if (pdFAIL == xTimerStart(xTimerStream, 100))
		return H2BR1_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100))
		return H2BR1_ERROR;

	return Status;
}

/***************************************************************************/
/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., HR, SPO2, FINGER_STATE).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout) {
    switch (function) {
        case HR:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleHRBuf);
            break;
        case SPO2:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleSPO2Buf);
            break;
        default:
            return H2BR1_ERR_WRONGPARAMS;
    }
}
/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE StreamSPO2Command(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *const HRCmdName = "hr";
	const char *const SPO2CmdName = "spo2";
	const char *pSensName = NULL;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	uint8_t port = 0;
	uint8_t module = 0;
	uint32_t timeout = 0;
	uint32_t Numofsamples = 0;

	portBASE_TYPE sensNameLen = 0;

	/* Make sure we return something */
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, HRCmdName, strlen(HRCmdName))) {
			if (portOrCLI)
				StreamToCLI(Numofsamples, timeout, SampleHRToString);
			else
				StreamToPort(module, port, HR, Numofsamples, timeout);

		} else if (!strncmp(pSensName, SPO2CmdName, strlen(SPO2CmdName))) {
			if (portOrCLI)
				StreamToCLI(Numofsamples, timeout, SampleSPO2ToString);
			else
				StreamToPort(module, port, SPO2, Numofsamples, timeout);
		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;

	} while (0);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");

	return pdFALSE;
}

/***************************************************************************/
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
		bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule) {
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;
	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;
	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/***************************************************************************/
portBASE_TYPE CLI_HR_SampleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H2BR1_OK;
	uint8_t heartRate = 0;
	static const int8_t *pcOKMessage = (int8_t*) "Heart Rate is : %d bpm\n\r";
	static const int8_t *pcErrorsMessage = (int8_t*) "Error Params!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = HR_Sample(&heartRate);

	if (status == H2BR1_OK)
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, heartRate);

	else if (status == H2BR1_ERROR)
		strcpy((char*) pcWriteBuffer, (char*) pcErrorsMessage);

	return pdFALSE;

}

/***************************************************************************/
portBASE_TYPE CLI_SPO2_SampleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H2BR1_OK;
	uint8_t oxygenationRate = 0;
	static const int8_t *pcOKMessage = (int8_t*) "Oxygenation Rate(SPO2) is : %d %%\n\r";
	static const int8_t *pcErrorsMessage = (int8_t*) "Error Params!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = SPO2_Sample(&oxygenationRate);

	if (status == H2BR1_OK)
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, oxygenationRate);

	else if (status == H2BR1_ERROR)
		strcpy((char*) pcWriteBuffer, (char*) pcErrorsMessage);

	return pdFALSE;

}

/***************************************************************************/
portBASE_TYPE CLI_FingerStateCommand(int8_t *pcWriteBuffer,	size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H2BR1_OK;
	FINGER_STATE fingerState = 0;

	static const int8_t *pcOKMessage = (int8_t*) "FingerState is : %d \n\r";
	static const int8_t *pcErrorsMessage = (int8_t*) "Error Params!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = FingerState(&fingerState);

	if (status == H2BR1_OK)
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, fingerState);

	else if (status == H2BR1_ERROR)
		strcpy((char*) pcWriteBuffer, (char*) pcErrorsMessage);

	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
