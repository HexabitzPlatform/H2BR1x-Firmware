/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H2BR1.c
 Description   : Source code for module H2BR1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H2BR1_inputs.h"
#include "H2BR1_i2c.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
typedef void (*SampleMemsToString)(char *, size_t);
/* Exported functions */

//uint32_t lastTick;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
#define MIN_PERIOD_MS				100
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule);

/* Private variables ---------------------------------------------------------*/
static bool stopStream = false;
MAX30100_s MaxStruct;

uint8_t flag ;



TimerHandle_t xTimerStream = NULL;

/* Stream to port variables */
volatile uint32_t PortNumOfSamples = 0u;    /* Number of samples for port streaming */
volatile uint32_t PortSamples = 0u;         /* Current sample count for port (if needed separately) */
uint8_t PortModule = 0u;           /* Module ID for port streaming */
uint8_t PortNumber = 0u;           /* Port number for streaming */
Sensor PortFunction;                    /* Function pointer or struct for port streaming */

/* Stream to terminal variables */
volatile uint32_t TerminalNumOfSamples = 0u; /* Number of samples for terminal streaming */
volatile uint8_t TerminalPort = 0u;          /* Port number for terminal streaming */
uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */
MAX30100_MODE TerminalFunction;                   /* Function pointer or struct for terminal streaming */
uint8_t StreamMode = 0u;                     /* Streaming mode selector (port or terminal) */
uint8_t StopeCliStreamFlag = 0u;             /* Flag to stop CLI streaming */
/* General streaming variable */
uint32_t SampleCount = 0u;                   /* Total sample counter */


/* Private function prototypes -----------------------------------------------*/
void StreamTimeCallback(TimerHandle_t xTimerStream);
Module_Status SampleToTerminal(uint8_t dstPort, MAX30100_MODE mode);
//Module_Status ExportStreanToTerminal (uint8_t port,Sensor Sensor,uint32_t Numofsamples,uint32_t timeout);
Module_Status ExportStreanToPort (uint8_t module,uint8_t port,Sensor Sensor,uint32_t Numofsamples,uint32_t timeout);
Module_Status Init_MAX30100(void);
//void EXGTask(void *argument);
void MAX30100_Reset();
void MAX30100_Enable_Interrupt(INTERRUPT_EN_A_FULL_BIT aFull, INTERRUPT_EN_TEMP_RDY_BIT tempRdy, INTERRUPT_EN_HR_RDY_BIT hrRdy, INTERRUPT_EN_SPO2_RDY_BIT Spo2Rdy);
void MAX30100_Set_Mode(MAX30100_MODE mode);
void MAX30100_Set_SpO2_SampleRate(MAX30100_SpO2_SR sampleRate);
void MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW pulseWidth );
void MAX30100_Set_Led_Current(MAX30100_LED_Current redPa, MAX30100_LED_Current irPa );
void MAX30100_Read_FIFO();
void MAX30100_Clear_FIFO(void);

void Oxymeter_Modify_Led_Current_Bias();
void Oxymeter_Add_Samples_To_Buffers();
void Oxymeter_Detect_Finger();
void Oxymeter_Signal_Processing();
void SampleHRToString(char *cstring, size_t maxLen);
void SampleSPO2ToString(char *cstring, size_t maxLen);
//These two functions should be put in external interrupt service routine
void Read_Data_When_Interrupt();
void Oxymeter_Calculating_HR_SPO2();


/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_HR_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SPO2_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_FingerStateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE StreamEXGCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/*-----------------------------------------------------------*/
/* CLI command structure : HR_Sample */
const CLI_Command_Definition_t CLI_HR_SampleCommandDefinition =
{
	( const int8_t * ) "hrsample", /* The command string to type. */
	( const int8_t * ) "hrsample:\r\nTake one sample measurement to measure heart rate after 6 seconds from placing the hand on the sensor.\r\n\r\n",
	CLI_HR_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : SPO2_Sample */
const CLI_Command_Definition_t CLI_SPO2_SampleCommandDefinition =
{
	( const int8_t * ) "spo2sample", /* The command string to type. */
	( const int8_t * ) "spo2sample:\r\nTake one sample measurement to measure oxygenation rate after 6 seconds from placing the hand on the sensor.\r\n\r\n",
	CLI_SPO2_SampleCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [EMG]/[EEG]/[EOG]/[ECG] (Numofsamples ) (time in ms) [port] [module].\r\n\r\n",
	StreamEXGCommand,
	-1
};
/*-----------------------------------------------------------*/
/* CLI command structure : FingerState */
const CLI_Command_Definition_t CLI_FingerStateCommandDefinition =
{
	( const int8_t * ) "fingerstate", /* The command string to type. */
	( const int8_t * ) "fingerstate:\r\nFeel the presence of a finger on or near the sensor.\r\n\r\n",
	CLI_FingerStateCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/



/* -----------------------------------------------------------------------
 |						    	 Private Functions						 |
 -------------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
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
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
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
    RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}


/*-----------------------------------------------------------*/

/* --- Save Command Topology in Flash RO --- */

uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
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
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
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
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Save Command Snippets in Flash RO --- */

uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

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
	for(uint8_t index = 0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(snippets[index].cond.conditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[index],sizeof(snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
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
			for(uint8_t j = 0; j < ((strlen(snippets[index].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[index].cmd + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Clear array topology in SRAM and Flash RO --- */

uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H2BR1 module initialization.
 */
void Module_Peripheral_Init(void){

	 __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	MX_I2C2_Init();
	Init_MAX30100();
	 //Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}


	/* Create a timeout software timer StreamSamplsToPort() API */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);

}

/*-----------------------------------------------------------*/
/* --- H2BR1 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H2BR1_OK;
	 uint8_t fingerState =0;
	 uint8_t module=0;

	switch(code){
	case CODE_H2BR1_HR_Sample:
			{
				SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],HR);
				break;
			}
		case CODE_H2BR1_SPO2_Sample:
			{
				SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],SPO2);
				break;
			}
		case CODE_H2BR1_FingerState:
			{
				Module_Status status ;
		module = cMessage[port - 1][shift];
		port = cMessage[port - 1][1 + shift];
		status = FingerState(&fingerState);
		if (H2BR1_OK == status)
			messageParams[1] = BOS_OK;
		else
			messageParams[1] = BOS_ERROR;
		messageParams[0] = FMT_UINT8;
		messageParams[2] = 1;
		messageParams[3] = (uint8_t) fingerState;
		SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
		break;
	}
		default:
			    result =H2BR1_ERR_UNKNOWNMESSAGE;
			    break;
	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART6)
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

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_HR_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SPO2_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_FingerStateCommandDefinition);
}


/***************************************************************************/
/*
 * brief: Callback function triggered by a timer to manage data streaming.
 * param xTimerStream: Handle of the timer that triggered the callback.
 * retval: None
 */
void StreamTimeCallback(TimerHandle_t xTimerStream){
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if(STREAM_MODE_TO_PORT == StreamMode){
		if((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)){
			SampleToPort(PortModule,PortNumber,PortFunction);

		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if(STREAM_MODE_TO_TERMINAL == StreamMode){
		if((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)){
			SampleToTerminal(TerminalPort,TerminalFunction);
				}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |							 	Local  APIs			    		          | 																 	|
/* -----------------------------------------------------------------------
 * */
void MAX30100_Reset()
{
	uint8_t modeConfReg = 0;
	modeConfReg = modeConfReg & MAX30100_MODE_CONF_RESET_MASK;
	MAX30100_Write(MAX30100_MODE_CONF_ADDR, modeConfReg, 100);
	// delay until completing reset process
	HAL_Delay(1);
	MaxStruct.modeConfReg = modeConfReg;
}

/*-----------------------------------------------------------*/
void MAX30100_Enable_Interrupt(INTERRUPT_EN_A_FULL_BIT aFull, INTERRUPT_EN_TEMP_RDY_BIT tempRdy, INTERRUPT_EN_HR_RDY_BIT hrRdy, INTERRUPT_EN_SPO2_RDY_BIT Spo2Rdy)
{
	uint8_t interrEnReg = aFull | tempRdy | hrRdy | Spo2Rdy;
	MAX30100_Write(MAX30100_INTERRUPT_EN_ADDR , interrEnReg, 100);
	MaxStruct.interruptEnableReg = interrEnReg;

}

/*-----------------------------------------------------------*/
void MAX30100_Set_Mode(MAX30100_MODE mode)
{
	MaxStruct.mode = mode;
	uint8_t modeConfReg = 0;
	MAX30100_Read(MAX30100_MODE_CONF_ADDR, &modeConfReg, 1, 100);
    modeConfReg = (modeConfReg & ~MAX30100_MODE_CONF_MODE_MASK) | (mode << MAX30100_MODE_CONF_MODE_POSITION);
    if(mode == SPO2_MODE)
		//Enable Temp_En bit
		modeConfReg |= MAX30100_MODE_CONF_TEMP_EN_MASK;
    else
    	//Disable Temp_En bit
    	{modeConfReg &= ~MAX30100_MODE_CONF_TEMP_EN_MASK;}
	MAX30100_Write(MAX30100_MODE_CONF_ADDR, modeConfReg, 100);
	MaxStruct.modeConfReg = modeConfReg;
	if(mode == UNUSED_MODE)
		// disable all interrupt bits
	MAX30100_Enable_Interrupt(INTERRUPT_EN_AFULL_DISABLED, INTERRUPT_EN_TEMP_RDY_DISABLED, INTERRUPT_EN_HR_RDY_DISABLED, INTERRUPT_EN_SPO2_RDY_DISABLED);
	else
		// enable A_Full interrupt bit
	MAX30100_Enable_Interrupt(INTERRUPT_EN_AFULL_ENABLED, INTERRUPT_EN_TEMP_RDY_DISABLED, INTERRUPT_EN_HR_RDY_DISABLED, INTERRUPT_EN_SPO2_RDY_DISABLED);
}

/*-----------------------------------------------------------*/
void MAX30100_Set_SpO2_SampleRate(MAX30100_SpO2_SR sampleRate )
{
	MaxStruct.SPO2SampleRate= sampleRate;
	uint8_t SPO2ConfReg = 0;
	MAX30100_Read(MAX30100_SPO2_CONF_ADDR, &SPO2ConfReg, 1, 100);
	SPO2ConfReg = (SPO2ConfReg & ~MAX30100_SPO2_CONF_SR_MASK) | (sampleRate << MAX30100_SPO2_CONF_SR_POSITION);
	MAX30100_Write(MAX30100_SPO2_CONF_ADDR , SPO2ConfReg, 100);
	MaxStruct.SPO2ConfReg = SPO2ConfReg;
}

/*-----------------------------------------------------------*/
void MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW pulseWidth )
{
	MaxStruct.ledPulseWidth = pulseWidth;
	uint8_t SPO2ConfReg = 0;
	MAX30100_Read(MAX30100_SPO2_CONF_ADDR, &SPO2ConfReg, 1, 100);
	SPO2ConfReg = (SPO2ConfReg & ~MAX30100_SPO2_CONF_LED_PW_MASK) | ((pulseWidth << MAX30100_SPO2_CONF_LED_PW_POSITION) & MAX30100_SPO2_CONF_LED_PW_MASK);
	MAX30100_Write(MAX30100_SPO2_CONF_ADDR , SPO2ConfReg, 100);
	MaxStruct.SPO2ConfReg = SPO2ConfReg;
}

/*-----------------------------------------------------------*/
void MAX30100_Set_Led_Current(MAX30100_LED_Current redPa, MAX30100_LED_Current irPa )
{
	MaxStruct.redPa = redPa;
	uint8_t ledConfReg = 0;
	ledConfReg= (redPa << MAX30100_LED_CONF_RED_PA_POSITION) | (irPa << MAX30100_LED_CONF_IR_PA_POSITION);
	MAX30100_Write(MAX30100_LED_CONF_ADDR, ledConfReg, 100);
	MaxStruct.ledConfReg= ledConfReg;
}

/*-----------------------------------------------------------*/
void MAX30100_Read_FIFO()
{
	uint8_t fifoData[MAX30100_FIFO_DATA_SIZE] = {0};
	// timeout =1000msec; reading 16 samples needs from 15-300msec
	MAX30100_Read(MAX30100D_FIFO_DATA_ADDR, fifoData, MAX30100_FIFO_DATA_SIZE, 1000);
	uint8_t i=0, j=0;
	for (i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
	{
		// First read byte is MSB. Size of sample is 2 bytes
		MaxStruct.irSamples[i]  = (fifoData[j] << 8) | fifoData[j+1];
		MaxStruct.redSamples[i]= (fifoData[j+2] << 8) | fifoData[j+3];
		j+=4;
	}
 }

/*-----------------------------------------------------------*/
void MAX30100_Clear_FIFO(void)
{
	MAX30100_Write(MAX30100_FIFO_W_PTR_ADDR, 0x00, 100);
	MAX30100_Write(MAX30100_OVF_COUNTER_ADDR, 0x00, 100);
	MAX30100_Write(MAX30100_FIFO_R_PTR_ADDR, 0x00, 100);
}

/*-----------------------------------------------------------*/
void Oxymeter_Add_Samples_To_Buffers()
{
	uint16_t buffIndex = MaxStruct.bufferIndex; // last value of bufferIndex
	if (buffIndex >= OXYMETER_MAX_BUFFER_SIZE)
		buffIndex = 0;
	for(uint8_t i = 0; i< MAX30100_FIFO_SAMPLES_SIZE; i++)
	{
		if (buffIndex < OXYMETER_MAX_BUFFER_SIZE)
		{
			MaxStruct.irRawBuffer[buffIndex] = MaxStruct.irSamples[i];
			MaxStruct.redRawBuffer [buffIndex] = MaxStruct.redSamples[i];
			buffIndex++;
		}
	}
	MaxStruct.bufferIndex = buffIndex;
}

/*-----------------------------------------------------------*/
void Oxymeter_Modify_Led_Current_Bias()
{
	if(MaxStruct.mode == SPO2_MODE)
	{
		uint8_t redLedCurrentIndex = (uint8_t) (MaxStruct.redPa);
		uint32_t tLastModified = MaxStruct.timeLastBiasModified;
		uint8_t needModifying = 0;
		if (HAL_GetTick() - tLastModified > CURRENT_MODIFYING_DURATION_MS)
		{
			if (MaxStruct.irSamples[0] - MaxStruct.redSamples[0] >  CURRENT_MODIFYING_THRESHOLD  && redLedCurrentIndex < MAX30100_LED_CURRENT_50P0)
			{
				++redLedCurrentIndex;
				needModifying = 1;
			}

			else if (MaxStruct.redSamples[0] - MaxStruct.irSamples[0] >  CURRENT_MODIFYING_THRESHOLD  && redLedCurrentIndex > MAX30100_LED_CURRENT_0P0)
			{
				--redLedCurrentIndex;
				needModifying = 1;
			}
			else MaxStruct.currentBiasState = CURRENT_BIAS_STATE_OK;

			if (needModifying == 1)
			{
				MaxStruct.currentBiasState = CURRENT_BIAS_STATE_NOT_OK;
				MAX30100_Set_Led_Current(redLedCurrentIndex, MAX30100_LED_CURRENT_50P0);
				MaxStruct.timeLastBiasModified = HAL_GetTick();
			}
		}
	}
}

/*-----------------------------------------------------------*/
void Oxymeter_Detect_Finger()
{
	MaxStruct.fingerState = FINGER_STATE_NOT_DETECTED;
	for (uint8_t i=0; i<MAX30100_FIFO_SAMPLES_SIZE; i++)
		{
			if (MaxStruct.irSamples[i] > FINGER_DETECTING_THRESHOLD)
				MaxStruct.fingerState = FINGER_STATE_DETECTED;
			else
			{
				MaxStruct.fingerState = FINGER_STATE_NOT_DETECTED;
				MaxStruct.bufferIndex = 0;
				MaxStruct.heartRate = 0;
				MaxStruct.SPO2 =0;
				break;
			}
		}
}

/*-----------------------------------------------------------*/
void Oxymeter_Signal_Processing()
{
	MaxStruct. processStartTick = HAL_GetTick();
	float ts= 0.009407; // should be 0.01 but period of sample from IC not accurate (0.009407); fs=100
	/************Removing DC (HPF)*************/
	float irRemovedDC [OXYMETER_MAX_BUFFER_SIZE]={0};
	float redRemovedDC [OXYMETER_MAX_BUFFER_SIZE]={0};
	// HPF: 1st order, Fc=0.8Hz
	uint16_t previousInput=0;
	uint16_t currentInput=0;
	// HPF: 1st order, Fc=0.8Hz, fs=100
	for(uint16_t i = 1; i< OXYMETER_MAX_BUFFER_SIZE; i++)
	{
		previousInput = MaxStruct.irRawBuffer[i-1];
		currentInput = MaxStruct.irRawBuffer[i];
		irRemovedDC[i] = 0.9755 * (float)currentInput - 0.9755 * (float)previousInput + 0.9510 * irRemovedDC[i-1];
	}
	// HPF: 1st order, Fc=0.8Hz, fs=100
	for(uint16_t i = 1; i< OXYMETER_MAX_BUFFER_SIZE; i++)
	{
		previousInput = MaxStruct.redRawBuffer[i-1];
		currentInput = MaxStruct.redRawBuffer[i];
		redRemovedDC[i] = 0.9755 * (float)currentInput - 0.9755 * (float)previousInput + 0.9510 * redRemovedDC[i-1];
	}
	/************LPF Filtering************/
	float irFiltered [OXYMETER_MAX_BUFFER_SIZE]={0};
	// HPF: 2nd order, Fc=2Hz, fs=100
	for(uint16_t i = 2; i< OXYMETER_MAX_BUFFER_SIZE; i++)
		irFiltered[i] = 0.0036 * irRemovedDC[i] + 0.0072 * irRemovedDC[i-1] + 0.0036 * irRemovedDC[i-2] + 1.8227 * irFiltered[i-1] - 0.8372 * irFiltered[i-2];
	/************Detecting Peaks***********/
	uint8_t numOfPeaks=0;
	uint16_t peakIndex[3]={0};
	for(uint16_t i = (FILTERING_SAMPLES_SHIFT + WINDOW_FOR_DETECING_PEAK); i< (OXYMETER_MAX_BUFFER_SIZE - WINDOW_FOR_DETECING_PEAK); i++)
	{
		if (irFiltered[i] < irFiltered[i-1] && irFiltered[i] < irFiltered[i+1]) // min peak
		{
			if (irFiltered[i] < irFiltered[i - WINDOW_FOR_DETECING_PEAK] && irFiltered[i] < irFiltered[i + WINDOW_FOR_DETECING_PEAK])
			{
				numOfPeaks++;
				peakIndex[numOfPeaks - 1] = i;
				if (numOfPeaks == 3)
					break;
			}
		}
	}
	MaxStruct.numOfPeaks = numOfPeaks;
	/*****Calculating Heart Rate (HR)******/
	uint16_t  peakInd1=0,  peakInd2=0,  peakInd3=0;
	float durP1_P2=0, durP2_P3=0, samplesP1_P3=0;
	float period=0;
	float HR=0;
	if (numOfPeaks == 3)
	{
		 peakInd1 = peakIndex[0];
		 peakInd2 = peakIndex[1];
		 peakInd3 = peakIndex[2];
		 durP1_P2 = (peakInd2 - peakInd1) * ts;
		 durP2_P3 = (peakInd3 - peakInd2) * ts;
		 samplesP1_P3 = peakInd3 - peakInd1;
		 MaxStruct.durationBetweenPeak1Peak2 = durP1_P2;
		 MaxStruct.durationBetweenPeak2Peak3 = durP2_P3;
		 MaxStruct.samplesBetweenPeak1Peak3 = samplesP1_P3;
		 if (durP1_P2 > MIN_HEART_PERIOD_SEC && durP1_P2 < MAX_HEART_PERIOD_SEC)
		 {
			 if (durP1_P2 <= 1.2 * durP2_P3 && durP1_P2 >= 0.8 * durP2_P3)
			 {
				 period = (durP1_P2 + durP2_P3) / 2.0 ; // period = mean of (durP1_P2, durP2_P3)
				 HR = 60.0 / period; // HR=60/T
				 MaxStruct.heartRate = roundf(HR);
			 }
		 }
		 else MaxStruct.heartRate = 0;
	}
	else MaxStruct.heartRate = 0;
	/**********Calculating SPO2***********/
	// Calibration array // SaO2 Look-up Table(http://www.ti.com/lit/an/slaa274b/slaa274b.pdf)
	uint8_t spO2LUT [43]={100,100,100,100,99,99,99,99,99,99,98,98,98,98,98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,95,95,95,95,94,94,94,94,94,93,93,93,93,93};
	float irACSqSum=0, redACSqSum=0;
	uint8_t SPO2Iindex=0, ratio=0, SPO2Value;
	if(MaxStruct.mode == SPO2_MODE)
	{
	    if (HR !=0 )
	    {
	        for (uint16_t i=peakInd1; i<=peakInd3; i++)
	        {
	        	irACSqSum = irACSqSum + (irRemovedDC[i] * irRemovedDC[i]);
	        	redACSqSum = redACSqSum + (redRemovedDC[i] * redRemovedDC[i]);
	        }
	        ratio = roundf(100.0 * log(redACSqSum/samplesP1_P3) / log(irACSqSum/samplesP1_P3));
	        if(ratio > 66)
	        	SPO2Iindex = ratio - 66;
	        else if (ratio > 50)
	        	SPO2Iindex = ratio - 50;
	        SPO2Value = spO2LUT[SPO2Iindex];
	        if(SPO2Value >= 93 && SPO2Value <= 100)
	        	MaxStruct.SPO2 = SPO2Value;
	        else
	        	MaxStruct.SPO2 = 0;
	    }
	    else MaxStruct.SPO2 = 0;
	}

    MaxStruct.processEndTick = HAL_GetTick();
    MaxStruct.processTimeMs = MaxStruct.processEndTick - MaxStruct.processStartTick;
}

/*-----------------------------------------------------------*/
// This function should put within external interrupt function
void Read_Data_When_Interrupt()
{
	uint8_t interruptReg = 0;
	MAX30100_Read(MAX30100_INTERRUPT_ADDR, &interruptReg, 1, 100);
	// if Samples FIFO Buffer is full (A_Full==1)
	if ( (interruptReg & MAX30100_INTERRUPT_A_FULL_MASK) >> MAX30100_INTERRUPT_A_FULL_POSITION )
	{
		MAX30100_Read_FIFO(MaxStruct);
		MaxStruct.dataReadingFlag1 = 1;
		MaxStruct.dataReadingFlag2 = 1;
	}
}

/*-----------------------------------------------------------*/
void Oxymeter_Calculating_HR_SPO2()
{
	Oxymeter_Detect_Finger(MaxStruct);
	if (MaxStruct.fingerState == FINGER_STATE_DETECTED)
	{
		Oxymeter_Add_Samples_To_Buffers(MaxStruct);
		Oxymeter_Modify_Led_Current_Bias(MaxStruct);
	}
	if (MaxStruct.bufferIndex == OXYMETER_MAX_BUFFER_SIZE)  // buffer is full
			Oxymeter_Signal_Processing(MaxStruct);
}

/* -----------------------------------------------------------------------
 |								  APIs							          |
/* -----------------------------------------------------------------------
 */
/*
 * @brief: initialize MAX30100
 * @retval: status
 */
Module_Status Init_MAX30100(void)
{
	uint8_t status = H2BR1_OK;
	uint8_t mode = SPO2_MODE;

		MAX30100_Set_Led_Current(MAX30100_LED_CURRENT_33P8, MAX30100_LED_CURRENT_50P0); // primary values of RedLed 33.8 mA and IrLed 50mA so that received light intensity (from Red and Ir) is nearly adjacent but it is different from finger to other..
		MAX30100_Set_Led_PulseWidth(MAX30100_LED_PW_1600);
		MAX30100_Set_SpO2_SampleRate(MAX30100_SPO2_SR_100);
		MAX30100_Set_Mode(mode);
		uint8_t interruptReg = 0;
		if (MAX30100_Read(MAX30100_INTERRUPT_ADDR, &interruptReg, 1, 100) == H2BR1_OK) // InterruptReg should be read firstly so that interrupt pin goes 'high'. When max30100 is booted interrupt pin is 'low'
			status = H2BR1_OK;
	    else
		    status = H2BR1_ERR_WRONGPARAMS;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief  Streams heart rate (HR) or oxygen saturation (SPO2) samples to the terminal.
 * @param  dstPort: The port number used for data transmission.
 * @param  mode: The mode of operation (HR_MODE for heart rate, SPO2_MODE for oxygenation).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToTerminal(uint8_t dstPort, MAX30100_MODE mode)
{
    Module_Status status = H2BR1_OK;   /* Initialize operation status as success */
    char formattedString[20];          /* Buffer for formatted output string */

    /* Validate the port number */
    if (dstPort == 0)
    {
        return H2BR1_ERR_WRONGPARAMS;  /* Return error for invalid port */
    }

    /* Process data based on the selected mode */
    switch (mode)
    {
        case HR_MODE:
            /* Check if new IR data is available */
            if (MaxStruct.dataReadingFlag1 == 1)
            {
                MaxStruct.dataReadingFlag1 = 0;

                /* Loop through IR samples and send them to the terminal */
                for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
                {
                    snprintf(formattedString, sizeof(formattedString), "i%d\r\n", MaxStruct.irSamples[i]);
                    HAL_UART_Transmit(GetUart(dstPort), (uint8_t*)formattedString, strlen(formattedString), 100);
                }
            }
            break;

        case SPO2_MODE:
            /* Check if new IR and RED data is available */
            if (MaxStruct.dataReadingFlag1 == 1)
            {
                MaxStruct.dataReadingFlag1 = 0;

                /* Loop through IR and RED samples and send them to the terminal */
                for (uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
                {
                    snprintf(formattedString, sizeof(formattedString), "i%dr%d\r\n", MaxStruct.irSamples[i], MaxStruct.redSamples[i]);
                    HAL_UART_Transmit(GetUart(dstPort), (uint8_t*)formattedString, strlen(formattedString), 100);
                }
            }
            break;

        default:
            status = H2BR1_ERR_WRONGPARAMS; /* Return error for invalid mode */
            break;
    }

    /* Return final status indicating success or prior error */
    return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reads infrared samples from MaxStruct and stores them in the provided buffer.
 * (Note: You can use them specifically to draw the signal).
 * @param1: pointer to an array to store the infrared samples (16 samples).
 * @retval: status
 */
Module_Status HR_ReadBuffer(uint16_t *irSampleBuffer)
{
	uint8_t status = H2BR1_OK;
	for(uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
		irSampleBuffer[i] = MaxStruct.irSamples[i];
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: reads red and infrared samples from MaxStruct and stores them in the provided buffers.
 * (Note: You can use them specifically to draw the signal).
 * @param1: pointer to an array to store red samples (16 samples).
 * @param2: pointer to an array to store infrared samples (16 samples).
 * @retval: status
 */
Module_Status SPO2_ReadBuffer(uint16_t *redSampleBuffer, uint16_t *irSampleBuffer)
{
	uint8_t status = H2BR1_OK;
	for(uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
	{
		redSampleBuffer[i] = MaxStruct.redSamples[i];
		irSampleBuffer[i] = MaxStruct.irSamples[i];
	}
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Sample Read Flag for IC MAX30100
 * @param1: pointer to a buffer to store value it always gives 1
 * @retval: status
 */
Module_Status SampleReadFlag(uint8_t *sampleReadFlag)
{
	uint8_t status = H2BR1_OK;
	*sampleReadFlag = MaxStruct.dataReadingFlag2;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Reset Sample Read Flag for IC MAX30100
 * @retval: status
 */
Module_Status ResetSampleReadFlag()
{
	uint8_t status = H2BR1_OK;
	MaxStruct.dataReadingFlag2 = 0;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: read the presence of a finger on or near the sensor.
 * @param1: pointer to a buffer to store value.
 * @retval: status
 */
Module_Status FingerState(FINGER_STATE *fingerState)
{
	uint8_t status = H2BR1_OK;
	*fingerState = MaxStruct.fingerState;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: read heart rate after 6 seconds from placing the hand on the sensor.
 * @param1: pointer to a buffer to store value
 * @retval: status
 */
Module_Status HR_Sample(uint8_t *heartRate)
{
	uint8_t status = H2BR1_OK;
	*heartRate = MaxStruct.heartRate;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: read oxygenation rate after 6 seconds from placing the hand on the sensor.
 * @param1: pointer to a buffer to store value
 * @retval: status
 */
Module_Status SPO2_Sample(uint8_t *SPO2)
{
	uint8_t status = H2BR1_OK;
	*SPO2 = MaxStruct.SPO2;
	return status;
}
/*-----------------------------------------------------------*/
/*
 * @brief  Samples data from a heart rate (HR) or oxygen saturation (SPO2) sensor
 *         and exports it to a specified port or module.
 * @param  dstModule: The module number to export data to.
 * @param  dstPort: The port number to export data to.
 * @param  sensorType: The type of sensor (HR for heart rate, SPO2 for oxygen saturation).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, Sensor dataFunction)
{
    uint8_t hrValue = 0;        /* Variable to store heart rate sample */
    uint8_t spo2Value = 0;      /* Variable to store oxygen saturation sample */
    Module_Status status = H2BR1_OK; /* Initialize operation status as success */

    /* Validate the port and module ID */
    if (dstPort == 0 && dstModule == myID)
    {
        return H2BR1_ERR_WRONGPARAMS; /* Return error for invalid parameters */
    }

    /* Process data based on the requested sensor type */
    switch (dataFunction)
    {
        case HR:
        {
            /* Sample heart rate data */
            status = HR_Sample(&hrValue);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                writePxITMutex(dstPort, (char*)&hrValue, sizeof(uint8_t), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H2BR1_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_UINT8;
                messageParams[2] = 1;
                messageParams[3] = hrValue;

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
            }
            break;
        }

        case SPO2:
        {
            /* Sample oxygen saturation data */
            status = SPO2_Sample(&spo2Value);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                writePxITMutex(dstPort, (char*)&spo2Value, sizeof(uint8_t), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H2BR1_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_UINT8;
                messageParams[2] = 1;
                messageParams[3] = spo2Value;

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
            }
            break;
        }

        default:
            status = H2BR1_ERR_WRONGPARAMS; /* Return error for invalid sensor type */
            break;
    }

    /* Return final status indicating success or prior error */
    return status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,Sensor dataFunction,uint32_t numOfSamples,uint32_t streamTimeout)
 {
	Module_Status Status = H2BR1_OK;

	uint32_t SamplePeriod = 0u;

	/* Check timer handle and timeout validity */
	if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)) {
		return H2BR1_ERROR; /* Assuming H2BR1_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortFunction =dataFunction;
	PortNumOfSamples =numOfSamples;
	/* Calculate the period from timeout and number of samples */
	SamplePeriod = streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if (xTimerIsTimerActive(xTimerStream)) {
		if (pdFAIL == xTimerStop(xTimerStream, 100)) {
			return H2BR1_ERROR;
		}
	}

	/* Start the stream timer */
	if (pdFAIL == xTimerStart(xTimerStream, 100)) {
		return H2BR1_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100)) {
		return H2BR1_ERROR;
	}

	return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort,MAX30100_MODE dataFunction,uint32_t numOfSamples,uint32_t streamTimeout)
{
	Module_Status Status =H2BR1_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H2BR1_ERROR; /* Assuming H2BR1_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
		StreamMode = STREAM_MODE_TO_TERMINAL;
		TerminalPort =dstPort;
		TerminalFunction =dataFunction;
		TerminalTimeout =streamTimeout;
		TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H2BR1_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H2BR1_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H2BR1_ERROR;
	}

	return Status;
}

/*-----------------------------------------------------------*/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=1 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				flag=1;
				return H2BR1_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H2BR1_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H2BR1_OK;
}
/*-----------------------------------------------------------*/
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H2BR1_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H2BR1_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' ) {
				UARTRxBuf[PcPort - 1][chr] = 0;
			}
		}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(PcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,Numofsamples) != H2BR1_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}

void SampleHRToString(char *cstring, size_t maxLen) {
	uint8_t heartRate;
	HR_Sample(&heartRate);
	snprintf(cstring, maxLen, "HeartRate: %d \r\n", heartRate);
}
/*-----------------------------------------------------------*/
void SampleSPO2ToString(char *cstring, size_t maxLen) {
	uint8_t SPO2;
	SPO2_Sample(&SPO2);
	snprintf(cstring, maxLen, "SPO2: %d \r\n", SPO2);
}
/*-----------------------------------------------------------*/
Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout,
		Sensor Sensor) {

	switch (Sensor) {
	case HR:
		StreamMemsToCLI(Numofsamples, timeout, SampleHRToString);
		break;
	case SPO2:
		StreamMemsToCLI(Numofsamples, timeout, SampleSPO2ToString);
		break;
	default:
		break;
	}

}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE StreamEXGCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const HRCmdName = "hr";
	const char *const SPO2CmdName = "spo2";


	uint32_t Numofsamples = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, HRCmdName, strlen(HRCmdName))) {
			if (portOrCLI) {

				StreamToCLI(Numofsamples, timeout,HR);

			} else {

				StreamToPort(module, port, HR, Numofsamples, timeout);

			}

		} else if (!strncmp(pSensName, SPO2CmdName, strlen(SPO2CmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SPO2);

			} else {

				StreamToPort(module, port, SPO2, Numofsamples, timeout);
			}



		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen,
					"Invalid Arguments\r\n");
		}

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule)
{
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

portBASE_TYPE CLI_HR_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR1_OK;
	uint8_t heartRate=0;
	static const int8_t *pcOKMessage=(int8_t* )"Heart Rate is : %d bpm\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=HR_Sample(&heartRate);

	 if(status == H2BR1_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,heartRate);

	 }

	 else if(status == H2BR1_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_SPO2_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR1_OK;
	uint8_t oxygenationRate=0;
	static const int8_t *pcOKMessage=(int8_t* )"Oxygenation Rate(SPO2) is : %d %%\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=SPO2_Sample(&oxygenationRate);

	 if(status == H2BR1_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,oxygenationRate);

	 }

	 else if(status == H2BR1_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_FingerStateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H2BR1_OK;
	FINGER_STATE fingerState =0;
	static const int8_t *pcOKMessage=(int8_t* )"FingerState is : %d \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=FingerState(&fingerState);

	 if(status == H2BR1_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,fingerState);

	 }

	 else if(status == H2BR1_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
