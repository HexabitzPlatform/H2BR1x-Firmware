/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
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

/* Exported functions */

//uint32_t lastTick;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
#define MIN_PERIOD_MS				100
/* Private variables ---------------------------------------------------------*/

MAX30100_s MaxStruct;

/* Private function prototypes -----------------------------------------------*/
Module_Status Init_MAX30100(void);

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

//These two functions should be put in external interrupt service routine
void Read_Data_When_Interrupt();
void Oxymeter_Calculating_HR_SPO2();


/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_HR_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SPO2_SampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_FingerStateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

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
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
//	__HAL_RCC_TIM1_CLK_ENABLE();
//	__HAL_RCC_TIM2_CLK_ENABLE();

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE();

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
		}
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
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
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		}
	}

	/* Create module special task (if needed) */
//	if(EXGTaskHandle == NULL)
//		xTaskCreate(EXGTask,(const char* ) "EXGTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&EXGTaskHandle);

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
				SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],HR);
				break;
			}
		case CODE_H2BR1_SPO2_Sample:
			{
				SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],SPO2);
				break;
			}
		case CODE_H2BR1_FingerState:
			{
				module = cMessage[port-1][shift];
				port = cMessage[port-1][1+shift];
				FingerState(&fingerState);
				messageParams[0] =port;
				messageParams[1] =(uint8_t)fingerState;
				SendMessageToModule(module,CODE_PORT_FORWARD,2);
				break;
			}
		default:
			    result =H2BR1_ERR_UnknownMessage;
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
	FreeRTOS_CLIRegisterCommand(&CLI_HR_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_SPO2_SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_FingerStateCommandDefinition);
}

/*-----------------------------------------------------------*/

///* Module special task function (if needed) */
//void EXGTask(void *argument) {
//
//	uint8_t cases; // Test variable.
//
//	/* Infinite loop */
//	for (;;) {
//		/*  */
//
//		switch (cases) {
//
//
//
//	default:
//		osDelay(10);
//		break;
//		}
//
//		taskYIELD();
//	}
//
//}


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
		    status = H2BR1_ERR_WrongParams;
	return status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: Send (irSamples) or (irSamples+redSamples) to display on Terminal or draw
 * signals for heart rate and oxygenation.
 * @param1: The port you want to send from
 * @param2: The mode to be selected (HR or SPO2)
 * @retval: status
 */
Module_Status PlotToTerminal(uint8_t port, MAX30100_MODE mode)
{
	uint8_t status = H2BR1_OK;
	char sendData[20];
	if(port == 0)
	return H2BR1_ERR_WrongParams;

	if(mode == HR_MODE)
	{
		if(MaxStruct.dataReadingFlag1 == 1)
		{
			MaxStruct.dataReadingFlag1 = 0;
				for(uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
				{
					sprintf(sendData, "i%d\r\n", MaxStruct.irSamples[i]);
					HAL_UART_Transmit(GetUart(port),(uint8_t *)sendData, strlen(sendData), 100);
				}
		}
	}
	else if (mode == SPO2_MODE)
	{
		if(MaxStruct.dataReadingFlag1 == 1)
		{
			MaxStruct.dataReadingFlag1 = 0;
				for(uint8_t i = 0; i < MAX30100_FIFO_SAMPLES_SIZE; i++)
				{
					sprintf(sendData, "i%dr%d\r\n", MaxStruct.irSamples[i], MaxStruct.redSamples[i]);
					HAL_UART_Transmit(GetUart(port),(uint8_t *)sendData, strlen(sendData), 100);
				}
		}
	}
	else
		status = H2BR1_ERR_WrongParams;
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
 * @brief: send a sample on the required port or send it to another module and
 * graduate the value on the required port.
 * @brief: if the topology file is not activated, therefore The module number is 0
 * @param1: destination module.
 * @param2: port number.
 * @param3: hr or spo2 sensor.
 * @retval: status
 */
Module_Status SampletoPort(uint8_t module,uint8_t port, Sensor Sensor)
{
	uint8_t HRValue = 0 ;
	uint8_t SPO2Value = 0;
	uint8_t status =H2BR1_OK;

	if(port == 0)
	return H2BR1_ERR_WrongParams;

	switch (Sensor)
	{
	case HR:
		status = HR_Sample(&HRValue);

		if (module == myID)
		{
			writePxITMutex(port,(char* )&HRValue,sizeof(uint8_t),10);
		}
		else
		{
			messageParams[0] =port;
			messageParams[1] =(uint8_t)HRValue;
			SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(uint8_t)+1);
		}
		break;
	case SPO2:
		status = SPO2_Sample(&SPO2Value);

		if (module == myID)
		{
			writePxITMutex(port,(char* )&SPO2Value,sizeof(uint8_t),10);
		}
		else
		{
			messageParams[0] =port;
			messageParams[1] =(uint8_t)SPO2Value;
			SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(uint8_t)+1);
		}
		break;

	default:
			status=H2BR1_ERR_WrongParams;
	    break;

	}
	return status;
}
/*-----------------------------------------------------------*/
/*
 * @brief: send a Stream  on the required port or send it to another module and graduate
 * the value on the required port.
 * @brief: if the topology file is not activated, therefore The module number is 0
 * @param1: destination module.
 * @param2: port number.
 * @param3: hr or spo2 sensor.
 * @param4: number of samples to be send.
 * @param5: timeout (Note: the time required to send a single sample is 6000 milliseconds).
 * @retval: status
 */
Module_Status StreamtoPort(uint8_t module,uint8_t port,Sensor Sensor,uint32_t Numofsamples,uint32_t timeout)
{
	uint8_t status =H2BR1_OK;
	uint32_t samples=0;
	uint32_t period=0;
	period=timeout/Numofsamples;

	if (timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
		return H2BR1_ERR_WrongParams;

	while(samples < Numofsamples)
	{
	status=SampletoPort(module,port,Sensor);
	vTaskDelay(pdMS_TO_TICKS(period));
	samples++;
	}
	samples=0;
	return status;

}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */

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
