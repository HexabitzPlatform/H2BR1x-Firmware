/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void) {

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for (;;) {
	}
}
uint8_t spo ,ss[50] ,k;
FINGER_STATE d ;
/***************************************************************************/
/* User Task */
void UserTask(void *argument) {

	/* put your code here, to run repeatedly. */
	while (1) {
		SPO2_Sample(&spo);
		FingerState(&d);
		SampleReadFlag(ss);
		HR_Sample(&k);
	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
