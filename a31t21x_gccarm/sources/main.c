/**
 *******************************************************************************
 * @file        main.c
 * @author      ABOV R&D Division
 * @brief       Application main
 *
 * Copyright 2022 ABOV Semiconductor Co.,Ltd. All rights reserved.
 *
 * This file is licensed under terms that are found in the LICENSE file
 * located at Document directory.
 * If this file is delivered or shared without applicable license terms,
 * the terms of the BSD-3-Clause license shall be applied.
 * Reference: https://opensource.org/licenses/BSD-3-Clause
 ******************************************************************************/

#include "a31t21x.h"

void SystemInit(void)
{
	/* Disable Global Interrupt */
	__disable_irq();

	/* WDT Disable */
	WDT->CR = (0x5A69<<16)
		|(0x25<<10)
		|(0x1A<<4);

	/* GPIO Access Enable */
	PORTEN->EN = 0x15;
	PORTEN->EN = 0x51;

	/* Flash Access Time Configure */
	FMC->MR = 0x81;
	FMC->MR = 0x28;
	FMC->CFG = (0x7858 << 16) | (3 << 8);		// Flash Access in 4 cycles (3-wait)
	FMC->MR = 0;
}

int main(void)
{
	SystemInit();
	/* Peripheral Enable(0:Disable, 1:Enable) */
	SCU->PER1 = SCU->PER1
			| (1<<10)	// GPIOC
			;
	/* Peripheral Clock Enable(0:Disable, 1:Enable) */
	SCU->PCER1 = SCU->PCER1
			| (1<<10)	// GPIOC
			;
	PC->MOD = (1 << 0) | (1 << 2);

	while(1) 
		{
			PC->BCR = (1 << 0) | (1 << 1);
			for(int i=0; i<5000; i++); 
			PC->BSR = (1 << 0) | (1 << 1);
			for(int i=0; i<5000; i++); 
		}
	return 0;
}

