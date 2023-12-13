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

// #include "a31t21x.h"
#include <stdint.h>
#define SCU_PER1 ((volatile uint32_t*) 0x40000028)
#define SCU_PCER1 ((volatile uint32_t*) 0x40000030)
#define PC_MOD	((volatile uint32_t*) 0x40001200)

void SystemInit(void)
{
	/* Disable Global Interrupt */
	// __disable_irq();
//   __ASM volatile ("cpsid i" : : : "memory");

	/* WDT Disable */
  *((uint32_t*)0x40001A00) = (0x5A69<<16)
		|(0x25<<10)
		|(0x1A<<4);

	/* GPIO Access Enable */
	// PORTEN->EN = 0x15;
	// PORTEN->EN = 0x51;
	*((uint32_t*)0x40001FF0) = 0x15;
	*((uint32_t*)0x40001FF0) = 0x51;
	
}

int main(void)
{
	SystemInit();
	/* Peripheral Enable(0:Disable, 1:Enable) */
	*(SCU_PER1) = (1<<10)	// GPIOC
			;
	/* Peripheral Clock Enable(0:Disable, 1:Enable) */
	*(SCU_PCER1) = (1<<10)	// GPIOC
	;
	*(PC_MOD) = (1 << 0) | (1 << 2);		

	while(1) 
		{
			*((uint32_t*)0x40001220) = (1 << 0) | (1 << 1);
			for(int i=0; i<20000; i++); 
			*((uint32_t*)0x4000121C) = (1 << 0) | (1 << 1);
			for(int i=0; i<20000; i++); 
		}
	return 0;
}

