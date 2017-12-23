/***************************************************************************
 * STM32 VGA demo
 * Copyright (C) 2012 Artekit Italy
 * http://www.artekit.eu
 * Written by Ruben H. Meleca
 
### sys.c
 
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

***************************************************************************/

#include "stm32f10x.h"
#include "sys.h"

volatile uint32_t	sysTiming;
volatile uint32_t	sysTicks = 0;
volatile uint32_t	sysDel1Ms;

//*****************************************************************************
//	Function sysInitSystemTimer(void)
//
//	Initialize system timer
//
//
//	return			0=Fail, 1=OK
//*****************************************************************************
uint8_t sysInitSystemTimer(void) {

	if (SysTick_Config((SystemCoreClock / 1000) -1 )) {
		return(0);
	}
	return(1);
}

//*****************************************************************************
//	Function sysDelay1Ms(void)
//
//	Performs delay 1 milliseconds
//
//	return			None
//*****************************************************************************
void sysDelay1Ms(void) {

	for (sysDel1Ms = 0; sysDel1Ms < 1000; sysDel1Ms++) { };
}


volatile uint16_t		*__timer = (uint16_t*) 0x40006c04;

//*****************************************************************************
//	Function sysDelayMs(u32 dly)
//
//	Performs delay "dly" milliseconds
//
//	parameters		dly		Delay value in milliseconds
//
//	return			None
//*****************************************************************************
void sysDelayMs(u32 dly) {

	sysTiming = dly;

	while (sysTiming > 0) { };

}

//*****************************************************************************
//	Function sysDelayMs(u32 dly)
//
//	Performs delay "dly" milliseconds
//
//	parameters		dly		Delay value in milliseconds
//
//	return			None
//*****************************************************************************

void sysTickCount(void) {

	sysTicks++;
	if (sysTiming > 0) --sysTiming;
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void) {

	sysTicks++;
	if (sysTiming > 0) --sysTiming;
}
