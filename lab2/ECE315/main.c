// Copyright (c) 2014, Joe Krachey
// All rights reserved.
//
// Redistribution and use in binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//*****************************************************************************
// main.c
// Author: jkrachey@wisc.edu
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#include "TM4C123.h"
#include "boardUtil.h"
#include "drv8833.h"
#include "interrupts.h"

//*****************************************************************************
// Global Variables
//*****************************************************************************
int secCount = 0;
  
//*****************************************************************************
//*****************************************************************************


void initializeBoard(void)
{
  DisableInterrupts();
  serialDebugInit();
	  drv8833_gpioInit();
  EnableInterrupts();
}


//*****************************************************************************
//*****************************************************************************
int 
main(void)
{
  initializeBoard();
	
	SysTick_Config(2500);
	
  uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  
	//drv8833_rightForward(30);
	GPIOF->DATA |= PF3;
  // Infinite Loop
  while(1)
  {
		if(secTick){
			secCount++;
			secTick = false;
		}
		if(secCount < 2) {
			drv8833_leftForward(90);
			drv8833_rightForward(90);
		} else if (secCount < 4) {
			drv8833_leftReverse(90);
			drv8833_rightReverse(90);
		} else if (secCount < 9) {
			drv8833_turnLeft(75);
		} else if (secCount < 14) {
			drv8833_turnRight(75);
		}
		else {
			GPIOF->DATA &= ~PF3;
		}
	}
}
