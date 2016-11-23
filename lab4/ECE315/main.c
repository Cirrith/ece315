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
#include "encoders.h"
#include "ece315_lab3.h"

//*****************************************************************************
// Global Variables
//*****************************************************************************
int secCount = 0;
char t [25];
//*****************************************************************************
//*****************************************************************************


void initializeBoard(void)
{
  DisableInterrupts();
  serialDebugInit();
	// Lab 1
	sensor_Init();
	SysTick_Config(2500);
	
	// Lab 2
	drv8833_gpioInit();
	
	// Lab3 Stuff
	encodersInit();
	rfInit();
  EnableInterrupts();
}

void getMessage(uint32_t* data) {
	char msg[80];
	wireless_com_status_t status;
	char interrupts[80];
	status = wireless_get_32(false, data);
	if(status == NRF24L01_RX_SUCCESS)	{
		memset (msg,0,80);
		sprintf(msg,"Data RXed: %c%c %d\n\r", *data>>24, *data>>16, *data & 0xFFFF);
		uartTxPoll(UART0_BASE, msg);
	}
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
  
	while(1){
		//if(UART7Entered){
			//  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
			//	UART7Entered = false;
	//	}
		
		/*if(sysTick){
			GPIOF->DATA ^= PF1;
			//UART calculations
			if(uartRxPoll(UART7_BASE, 0) == 'R') {
				uartVal[0] = uartRxPoll(UART7_BASE, 1);
				uartVal[1] = uartRxPoll(UART7_BASE, 1);
				uartVal[2] = uartRxPoll(UART7_BASE, 1);
			}
			uartDistance = (((uartVal[0]-48)* 100) + ((uartVal[1]-48)*10) + (uartVal[2]-48));
		}*/
		if(secTick){
			sprintf(t, "Left = %d\n\r", uartDistance);
		  uartTxPoll(UART0_BASE, t);
			secTick = false;
		}
	}
}
