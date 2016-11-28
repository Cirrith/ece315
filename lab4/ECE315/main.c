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
#include "lcd.h"

//*****************************************************************************
// Global Variables
//*****************************************************************************
int secCount = 0;
char t [25];
char dist [25];
int direction = 0;  // 0 = FWD, 1 = REV, 2 = TURN
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
	ece315_lcdInit();
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
	uint32_t data;
	bool motorDisabled = false;
	
  initializeBoard();
	
	SysTick_Config(2500);	
	
	uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  ece315_lcdClear();

	
	while(1){
		// Lab 3 Driving
		getMessage(&data);
		if(data>>24 == 'F' && (data>>16 & 0xFF) == 'W'){
			if(motorDisabled == true){
				GPIOF->DATA |= PF3;
				motorDisabled = false;
			}
			direction = 0;
			drv8833_leftForward(data & 0xFFFF);
			drv8833_rightForward(data & 0xFFFF);
		}
		else if (data>>24 == 'R' && (data>>16 & 0xFF) == 'V'){
			if(motorDisabled == true){
				GPIOF->DATA |= PF3;
				motorDisabled = false;
			}
			direction = 1;
			drv8833_leftReverse(data & 0xFFFF);
			drv8833_rightReverse(data & 0xFFFF);
		}
		else if (data>>24 == 'R' && (data>>16 & 0xFF) == 'T'){
			if(motorDisabled == true){
				GPIOF->DATA |= PF3;
				motorDisabled = false;
			}
			direction = 2;
			drv8833_turnRight(data & 0xFFFF);
		}
		else if (data>>24 == 'L' && (data>>16 & 0xFF) == 'F'){
			if(motorDisabled == true){
				GPIOF->DATA |= PF3;
				motorDisabled = false;
			}
			direction = 2;
			drv8833_turnLeft(data & 0xFFFF);
		}
		else{
			GPIOF->DATA &= ~PF3;
			motorDisabled = true;
		}
		
		// Lab 4
		if(secTick){
			sprintf(t, "Left = %d\n\r", uartDistance);
		  uartTxPoll(UART0_BASE, t);
			secTick = false;		
		}
		if(uartTick){
			ece315_lcdClear();
			sprintf(dist, "DIST: %0.1f\n\r", (double)uartDistance);
			ece315_lcdWriteString(0, dist);
			switch(direction) {
				case 0:
					ece315_lcdWriteString(1, "DIR: FWD\n");
					break;
				case 1:
					ece315_lcdWriteString(1, "DIR: REV\n");
					break;
				case 2:
					ece315_lcdWriteString(1, "DIR: TURN\n");
					break;
				default:
					ece315_lcdWriteString(1, "DIR: ERROR\n");
					break;
			}
			
			
			uartTick = false;
		}
		
	}
}
