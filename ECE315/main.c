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

#include "interrupts.h"
#include "TM4C123.h"
#include "boardUtil.h"
#include "adc.h"


//*****************************************************************************
// Global Variables
//*****************************************************************************
  uint32_t distance;
  char t [100];
//*****************************************************************************
//*****************************************************************************
void Sensor_Init(){
	gpio_enable_port(GPIOE_BASE);
	gpio_config_enable_input(GPIOE_BASE, PE2 | PE3 | PE0 | PE1);
	gpio_config_digital_enable(GPIOE_BASE, PE2|PE0|PE1); 
	gpio_config_analog_enable(GPIOE_BASE, PE3);
	gpio_config_alternate_function(GPIOE_BASE, PE3|PE0|PE1);
	initializeADC(ADC0_BASE);
	
	//uart stuff
	gpio_config_port_control( GPIOE_BASE, GPIO_PCTL_PE0_U7RX | GPIO_PCTL_PE1_U7TX);
	uart_init_115K(UART7_BASE, SYSCTL_RCGCUART_R7, 
    SYSCTL_PRUART_R7);
	
	//testing purposes
	gpio_enable_port(GPIOF_BASE);
	gpio_config_enable_output(GPIOF_BASE, PF1|PF4);
	gpio_config_digital_enable(GPIOF_BASE, PF1|PF4);

}

void initializeBoard(void)
{
  DisableInterrupts();
  serialDebugInit();
	SysTick_Config(2500);
	Sensor_Init();
	GPIOF->DATA = 0;
  EnableInterrupts();
}
//*****************************************************************************
//*****************************************************************************

int 
main(void)
{
  initializeBoard();
	//config an output pin
	
	
	//sprintf(t, "jhdkjaakja %d", test);
	//uarttxpoll(uart0base, t)
	
	
	
	//gpio_config_enable_input(Gpio
  uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  
  // Infinite Loop
  while(1)
  {
		if(AlertSysTick){
			GPIOF->DATA ^= PF1;
			AlertSysTick = false;
		}
		if(analogTick){
			GPIOF->DATA ^= PF4;
			distance = getADCValue(ADC0_BASE, 0);
			analogTick = false;
		}
		if(secTick){
			sprintf(t, "ADC Value: %d\n", distance);
		  uartTxPoll(UART0_BASE, t);
			secTick = false;
		}
		 
  }
}
