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
	bool wasHigh; //test if the PWM signal was high so that we can get the full signal before testing again
  char t [25]; //buffer for printing to serial decode
	char uartVal[3];
	int uartDistance;
	int pwmCount; //counter for the pwm
	uint32_t pwmPin;
	float pwmDistance = 0;
	float analogDistance = 0;
//*****************************************************************************
//*****************************************************************************

//initialize UART7 to a baud rate of 9600 with FIFO enabled
void uart_init_9600(){
	UART0_Type *myUart = (UART0_Type*) UART7_BASE;
	SYSCTL->RCGCUART |= SYSCTL_RCGCUART_UART7;
	while((SYSCTL->PRUART & SYSCTL_PRUART_R7) == 0);
	
	myUart->CTL &= ~UART_CTL_UARTEN;
	
	myUart->IBRD = 325;
	myUart->FBRD = 33;
	
	myUart->LCRH = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
	myUart->CTL = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
}

void Sensor_Init(){
	gpio_enable_port(GPIOE_BASE);
	gpio_config_enable_input(GPIOE_BASE, PE2 | PE3 | PE0 | PE1);
	gpio_config_digital_enable(GPIOE_BASE, PE2|PE0|PE1); 
	gpio_config_analog_enable(GPIOE_BASE, PE3);
	gpio_config_alternate_function(GPIOE_BASE, PE3|PE0|PE1);
	initializeADC(ADC0_BASE);
	
	//uart initialization
	gpio_config_port_control( GPIOE_BASE, GPIO_PCTL_PE0_U7RX | GPIO_PCTL_PE1_U7TX);
	uart_init_9600();
	
	//testing purposes
	gpio_enable_port(GPIOF_BASE);
	gpio_config_enable_output(GPIOF_BASE, PF1|PF4|PF3);
	gpio_config_digital_enable(GPIOF_BASE, PF1|PF4|PF3);

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

	//gpio_config_enable_input(Gpio
  uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  
  // Infinite Loop
  while(1)
  {
		//interrupt for getting the left and right sensors data and doing the conversion of these values to display inches
		if(AlertSysTick){
			GPIOF->DATA ^= PF1;
			//UART calculations
			if(uartRxPoll(UART7_BASE, 0) == 'R') {
				uartVal[0] = uartRxPoll(UART7_BASE, 1);
				uartVal[1] = uartRxPoll(UART7_BASE, 1);
				uartVal[2] = uartRxPoll(UART7_BASE, 1);
			}
			uartDistance = (((uartVal[0]-48)* 100) + ((uartVal[1]-48)*10) + (uartVal[2]-48));
			
			//PWM Calculations 
			pwmPin = GPIOE->DATA & (1 << 2);
			if(pwmPin != 0){
				pwmCount++;
				wasHigh = true;
			}
			else if(wasHigh){
				pwmDistance = ((pwmCount * 50)/147);
				wasHigh = false;
				pwmCount = 0;
			}
			else{
				pwmCount = 0;
			}
			AlertSysTick = false;
		}
		
		//interrupt for the analog timer which is 10ms for the center sensor
		if(analogTick){
			GPIOF->DATA ^= PF4;
			distance = getADCValue(ADC0_BASE, 0);
			analogDistance = (float)distance * 0.125;
			analogTick = false;
		}
		
		//1 second timer to display the distances for the three sensors
		if(secTick){
			sprintf(t, "Center = %.2f\n\r Left = %d\n\r Right = %.2f\n\r", analogDistance, uartDistance, pwmDistance);
		  uartTxPoll(UART0_BASE, t);
			secTick = false;
		}
  }
}
