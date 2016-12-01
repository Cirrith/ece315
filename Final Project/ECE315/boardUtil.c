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

#include "TM4C123GH6PM.h"
#include "boardUtil.h"
#include "../include/sysctrl.h"
#include "drv8833.h"
#include "gpioPort.h"
#include "TM4C123.h"
#include "adc.h"




void DisableInterrupts(void)
{
  __asm {
         CPSID  I
  }
}
void EnableInterrupts(void)
{
  __asm {
    CPSIE  I
  }
}


//*****************************************************************************
// Configure PA0 and PA1 to be UART pins
//*****************************************************************************
void uart0_config_gpio(void)
{
   gpio_enable_port(GPIOA_BASE);
   gpio_config_digital_enable( GPIOA_BASE, PA0 | PA1);
   gpio_config_alternate_function( GPIOA_BASE, PA0 | PA1);
   gpio_config_port_control( GPIOA_BASE, GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX);
}

//*****************************************************************************
//*****************************************************************************
void serialDebugInit(void)
{
  // Configure GPIO Pins
  uart0_config_gpio();
 
  // Initialize UART0 for 8N1, interrupts enabled.
  uart_init_115K(UART0_BASE, SYSCTL_RCGCUART_R0, SYSCTL_PRUART_R0);
}

//*****************************************************************************
// Setup Sensors (Right = PWM, Center = Analog, Left = UART)
//		PE0 = UART_RX, PE1 = UART_TX, PE2 = PWM, PE3 = Analog
//*****************************************************************************
void sensor_Init() {
	gpio_enable_port(GPIOE_BASE);
	
	// Right Sensor
	gpio_config_enable_input(GPIOE_BASE, PE2);
	gpio_config_digital_enable(GPIOE_BASE, PE2);
	
	// Center Sensor
	gpio_config_enable_input(GPIOE_BASE, PE3);
	gpio_config_analog_enable(GPIOE_BASE, PE3);
	gpio_config_alternate_function(GPIOE_BASE, PE3);
	initializeADC(ADC0_BASE);
	
	// Left Sensor
	gpio_config_enable_input(GPIOE_BASE, PE0 | PE1);
	gpio_config_digital_enable(GPIOE_BASE, PE0 | PE1);
	gpio_config_alternate_function(GPIOE_BASE, PE0 | PE1);
	gpio_config_port_control(GPIOE_BASE, GPIO_PCTL_PE0_U7RX | GPIO_PCTL_PE1_U7TX);
	uart_init_9600(UART7_BASE, SYSCTL_RCGCUART_R7, SYSCTL_PRUART_R7);
}

//*****************************************************************************
// Setup UART Module for transmitting at 9600 baud and enable interrupts
//*****************************************************************************
void uart_init_9600(uint32_t base, uint32_t rcgc_mask, uint32_t pr_mask) {
	  UART0_Type *myUart;
	
    myUart = (UART0_Type *)base;
    
    // Enable UART Clock
    SYSCTL->RCGCUART |= rcgc_mask;
    
    // Wait until the UART is ready
    while( (SYSCTL->PRUART & pr_mask) == 0);
    
    // Set the baud rate
    myUart->IBRD = 325;
    myUart->FBRD = 33;
    
    // Disable UART
    myUart->CTL &= ~UART_CTL_UARTEN;
    
    // Configure the Line Control for 8N1, FIFOs
    myUart->LCRH = UART_LCRH_WLEN_8 | UART_LCRH_FEN;  
		myUart->IM |= UART_IM_RXIM | UART_IM_RTIM;
	
    // Enable Tx, Rx, and the UART
    myUart->CTL =  (UART_CTL_RXE |  UART_CTL_TXE |  UART_CTL_UARTEN);
		
		NVIC_SetPriority(UART7_IRQn, 1);
		NVIC_EnableIRQ(UART7_IRQn);
}

//Lab3
void encodersInit(){
	GPIOA_Type *gpioc;
	GPIOA_Type *gpiof;
	
	gpioc = (GPIOA_Type*) GPIOC_BASE;
	gpiof = (GPIOA_Type*) GPIOF_BASE;
	
	gpio_enable_port(GPIOF_BASE);
	gpio_enable_port(GPIOC_BASE);
	gpio_config_digital_enable(GPIOF_BASE, PF0 | PF1);
	gpio_config_digital_enable(GPIOC_BASE, PC5 | PC6);
	gpio_config_enable_input(GPIOF_BASE, PF0 | PF1);
	gpio_config_enable_input(GPIOC_BASE, PC5 | PC6);
	
	gpioc->ICR |= 0x60;
  gpioc->IM &= ~0x60;
	gpioc->IS &= ~0x60;
	gpioc->IBE &= ~0x60;
	gpioc->IEV |= 0x60;
	gpioc->IM |= 0x60;
	NVIC_SetPriority(GPIOC_IRQn, 1);
	NVIC_EnableIRQ(GPIOC_IRQn);

	gpiof->ICR |= 0x03;
	gpiof->IM &= ~0x03;
	gpiof->IS &= ~0x03;
	gpiof->IBE &= ~0x03;
	gpiof->IEV |= 0x03;
	gpiof->IM |= 0x03;
	NVIC_SetPriority(GPIOF_IRQn, 1);
	NVIC_EnableIRQ(GPIOF_IRQn);
}

