#include "interrupts.h"
#include "gpioPort.h"
#include "TM4C123GH6PM.h"

// Lab 1
volatile bool sysTick = false;
volatile bool analogTick = false;
volatile bool secTick = false;

// Lab 3
volatile uint32_t f0 = 0;
volatile uint32_t f1 = 0;
volatile uint32_t c5 = 0;
volatile uint32_t c6 = 0;

// Lab 4
//volatile char uartVal[3];
//volatile int uartDistance; 
volatile bool uartTick = false;

void SysTick_Handler(void){
	uint32_t val;
	// Counter for Analog
	static int count = 0;
	
	// Counter for 1 second
	static int secCount = 0;
	
	//counter for 1 microsecond
	sysTick = true;

	count++;
	secCount++;
	
	// 10 ms
	if(count == 200){
		analogTick = true;
		count = 0;
	}
	
	// 1 Second
	if(secCount == 20000){
		secTick = true;
		secCount = 0;
	}
	
	// Reset interrupt 
	val = SysTick->VAL;
}

void UART7_Handler(void){
	static int count = 0;
	UART0_Type *myUart;
	
  myUart = (UART0_Type *)UART7_BASE;
	//UART Tick
	if(count == 5){
		uartTick = true;
		count = 0;
	}
	count++;
	myUart->ICR |= UART_IM_RXIM | UART_IM_RTIM;
}

////////////////
//LAB 3 stuff //
////////////////
void GPIOF_Handler(void) {
	GPIOA_Type *mygpio;
	mygpio = (GPIOA_Type*) GPIOF_BASE;
	
	//PF1
	if((mygpio->RIS & 0x2) == 0x2){
		if(f1 > 0)
			f1--;
	}
	//PF0
	if((mygpio->RIS & 0x1) == 1){
		if(f0 > 0)
			f0--;
	}
	mygpio->ICR |= 0x3;
}

void GPIOC_Handler(void) {
	GPIOA_Type *mygpio;
	mygpio = (GPIOA_Type*) GPIOC_BASE;
	
	//PC5
	if((mygpio->RIS & 0x20) == 0x20){
		if(c5 > 0)
			c5--;
	}
	//PC6
	if((mygpio->RIS & 0x40) == 0x40){
		if(c6 > 0)
			c6--;
	}
	mygpio->ICR |= 0x60;
}


