#include "interrupts.h"
#include "gpioPort.h"
#include "TM4C123GH6PM.h"

volatile bool AlertSysTick = false;
volatile bool analogTick = false;
volatile bool secTick = false;

//lab3 variables
volatile uint32_t f0 = 0;
volatile uint32_t f1 = 0;
volatile uint32_t c5 = 0;
volatile uint32_t c6 = 0;

void SysTick_Handler(void){
	uint32_t val;
	static int count = 0;
	static int secCount = 0;
	AlertSysTick = true;

	count++;
	secCount ++;
	
	if(count == 200){
		analogTick = true;
		count = 0;
	}
	if(secCount == 20000){
		secTick = true;
		secCount = 0;
	}
	
	val = SysTick->VAL;
}

void GPIOF_Handler(void) {
	GPIOA_Type *mygpio;
	mygpio = (GPIOA_Type*) GPIOF_BASE;
	
	//PF1
	if((mygpio->RIS & 0x2) == 0x2){
		f1++;
	}
	//PF0
	if((mygpio->RIS & 0x1) == 1){
		f0++;
	}
	mygpio->ICR |= 0x3;
}

void GPIOC_Handler(void) {
	GPIOA_Type *mygpio;
	mygpio = (GPIOA_Type*) GPIOC_BASE;
	
	//PC5
	if((mygpio->RIS & 0x20) == 0x20){
		c5++;
	}
	//PC6
	if((mygpio->RIS & 0x40) == 0x40){
		c6++;
	}
	mygpio->ICR |= 0x60;
}


