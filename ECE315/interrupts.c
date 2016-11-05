#include "interrupts.h"
#include "gpioPort.h"

volatile bool AlertSysTick = false;
volatile bool analogTick = false;
volatile bool secTick = false;

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
