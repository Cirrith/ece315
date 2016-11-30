#ifndef __ECE353_INTERRUPTS_H__
#define __ECE353_INTERRUPTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123.h"

#include "uart.h"

extern volatile bool sysTick;
extern volatile bool analogTick;
extern volatile bool secTick;
extern volatile uint32_t f0;
extern volatile uint32_t f1;
extern volatile uint32_t c5;
extern volatile uint32_t c6;

void SysTick_Handler(void);
void GPIOF_Handler(void);
void GPIOC_Handler(void);
#endif
