#ifndef __ECE353_INTERRUPTS_H__
#define __ECE353_INTERRUPTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123.h"

#include "uart.h"
//#include "pc_buffer.h"

extern volatile bool AlertSysTick;
extern volatile bool analogTick;
extern volatile bool secTick;

void SysTick_Handler(void);

#endif
