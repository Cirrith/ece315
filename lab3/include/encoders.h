#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <stdio.h>
#include "TM4C123GH6PM.h"
#include "interrupts.h"

void setEncodeR(uint32_t dist);

void setEncodeL(uint32_t dist);

#endif