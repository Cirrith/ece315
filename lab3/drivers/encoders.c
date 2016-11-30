#include "encoders.h"

void setEncodeR(uint32_t dist) {
	c5 = dist * 6;
}

void setEncodeL(uint32_t dist) {
	f0 = dist * 12;
}