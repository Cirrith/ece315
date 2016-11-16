#include "pwm.h"

uint8_t pwmConfig(uint32_t base, uint8_t pwm_generator, uint32_t load, uint32_t cmpa, 
  uint32_t cmpb, uint32_t gena, uint32_t genb)
{
	PWM0_Type *myPWM;
	switch(base){
		case(PWM0_BASE): break;
	  case(PWM1_BASE): break;
	  default: return 1;
	}
	
	myPWM = (PWM0_Type*) base;
	/*
	if(base == PWM0_BASE) {
		myPWM->_1_CTL = 0;
		myPWM->_1_GENA = gena;
		myPWM->_1_GENB = genb;
		myPWM->_1_LOAD = load;
		myPWM->_1_CMPA = cmpa;
		myPWM->_1_CMPB = cmpb;
		myPWM->_1_CTL = 1;
	}
	else{
		myPWM->_1_GENA |= gena;
		myPWM->_1_GENB |= genb;
		myPWM->_1_LOAD |= load;
		myPWM->_1_CMPA |= cmpa;
		myPWM->_1_CMPB |= cmpb;
		myPWM->_1_CTL |= 1;
	}
	myPWM->ENABLE |= 0xff;
	*/
	
	switch(pwm_generator) {
		case 0:
		{
			myPWM->_0_GENA = gena;
			myPWM->_0_GENB = genb;
			myPWM->_0_LOAD = load;
			myPWM->_0_CMPA = cmpa;
			myPWM->_0_CMPB = cmpb;
			myPWM->_0_CTL = 1;
			break;
		}
		
		case 1:
		{
			myPWM->_1_GENA = gena;
			myPWM->_1_GENB = genb;
			myPWM->_1_LOAD = load;
			myPWM->_1_CMPA = cmpa;
			myPWM->_1_CMPB = cmpb;
			myPWM->_1_CTL = 1;
			break;
		}
		
		case 2:
		{
			myPWM->_2_GENA = gena;
			myPWM->_2_GENB = genb;
			myPWM->_2_LOAD = load;
			myPWM->_2_CMPA = cmpa;
			myPWM->_2_CMPB = cmpb;
			myPWM->_2_CTL = 1;
			break;
		}
		
		case 3:
		{
			myPWM->_3_GENA = gena;
			myPWM->_3_GENB = genb;
			myPWM->_3_LOAD = load;
			myPWM->_3_CMPA = cmpa;
			myPWM->_3_CMPB = cmpb;
			myPWM->_3_CTL = 1;
			break;
		}
	}
	
	myPWM->ENABLE |= 0xff;
	
	return 1;
	}
