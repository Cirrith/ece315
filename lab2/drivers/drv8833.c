#include "drv8833.h"
#include "system_TM4C123.h"


#define PWM_LOAD_VAL    10000
#define PWM_CHANNEL_PWM   (PWM_GEN_ACTCMPAD_LOW | PWM_GEN_ACTLOAD_HIGH | PWM_GEN_ACTZERO_NOTHING)
#define PWM_CHANNEL_LOW   (PWM_GEN_ACTCMPBD_LOW | PWM_GEN_ACTLOAD_LOW | PWM_GEN_ACTZERO_NOTHING)
#define PWM_CHANNEL_HIGH  (PWM_GEN_ACTCMPBD_HIGH | PWM_GEN_ACTLOAD_HIGH | PWM_GEN_ACTZERO_NOTHING)

//*****************************************************************************
// Initializes the 6 pins needed to control the DRV8833
//*****************************************************************************
void  drv8833_gpioInit(void)
{
	GPIOA_Type *myGPIO;
  gpio_enable_port(GPIOE_BASE);
	gpio_enable_port(GPIOB_BASE);
	gpio_enable_port(GPIOF_BASE);
	
	gpio_config_enable_output(GPIOB_BASE, PB5 | PB4 );
	gpio_config_enable_output(GPIOE_BASE, PE5 | PE4 );
	gpio_config_enable_input(GPIOF_BASE, PF2 );
	gpio_config_enable_output(GPIOF_BASE, PF3 );

	gpio_config_digital_enable(GPIOB_BASE, PB4|PB5);
	gpio_config_digital_enable(GPIOE_BASE, PE4|PE5); 
	gpio_config_digital_enable(GPIOF_BASE, PF2|PF3); 

	gpio_config_alternate_function(GPIOB_BASE, PB5|PB4);
	gpio_config_alternate_function(GPIOE_BASE, PE5|PE4);
	
	gpio_config_port_control(GPIOB_BASE, 0x00040000);
	gpio_config_port_control(GPIOB_BASE, 0x00400000);
	gpio_config_port_control(GPIOE_BASE, 0x00050000);
	gpio_config_port_control(GPIOE_BASE, 0x00500000);
	
	
	//myGPIO = (GPIOA_Type*) GPIOB_BASE;
	//myGPIO->PCTL = 0x00440000;
	//myGPIO = (GPIOA_Type*) GPIOE_BASE;
	//myGPIO->PCTL = 0x00550000;
	
	
}
uint8_t pwmConfig(uint32_t base, uint8_t pwm_generator, uint32_t load, uint32_t cmpa, 
  uint32_t cmpb, uint32_t gena, uint32_t genb
){
	PWM0_Type *myPWM;
	myPWM = (PWM0_Type*) base;
	SYSCTL->RCGCPWM |= pwm_generator;
	SYSCTL->RCGC2 |= (2 | 16);
	SYSCTL->RCC |= ((1 << 20) | (2 << 17));
	myPWM->CTL = 0x00000000;
	if(base == PWM0_BASE){
		myPWM->_0_CTL = 0;
		myPWM->_0_GENA |= gena;
		myPWM->_0_GENB |= genb;
		myPWM->_0_LOAD |= load;
		myPWM->_0_CMPA |= cmpa;
		myPWM->_0_CMPB |= cmpb;
		myPWM->_0_CTL |= 0x00000002;
	}
	else{
		myPWM->_1_CTL = 0;
		myPWM->_1_GENA |= gena;
		myPWM->_1_GENB |= genb;
		myPWM->_1_LOAD |= load;
		myPWM->_1_CMPA |= cmpa;
		myPWM->_1_CMPB |= cmpb;
		myPWM->_1_CTL |= 0x00000002;
	}
	
	myPWM->ENABLE |= 0x00000003;
	
	return 1;
	}

//*****************************************************************************
//*****************************************************************************
void  drv8833_leftForward(uint8_t dutyCycle)
{
  pwmConfig(PWM0_BASE, 1, PWM_LOAD_VAL , (PWM_LOAD_VAL-(dutyCycle*100)), 0, PWM_CHANNEL_PWM, PWM_CHANNEL_LOW   ) ;
	
}

//*****************************************************************************
//*****************************************************************************
void  drv8833_leftReverse(uint8_t dutyCycle)
{
	  pwmConfig(PWM0_BASE, 1, PWM_LOAD_VAL , (PWM_LOAD_VAL-(dutyCycle*100)), 0, PWM_CHANNEL_PWM,PWM_CHANNEL_LOW   ) ;

}


//*****************************************************************************
//*****************************************************************************
void  drv8833_rightForward(uint8_t dutyCycle)
{
	  pwmConfig(PWM1_BASE, 1, PWM_LOAD_VAL , (PWM_LOAD_VAL-(dutyCycle*100)), 0, 0,PWM_CHANNEL_PWM  ) ;

}

//*****************************************************************************
//*****************************************************************************
void  drv8833_rightReverse(uint8_t dutyCycle)
{
	  pwmConfig(PWM1_BASE, 1, PWM_LOAD_VAL , (PWM_LOAD_VAL-(dutyCycle*100)), 0, PWM_CHANNEL_PWM,PWM_CHANNEL_LOW   ) ;

}

//*****************************************************************************
//*****************************************************************************
void  drv8833_turnLeft(uint8_t dutyCycle)
{
}

//*****************************************************************************
//*****************************************************************************
void  drv8833_turnRight(uint8_t dutyCycle)
{
}
