#include "drv8833.h"
#include "system_TM4C123.h"


#define PWM_LOAD_VAL    10000
#define PWM_CHANNEL_PWM   (PWM_GEN_ACTCMPAD_LOW | PWM_GEN_ACTLOAD_HIGH | PWM_GEN_ACTZERO_NOTHING)
#define PWM_CHANNEL_LOW   (PWM_GEN_ACTCMPBD_LOW | PWM_GEN_ACTLOAD_LOW | PWM_GEN_ACTZERO_NOTHING)
#define PWM_CHANNEL_HIGH  (PWM_GEN_ACTCMPBD_HIGH | PWM_GEN_ACTLOAD_HIGH | PWM_GEN_ACTZERO_NOTHING)

#define USEPWMDIV (1 << 20)
#define DIV2 ~(7 << 17)
//*****************************************************************************
// Initializes the 6 pins needed to control the DRV8833
//*****************************************************************************
void  drv8833_gpioInit(void)
{
  gpio_enable_port(GPIOE_BASE);
	gpio_enable_port(GPIOB_BASE);
	gpio_enable_port(GPIOF_BASE);
	
	gpio_config_digital_enable(GPIOB_BASE, PB4|PB5);
	gpio_config_digital_enable(GPIOE_BASE, PE4|PE5); 
	gpio_config_digital_enable(GPIOF_BASE, PF2|PF3);
	
	gpio_config_enable_output(GPIOB_BASE, PB5 | PB4 );
	gpio_config_enable_output(GPIOE_BASE, PE5 | PE4 );
	gpio_config_enable_input(GPIOF_BASE, PF2 );
	gpio_config_enable_output(GPIOF_BASE, PF3 );

	gpio_config_alternate_function(GPIOB_BASE, PB5|PB4);
	gpio_config_alternate_function(GPIOE_BASE, PE5|PE4);
	
	gpio_config_port_control(GPIOB_BASE, ((4<<16) | (4<<20)));
	gpio_config_port_control(GPIOE_BASE, ((5<<16) | (5<<20)));
	
	SYSCTL->RCGCPWM |= 0x3;
	SYSCTL->RCC |= USEPWMDIV;
	SYSCTL->RCC &= DIV2;
}

//*****************************************************************************
//*****************************************************************************
void  drv8833_leftForward(uint8_t dutyCycle)
{
	uint32_t comparison = (PWM_LOAD_VAL-(dutyCycle*100));
  pwmConfig(PWM0_BASE, 1, PWM_LOAD_VAL , comparison, 0, PWM_CHANNEL_PWM, PWM_CHANNEL_LOW) ;
	
}

//*****************************************************************************
//*****************************************************************************
void  drv8833_leftReverse(uint8_t dutyCycle)
{
	  uint32_t comparison = (PWM_LOAD_VAL-(dutyCycle*100));
	  pwmConfig(PWM0_BASE, 1, PWM_LOAD_VAL , 0, comparison, PWM_CHANNEL_LOW,PWM_CHANNEL_PWM) ;

}


//*****************************************************************************
//*****************************************************************************
void  drv8833_rightForward(uint8_t dutyCycle)
{
	  uint32_t comparison = (PWM_LOAD_VAL-(dutyCycle*100));
	  pwmConfig(PWM1_BASE, 1, PWM_LOAD_VAL , comparison, 0, PWM_CHANNEL_PWM, PWM_CHANNEL_LOW) ;

}

//*****************************************************************************
//*****************************************************************************
void  drv8833_rightReverse(uint8_t dutyCycle)
{
		uint32_t comparison = (PWM_LOAD_VAL-(dutyCycle*100));
	  pwmConfig(PWM1_BASE, 1, PWM_LOAD_VAL , 0, comparison, PWM_CHANNEL_LOW,PWM_CHANNEL_PWM) ;

}

//*****************************************************************************
//*****************************************************************************
void  drv8833_turnLeft(uint8_t dutyCycle)
{
	drv8833_rightForward(dutyCycle);
	drv8833_leftReverse(dutyCycle);
}

//*****************************************************************************
//*****************************************************************************
void  drv8833_turnRight(uint8_t dutyCycle)
{
	drv8833_rightReverse(dutyCycle);
	drv8833_leftForward(dutyCycle);
}
