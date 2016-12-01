// Copyright (c) 2014, Joe Krachey
// All rights reserved.
//
// Redistribution and use in binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//*****************************************************************************
// main.c
// Author: jkrachey@wisc.edu
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#include "TM4C123.h"
#include "boardUtil.h"
#include "drv8833.h"
#include "interrupts.h"
#include "encoders.h"
#include "ece315_lab3.h"
#include "lcd.h"
#include "led_controller.h"
#include "adc.h"
//*****************************************************************************
// Global Variables
//*****************************************************************************
int secCount = 0;
char t [25];
char dist [25];
int direction = 0;  // 0 = FWD, 1 = REV, 2 = TURN
int pwmCount = 0; //counter for the pwm
uint32_t pwmPin;
bool wasHigh;
uint32_t adcData;
//*****************************************************************************
//*****************************************************************************


void initializeBoard(void)
{
  DisableInterrupts();
	
	SysTick_Config(2500);	
	
  serialDebugInit();
	// Lab 1
	sensor_Init();
	SysTick_Config(2500);
	
	// Lab 2
	drv8833_gpioInit();
	
	// Lab3 Stuff
	encodersInit();
	rfInit();
	ece315_lcdInit();
	
	//final project stuff
	ledController_init(IO_I2C_BASE);
  EnableInterrupts();
}

void getMessage(uint32_t* data) {
	char msg[80];
	wireless_com_status_t status;
	char interrupts[80];
	status = wireless_get_32(false, data);
	if(status == NRF24L01_RX_SUCCESS)	{
		memset (msg,0,80);
		sprintf(msg,"Data RXed: %c%c %d\n\r", *data>>24, *data>>16, *data & 0xFFFF);
		uartTxPoll(UART0_BASE, msg);
	}
}

void setLed(int distance, int side){
				//0 = left, 1 = center, 2 = right
				uint8_t val;
				switch (side){
					case(0):
						val = 0x07;
						break;
					case 1:
						val = 0x0A;
						break;
					case 2:
						val = 0x0D;
						break;
				}
				if(distance <= 8){
				  led_controller_byte_write(IO_I2C_BASE, val, 0xFF);
					led_controller_byte_write(IO_I2C_BASE, val+1, 0x00);
					led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
				  led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
			}
			else if (distance < 16 && distance > 9){
				  led_controller_byte_write(IO_I2C_BASE, val, 0xff);
					led_controller_byte_write(IO_I2C_BASE, val+1, 0xff);
					led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
					led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
			}
			else if (distance >= 16){
				  led_controller_byte_write(IO_I2C_BASE, val, 0x00);
					led_controller_byte_write(IO_I2C_BASE, val+1, 0xff);
					led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
					led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
			}
}

int getUART(void) {
	char uartVal[3];
	
	while(uartRxPoll(UART7_BASE, 1) != 'R') {
			uartVal[0] = uartRxPoll(UART7_BASE, 1);
			uartVal[1] = uartRxPoll(UART7_BASE, 1);
			uartVal[2] = uartRxPoll(UART7_BASE, 1);
		}
	return (((uartVal[0]-48)* 100) + ((uartVal[1]-48)*10) + (uartVal[2]-48));
	
}

void readSensors(int* uartDistance, int* analogDistance, int* pwmDistance) {
	static int pwmCount = 0;
	if(sysTick) {			
		pwmPin = (GPIOE->DATA & 0x4)>>2;  // Get Value from Digital Input
		if(pwmPin == 1){  // If high, count this cycle and make note that it was high
			pwmCount++;
			wasHigh = true;
		}	else if(wasHigh) {  // If falling edge, calculate and reset count
			*pwmDistance = (int)((pwmCount * 50)/147);
			wasHigh = false;
			pwmCount = 0;
		}
		sysTick = false;
	}
	
	if(analogTick){  // Every 10 mS get value from ADC
		adcData = getADCValue(ADC0_BASE, 0);
		*analogDistance = (int)(adcData * 0.125);
		analogTick = false;
	}
	
	if(uartTick){
		*uartDistance = getUART();
		uartTick = false;
	}	
}

void setLEDs(int lDist, int cDist, int rDist) {
	
}

//*****************************************************************************
//*****************************************************************************
int 
main(void)
{
	int lDist, cDist, rDist = 0;
	
	int lHist[5];
	int cHist[5];
	int rHist[5];

	
	uint32_t spiData;
	bool motorEnabled = false;
	
		
  initializeBoard();
	
	uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  ece315_lcdClear();

	while(1){
		
		readSensors(&lDist, &cDist, &rDist);
		setLEDs(lDist, cDist, rDist);
		
		

		
		// Three Condtions:
			// If can turn left, turn left
			// Else if can drive forward, drive forward
			// Else if can turn right, turn right
			// Else deadend, turn around
		
		if (uartDistance >= 8) {
			drv8833_turnLeft(50);
		} else if (analogDistance >= 8) {
			drv8833_rightForward(80);
			drv8833_leftForward(80);
		} else {
			drv8833_turnRight(50);
		}
		
	}
}
