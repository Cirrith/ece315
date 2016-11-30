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
float pwmDistance = 0;
float analogDistance = 0;
bool wasHigh;
uint32_t adcData;
//char uartVal[3];
//*****************************************************************************
//*****************************************************************************


void initializeBoard(void)
{
  DisableInterrupts();
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
//*****************************************************************************
//*****************************************************************************
int 
main(void)
{
	uint32_t data;
	bool motorDisabled = false;
	
  initializeBoard();
	
	SysTick_Config(2500);	
	
	uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  ece315_lcdClear();

	
	while(1){
		// Lab 4
		if(secTick){
			sprintf(t, "pwm = %.2f\n\r", pwmDistance);
		  uartTxPoll(UART0_BASE, t);
			secTick = false;		
		}
		
		if(sysTick){
			//UART calculations
			/*if(uartRxPoll(UART7_BASE, 0) == 'R') {
				uartVal[0] = uartRxPoll(UART7_BASE, 1);
				uartVal[1] = uartRxPoll(UART7_BASE, 1);
				uartVal[2] = uartRxPoll(UART7_BASE, 1);
			}
			uartDistance = (((uartVal[0]-48)* 100) + ((uartVal[1]-48)*10) + (uartVal[2]-48));*/
			
			
			pwmPin = GPIOE->DATA & (1 << 2);
			if(pwmPin == 4){
				pwmCount++;
				wasHigh = true;
			}
			else if(wasHigh){
				pwmDistance = ((pwmCount * 50)/147);
				wasHigh = false;
				setLed((int)pwmDistance, 2);
				pwmCount = 0;
			}
			else{
				pwmCount = 0;
			}
			sysTick = false;
		}
		
		if(analogTick){
			adcData = getADCValue(ADC0_BASE, 0);
			analogDistance = (float)adcData * 0.125;
			setLed((int)analogDistance, 1);
			analogTick = false;
		}
		
		/*if(uartTick){
			setLed(uartDistance, 0);
			ece315_lcdClear();
			sprintf(dist, "DIST: %0.1f\n\r", (double)uartDistance);
			ece315_lcdWriteString(0, dist);
			switch(direction) {
				case 0:
					ece315_lcdWriteString(1, "DIR: FWD\n");
					break;
				case 1:
					ece315_lcdWriteString(1, "DIR: REV\n");
					break;
				case 2:
					ece315_lcdWriteString(1, "DIR: TURN\n");
					break;
				default:
					ece315_lcdWriteString(1, "DIR: ERROR\n");
					break;
			}
			
			
			uartTick = false;
		}*/
		
	}
}
