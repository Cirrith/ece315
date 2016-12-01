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
#define histSize 5

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

// Get a message from SPI, i.e. wireless module
void getMessage(uint32_t* data) {
	char msg[80];
	wireless_com_status_t status;
	status = wireless_get_32(false, data);
	if(status == NRF24L01_RX_SUCCESS)	{
		memset (msg,0,80);
		sprintf(msg,"Data RXed: %c%c %d\n\r", *data>>24, *data>>16, *data & 0xFFFF);
		uartTxPoll(UART0_BASE, msg);
	}
}

// Turn LED accodring to distance
// < 8" = Red
// 8" < 15" = Yellow
// 15" < = Green
//0 = left, 1 = center, 2 = right
void setLED(int distance, int side){
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
	if(distance <= 8) {
		led_controller_byte_write(IO_I2C_BASE, val, 0xFF);
		led_controller_byte_write(IO_I2C_BASE, val+1, 0x00);
		led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
		led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
	}	else if (distance < 16 && distance > 9) {
		led_controller_byte_write(IO_I2C_BASE, val, 0xff);
		led_controller_byte_write(IO_I2C_BASE, val+1, 0xff);
		led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
		led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
	} else if (distance >= 16){
		led_controller_byte_write(IO_I2C_BASE, val, 0x00);
		led_controller_byte_write(IO_I2C_BASE, val+1, 0xff);
		led_controller_byte_write(IO_I2C_BASE, val+2, 0x00);
		led_controller_byte_write(IO_I2C_BASE, 0x10, 0x00);
	}
}

int getUART(void) {
	char uartVal[3];
	
	while(uartRxPoll(UART7_BASE, 1) != 'R') {  // Changed if to while, should keep going through data
		uartVal[0] = uartRxPoll(UART7_BASE, 1);
		uartVal[1] = uartRxPoll(UART7_BASE, 1);
		uartVal[2] = uartRxPoll(UART7_BASE, 1);
	}
	return (((uartVal[0]-48)* 100) + ((uartVal[1]-48)*10) + (uartVal[2]-48));
}

void readSensors(int* uartDistance, int* analogDistance, int* pwmDistance) {
	static bool wasHigh = false;
	static int pwmCount = 0;
	int pwmData;
	int adcData;
	
	if(sysTick) {			
		pwmData = (GPIOE->DATA & 0x4)>>2;  // Get Value from Digital Input
		if(pwmData == 1){  // If high, count this cycle and make note that it was high
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
	setLED(lDist, 0);
	setLED(cDist, 1);
	setLED(rDist, 2);
}

void updateScreen(int dist) {
	char msg[25];
	sprintf(msg, "Distance: %d", dist);
	ece315_lcdWriteString(1, msg);
}

//*****************************************************************************
//*****************************************************************************
int 
main(void) {
	int lDist, cDist, rDist = 0;
	int lTrend, cTrend, rTrend = 0;
	
	int lHist[histSize];
	int cHist[histSize];
	int rHist[histSize];

	int histTime = 0;  // Oldest value in Hist
		
  initializeBoard();
	
	uartTxPoll(UART0_BASE, "\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  uartTxPoll(UART0_BASE,"* ECE315 Default Project\n\r");
  uartTxPoll(UART0_BASE,"**************************************\n\r");
  
	ece315_lcdClear();

	GPIOF->DATA |= PF3;  // Enable motors
	
	while(1) {
		readSensors(&lDist, &cDist, &rDist);
		setLEDs(lDist, cDist, rDist);

		if(quatTick) {
			updateScreen(((encodeL*12)+(encodeR*6))/2);  // Display average of left and right
			
			lHist[histTime] = lDist;  // Save current distance in hist
			cHist[histTime] = cDist;
			rHist[histTime] = rDist;
			
			histTime = (histTime+1)%histSize;  // Increment Time
			
			lTrend = lHist[(histTime+histSize-1)%histSize] - lHist[histTime];  // Newest - Oldest
			cTrend = cHist[(histTime+histSize-1)%histSize] - cHist[histTime];
			rTrend = rHist[(histTime+histSize-1)%histSize] - rHist[histTime];
			
			// Three Condtions:
				// If can turn left, turn left
				// Else if can drive forward, drive forward
				// Else if can turn right, turn right
				// Else deadend, turn around
			
			if (abs(lTrend) >= 8) {  // Can Turn Left
				drv8833_turnLeft(30);
			} else if (abs(cTrend) >= 8) {  // Can Drive Forward
					if(abs(lTrend) < 3) {  // Basically Straight
						// Drive straight forward
						drv8833_rightForward(40);
						drv8833_leftForward(40);
					} else if (lTrend < 0) {  // Curving to the left
						// Curve to the Right
						drv8833_rightForward(30);
						drv8833_leftForward(40);
					} else {
						// Curve to the left
						drv8833_rightForward(40);
						drv8833_leftForward(30);
					}
			} else {  // Have to Turn Right
				drv8833_turnRight(30);
			}
			quatTick = false;
		}
	}
}
