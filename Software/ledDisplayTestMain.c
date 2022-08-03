//------------------------------------------------------------------------------------------------------
// Demonstration program for Cortex-M0 SoC design - simple version, no CMSIS
// 1)Enable the interrupt for UART - interrupt when character received
// 2)Go to sleep mode
// 3)On interruption, echo character back to the UART and collect into buffer
// 4)When a whole sentence has been received, invert the case and send it back
// 5)Loop forever doing the above.
//
// Version 5 - March 2022
//------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <math.h>
#include "DES_M0_SoC.h"

#define BUF_SIZE						100
#define ASCII_CR						'\r'
#define CASE_BIT						('A' ^ 'a')
#define nLOOPS_per_DELAY		1000000

#define INVERT_LEDS					(gpioLed ^= 0xff)

#define ARRAY_SIZE(__x__)   (sizeof(__x__)/sizeof(__x__[0]))

#define NUM_DIGITS 4 	// Number of digits enabled in 7 segment display

volatile uint8  counter  = 0; // current number of char received on UART currently in RxBuf[]
volatile uint8  BufReady = 0; // Flag to indicate if there is a sentence worth of data in RxBuf
volatile uint8  RxBuf[BUF_SIZE];

//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()	
{
	// Do nothing - this interrupt is not used here
}

//////////////////////////////////////////////////////////////////
// Interrupt service routine, runs when UART interrupt occurs - see cm0dsasm.s
//////////////////////////////////////////////////////////////////
void UART_ISR()		
{
	
}

// function to display temperature code
uint32 dispTempLED(uint8 number)
{
	uint32 gpioLed;
	
	switch(number)
	{
		case 0:
			gpioLed = 0x0000;
			break;
			
		case 1:
			gpioLed = 0x0001;
			break;
		
		case 2:
			gpioLed = 0x0003;
			break;
			
		case 3:
			gpioLed = 0x0007;
			break;
			
		case 4:
			gpioLed = 0x000F;
			break;
			
		case 5:
			gpioLed = 0x001F;
			break;
			
		case 6:
			gpioLed = 0x003F;
			break;
			
		case 7:
			gpioLed = 0x007F;
			break;
			
		case 8: 
			gpioLed = 0x00FF;
			break;
			
		case 9:
			gpioLed = 0x01FF;
			break;
			
		case 10:
			gpioLed = 0x03FF;
			break;
			
		case 11:
			gpioLed = 0x07FF;
			break;
			
		case 12:
			gpioLed = 0x0FFF;
			break;
			
		case 13:
			gpioLed = 0x1FFF;
			break;
			
		case 14:
			gpioLed = 0x3FFF;
			break;
			
		case 15:
			gpioLed = 0x7FFF;
			break;
			
		case 16:
			gpioLed = 0xFFFF;
			break;
			
		default:
			gpioLed = 0x0000;
			break;
	}
	return gpioLed;
}

// configuration function for 7 segment display
void segDispConfig()
{
	// first 8 bits are not used, 
	// second byte enables digits
	// third byte sets hex mode or raw mode
	// 4th byte sets dots for each digit
	CONTROL7 = 0x00ff7700;
//	CONTROL7 = 0x00ffff00;
//	CONTROL7 = 0xffffffff;
}
	
// function to send raw data to raw registers in 7-segment display
void sendRaw(uint32 raw, int low)
{
	if(low)
	{
		RAW_LOW = raw;
	}
	else
	{
		RAW_HIGH = raw;
	}	
}


	
// function to display signed number on 7 segment 
void displayNumber(int number)
{
	int i;
	uint16 hex = 0;
	uint8 digitValue;
	
	if(number < 0)
	{
		// send a dash to the fourth digit from the left
		sendRaw(0x02000000,1);
		number = -number;
	}
	else
	{
		// display 0 on the fourth digit from the left
		sendRaw(0xfc000000,1);
	}
	i=0;
	while(number > 0)
	{
		digitValue = number % 10;
		hex += digitValue<<(4*i);
		number /= 10;
		i++;
	}
	HEX_DATA = hex;
}

	
//////////////////////////////////////////////////////////////////
// Software delay function
// As a rough guide, looping 1000 times takes 160 us to 220 us, 
// depending on the compiler optimisation level.
//////////////////////////////////////////////////////////////////
void delay(uint32 n) 
{
	volatile uint32 i;
	for(i=0;i<n;i++)
	{
	}
}


//////////////////////////////////////////////////////////////////
// Main Function
//////////////////////////////////////////////////////////////////
int main(void) 
{
	int j;
	
	// configure 7 segment display
	segDispConfig();
	
	while(1)		// loop forever
	{	
		
		// loop through the temperature LED code indefinitely
		for(j=-111;j<=111;j++)
		{
			displayNumber(j);
			delay(4000000);
		}
	
//	//HEX_DATA = 0x111;		
//	displayNumber(111);

	} // end of infinite loop

}  // end of main
