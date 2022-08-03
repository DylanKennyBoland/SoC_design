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
#include "DES_M0_SoC.h"

#define BUF_SIZE						100
#define ASCII_CR						'\r'
#define CASE_BIT						('A' ^ 'a')
#define nLOOPS_per_DELAY		1000000

#define INVERT_LEDS					(GPIO_LED ^= 0xff)

#define ARRAY_SIZE(__x__)   (sizeof(__x__)/sizeof(__x__[0]))

volatile uint8  counter  = 0; // current number of char received on UART currently in RxBuf[]
volatile uint8  BufReady = 0; // Flag to indicate if there is a sentence worth of data in RxBuf
volatile uint8  RxBuf[BUF_SIZE];


//////////////////////////////////////////////////////////////////
// Interrupt service routine, runs when UART interrupt occurs - see cm0dsasm.s
//////////////////////////////////////////////////////////////////
void UART_ISR()		
{
	
}

//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()	
{
	// Do nothing - this interrupt is not used here
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
	uint8 i;
	uint8 TxBuf[ARRAY_SIZE(RxBuf)];
	
	delay(nLOOPS_per_DELAY);						// wait a little
	
	printf("\nWelcome to Cortex-M0 SoC\n");		// output welcome message
	
	printf("\nSPI Control Register: %x\n", SPI_CTRL);
	
	SPI_CTRL = 0xFF;
	
	printf("\nSPI Control Register: %x\n", SPI_CTRL);
	
	SPI_CTRL = 0xFe;
	
	printf("\nSPI Control Register: %x\n", SPI_CTRL);
	
	while(1)		// loop forever
	{	
		
	} // end of infinite loop*/

}  // end of main


