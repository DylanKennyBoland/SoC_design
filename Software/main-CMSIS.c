/*---------------------------------------------------------------------------------
	Demonstration program for Cortex-M0 SoC design.
	Enables two interrupts: UART character received and SysTick timer.
	Repeat:
		Set the 8 leftmost LEDs to match the switches, then flashes them
		Disable SysTick interrupt if button BTND is pressed
		Print some registers and measure a delay time
		Invite input, then sleep while waiting for an interrupt
			-	on UART interrupt, put the received character in a buffer array
				and echo it back to the UART output
			- on SysTick interrupt, change the state of leftmost LED
		When CR character is received, send out all the received characters
		with their case inverted
	
	Version 5 - April 2021
	-------------------------------------------------------------------------------*/
#include <stdio.h>
#include "ARMCM0.h"
#include "DES_M0_SoC.h"

#define BUF_SIZE						100
#define ASCII_CR						'\r'
#define CASE_BIT						('A' ^ 'a')
#define FLASH_DELAY					1000000		// delay for flashing LEDs, ~220 ms
#define TEST_DELAY					1000			// delay for testing delay function

#define INVERT_LEDS_Hi			(GPIO_LED_Hi ^= 0xff)
#define MSB8								0x80			// most significant bit of 8-bit value
#define ST_INT_MASK					0x0002		// SysTick interrupt enable bit mask

#define ARRAY_SIZE(__x__)       (sizeof(__x__)/sizeof(__x__[0]))


// Global variables - shared between main and UART_ISR
volatile uint8  RxBuf[BUF_SIZE];	// array to hold received characters
volatile uint8  counter  = 0; 		// current number of characters in RxBuf[]
volatile uint8  BufReady = 0; 		// flag indicates data in RxBuf is ready for processing


//////////////////////////////////////////////////////////////////
// Interrupt service routine, runs when UART has received a character
//////////////////////////////////////////////////////////////////
void UART_ISR()		
{
	char c;
	c = UART_RXD;	 				// read character from UART (there must be one waiting)
	RxBuf[counter]  = c;  // store in buffer
	counter++;            // increment counter, number of characters in buffer
	UART_TXD = c;  				// write character to UART (assuming transmit queue not full)

	/* Counter is now the position in the buffer that the next character should go into.
		If this is the end of the buffer, i.e. if counter == BUF_SIZE-1, then null terminate
		and indicate that a complete sentence has been received.
		If the character just put in was a carriage return, do the same.  */
	if (counter == BUF_SIZE-1 || c == ASCII_CR)  
	{
		counter--;							// decrement counter (CR will be over-written)
		RxBuf[counter] = NULL;  // null terminate
		BufReady       = 1;	    // indicate that data is ready for processing
	}
}


//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()	
{
	GPIO_LED_Hi ^= MSB8;		// flip the leftmost LED
}


//////////////////////////////////////////////////////////////////
// Software delay function
// As a rough guide, looping 1000 times takes 160 us to 220 us, 
// depending on the compiler optimisation level.
//////////////////////////////////////////////////////////////////
void delay (uint32 n) 
{
	volatile uint32 i;
		for(i=0; i<n; i++);		// do nothing n times
}


//////////////////////////////////////////////////////////////////
// Main Function
//////////////////////////////////////////////////////////////////
int main(void) 
{
	uint8 i;		// used in for loop
	uint8 TxBuf[ARRAY_SIZE(RxBuf)];		// serial transmit buffer
	uint32 timeStart, timeEnd;	// variables to hold SysTick values for time measurement
	
	delay(FLASH_DELAY);										// wait a little
	
	printf("\nWelcome to Cortex-M0 SoC with CMSIS functions\n");			// output welcome message
	printf("CPU ID %x SysTick Calibration %x\n", SCB->CPUID, SysTick->CALIB);

	// Configure UART to cause an interrupt, and enable that interrupt in NVIC
	UART_CTL = (1 << UART_RX_FIFO_NOTEMPTY_BIT_POS);		// turn on rx data available interrupt
  NVIC_EnableIRQ(UART_IRQn);													// enable UART interrupt
	
	// Configure SysTick timer for slowest possible interrupt rate (~2.98 Hz)
	if ( SysTick_Config(0x1000000) )	// returns 1 on failure
	{
		printf("Failed to configure SysTick timer\n");
	}

	// Delay measurement, to calibrate the delay function
	timeStart = SysTick->VAL;		// capture start time
	delay( TEST_DELAY );				// delay a short time
	timeEnd = SysTick->VAL;			// capture end time (lower than start, as counting down)
	printf("delay( %d ) used %d clock cycles\n", TEST_DELAY, 
					(timeStart - timeEnd) & 0x00ffffff);  // mask to 24 bits hides overflow
	
	
	while(1){			// loop forever
			
		// Do some work before entering Sleep Mode
		GPIO_LED	= GPIO_SW; 			// echo 16 switches onto corresponding LEDs
		delay(FLASH_DELAY);				// delay a little
		INVERT_LEDS_Hi;						// invert the 8 leftmost LEDs
		delay(FLASH_DELAY);				// delay a little
		INVERT_LEDS_Hi;						// invert the 8 leftmost LEDs
		delay(FLASH_DELAY);				// delay a little

		/* If BTND is pressed, disable SysTick interrupt for this cycle.
		The NVIC enable register does not control this, but there is a bit in the
		SysTick control register to enable an overflow to cause an interrupt.  */
		if ( GPIO_BUTTON & BTND_MASK ) 		// if button D is pressed at this point
			SysTick->CTRL &= ~ST_INT_MASK;	// disable SysTick interrupt
		else 															// normal situation - button not pressed
			SysTick->CTRL |= ST_INT_MASK;		// enable SysTick interrupt
		
		// Print the values of some system registers for information
		printf("\nSysTick: Control %x Reload %x Current %x\n", 
								SysTick->CTRL, SysTick->LOAD, SysTick->VAL);		// SysTick registers
		printf("\nNVIC: Enable %x Pending %x Priority %x\n", 
								NVIC->ISER[0], NVIC->ISPR[0], NVIC->IP[0]);			// NVIC registers

		// Invite user input
		printf("\nType some characters: ");
		
		while (BufReady == 0)		// loop until input is ready to process
		{			
			__wfi();  // Wait For Interrupt: enter Sleep Mode until interrupt occurs
			// interrupt could be due to received character or SysTick timer
			if ( counter > 0 )	// if at least one character has been received
				GPIO_LED_Lo = RxBuf[counter-1];  // display code for last character received
		}

		/* Get here when CR entered or buffer full - data ready for processing.
			Process the data with UART interrupts disabled, so data does not change.  */
		
		NVIC_DisableIRQ(UART_IRQn);		// ---- start of critical section ----
		
		for (i=0; i<=counter; i++)		// step through all the bytes received
		{
			if (RxBuf[i] >= 'A') {						// if this character is a letter (roughly)
				TxBuf[i] = RxBuf[i] ^ CASE_BIT; // copy to transmit buffer, changing case
			}
			else {
				TxBuf[i] = RxBuf[i];            // non-letter so don't change case
			}
		} 
		
		BufReady = 0;			// clear the flag
		counter  = 0; 		// clear the counter for next sentence	
		
		NVIC_EnableIRQ(UART_IRQn);		// ---- end of critical section ----	
		
		
		printf("\n:--> |%s|\n", TxBuf);  // print the results between bars
		
	} // end of infinite loop

}  // end of main


