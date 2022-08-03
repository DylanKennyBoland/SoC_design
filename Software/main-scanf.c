//------------------------------------------------------------------------------------------------------
// Demonstration program for Cortex-M0 SoC design, showing user input options
// 1) Copy the switch signals to the LEDs
// 2) Invert the rightmost 8 LEDs twice (flashing effect)
// 3) Ask user for text input in various forms and print what is received
// 4) Loop forever doing the above.
//
// Version 2 - March 2022
//------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include "DES_M0_SoC.h"

#define nLOOPS_per_DELAY		1000000
#define INVERT_LEDS				(GPIO_LED ^= 0xff)

//////////////////////////////////////////////////////////////////
// Interrupt service routine, runs when UART interrupt occurs - see cm0dsasm.s
//////////////////////////////////////////////////////////////////
void UART_ISR()		
{	
	// Do nothing - UART interrupts are not used here
}

//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()	
{
	// Do nothing - SysTick is not used here
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
int main(void) {
	int number;
	char message[30];

	delay(nLOOPS_per_DELAY);										// wait a little
	
	printf("\nWelcome to DES SoC (scanf version)\n");			// output welcome message
	
	while(1){			// loop forever
		
		// Flash the LEDs 
		GPIO_LED = GPIO_SW;	// Echo the switches onto the LEDs
		delay(nLOOPS_per_DELAY);		// delay a little
		INVERT_LEDS;						// invert the 8 rightmost LEDs
		delay(nLOOPS_per_DELAY);
		INVERT_LEDS;						// invert the same LEDs again
		delay(nLOOPS_per_DELAY);
		
		// Prompt for user input using scanf() - stops at space, tab or enter
		printf("\nEnter a word (no spaces): ");
		scanf("%28s", message);
		printf("\nYour word was |%s|\n", message);

		// Prompt for user input using fgets() - stops at enter
		printf("\nEnter a message (up to 29 characters): ");
		fgets(message, 30, stdin);
		printf("\nYour message was |%s|\n", message);

		// Prompt for the user to enter a number - stops at anything not part of a number
		printf("\nEnter an integer: ");
		scanf("%d", &number);
		printf("\nYou entered %d\n", number);
			
		} // end of infinite loop

}  // end of main


