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
#include <math.h>

#define BUF_SIZE						100
#define ASCII_CR						'\r'
#define CASE_BIT						('A' ^ 'a')
#define nLOOPS_per_DELAY		1000000

// Macros for the ADXL362 accelerometer
#define FILTER_CTL_ADDR 0x2c
#define POWER_CTL_ADDR 0x2d // power control reg. address
#define DEV_ID_ADDR 0x00 // the device ID reg. address
// addresses for the acceleration-data registers
#define XDATA_L_ADDR 0x0e
#define XDATA_H_ADDR 0x0f
#define YDATA_L_ADDR 0x10
#define YDATA_H_ADDR 0x11
#define ZDATA_L_ADDR 0x12
#define ZDATA_H_ADDR 0x13
#define XDATA 0x08
#define ZDATA 0x0A

// command Bytes for the accelerometer
#define WRITE_TO_REG 0x0a // this Byte in a SPI transaction specifies a write command
#define READ_REG 0x0b // this Bytes signifies a read is to be done

#define SPI_CTL_MASK 0x80 // high nibble is 1000, low nibble is 0000 - use this to get MSB (BUSY flag) of CTL reg.

// Macros for the 8-digit 7-segment display interace on the AHB-Lite bus:
// First, reg. addresses:
#define RAW_LOW_ADDR 0x0
#define RAW_HIGH_ADDR 0x4
#define HEX_DATA_ADDR 0x8
#define CTL_ADDR 0xc

#define INVERT_LEDS					(GPIO_LED ^= 0xff)

#define ARRAY_SIZE(__x__)   (sizeof(__x__)/sizeof(__x__[0]))

volatile uint8  counter  = 0; // current number of char received on UART currently in RxBuf[]
volatile uint8  BufReady = 0; // Flag to indicate if there is a sentence worth of data in RxBuf
volatile uint8  RxBuf[BUF_SIZE];

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
// Interrupt service routine, runs when UART interrupt occurs - see cm0dsasm.s
//////////////////////////////////////////////////////////////////
void UART_ISR()		
{
	char c;
	c = UART_RXD;	 // read a character from UART - interrupt only occurs when character waiting
	RxBuf[counter]  = c;   // Store in buffer
	counter++;             // Increment counter to indicate that there is now 1 more character in buffer
	UART_TXD = c;   // write (echo) the character to UART (assuming transmit queue not full!)
	// counter is now the position that the next character should go into
	// If this is the end of the buffer, i.e. if counter==BUF_SIZE-1, then null terminate
	// and indicate the a complete sentence has been received.
	// If the character just put in was a carriage return, do likewise.
	if (counter == BUF_SIZE-1 || c == ASCII_CR)  
	{
		counter--;							// decrement counter (CR will be over-written)
		RxBuf[counter] = NULL;  // Null terminate
		BufReady       = 1;	    // Indicate to rest of code that a full "sentence" has being received (and is in RxBuf)
	}
}

//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()	
{
	// Do nothing - this interrupt is not used here
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
}

// this version of the function configures
// the display when the full 12-bit acceleration
// values are being read and displayed
void segDispConfig2()
{
	CONTROL7 = 0x00ff7f00;
}
	
// function to send raw data to raw registers in 7-segment display
void sendRaw(uint32 raw, int low)
{
	if(low==1)
	{
		RAW_LOW = raw;
	}
	else
	{
		RAW_HIGH = raw;
	}	
}

// this version of the function is used when the full 12-bit
// acceleration values are being read and displayed
void sendRaw2(uint32 raw) {
		RAW_HIGH = raw;
}

// function to display signed number on 7 segment 
void displayNumber(int16 number, int low)
{
	int i;
	uint32 hex = 0;
	uint8 digitValue;
	
	if(number < 0)
	{
		// send a dash to the fourth digit from the left
		sendRaw(0x02000000,low);
		number = -number;
	}
	else
	{
		// display 0 on the fourth digit from the left
		sendRaw(0xfc000000,low);
	}
	
	// make sure number doesn't overwrite sign digit
	if(number > 999){
		number = 999;
	}
	
	if(low == 1){
		i=0;
	}
	else{
		i=4;
	}
	
	while(number > 0)
	{
		digitValue = number % 10;
		hex += digitValue<<(4*i);
		number /= 10;
		i++;
	}
	HEX_DATA = hex;
}

// function to display the 12-bit acceleration values on the
// 8-digit display:
void displayHalfWordNum(int16 number)
{
	int i;
	uint32 hex = 0;
	uint8 digitValue;
	
	if(number < 0) {
		sendRaw2(0x02000000);
		number = -number;
	} else {
		sendRaw2(0xfc000000);
	}
	
	i = 0; // initialise i for the logic below
	
	while(number > 0)
	{
		digitValue = number % 10;
		hex += digitValue<<(4*i);
		number /= 10;
		i++;
	}
	HEX_DATA = hex;
}

// function to send (and receive) a Byte over SPI interface
uint8 spiByte(uint8 txByte) {
    uint8 busyStatus;
	uint8 rxByte;
	SPI_TX_DATA = txByte; // loading the transmit reg. with the value to be sent
    do {
		busyStatus = (SPI_CTRL & SPI_CTL_MASK);
    } while (busyStatus == 0x80); // wait until transaction is done...
    // Transaction done. Now let us collect the received Byte:
	rxByte = SPI_RX_DATA; // assign value in received-data reg. to rxByte var.
    return rxByte; // return the Byte
}

// function to read a register in the accelerometer (ADXL362)
uint8 getByteADXL(uint8 address) {
	uint8 rxByte;
	uint8 csStatus;
	do {
		csStatus = SPI_CTRL & 0x01;
	} while (csStatus == 1); // the chip is already selected... wait...
	SPI_CTRL = 1; // select the ADXL362 device by writing a 1 to CTL reg
	spiByte(READ_REG); // first sent the command Byte for a read operation
	spiByte(address); // specify the address
	rxByte = spiByte(0); // get the Byte
	SPI_CTRL = 0;
	return rxByte;
}

// function to read two consecutive registers in the accelerometer
uint16 getHalfWordADXL(uint8 lowAddress) {
	uint8 rxByte1;
	uint8 rxByte2;
	uint16 halfWord;
	uint8 csStatus; 
	do {
		csStatus = SPI_CTRL & 0x01; // get the chip-select bit or status
	} while (csStatus == 1); // the chip is already selected... wait...
	SPI_CTRL = 1; // select the ADXL362 device by setting LSB of CTRL reg. to 1
	spiByte(READ_REG); // first sent the command Byte for a read operation
	spiByte(lowAddress); // now specify the address
	rxByte1 = spiByte(0);
	rxByte2 = spiByte(0); // this Byte will hopefully be from (lowAddress + 1)
	SPI_CTRL = 0; // transaction is done... unselect the ADXL362 chip
	// Combine these values:
	halfWord = rxByte1 | (rxByte2 << 8);
	return halfWord; // return the 16-bit value
}

// function to send a Byte to a register in the accelerometer
void sendData2ADXL (uint8 address, uint8 data) {
	uint8 csStatus;
	do {
		csStatus = SPI_CTRL & 0x01; // get the chip-select bit or status
	} while (csStatus == 1); // the chip is already selected... wait...
	SPI_CTRL = 1; // select the ADXL362 device by writing a 1 to CTL reg. 
	spiByte(WRITE_TO_REG); // first sent the command Byte for a write operation
	spiByte(address); // now we will send the address of the reg. on the ADXL362 device
	spiByte(data); // and sent the data Byte to the address specified by prev. line
	SPI_CTRL = 0; // unselect the device
}

int main(void) 
{
	// Variables
	uint8 i; // used in for loop
	uint8 TxBuf[ARRAY_SIZE(RxBuf)];	// serial transmit buffer
	uint32 timeStart, timeEnd;	// variables to hold SysTick values for time measurement
	int16 fullAccelXdata; // variable to hold x-axis acceleration

	// Configure the 8-digit display for 12-bit representation mode:
	segDispConfig2();
	// Configure the UART - the control register decides which events cause interrupts:
	UART_CTL = (1 << UART_RX_FIFO_NOTEMPTY_BIT_POS); // Enable rx data available interrupt only
	// Configure the interrupt system in the processor (NVIC):
	NVIC_Enable = (1 << NVIC_UART_BIT_POS); // Enable the UART interrupt
	
	// Configuration of the ADXL362 device:
	/* Write 0000 1010 to POWER_CTL reg. in order to wake the device
	up, and select the measurement mode.
	bit 3: the wake-up bit
	2 LSBs: for the measurement mode
	Write 0000 0011 to FILTER_CTL reg. in order to select 2g range, and
	an output data rate of 100 Hz.
	2 MSBs: for range
	3 LSBs: for output data rate (ODR)
	bit 4: for halved bandwidth
	*/
	sendData2ADXL(POWER_CTL_ADDR, 0x0a); // config. power reg.
	sendData2ADXL(FILTER_CTL_ADDR, 0x03); // config. filter reg.
	delay(nLOOPS_per_DELAY); // wait a little
	
	while(1) // The main loop
	{	
		fullAccelXdata = (int16)getHalfWordADXL(XDATA_L_ADDR); // try and read low Byte of accel.
		displayHalfWordNum(fullAccelXdata);
		delay(nLOOPS_per_DELAY); // small delay so that we do not update or refresh the display too quickly
	} // end of infinite loop
}  // end of main

