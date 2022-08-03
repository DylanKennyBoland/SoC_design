`timescale 1ns / 1ns
/////////////////////////////////////////////////////////////////
// Module Name: TB_AHBgpio - testbench for AHBgpio block
/////////////////////////////////////////////////////////////////
module TB_AHBgpio(    );

// AHB-Lite Bus Signals
	reg HCLK;					// bus clock
	reg HRESETn;				// bus reset, active low
	reg HSELx = 1'b0;			// selects this slave
	reg [31:0] HADDR = 32'h0;	// address
	reg [1:0] HTRANS = 2'b0;	// transaction type (only two types used)
	reg HWRITE = 1'b0;			// write transaction
	reg [2:0] HSIZE = 3'b0;		// transaction width (max 32-bit supported)
	reg [31:0] HWDATA = 32'h0;	// write data
	reg [2:0] HBURST = 3'b0;	// burst type
	reg [3:0] HPROT = 4'b0011;	// privileged data transfer
	wire [31:0] HRDATA;			// read data from slave
    wire HREADY;             	// ready signal - to master and to all slaves
    wire HREADYOUT;         	// ready signal output from this slave

// GPIO input and output signals
	reg [15:0] gpio_in0 = 16'h5555, gpio_in1 = 16'haaaa;	// input ports with initial values
	wire [15:0] gpio_out0, gpio_out1;	// output ports

   
// Define names for some of the bus signal values and for the device registers
	localparam [2:0] BYTE = 3'b000, HALF = 3'b001, WORD = 3'b010;			// HSIZE values
	localparam [1:0] IDLE = 2'b00, NONSEQ = 2'b10;							// HTRANS values
	localparam [31:0] OUT0 = 32'h0, OUT1 = 32'h4, IN0 = 32'h8, IN1 = 32'hc;	// word addresses
	localparam [31:0] OUT0L = 32'h0, OUT0H = 32'h1, OUT1L = 32'h4, OUT1H = 32'h5;	// byte addresses

// Instantiate the design under test and connect it to the testbench signals
	AHBgpio dut(
		.HCLK(HCLK),
		.HRESETn(HRESETn),
		.HSEL(HSELx),
		.HREADY(HREADY),
		.HADDR(HADDR),
		.HTRANS(HTRANS),
		.HWRITE(HWRITE),  
		.HSIZE(HSIZE), 
		.HWDATA(HWDATA),  
		.HRDATA(HRDATA),  
		.HREADYOUT(HREADYOUT),
		.gpio_out0(gpio_out0),
		.gpio_out1(gpio_out1),
		.gpio_in0(gpio_in0),
		.gpio_in1(gpio_in1)
		);

// Generate the clock signal at 50 MHz - period 20 ns
	initial
		begin
			HCLK = 1'b0;
			forever
				#10 HCLK = ~HCLK;  // invert clock every 10 ns
		end

// Generate reset pulse and simulate some bus transactions to implement verification plan
	initial
		begin
			HRESETn = 1'b1;			// reset inactive at start
			#20 HRESETn = 1'b0;		// reset active on falling edge of clock
			#20 HRESETn = 1'b1;		// inactive after one clock cycle
			#50;					// delay to see what happens
			AHBwrite(WORD, OUT0, 32'h1a2b3c4d);	// write word to address for output 0 register
			AHBwrite(WORD, OUT1, 32'h12345678); // write word to output 1 register
			AHBread (WORD, OUT1, 32'h00005678);	// read output 1 register, expect 16 bits of value written
			AHBwrite(WORD, IN0, 32'hf0f0f1d8);	// read-only register, write should have no effect
			AHBread (WORD, IN0, 32'h00005555);	// read in0 port, should still be 5555
			AHBidle;	// must put bus into idle state unless there is another transaction following
			repeat(5)
				@ (posedge HCLK);		// wait 5 clock cycles
			gpio_in0 = 16'h4321;		// change GPIO input values
			gpio_in1 = 16'hdcba;
			AHBread (WORD, OUT0, 32'h00003c4d);	// read all registers
			AHBread (WORD, IN0, 32'h00001234);	// wrong expected data to show error counting
			AHBread (WORD, IN1, 32'h0000dcba);
			AHBread (WORD, OUT1, 32'h00005678);
			AHBidle;
			repeat(5)
				@ (posedge HCLK);		// wait 5 clock cycles
			AHBwrite(BYTE, OUT0H, 32'h00005500);	// test the byte write facility
			AHBwrite(BYTE, OUT1L, 32'h000000ee);	// only half of each register should change
			AHBread (BYTE, OUT1L, 32'h000056ee);	// read to check the result
			AHBread (WORD, OUT0, 32'h0000554d);
			AHBidle;
			#50;			// wait a while to allow the last transaction to complete
			$stop;			// stop the simulation
		end

// =========== AHB bus tasks - crude models of bus activity =========================
// To use these tasks, include everything below this line, until the next ===== line
// Read and Write tasks do not restore the bus to idle, as another transaction might follow.
// Use Idle task immediately after read or write if no transaction follows immediately.

	reg [31:0] nextWdata = 32'h0;		// delayed data for write transactions
	reg [31:0] expectRdata = 32'h0;		// expected read data for read transactions
	reg [31:0] rExpectRead;				// store expected read data
	reg checkRead;						// remember that read is in progress
	reg transState;						// state of our transaction - 1 if in data phase
	reg error = 1'b0;  // read error signal - asserted for one cycle AFTER read completes
	integer errCount = 0;				// error counter
    
// Task to simulate a write transaction on AHB Lite
	task AHBwrite ( 
			input [2:0] size,	// transaction width - BYTE, HALF or WORD
			input [31:0] addr,	// address
			input [31:0] data );	// data to be written
		begin
			wait (HREADY == 1'b1);	// wait for ready signal - previous transaction completing
			@ (posedge HCLK);	// align with clock
			#1 HSIZE = size;	// set up signals for address phase, just after clock edge
			HTRANS = NONSEQ;	// transaction type non-sequential
			HWRITE = 1'b1;		// write transaction
			HADDR = addr;		// put address on bus
			HSELx = 1'b1;		// select this slave
			#1 nextWdata = data;	// a little later, store data for use in the data phase
		end
	endtask

// Task to simulate a read transaction on AHB Lite
	task AHBread (
			input [2:0] size,	// transaction width - BYTE, HALF or WORD
			input [31:0] addr,	// address
			input [31:0] data );	// expected data from slave
		begin  
			wait (HREADY == 1'b1);	// wait for ready signal - previous transaction completing
			@ (posedge HCLK);	// align with clock
			#1 HSIZE = size;	// set up signals for address phase, just after clock edge
			HTRANS = NONSEQ;	// transaction type non-sequential
			HWRITE = 1'b0;		// read transaction
			HADDR = addr;		// put address on bus
			HSELx = 1'b1;		// select this slave
			#1 expectRdata = data;	// a little later, store expected data for checking in the data phase
		end
	endtask

// Task to put bus in idle state after read or write transaction
	task AHBidle;
		begin  
			wait (HREADY == 1'b1); // wait for ready signal - previous transaction completing
			@ (posedge HCLK);	// then wait for clock edge
			#1 HTRANS = IDLE;	// set transaction type to idle
			HSELx = 1'b0;		// deselect the slave
		end
	endtask

// Control the HWDATA signal during the data phase
	always @ (posedge HCLK)
		if (~HRESETn) HWDATA <= 32'b0;
		else if (HSELx && HWRITE && HTRANS && HREADY) // our write transaction is moving to data phase
			#1 HWDATA <= nextWdata;	// change HWDATA shortly after the clock edge
		else if (HREADY)	// some other transaction in progress
			#1 HWDATA <= {HADDR[31:24], HADDR[11:0], 12'hbad}; // put rubbish on HWDATA

// Register to hold expected read data during data phase, and flag to indicate that read is in progress
	always @ (posedge HCLK)
		if (~HRESETn)
			begin
				rExpectRead <= 32'b0;
				checkRead <= 1'b0;
			end
		else if (HSELx && ~HWRITE && HTRANS && HREADY)  // our read transaction moving to data phase
			begin
				rExpectRead <= expectRdata;	// update register with expected data
				checkRead <=1'b1;			// set flag to get it checked
			end
		else if (HREADY)	// some other transaction moving to data phase
				checkRead <= 1'b0;			// clear flag - no check needed

// Check the read data as the read transaction completes
// error signal will be asserted for one cycle AFTER problem detected
	always @ (posedge HCLK)
		if (~HRESETn) error <= 1'b0;
		else if (checkRead & HREADY)	// our read transaction is completing on this clock edge
			if (HRDATA != rExpectRead)	// the read data is not as expected
				begin
					error <= 1'b1;		// so flag this as an error
					errCount = errCount + 1;	// and increment the error counter
				end
			else error <= 1'b0;			// otherwise our read transaction is OK
		else		// this is some other transaction 
			error <= 1'b0;	// so no error 
			
// Control the HREADY signal during the data phase
	always @ (posedge HCLK)
		if (~HRESETn) transState <= 1'b0;	// after reset, this is not the data phase of our transaction
		else if (HSELx && HTRANS && HREADY) // transaction with this slave is moving to data phase
			#1 transState <= 1'b1;			// so this slave controls HREADY
		else if (HREADY)					// some other transaction is moving to data phase
			#1 transState <= 1'b0;			// some other slave controls HREADY
			
	assign HREADY = transState ? HREADYOUT : 1'b1;     // other slave is always ready

//============================= END of AHB bus tasks =========================================
		
endmodule
