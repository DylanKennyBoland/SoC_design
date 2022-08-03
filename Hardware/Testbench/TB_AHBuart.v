`timescale 1ns / 1ns
/////////////////////////////////////////////////////////////////
// Module Name: TB_AHBuart - testbench for AHB uart block
/////////////////////////////////////////////////////////////////
module TB_AHBuart(    );

// AHB-Lite Bus Signals
	reg HCLK;					// bus clock
	reg HRESETn;				// bus reset, active low
	reg HSELx = 1'b0;			// selects this slave
	reg [31:0] HADDR = 32'h0;	// address
	reg [1:0] HTRANS = 2'b0;	// transaction type (only two tpyes used)
	reg HWRITE = 1'b0;			// write transaction
	reg [2:0] HSIZE = 3'b0;		// transaction width (max 32-bit supported)
	reg [31:0] HWDATA = 32'h0;	// write data
	reg [2:0] HBURST = 3'b0;	// burst type
	reg [3:0] HPROT = 4'b0011;	// privileged data transfer
	wire [31:0] HRDATA;			// read data from slave
    wire HREADY;             	// ready signal - to master and to all slaves
    wire HREADYOUT;             // ready signal output from this slave

// Other signals to connect to the module under test
	wire serialRx, serialTx, uart_IRQ;
	assign serialRx = serialTx;  // loopback connection for testing

// Integer is used in for loops
	integer i = 0;

// Define names for some of the bus signal values and for the device registers
	localparam [2:0] BYTE = 3'b000, HALF = 3'b001, WORD = 3'b010;	// HSIZE values
	localparam [1:0] IDLE = 2'b00, NONSEQ = 2'b10;					// HTRANS values
	localparam [31:0] RXDATA = 32'h0, TXDATA = 32'h4, STATUS = 32'h8, CONTRL = 32'hc;	// registers

// Instantiate the design under test and connect it to the testbench signals
// Some bus signals are not used - this design ignores HSIZE, for example
	AHBuart dut(
		.HCLK(HCLK),
		.HRESETn(HRESETn),
		.HSEL(HSELx),
		.HREADY(HREADY),
		.HADDR(HADDR),
		.HTRANS(HTRANS),
		.HWRITE(HWRITE),  
		.HWDATA(HWDATA),  
		.HRDATA(HRDATA),  
		.HREADYOUT(HREADYOUT),
		.serialRx(serialRx),
		.serialTx(serialTx),
		.uart_IRQ(uart_IRQ)
		);

// Generate the clock signal at 50 MHz - period 20 ns
	initial
		begin
			HCLK = 1'b0;
			forever
				#10 HCLK = ~HCLK;  // invert clock every 10 ns
		end

// Generate reset pulse and simulate some bus transactions
	initial
		begin
			HRESETn = 1'b1;			// reset inactive at start
			#20 HRESETn = 1'b0;		// reset active on falling edge of clock
			#20 HRESETn = 1'b1;		// inactive after one clock cycle
			#50;					// delay to see what happens
			AHBwrite(WORD, RXDATA, 32'h1a2b3c4d);	// write to address 0 - read only register
			AHBidle;	// need to put bus in idle state unless there is another transaction following
			#20;		// leave a short gap
			AHBwrite(WORD, TXDATA, 32'h12345678);	// transmit data - only 8 bits are used
			AHBread (WORD, TXDATA, 32'h78);			// read back data
			AHBwrite(WORD, CONTRL, 32'hc);	// enable rx interrupts
			AHBread (WORD, STATUS, 32'h2);	// read status: expect tx buffer empty, rx buffer empty
			AHBidle;	// need to put bus in idle state unless there is another transaction following
			wait (uart_IRQ == 1'b1);		// wait for interrupt when first byte has been received
			AHBread (WORD, STATUS, 32'ha);	// read status: expect rx buffer not empty, tx empty
            AHBread (WORD, RXDATA, 32'h78);    // read received data - should be the first byte sent
			AHBidle;	// need to put bus in idle state unless there is another transaction following
            #40;        // leave a short gap
			AHBwrite(WORD, TXDATA, 32'h56);	// send more data
			AHBwrite(WORD, TXDATA, 32'h34);
			AHBread (WORD, STATUS, 32'h0);	// read status: expect tx buffer not empty, rx still empty
			AHBread (WORD, CONTRL, 32'hC);	// readback of control register
			for (i=0; i<20; i=i+1)
				AHBwrite(WORD, TXDATA, i+20);	// transmit data to fill the transmit buffer
			AHBread (WORD, STATUS, 32'h1);	// read status: expect tx full, rx still empty
			AHBidle;
			wait (uart_IRQ == 1'b1);		// wait for interrupt when a byte has been received
			AHBread (WORD, STATUS, 32'h8);	// read status: expect rx buffer not empty, tx no longer full
			AHBread (WORD, RXDATA, 32'h56);	// read received data - should be the first byte sent
			AHBidle;
			@ (posedge uart_IRQ );			// another way to wait for interrupt
			AHBread (WORD, STATUS, 32'h8);	// read status: expect rx buffer not empty, tx not full
			AHBread (WORD, RXDATA, 32'h34);	// read received data, expect second byte send
			AHBidle;
			@ (posedge uart_IRQ );			// wait for interrupt
			AHBread (WORD, STATUS, 32'h8);	// read status: expect both buffers have some data
			AHBread (WORD, RXDATA, 32'd20);	// read received data - expect third byte
			AHBread (WORD, STATUS, 32'h0);	// read status: expect tx buffer has data, rx empty
			AHBwrite(WORD, CONTRL, 32'h2);	// enable tx buffer empty interrupt only
			AHBidle;
			@ (posedge uart_IRQ );			// wait for interrupt - tx buffer is now empty
			AHBread (WORD, STATUS, 32'ha);	// read status: tx buffer empty, rx buffer has data
			AHBread (WORD, RXDATA, 32'd21);	// read received data - expect fourth byte
			AHBidle;
			#5000;							// delay to allow actions to complete
			$stop;							// stop the simulation
		end


// =========== AHB bus tasks - crude models of bus activity =========================
// To use these tasks, include everything below this line, until the next ===== line
// Read and Write tasks do not restore the bus to idle, as another transaction might follow.
// Use Idle task immediately after read or write if no transaction follows immediately.

    reg [31:0] nextWdata = 32'h0;        // delayed data for write transactions
    reg [31:0] expectRdata = 32'h0;        // expected read data for read transactions
    reg [31:0] rExpectRead;                // store expected read data
    reg checkRead;                        // remember that read is in progress
    reg transState;                        // state of our transaction - 1 if in data phase
    reg error = 1'b0;  // read error signal - asserted for one cycle AFTER read completes
    integer errCount = 0;                // error counter
    
// Task to simulate a write transaction on AHB Lite
    task AHBwrite ( 
            input [2:0] size,    // transaction width - BYTE, HALF or WORD
            input [31:0] addr,    // address
            input [31:0] data );    // data to be written
        begin
            wait (HREADY == 1'b1);    // wait for ready signal - previous transaction completing
            @ (posedge HCLK);    // align with clock
            #1 HSIZE = size;    // set up signals for address phase, just after clock edge
            HTRANS = NONSEQ;    // transaction type non-sequential
            HWRITE = 1'b1;        // write transaction
            HADDR = addr;        // put address on bus
            HSELx = 1'b1;        // select this slave
            #1 nextWdata = data;    // a little later, store data for use in the data phase
        end
    endtask

// Task to simulate a read transaction on AHB Lite
    task AHBread (
            input [2:0] size,    // transaction width - BYTE, HALF or WORD
            input [31:0] addr,    // address
            input [31:0] data );    // expected data from slave
        begin  
            wait (HREADY == 1'b1);    // wait for ready signal - previous transaction completing
            @ (posedge HCLK);    // align with clock
            #1 HSIZE = size;    // set up signals for address phase, just after clock edge
            HTRANS = NONSEQ;    // transaction type non-sequential
            HWRITE = 1'b0;        // read transaction
            HADDR = addr;        // put address on bus
            HSELx = 1'b1;        // select this slave
            #1 expectRdata = data;    // a little later, store expected data for checking in the data phase
        end
    endtask

// Task to put bus in idle state after read or write transaction
    task AHBidle;
        begin  
            wait (HREADY == 1'b1); // wait for ready signal - previous transaction completing
            @ (posedge HCLK);    // then wait for clock edge
            #1 HTRANS = IDLE;    // set transaction type to idle
            HSELx = 1'b0;        // deselect the slave
        end
    endtask

// Control the HWDATA signal during the data phase
    always @ (posedge HCLK)
        if (~HRESETn) HWDATA <= 32'b0;
        else if (HSELx && HWRITE && HTRANS && HREADY) // our write transaction is moving to data phase
            #1 HWDATA <= nextWdata;    // change HWDATA shortly after the clock edge
        else if (HREADY)    // some other transaction in progress
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
                rExpectRead <= expectRdata;    // update register with expected data
                checkRead <=1'b1;            // set flag to get it checked
            end
        else if (HREADY)    // some other transaction moving to data phase
                checkRead <= 1'b0;            // clear flag - no check needed

// Check the read data as the read transaction completes
// error signal will be asserted for one cycle AFTER problem detected
    always @ (posedge HCLK)
        if (~HRESETn) error <= 1'b0;
        else if (checkRead & HREADY)    // our read transaction is completing on this clock edge
            if (HRDATA != rExpectRead)    // the read data is not as expected
                begin
                    error <= 1'b1;        // so flag this as an error
                    errCount = errCount + 1;    // and increment the error counter
                end
            else error <= 1'b0;            // otherwise our read transaction is OK
        else        // this is some other transaction 
            error <= 1'b0;    // so no error 
            
// Control the HREADY signal during the data phase
    always @ (posedge HCLK)
        if (~HRESETn) transState <= 1'b0;    // after reset, this is not the data phase of our transaction
        else if (HSELx && HTRANS && HREADY) // transaction with this slave is moving to data phase
            #1 transState <= 1'b1;            // so this slave controls HREADY
        else if (HREADY)                    // some other transaction is moving to data phase
            #1 transState <= 1'b0;            // some other slave controls HREADY
            
    assign HREADY = transState ? HREADYOUT : 1'b1;     // other slave is always ready

//============================= END of AHB bus tasks =========================================

endmodule
