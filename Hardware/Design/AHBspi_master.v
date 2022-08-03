`timescale 1ns / 1ns

module AHBspi_master(
			// Bus signals
			input wire HCLK,			// bus clock
			input wire HRESETn,			// bus reset, active low
			input wire HSEL,			// selects this slave
			input wire HREADY,			// indicates previous transaction completing
			input wire [31:0] HADDR,	// address
			input wire [1:0] HTRANS,	// transaction type (only bit 1 used)
			input wire HWRITE,			// write transaction
//			input wire [2:0] HSIZE,		// transaction width ignored
			input wire [31:0] HWDATA,	// write data
			output wire [31:0] HRDATA,	// read data from slave
            output reg MOSI,           // to slaves
            output wire SCLK,           // to slaves
			input wire MISO,            // from selected slave
            output wire ACCELEROMETER_SELECT_N,	// to accelerometer slave
            output wire HREADYOUT		
    );
	
	// Registers to hold signals from address phase
	reg [1:0] rHADDR;	// only need two bits of address
	reg rWrite;	// write enable signals
	wire send = (rWrite && (rHADDR == 2'b01) && ~busy);

 	// Capture bus signals in address phase
	always @(posedge HCLK)
		if(!HRESETn)
			begin
				rHADDR <= 2'b0;
				rWrite <= 1'b0;
			end
		else if(HREADY)
		 begin
			rHADDR <= HADDR[3:2];                   // capture address bits for for use in data phase
			rWrite <= HSEL & HWRITE & HTRANS[1];	// slave selected for write transfer       
		 end

    assign HREADYOUT = 1'b1;
    // COUNT is an 8 bit counter where 
    // COUNT[6:4] are used to select MOSI and MISO bits
    // Also, busy and SCLK wires epend on COUNT[7] and COUNT[3] respectively

    wire busy;
    assign busy = ~COUNT[7];
    assign SCLK = COUNT[3];

    reg [7:0] NEXT_COUNT;
	reg [7:0] COUNT;	// holds interrupt enable bits
	
	wire shift = (COUNT[3:0] == 4'b0111);	// shift at posedge of SCLK

	always @(posedge HCLK)
	begin
       if (!HRESETn) COUNT <= 8'h80;  // reset to max value
       else COUNT <= NEXT_COUNT;
     end
	 
	always @( * )
	begin
       if (busy) NEXT_COUNT = COUNT + 1'b1;
	   else if (send) NEXT_COUNT = 8'b0;
	   else NEXT_COUNT = 8'h80;
    end
		
		
	// SPICON register
    reg ACCELEROMETER_SELECT_REG;
	wire [7:0] SPICON;	// holds interrupt enable bits

	always @(posedge HCLK)
	begin
        if (!HRESETn) 
            begin
                ACCELEROMETER_SELECT_REG <= 1'b0;    
            end
        else
            begin
                if (rWrite && (rHADDR == 2'b00) && ~busy) ACCELEROMETER_SELECT_REG <= HWDATA[0]; 
            end
     end
	 
	 assign SPICON = {busy, 6'b0, ACCELEROMETER_SELECT_REG};

    assign ACCELEROMETER_SELECT_N = ~ACCELEROMETER_SELECT_REG;

	// TX
	reg [7:0] TX_DATA;	// holds interrupt enable bits

	
	
	always @(posedge HCLK)
        if (!HRESETn) TX_DATA <= 8'b0;   
        else if (send) TX_DATA <= HWDATA[7:0]; 
    
    always @( * )
        case (COUNT[6:4])
            3'b000:  MOSI = TX_DATA[7];
            3'b001:  MOSI = TX_DATA[6];
            3'b010:  MOSI = TX_DATA[5];
            3'b011:  MOSI = TX_DATA[4];
            3'b100:  MOSI = TX_DATA[3];
            3'b101:  MOSI = TX_DATA[2];
            3'b110:  MOSI = TX_DATA[1];
            3'b111:  MOSI = TX_DATA[0];
        endcase

    // Read data
    reg [7:0] READ_DATA;
    reg[7:0] SHIFT_REG;
    
    always @( * )
    begin
        case (rHADDR)
            2'b00:  READ_DATA = SPICON;
            2'b01:  READ_DATA = TX_DATA;
            2'b10:  READ_DATA = SHIFT_REG;
            2'b11:  READ_DATA = SPICON;
        endcase
        
     end
     
     assign HRDATA = {24'b0, READ_DATA};
	
	
	
	// shift register	
	always @(posedge HCLK)
	begin
	if (!HRESETn) SHIFT_REG <= 8'b0;
	else if (shift) SHIFT_REG <= {SHIFT_REG[6:0],MISO};
	end

endmodule
