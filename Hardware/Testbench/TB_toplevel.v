`timescale 1ns / 1ns
/////////////////////////////////////////////////////////////////
// Module Name: TB_toplevel
// Simple testbench for SoC - no program load, just clock and reset
/////////////////////////////////////////////////////////////////
module TB_toplevel(    );
     
    reg btnCpuResetn, clk100, btnU; 
	reg [15:0] sw;		// switch inputs
    wire [15:0] LED;
    wire RsRx = 1'b1;		// serial receive at idle
    wire RsTx;        // serial transmit
     
    AHBliteTop dut(
        .clk(clk100), 
        .btnCpuResetn(btnCpuResetn),
        .btnU(btnU),
        .RsRx(RsRx),
		.sw(sw),
        .led(LED), 
        .RsTx(RsTx)
         );
         
 
   initial
    begin
        clk100 = 0;
        forever	// generate 100 MHz clock
        begin
          #5 clk100 = ~clk100;  // invert clock every 5 ns
         end
    end

    initial
    begin
		sw = 16'h5a4b;			// set a value on the switches
        btnCpuResetn = 1'b1;		// start with reset inactive
        btnU = 1'b0;				// loader button not pressed
        #30 btnCpuResetn = 1'b0;    // active low reset
        #70 btnCpuResetn = 1'b1;    // release reset
        
        #20000;      // delay 20 us or 1000 clock cycles

		$stop;
        
      end
     
     
        
endmodule
