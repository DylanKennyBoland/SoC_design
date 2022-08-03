// Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "blk_mem_gen_v8_2,Vivado 2015.2" *)
module blk_mem_4Kword(clka, wea, addra, dina, clkb, addrb, doutb);
  input clka;
  input [3:0]wea;
  input [11:0]addra;
  input [31:0]dina;
  input clkb;
  input [11:0]addrb;
  output [31:0]doutb;
endmodule
