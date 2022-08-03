// Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module CORTEXM0DS(HCLK, HRESETn, HADDR, HBURST, HMASTLOCK, HPROT, HSIZE, HTRANS, HWDATA, HWRITE, HRDATA, HREADY, HRESP, NMI, IRQ, TXEV, RXEV, LOCKUP, SYSRESETREQ, SLEEPING);
  input HCLK;
  input HRESETn;
  output [31:0]HADDR;
  output [2:0]HBURST;
  output HMASTLOCK;
  output [3:0]HPROT;
  output [2:0]HSIZE;
  output [1:0]HTRANS;
  output [31:0]HWDATA;
  output HWRITE;
  input [31:0]HRDATA;
  input HREADY;
  input HRESP;
  input NMI;
  input [15:0]IRQ;
  output TXEV;
  input RXEV;
  output LOCKUP;
  output SYSRESETREQ;
  output SLEEPING;
endmodule
