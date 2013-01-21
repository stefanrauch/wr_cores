//
// Title          : Software Wishbone master unit for testbenches
//
// File           : wishbone_master_tb.v
// Author         : Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
// Created        : Tue Mar 23 12:19:36 2010
// Standard       : Verilog 2001
//


// Default values of certain WB parameters.

`timescale 1ns/1ps

// Bus clock period
`ifndef WB_CLOCK_PERIOD
`define WB_CLOCK_PERIOD 16
`define WB_RESET_DELAY (3*`WB_CLOCK_PERIOD)
`endif

// Widths of wishbone address/data/byte select
`ifndef WB_DATA_BUS_WIDTH
 `define WB_DATA_BUS_WIDTH 32
`endif

`ifndef WB_ADDRESS_BUS_WIDTH
 `define WB_ADDRESS_BUS_WIDTH 32
`endif


`define WB_BWSEL_WIDTH ((`WB_DATA_BUS_WIDTH + 7) / 8)

module WB_TEST_MASTER (
`ifdef WB_USE_EXTERNAL_CLOCK		       
		       input clk_i, 
		       input rst_n_i 
`endif
);
// these signals make the WB bus, which can be accessed from outside the module
   reg [`WB_ADDRESS_BUS_WIDTH - 1 : 0] wb_addr = 0;
   reg [`WB_DATA_BUS_WIDTH - 1 : 0]    wb_data_o = 0;
   reg [`WB_BWSEL_WIDTH - 1 : 0]       wb_bwsel = 0;
   wire [`WB_DATA_BUS_WIDTH -1 : 0]       wb_data_i;
   wire wb_ack;
   reg 	wb_cyc 	       = 0;
   reg 	wb_stb 	       = 0;
   reg 	wb_we 	       = 0;
   reg 	rst_reg        = 0;
   reg 	clk_reg        = 1;

   wire wb_clk, wb_rst;
   

   
   reg wb_tb_verbose   = 1;
   reg wb_monitor_bus  = 1;
   time last_access_t  = 0;
   reg [`WB_DATA_BUS_WIDTH -1 : 0] dummy;

// ready signal. 1 indicates that WB_TEST unit is initialized and ready for commands
   reg ready = 0;

`ifndef WB_USE_EXTERNAL_CLOCK
   
// generate the WB bus clock
   always #(`WB_CLOCK_PERIOD/2) clk_reg <= ~clk_reg;

// generate the reset and ready signals
   initial begin
      #(`WB_RESET_DELAY) rst_reg <= 1;
      #(`WB_CLOCK_PERIOD*2) ready <= 1;
      end

   assign wb_clk  = clk_reg;
   assign wb_rst  = rst_reg;
   
`else // !`ifdef WB_USE_OWN_CLOCK

   assign wb_clk  = clk_i;
   assign wb_rst  = rst_n_i;


   initial begin repeat(3) @(posedge wb_clk); ready = 1; end
      
       
   
`endif // !`ifdef WB_USE_OWN_CLOCK
   
   
// enables/disables displaying information about each read/write operation.
   task verbose;
      input onoff;
      begin
	 wb_tb_verbose = onoff;
      end
   endtask // wb_verbose

   task monitor_bus;
      input onoff;
      begin
	 wb_monitor_bus = onoff;
      end

   endtask // monitor_bus
   
   task rw_generic;
      input [`WB_ADDRESS_BUS_WIDTH - 1 : 0] addr;
      input [`WB_DATA_BUS_WIDTH - 1 : 0] data_i;
      output [`WB_DATA_BUS_WIDTH - 1 : 0] data_o;
      input rw;
      input [3:0] size;
      begin : rw_generic_main
	 if(wb_tb_verbose && rw) 
	   $display("WB write %s: addr %x, data %x",
		    (size==1?"byte":((size==2)?"short":"int")), 
		    addr, data_i);

	 if($time != last_access_t) begin
	    @(posedge wb_clk);
	 end
	 	 
	 wb_stb<=1;
	 wb_cyc<=1;
	 wb_addr <= {2'b00, addr[31:2]};
	 wb_we <= rw;

	 if(rw) begin
	    case(size)
	      4: begin wb_data_o<=data_i; wb_bwsel <= 4'b1111; end
		 
	      2: begin
		 
		 if(addr[1]) begin
		    wb_data_o[31:16] <= data_i[15:0];
		    wb_bwsel <= 4'b1100;
		 end else begin
		    wb_data_o[15:0] <= data_i[15:0];
		    wb_bwsel <= 4'b0011;
		 end
	      end
	      1: begin
		 case(addr[1:0])
		   0: begin wb_data_o[31:24] <= data_i[7:0]; wb_bwsel <= 4'b1000; end
		   1: begin wb_data_o[23:16] <= data_i[7:0]; wb_bwsel <= 4'b0100; end
		   2: begin wb_data_o[15:8] <= data_i[7:0]; wb_bwsel <= 4'b0010; end
		   3: begin wb_data_o[7:0] <= data_i[7:0]; wb_bwsel <= 4'b0001; end
		 endcase // case(addr[1:0])
	      end
	     
	    endcase // case(size)
	 end // if (rw)
	 

	 @(posedge wb_clk);
	 	 
	 if(wb_ack == 0) begin
	    while(wb_ack == 0) begin @(posedge wb_clk); end
	 end

	 data_o = wb_data_i;
 	 wb_cyc <= 0;
	 wb_we<=0;
	 wb_stb<=0;

	 if(wb_tb_verbose && !rw) 
	   $display("WB read %s: addr %x, data %x",
		    (size==1?"byte":((size==2)?"short":"int")), 
		    addr, wb_data_i);


	 last_access_t = $time;
      end
   endtask // rw_generic

   task write8;
      input [`WB_ADDRESS_BUS_WIDTH - 1 : 0] addr;
      input [7 : 0] data_i;
      begin
	 rw_generic(addr, data_i, dummy, 1, 1);
      end
   endtask // write8

   task read8;
      input [`WB_ADDRESS_BUS_WIDTH - 1 : 0] addr;
      output [7 : 0] data_o;
      begin : read8_body
	 reg [`WB_DATA_BUS_WIDTH - 1 : 0] rval;
	 rw_generic(addr, 0, rval, 0, 1);
	 data_o = rval[7:0];
      end
   endtask // write8

   task write32;
      input [`WB_ADDRESS_BUS_WIDTH - 1 : 0] addr;
      input [31 : 0] data_i;
      begin
	 rw_generic(addr, data_i, dummy, 1, 4);
      end
   endtask // write32

   task read32;
      input [`WB_ADDRESS_BUS_WIDTH - 1 : 0] addr;
      output [31 : 0] data_o;
      begin : read32_body
	 reg [`WB_DATA_BUS_WIDTH - 1 : 0] rval;
	 rw_generic(addr, 0, rval, 0, 4);
	 data_o = rval[31:0];
      end
   endtask // write32

// bus monitor   
   always@(posedge wb_clk) begin
      if(wb_monitor_bus && wb_cyc && wb_stb && wb_ack)begin
	 if(wb_we) $display("ACK-Write: addr %x wdata %x bwsel %b", wb_addr, wb_data_o, wb_bwsel);
	 else $display("ACK-Read: addr %x rdata %x", wb_addr, wb_data_i);
      end
   end


   
endmodule