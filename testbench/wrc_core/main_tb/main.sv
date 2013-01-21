`timescale 1ns/1ps

   
`include "tbi_utils.sv"

`include "simdrv_defs.svh"
`include "if_wb_master.svh"
`include "wrc_syscon_regs.vh"

`define BASE_SYSCON 'h40400

module main;

   wire clk_ref;
   wire clk_sys;
   wire rst_n;

   IWishboneMaster WB (
      .clk_i(clk_sys),
      .rst_n_i(rst_n)); 
   tbi_clock_rst_gen
     #(
       .g_rbclk_period(8000))
     clkgen(
	    .clk_ref_o(clk_ref),
	    .clk_sys_o(clk_sys),
	    .phy_rbclk_o(phy_rbclk),
	    .rst_n_o(rst_n)
	    );

   wire clk_sys_dly;

   assign  #10 clk_sys_dly  = clk_sys;
    wire   [7:0]phy_tx_data      ;
   wire   phy_tx_k         ;
   wire   phy_tx_disparity ;
   wire   phy_tx_enc_err   ;
   wire   [7:0]phy_rx_data      ;
   wire   phy_rx_rbclk     ;
   wire   phy_rx_k         ;
   wire   phy_rx_enc_err   ;
   wire   [3:0]phy_rx_bitslide  ;
   wire   phy_rst          ;
   wire   phy_loopen;

   wr_core #(
             .g_simulation             (1),
             .g_dpram_initf            ("sw/main.ram"),
             .g_dpram_size             (16384)
    )
   DUT (
	.clk_sys_i      (clk_sys),
	.clk_dmtd_i     (clk_ref),
	.clk_ref_i      (clk_ref),
  .clk_aux_i      (clk_ref),
	.rst_n_i        (rst_n),

	.pps_p_o        (),

	.dac_hpll_load_p1_o (),
	.dac_hpll_data_o (),

	.dac_dpll_load_p1_o (),
	.dac_dpll_data_o (),

	.uart_rxd_i       (1'b0),
	.uart_txd_o       (),

        .scl_i(scl_loop),
        .scl_o(scl_loop),

        .sda_i(sda_loop),
        .sda_o(sda_loop),

        .btn1_i(1'b0),
        .btn2_i(1'b0),
        
	.wb_adr_i      (WB.master.adr[31:0]),
	.wb_dat_i      (WB.master.dat_o),
	.wb_dat_o      (WB.master.dat_i),
	.wb_sel_i       (4'b1111),
	.wb_we_i        (WB.master.we),
	.wb_cyc_i       (WB.master.cyc),
	.wb_stb_i       (WB.master.stb),
	.wb_ack_o       (WB.master.ack),
  .wb_stall_o     (WB.master.stall),

        .phy_ref_clk_i(clk_ref),
        .phy_tx_data_o(phy_tx_data),
        .phy_tx_k_o(phy_tx_k),
        .phy_tx_disparity_i(phy_tx_disparity),
        .phy_tx_enc_err_i(phy_tx_enc_err),
        .phy_rx_data_i(phy_rx_data),
        .phy_rx_rbclk_i(phy_rx_rbclk),
        .phy_rx_k_i(phy_rx_k),
        .phy_rx_enc_err_i(phy_rx_enc_err),
        .phy_rx_bitslide_i(phy_rx_bitslide),
        .phy_rst_o(phy_rst),
        .phy_loopen_o(phy_lo)

        );

   assign phy_rx_data       = phy_tx_data;
   assign phy_rx_k          = phy_tx_k;
   assign phy_tx_disparity  = 0;
   assign phy_tx_enc_err    = 0;
   assign phy_rx_enc_err    = 0;
   

   initial begin
      CWishboneAccessor acc;
      
      @(posedge rst_n);
      repeat(3) @(posedge clk_sys);

      #1us;
      
      
      acc  = WB.get_accessor();
      
      acc.set_mode(PIPELINED);
      #1us;
      acc.write(`BASE_SYSCON + `ADDR_SYSC_RSTR, 'hdeadbee | `SYSC_RSTR_RST);
      

      #1us;
      acc.write(`BASE_SYSCON + `ADDR_SYSC_RSTR, 'hdeadbee );
      
      
   end
   
   
endmodule // main

