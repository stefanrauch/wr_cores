library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package ddr3_mem_pkg is
  component ddr3_mem_phy
	PORT (
		pll_ref_clk	: IN STD_LOGIC;
		global_reset_n	: IN STD_LOGIC;
		soft_reset_n	: IN STD_LOGIC;
		ctl_dqs_burst	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
		ctl_wdata_valid	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
		ctl_wdata	: IN STD_LOGIC_VECTOR (63 DOWNTO 0);
		ctl_dm	: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		ctl_addr	: IN STD_LOGIC_VECTOR (25 DOWNTO 0);
		ctl_ba	: IN STD_LOGIC_VECTOR (5 DOWNTO 0);
		ctl_cas_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_cke	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_cs_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_odt	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_ras_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_we_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_rst_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_mem_clk_disable	: IN STD_LOGIC_VECTOR (0 DOWNTO 0);
		ctl_doing_rd	: IN STD_LOGIC_VECTOR (3 DOWNTO 0);
		ctl_cal_req	: IN STD_LOGIC;
		ctl_cal_byte_lane_sel_n	: IN STD_LOGIC_VECTOR (1 DOWNTO 0);
		oct_ctl_rs_value	: IN STD_LOGIC_VECTOR (13 DOWNTO 0);
		oct_ctl_rt_value	: IN STD_LOGIC_VECTOR (13 DOWNTO 0);
		dqs_offset_delay_ctrl	: IN STD_LOGIC_VECTOR (5 DOWNTO 0);
		dqs_delay_ctrl_import	: IN STD_LOGIC_VECTOR (5 DOWNTO 0);
		dbg_clk	: IN STD_LOGIC;
		dbg_reset_n	: IN STD_LOGIC;
		dbg_addr	: IN STD_LOGIC_VECTOR (12 DOWNTO 0);
		dbg_wr	: IN STD_LOGIC;
		dbg_rd	: IN STD_LOGIC;
		dbg_cs	: IN STD_LOGIC;
		dbg_wr_data	: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		reset_request_n	: OUT STD_LOGIC;
		ctl_clk	: OUT STD_LOGIC;
		ctl_reset_n	: OUT STD_LOGIC;
		ctl_wlat	: OUT STD_LOGIC_VECTOR (4 DOWNTO 0);
		ctl_rdata	: OUT STD_LOGIC_VECTOR (63 DOWNTO 0);
		ctl_rdata_valid	: OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		ctl_rlat	: OUT STD_LOGIC_VECTOR (4 DOWNTO 0);
		ctl_cal_success	: OUT STD_LOGIC;
		ctl_cal_fail	: OUT STD_LOGIC;
		ctl_cal_warning	: OUT STD_LOGIC;
		mem_addr	: OUT STD_LOGIC_VECTOR (12 DOWNTO 0);
		mem_ba	: OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
		mem_cas_n	: OUT STD_LOGIC;
		mem_cke	: OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
		mem_cs_n	: OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
		mem_dm	: OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		mem_odt	: OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
		mem_ras_n	: OUT STD_LOGIC;
		mem_we_n	: OUT STD_LOGIC;
		mem_reset_n	: OUT STD_LOGIC;
		dqs_delay_ctrl_export	: OUT STD_LOGIC_VECTOR (5 DOWNTO 0);
		dll_reference_clk	: OUT STD_LOGIC;
		dbg_rd_data	: OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
		dbg_waitrequest	: OUT STD_LOGIC;
		aux_half_rate_clk	: OUT STD_LOGIC;
		aux_full_rate_clk	: OUT STD_LOGIC;
		mem_clk	: INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
		mem_clk_n	: INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
		mem_dq	: INOUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		mem_dqs	: INOUT STD_LOGIC_VECTOR (1 DOWNTO 0);
		mem_dqs_n	: INOUT STD_LOGIC_VECTOR (1 DOWNTO 0)
	);
end component;
--component ddr3_mem_example_top is 
--        port (
--              -- inputs:
--                 clock_source : IN STD_LOGIC;
--                 global_reset_n : IN STD_LOGIC;
--
--              -- outputs:
--                 mem_addr : OUT STD_LOGIC_VECTOR (12 DOWNTO 0);
--                 mem_ba : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
--                 mem_cas_n : OUT STD_LOGIC;
--                 mem_cke : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_clk : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_clk_n : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_cs_n : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_dm : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_dq : INOUT STD_LOGIC_VECTOR (7 DOWNTO 0);
--                 mem_dqs : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_dqsn : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_odt : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
--                 mem_ras_n : OUT STD_LOGIC;
--                 mem_reset_n : OUT STD_LOGIC;
--                 mem_we_n : OUT STD_LOGIC;
--                 pnf : OUT STD_LOGIC;
--                 pnf_per_byte : OUT STD_LOGIC_VECTOR (3 DOWNTO 0);
--                 test_complete : OUT STD_LOGIC;
--                 test_status : OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
--              );
--end component;

component ddr3_mem_example_top is 
        port (
              -- inputs:
                 clock_source : IN STD_LOGIC;
                 global_reset_n : IN STD_LOGIC;

              -- outputs:
                 mem_addr : OUT STD_LOGIC_VECTOR (12 DOWNTO 0);
                 mem_ba : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
                 mem_cas_n : OUT STD_LOGIC;
                 mem_cke : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
                 mem_clk : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
                 mem_clk_n : INOUT STD_LOGIC_VECTOR (0 DOWNTO 0);
                 mem_cs_n : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
                 mem_dm : OUT STD_LOGIC_VECTOR (1 DOWNTO 0);
                 mem_dq : INOUT STD_LOGIC_VECTOR (15 DOWNTO 0);
                 mem_dqs : INOUT STD_LOGIC_VECTOR (1 DOWNTO 0);
                 mem_dqsn : INOUT STD_LOGIC_VECTOR (1 DOWNTO 0);
                 mem_odt : OUT STD_LOGIC_VECTOR (0 DOWNTO 0);
                 mem_ras_n : OUT STD_LOGIC;
                 mem_reset_n : OUT STD_LOGIC;
                 mem_we_n : OUT STD_LOGIC;
                 pnf : OUT STD_LOGIC;
                 pnf_per_byte : OUT STD_LOGIC_VECTOR (7 DOWNTO 0);
                 test_complete : OUT STD_LOGIC;
                 test_status : OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
              );
end component;
end ddr3_mem_pkg;