-------------------------------------------------------------------------------
-- Title      : 1000BaseT/X MAC Endpoint - PCS for 1000BaseX
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : ep_1000basex_pcs.vhd
-- Author     : Tomasz Wlostowski
-- Company    : CERN BE-CO-HT
-- Created    : 2010-11-18
-- Last update: 2011-04-10
-- Platform   : FPGA-generic
-- Standard   : VHDL'93
-------------------------------------------------------------------------------
-- Description: Module implements the top level of a 1000BaseX PCS
-- (Physical Coding Sublayer) dedicated for 10-bit interface PHYs and Xilinx
-- GTP transceivers.  
-------------------------------------------------------------------------------
--
-- Copyright (c) 2009 Tomasz Wlostowski / CERN
--
-- This source file is free software; you can redistribute it   
-- and/or modify it under the terms of the GNU Lesser General   
-- Public License as published by the Free Software Foundation; 
-- either version 2.1 of the License, or (at your option) any   
-- later version.                                               
--
-- This source is distributed in the hope that it will be       
-- useful, but WITHOUT ANY WARRANTY; without even the implied   
-- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
-- PURPOSE.  See the GNU Lesser General Public License for more 
-- details.                                                     
--
-- You should have received a copy of the GNU Lesser General    
-- Public License along with this source; if not, download it   
-- from http://www.gnu.org/licenses/lgpl-2.1.html
--
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author    Description
-- 2010-11-18  0.4      twlostow  Created (separeted from wrsw_endpoint)
-- 2011-02-07  0.5      twlostow  Tested on Spartan6 GTP
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.old_endpoint_pkg.all;

entity old_ep_1000basex_pcs is

  generic (
    g_simulation : integer;
    g_phy_mode   : string := "TBI");
  
  port (
    rst_n_i   : in std_logic;
    clk_sys_i : in std_logic;

    -- PCS <-> MAC Interface

    rxpcs_busy_o            : out std_logic;
    rxpcs_data_o            : out std_logic_vector(15 downto 0);
    rxpcs_bytesel_o         : out std_logic;
    rxpcs_sof_o             : out std_logic;
    rxpcs_eof_o             : out std_logic;
    rxpcs_error_o           : out std_logic;
    rxpcs_dreq_i            : in  std_logic;
    rxpcs_valid_o           : out std_logic;
    rxpcs_timestamp_stb_p_o : out std_logic;

    txpcs_data_i            : in  std_logic_vector(15 downto 0);
    txpcs_bytesel_i         : in  std_logic;
    txpcs_sof_i             : in  std_logic;
    txpcs_eof_i             : in  std_logic;
    txpcs_abort_i           : in  std_logic;
    txpcs_error_p_o         : out std_logic;
    txpcs_busy_o            : out std_logic;
    txpcs_valid_i           : in  std_logic;
    txpcs_fifo_almostfull_o : out std_logic;
    txpcs_timestamp_stb_p_o : out std_logic;

    link_ok_o : out std_logic;

    -- TBI interface (for TLK1221 PHY)

    tbi_rbclk_i  : in std_logic;
    tbi_rxdata_i : in std_logic_vector(9 downto 0);

    tbi_txclk_i  : in  std_logic;
    tbi_txdata_o : out std_logic_vector(9 downto 0);

    tbi_syncen_o : out std_logic;
    tbi_loopen_o : out std_logic;
    tbi_prbsen_o : out std_logic;
    tbi_enable_o : out std_logic;

    -- GTP Serdes interface

    gtp_tx_clk_i       : in  std_logic;
    gtp_tx_data_o      : out  std_logic_vector(7 downto 0);
    gtp_tx_k_o         : out  std_logic;
    gtp_tx_disparity_i : in std_logic;
    gtp_tx_enc_err_i   : in std_logic;

    gtp_rx_data_i     : in std_logic_vector(7 downto 0);
    gtp_rx_clk_i    : in std_logic;
    gtp_rx_k_i        : in std_logic;
    gtp_rx_enc_err_i  : in std_logic;
    gtp_rx_bitslide_i : in std_logic_vector(3 downto 0);

    gtp_rst_o    : out std_logic;
    gtp_loopen_o : out std_logic;
    
    -- RMON statistic counters
    rmon_syncloss_p_o     : out std_logic;
    rmon_invalid_code_p_o : out std_logic;
    rmon_rx_overrun_p_o   : out std_logic;
    rmon_tx_underrun_o    : out std_logic;


--  MDIO interface
    mdio_addr_i  : in  std_logic_vector(7 downto 0);
    mdio_data_i  : in  std_logic_vector(15 downto 0);
    mdio_data_o  : out std_logic_vector(15 downto 0);
    mdio_stb_i   : in  std_logic;
    mdio_rw_i    : in  std_logic;
    mdio_ready_o : out std_logic

    );

end old_ep_1000basex_pcs;

architecture rtl of old_ep_1000basex_pcs is

  component old_ep_pcs_tbi_mdio_wb
    port (
      rst_n_i                    : in  std_logic;
      wb_clk_i                   : in  std_logic;
      wb_addr_i                  : in  std_logic_vector(4 downto 0);
      wb_data_i                  : in  std_logic_vector(31 downto 0);
      wb_data_o                  : out std_logic_vector(31 downto 0);
      wb_cyc_i                   : in  std_logic;
      wb_sel_i                   : in  std_logic_vector(3 downto 0);
      wb_stb_i                   : in  std_logic;
      wb_we_i                    : in  std_logic;
      wb_ack_o                   : out std_logic;
      tx_clk_i                   : in  std_logic;
      rx_clk_i                   : in  std_logic;
      mdio_mcr_uni_en_o          : out std_logic;
      mdio_mcr_anrestart_o       : out std_logic;
      mdio_mcr_pdown_o           : out std_logic;
      mdio_mcr_anenable_o        : out std_logic;
      mdio_mcr_loopback_o        : out std_logic;
      mdio_mcr_reset_o           : out std_logic;
      mdio_msr_lstatus_i         : in  std_logic;
      mdio_msr_rfault_i          : in  std_logic;
      mdio_msr_anegcomplete_i    : in  std_logic;
      mdio_advertise_pause_o     : out std_logic_vector(1 downto 0);
      mdio_advertise_rfault_o    : out std_logic_vector(1 downto 0);
      mdio_lpa_full_i            : in  std_logic;
      mdio_lpa_half_i            : in  std_logic;
      mdio_lpa_pause_i           : in  std_logic_vector(1 downto 0);
      mdio_lpa_rfault_i          : in  std_logic_vector(1 downto 0);
      mdio_lpa_lpack_i           : in  std_logic;
      mdio_lpa_npage_i           : in  std_logic;
      mdio_wr_spec_tx_cal_o      : out std_logic;
      mdio_wr_spec_rx_cal_stat_i : in  std_logic;
      mdio_wr_spec_cal_crst_o    : out std_logic;
      mdio_wr_spec_bslide_i: in std_logic_vector(3 downto 0);
      lstat_read_notify_o        : out std_logic);
  end component;

  component old_ep_rx_pcs_tbi
    generic (
      g_simulation  : integer;
      g_phy_mode : string);
    port (
      clk_sys_i                  : in  std_logic;
      rst_n_i                    : in  std_logic;
      pcs_busy_o                 : out std_logic;
      pcs_data_o                 : out std_logic_vector(15 downto 0);
      pcs_bytesel_o              : out std_logic;
      pcs_sof_o                  : out std_logic;
      pcs_eof_o                  : out std_logic;
      pcs_error_o                : out std_logic;
      pcs_dreq_i                 : in  std_logic;
      pcs_valid_o                : out std_logic;
      timestamp_stb_p_o          : out std_logic;
      tbi_rbclk_i                : in  std_logic;
      tbi_rxdata_i               : in  std_logic_vector(9 downto 0);
     
      gtp_rx_clk_i               : in  std_logic;
      gtp_rx_data_i              : in  std_logic_vector(7 downto 0);
      gtp_rx_k_i                 : in  std_logic;
      gtp_rx_enc_err_i           : in  std_logic;
      mdio_mcr_pdown_i           : in  std_logic;
      mdio_wr_spec_cal_crst_i    : in  std_logic;
      mdio_wr_spec_rx_cal_stat_o : out std_logic;
      synced_o                   : out std_logic;
      sync_lost_o                : out std_logic;
      an_rx_en_i                 : in  std_logic;
      an_rx_val_o                : out std_logic_vector(15 downto 0);
      an_rx_valid_o              : out std_logic;
      an_idle_match_o            : out std_logic;
      rmon_syncloss_p_o          : out std_logic;
      rmon_invalid_code_p_o      : out std_logic;
      rmon_rx_overrun_p_o        : out std_logic);
  end component;


  component old_ep_tx_pcs_tbi
    generic (
      g_phy_mode : string);
    port (
      rst_n_i               : in  std_logic;
      clk_sys_i             : in  std_logic;
      pcs_data_i            : in  std_logic_vector(15 downto 0);
      pcs_bytesel_i         : in  std_logic;
      pcs_sof_i             : in  std_logic;
      pcs_eof_i             : in  std_logic;
      pcs_abort_i           : in  std_logic;
      pcs_error_p_o         : out std_logic;
      pcs_busy_o            : out std_logic;
      pcs_valid_i           : in  std_logic;
      pcs_fifo_almostfull_o : out std_logic;
      mdio_mcr_pdown_i      : in  std_logic;
      mdio_wr_spec_tx_cal_i : in  std_logic;
      an_tx_en_i            : in  std_logic;
      an_tx_val_i           : in  std_logic_vector(15 downto 0);
      timestamp_stb_p_o     : out std_logic;
      rmon_tx_underrun_o    : out std_logic;
      gtp_tx_clk_i          : in  std_logic;
      gtp_tx_data_o         : out std_logic_vector(7 downto 0);
      gtp_tx_k_o            : out std_logic;
      gtp_tx_disparity_i    : in  std_logic;
      gtp_tx_enc_err_i      : in  std_logic;
      tbi_txclk_i           : in  std_logic;
      tbi_txdata_o          : out std_logic_vector(9 downto 0));
  end component;


  component old_ep_autonegotiation
    generic (
      g_simulation : integer);
    port (
      clk_sys_i               : in  std_logic;
      rst_n_i                 : in  std_logic;
      pcs_synced_i            : in  std_logic;
      pcs_los_i               : in  std_logic;
      pcs_link_ok_o           : out std_logic;
      an_idle_match_i         : in  std_logic;
      an_rx_en_o              : out std_logic;
      an_rx_val_i             : in  std_logic_vector(15 downto 0);
      an_rx_valid_i           : in  std_logic;
      an_tx_en_o              : out std_logic;
      an_tx_val_o             : out std_logic_vector(15 downto 0);
      mdio_mcr_anrestart_i    : in  std_logic;
      mdio_mcr_anenable_i     : in  std_logic;
      mdio_msr_anegcomplete_o : out std_logic;
      mdio_advertise_pause_i  : in  std_logic_vector(1 downto 0);
      mdio_advertise_rfault_i : in  std_logic_vector(1 downto 0);
      mdio_lpa_full_o         : out std_logic;
      mdio_lpa_half_o         : out std_logic;
      mdio_lpa_pause_o        : out std_logic_vector(1 downto 0);
      mdio_lpa_rfault_o       : out std_logic_vector(1 downto 0);
      mdio_lpa_lpack_o        : out std_logic;
      mdio_lpa_npage_o        : out std_logic);
  end component;

  signal mdio_mcr_uni_en          : std_logic;
  signal mdio_mcr_anrestart       : std_logic;
  signal mdio_mcr_pdown           : std_logic;
  signal mdio_mcr_anenable        : std_logic;
  signal mdio_mcr_loopback        : std_logic;
  signal mdio_mcr_reset           : std_logic;
  signal mdio_msr_lstatus         : std_logic;
  signal mdio_msr_rfault          : std_logic;
  signal mdio_msr_anegcomplete    : std_logic;
  signal mdio_advertise_pause     : std_logic_vector(1 downto 0);
  signal mdio_advertise_rfault    : std_logic_vector(1 downto 0);
  signal mdio_lpa_full            : std_logic;
  signal mdio_lpa_half            : std_logic;
  signal mdio_lpa_pause           : std_logic_vector(1 downto 0);
  signal mdio_lpa_rfault          : std_logic_vector(1 downto 0);
  signal mdio_lpa_lpack           : std_logic;
  signal mdio_lpa_npage           : std_logic;
  signal mdio_wr_spec_tx_cal      : std_logic;
  signal mdio_wr_spec_rx_cal_stat : std_logic;
  signal mdio_wr_spec_cal_crst    : std_logic;
  signal mdio_wr_spec_bslide : std_logic_vector(3 downto 0);
  
  signal lstat_read_notify        : std_logic;

-------------------------------------------------------------------------------
-- Autonegotiation signals
-------------------------------------------------------------------------------

  signal an_tx_en      : std_logic;
  signal an_rx_en      : std_logic;
  signal an_tx_val     : std_logic_vector(15 downto 0);
  signal an_rx_val     : std_logic_vector(15 downto 0);
  signal an_rx_valid   : std_logic;
  signal an_idle_match : std_logic;

  signal pcs_enable        : std_logic;
  signal synced, sync_lost : std_logic;

  signal txpcs_busy_int : std_logic;
  signal link_ok        : std_logic;

  signal pcs_reset_n : std_logic;

  signal wb_stb, wb_ack : std_logic;

  signal dummy : std_logic_vector(31 downto 0);

  signal tx_clk, rx_clk : std_logic;
  
begin  -- rtl



  pcs_reset_n <= '0' when (mdio_mcr_reset = '1' or rst_n_i = '0') else '1';

-- the PCS state machines themselves

  U_TX_PCS : old_ep_tx_pcs_tbi
    generic map (
      g_phy_mode => g_phy_mode)
    port map (
      rst_n_i               => pcs_reset_n,
      clk_sys_i             => clk_sys_i,
      pcs_data_i            => txpcs_data_i,
      pcs_bytesel_i         => txpcs_bytesel_i,
      pcs_sof_i             => txpcs_sof_i,
      pcs_eof_i             => txpcs_eof_i,
      pcs_abort_i           => txpcs_abort_i,
      pcs_error_p_o         => txpcs_error_p_o,
      pcs_busy_o            => txpcs_busy_int,
      pcs_valid_i           => txpcs_valid_i,
      pcs_fifo_almostfull_o => txpcs_fifo_almostfull_o,

      mdio_mcr_pdown_i      => mdio_mcr_pdown,
      mdio_wr_spec_tx_cal_i => mdio_wr_spec_tx_cal,

      an_tx_en_i         => an_tx_en,
      an_tx_val_i        => an_tx_val,
      timestamp_stb_p_o  => txpcs_timestamp_stb_p_o,
      rmon_tx_underrun_o => rmon_tx_underrun_o,

      tbi_txclk_i        => tbi_txclk_i,
      tbi_txdata_o       => tbi_txdata_o,

      gtp_tx_clk_i          => gtp_tx_clk_i,
      gtp_tx_data_o         => gtp_tx_data_o,
      gtp_tx_k_o            => gtp_tx_k_o,
      gtp_tx_disparity_i    => gtp_tx_disparity_i,
      gtp_tx_enc_err_i      => gtp_tx_enc_err_i
      );


  U_RX_PCS : old_ep_rx_pcs_tbi
    generic map (
      g_simulation => g_simulation,
      g_phy_mode => g_phy_mode)
    port map (
      clk_sys_i         => clk_sys_i,
      rst_n_i           => pcs_reset_n,
      pcs_busy_o        => rxpcs_busy_o,
      pcs_data_o        => rxpcs_data_o,
      pcs_bytesel_o     => rxpcs_bytesel_o,
      pcs_sof_o         => rxpcs_sof_o,
      pcs_eof_o         => rxpcs_eof_o,
      pcs_error_o       => rxpcs_error_o,
      pcs_dreq_i        => rxpcs_dreq_i,
      pcs_valid_o       => rxpcs_valid_o,
   
      timestamp_stb_p_o => rxpcs_timestamp_stb_p_o,

      mdio_mcr_pdown_i           => mdio_mcr_pdown,
      mdio_wr_spec_cal_crst_i    => mdio_wr_spec_cal_crst,
      mdio_wr_spec_rx_cal_stat_o => mdio_wr_spec_rx_cal_stat,

      synced_o        => synced,
      sync_lost_o     => sync_lost,
      an_rx_en_i      => an_rx_en,
      an_rx_val_o     => an_rx_val,
      an_rx_valid_o   => an_rx_valid,
      an_idle_match_o => an_idle_match,

      rmon_syncloss_p_o     => rmon_syncloss_p_o,
      rmon_invalid_code_p_o => rmon_invalid_code_p_o,
      rmon_rx_overrun_p_o   => rmon_rx_overrun_p_o,

      tbi_rbclk_i       => tbi_rbclk_i,
      tbi_rxdata_i      => tbi_rxdata_i,

      gtp_rx_clk_i               => gtp_rx_clk_i,
      gtp_rx_data_i              => gtp_rx_data_i,
      gtp_rx_k_i                 => gtp_rx_k_i,
      gtp_rx_enc_err_i           => gtp_rx_enc_err_i
      );

  txpcs_busy_o <= '1' when (synced = '0' and mdio_mcr_uni_en = '0') else txpcs_busy_int;

  gen_tbi: if(g_phy_mode = "TBI") generate
      rx_clk <= tbi_rbclk_i;
      tx_clk <= tbi_txclk_i;

  -- drive the PHY control signals
      tbi_loopen_o <= mdio_mcr_loopback;
      tbi_enable_o <= (not mdio_mcr_pdown);
      tbi_prbsen_o <= '0';
      tbi_syncen_o <= not synced;

  end generate gen_tbi;

  gen_gtp:if(g_phy_mode = "GTP") generate

    rx_clk <= gtp_rx_clk_i;
    tx_clk <= gtp_tx_clk_i;

    gtp_rst_o <= (not pcs_reset_n) or mdio_mcr_pdown;
    gtp_loopen_o <= mdio_mcr_loopback;
    mdio_wr_spec_bslide <= gtp_rx_bitslide_i;
  end generate gen_gtp;
  
  U_MDIO_WB : old_ep_pcs_tbi_mdio_wb
    port map (
      rst_n_i                 => rst_n_i,
      wb_clk_i                => clk_sys_i,
      wb_addr_i               => mdio_addr_i(4 downto 0),
      wb_data_i(15 downto 0)  => mdio_data_i,
      wb_data_i(31 downto 16) => x"0000",
      wb_data_o(15 downto 0)  => mdio_data_o,
      wb_data_o(31 downto 16) => dummy(31 downto 16),

      wb_cyc_i => wb_stb,
      wb_sel_i => "1111",
      wb_stb_i => wb_stb,
      wb_we_i  => mdio_rw_i,
      wb_ack_o => wb_ack,
      tx_clk_i => tx_clk,
      rx_clk_i => rx_clk,

      mdio_mcr_uni_en_o          => mdio_mcr_uni_en,
      mdio_mcr_anrestart_o       => mdio_mcr_anrestart,
      mdio_mcr_pdown_o           => mdio_mcr_pdown,
      mdio_mcr_anenable_o        => mdio_mcr_anenable,
      mdio_mcr_loopback_o        => mdio_mcr_loopback,
      mdio_mcr_reset_o           => mdio_mcr_reset,
      mdio_msr_lstatus_i         => mdio_msr_lstatus,
      mdio_msr_rfault_i          => mdio_msr_rfault,
      mdio_msr_anegcomplete_i    => mdio_msr_anegcomplete,
      mdio_advertise_pause_o     => mdio_advertise_pause,
      mdio_advertise_rfault_o    => mdio_advertise_rfault,
      mdio_lpa_full_i            => mdio_lpa_full,
      mdio_lpa_half_i            => mdio_lpa_half,
      mdio_lpa_pause_i           => mdio_lpa_pause,
      mdio_lpa_rfault_i          => mdio_lpa_rfault,
      mdio_lpa_lpack_i           => mdio_lpa_lpack,
      mdio_lpa_npage_i           => mdio_lpa_npage,
      mdio_wr_spec_tx_cal_o      => mdio_wr_spec_tx_cal,
      mdio_wr_spec_rx_cal_stat_i => mdio_wr_spec_rx_cal_stat,
      mdio_wr_spec_cal_crst_o    => mdio_wr_spec_cal_crst,
      mdio_wr_spec_bslide_i => mdio_wr_spec_bslide,

      lstat_read_notify_o => lstat_read_notify
      );


  mdio_msr_rfault <= '0';
  
  U_AUTONEGOTIATION : old_ep_autonegotiation
    generic map (
      g_simulation => g_simulation)
    
    port map (
      clk_sys_i => clk_sys_i,
      rst_n_i   => pcs_reset_n,

      pcs_synced_i  => synced,
      pcs_los_i     => sync_lost,
      pcs_link_ok_o => link_ok,

      an_idle_match_i         => an_idle_match,
      an_rx_en_o              => an_rx_en,
      an_rx_val_i             => an_rx_val,
      an_rx_valid_i           => an_rx_valid,
      an_tx_en_o              => an_tx_en,
      an_tx_val_o             => an_tx_val,
      mdio_mcr_anrestart_i    => mdio_mcr_anrestart,
      mdio_mcr_anenable_i     => mdio_mcr_anenable,
      mdio_msr_anegcomplete_o => mdio_msr_anegcomplete,
      mdio_advertise_pause_i  => mdio_advertise_pause,
      mdio_advertise_rfault_i => mdio_advertise_rfault,
      mdio_lpa_full_o         => mdio_lpa_full,
      mdio_lpa_half_o         => mdio_lpa_half,
      mdio_lpa_pause_o        => mdio_lpa_pause,
      mdio_lpa_rfault_o       => mdio_lpa_rfault,
      mdio_lpa_lpack_o        => mdio_lpa_lpack,
      mdio_lpa_npage_o        => mdio_lpa_npage
      );

  -- process: translates the MDIO reads/writes into Wishbone read/writes
  -- inputs: mdio_stb_i, wb_ack
  -- ouputs: mdio_ready_o, wb_stb
  p_translate_mdio_wb : process(clk_sys_i, rst_n_i)
  begin
    if rising_edge(clk_sys_i) then
      if (rst_n_i = '0') then
        wb_stb       <= '0';
        mdio_ready_o <= '1';
      else
        if(mdio_stb_i = '1' and wb_stb = '0') then
          wb_stb <= '1';
          mdio_ready_o <= '0';
        elsif(wb_stb = '1' and wb_ack = '1') then
          mdio_ready_o <= '1';
          wb_stb       <= '0';
        end if;
      end if;
    end if;
  end process;

  -- process: handles the LSTATUS bit in MSR register
  -- inputs: sync_lost, synced, lstat_read_notify
  -- outputs: mdio_msr_lstatus
  p_gen_link_status : process(clk_sys_i, pcs_reset_n)
  begin
    if rising_edge(clk_sys_i) then
      if(pcs_reset_n = '0') then
        mdio_msr_lstatus <= '0';
      else
        if(sync_lost = '1') then
          mdio_msr_lstatus <= '0';
        elsif(lstat_read_notify = '1') then
          mdio_msr_lstatus <= synced and link_ok;
        end if;
      end if;
    end if;
  end process;

  link_ok_o <= link_ok and synced;
end rtl;
