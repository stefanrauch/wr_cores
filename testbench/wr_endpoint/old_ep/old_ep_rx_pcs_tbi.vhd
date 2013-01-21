-------------------------------------------------------------------------------
-- Title      : 1000BaseT/X MAC Endpoint - receive path PCS for 1000BaseX
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : ep_rx_pcs_tbi.vhd
-- Author     : Tomasz Wlostowski
-- Company    : CERN BE-CO-HT
-- Created    : 2009-06-16
-- Last update: 2011-05-11
-- Platform   : FPGA-generic
-- Standard   : VHDL'93
-------------------------------------------------------------------------------
-- Description: Module implements the reception path for 1000BaseX PCS
-- (Physical Coding Sublayer). It provides synchronization between the PHY RX
-- clock and system reference clock, elastic buffering, preamble, SFD and other
-- 8b10b patterns recognition. It also generates deterministic timestamping
-- pulses for RXed packets.
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
-- 2009-06-16  0.1      twlostow  Created (no error propagation supported yet)
-- 2010-10-25  0.2      twlostow  Names updated to comply with the coding,
--                                added some comments
-- 2010-11-18  0.4      twlostow  Added support for Xilinx GTP transceivers.
-- 2011-02-07  0.5      twlostow  Tested on Spartan6 GTP
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.gencores_pkg.all;
use work.genram_pkg.all;
use work.old_endpoint_pkg.all;

entity old_ep_rx_pcs_tbi is
  generic (
    g_simulation : integer;
    g_phy_mode   : string);
  port (
-- 62.5 MHz refclk divided by 2
    clk_sys_i : in std_logic;

-- reset (refclk2-synchronous)
    rst_n_i : in std_logic;

-- RX path busy indicator (active HI),
-- asserted means that receiver is in the middle of reception
-- of a frame
    pcs_busy_o : out std_logic;
    -- data FIFO output
    pcs_data_o : out std_logic_vector(15 downto 0);

-- HIGH level indicates that pcs_data_o contains the last single byte of
-- odd-sized frame.
    pcs_bytesel_o : out std_logic;

-- start-of-frame and end-of-frame indicators
    pcs_sof_o : out std_logic;
    pcs_eof_o : out std_logic;

-- reception error indicator
    pcs_error_o : out std_logic;

-- HI requests a single word from RX FIFO
    pcs_dreq_i : in std_logic;

    -- HI indicates that there is valid data present on pcs_data_o
    pcs_valid_o : out std_logic;


    timestamp_stb_p_o : out std_logic;  -- strobe for RX timestamping

-------------------------------------------------------------------------------
-- TBI interface
-------------------------------------------------------------------------------

    tbi_rbclk_i  : in std_logic;                     -- recovered byte clock
    tbi_rxdata_i : in std_logic_vector(9 downto 0);  -- 8b10b encoded PHY input

-------------------------------------------------------------------------------
-- Xilinx GTP Interface
-------------------------------------------------------------------------------    

    gtp_rx_clk_i     : in std_logic;
    gtp_rx_data_i    : in std_logic_vector(7 downto 0);
    gtp_rx_k_i       : in std_logic;
    gtp_rx_enc_err_i : in std_logic;

-------------------------------------------------------------------------------
-- Wishbone registers
-------------------------------------------------------------------------------    

    -- Receive control regsiter
    mdio_mcr_pdown_i           : in  std_logic;
    mdio_wr_spec_cal_crst_i    : in  std_logic;
    mdio_wr_spec_rx_cal_stat_o : out std_logic;

    synced_o    : out std_logic;
    sync_lost_o : out std_logic;

    -- autonegotiation signals
    an_rx_en_i      : in  std_logic;
    an_rx_val_o     : out std_logic_vector(15 downto 0);
    an_rx_valid_o   : out std_logic;
    an_idle_match_o : out std_logic;

    -- RMON statistic counters
    rmon_syncloss_p_o     : out std_logic;
    rmon_invalid_code_p_o : out std_logic;
    rmon_rx_overrun_p_o   : out std_logic
    );

end old_ep_rx_pcs_tbi;

architecture behavioral of old_ep_rx_pcs_tbi is

-- RX state machine definitions
  type t_tbif_rx_state is (RX_NOFRAME, RX_COMMA, RX_CR3, RX_CR4, RX_SPD_PREAMBLE, RX_PAYLOAD, RX_EXTEND);

-- size of the calibrration pattern detection counter. It counts up every time
-- a valid calibration pattern character is received and resets upon detecion
-- of non-calibration character. The pattern is validated when the counter
-- overflows (after 2**c_cal_pattern_counter_bits counts)


  function f_calc_pattern_counter_bits
    return integer is
  begin  -- f_calc_pattern_counter_bits
    if(g_simulation /= 0) then
      return 8;                         -- use smaller calibration counter to
                                        -- speed up the simulation
    else
      return 17;
    end if;
  end f_calc_pattern_counter_bits;

  constant c_cal_pattern_counter_bits : integer := f_calc_pattern_counter_bits;

  component old_ep_sync_detect
    port (
      rst_n_i  : in  std_logic;
      rbclk_i  : in  std_logic;
      en_i     : in  std_logic;
      data_i   : in  std_logic_vector(7 downto 0);
      k_i      : in  std_logic;
      err_i    : in  std_logic;
      synced_o : out std_logic;
      even_o   : out std_logic;
      cal_i    : in  std_logic);
  end component;

  signal reset_synced_rxclk : std_logic;

  signal rx_state             : t_tbif_rx_state;
  signal preamble_cntr        : unsigned(2 downto 0);
  signal rx_busy              : std_logic;
  signal rx_enable_synced     : std_logic;
  signal rx_rdreq, fifo_wrreq : std_logic;

  -- 8b10b decoding and postprocessing signals
  signal dec_err_code                               : std_logic;
  signal dec_err_rdisp                              : std_logic;
  signal d_is_k, d_err, d_is_comma, d_is_epd        : std_logic;
  signal d_is_spd, d_is_extend, d_is_idle, d_is_lcr : std_logic;
  signal d_is_sfd_char, d_is_preamble_char          : std_logic;
  signal d_data                                     : std_logic_vector(7 downto 0);
  signal d_is_even                                  : std_logic;
  signal d_is_cal                                   : std_logic;
  signal dec_out                                    : std_logic_vector(7 downto 0);
  signal dec_err, dec_is_k                          : std_logic;


  -- Clock alignment FIFO signals
  signal fifo_wr_toggle     : std_logic;
  signal fifo_in, fifo_out  : std_logic_vector(21 downto 0);
  signal fifo_rx_data       : std_logic_vector(15 downto 0);
  signal fifo_mask_write    : std_logic;
  signal fifo_bytesel       : std_logic;
  signal fifo_sof, fifo_eof : std_logic;
  signal fifo_error         : std_logic;
  signal fifo_almostfull    : std_logic;
  signal fifo_empty         : std_logic;
  signal fifo_clear_n       : std_logic;

-- Synchronization detection FSM signals
  signal rx_synced, rx_even : std_logic;
  signal rx_sync_lost_p     : std_logic;
  signal rx_sync_status     : std_logic;
  signal rx_sync_enable     : std_logic;

-- Autonegotiation control signals
  signal an_rx_en_synced : std_logic;

  signal lcr_ready         : std_logic;
  signal lcr_prev_val      : std_logic_vector(15 downto 0);
  signal lcr_cur_val       : std_logic_vector(15 downto 0);
  signal lcr_validity_cntr : unsigned(1 downto 0);

  signal an_idle_cntr      : unsigned(1 downto 0);
  signal an_idle_match_int : std_logic;

-- RMON counter pulses
  signal rmon_rx_overrun_p_int   : std_logic;
  signal rmon_syncloss_p_int     : std_logic;
  signal rmon_invalid_code_p_int : std_logic;

-- Misc. signals
  signal cal_pattern_cntr      : unsigned(c_cal_pattern_counter_bits-1 downto 0);
  signal mdio_mcr_pdown_synced : std_logic;

  signal rx_clk : std_logic;

  signal pcs_valid_int : std_logic;
  

begin
-------------------------------------------------------------------------------
-- synchronizer chains for Wishbone-accessible control signals
-------------------------------------------------------------------------------

  U_sync_pcs_busy : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => clk_sys_i,
      rst_n_i  => '1',
      data_i   => rx_busy,
      synced_o => pcs_busy_o,
      npulse_o => open,
      ppulse_o => open);


  U_sync_rx_reset : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => rx_clk,
      rst_n_i  => '1',
      data_i   => rst_n_i,
      synced_o => reset_synced_rxclk,
      npulse_o => open,
      ppulse_o => open);

  U_sync_an_rx_enable : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => rx_clk,
      rst_n_i  => reset_synced_rxclk,
      data_i   => an_rx_en_i,
      synced_o => an_rx_en_synced,
      npulse_o => open,
      ppulse_o => open);

  U_sync_power_down : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => rx_clk,
      rst_n_i  => '1',
      data_i   => mdio_mcr_pdown_i,
      synced_o => mdio_mcr_pdown_synced,
      npulse_o => open,
      ppulse_o => open);


  rx_sync_enable <= not mdio_mcr_pdown_synced;

-------------------------------------------------------------------------------
-- TBI version:  instantiate an 8b10b decoder
-------------------------------------------------------------------------------

  gen_tbi : if(g_phy_mode = "TBI") generate

    rx_clk <= tbi_rbclk_i after 1ns;    -- choose TBI clock as the RX clock

    U_DEC : old_ep_dec_8b10b

      port map (
        clk_i    => rx_clk,
        rst_n_i  => reset_synced_rxclk,
        in_10b_i => tbi_rxdata_i,

        ctrl_o      => dec_is_k,
        out_8b_o    => dec_out,
        code_err_o  => dec_err_code,
        rdisp_err_o => dec_err_rdisp);

    dec_err <= dec_err_code or dec_err_rdisp;
  end generate gen_tbi;

-------------------------------------------------------------------------------
-- GTP version: 8b10b is integrated in the transceiver
-------------------------------------------------------------------------------  
  gen_gtp : if(g_phy_mode = "GTP") generate
    rx_clk <= gtp_rx_clk_i after 1ns;   -- select GTP clock as the RX clock
    -- the after statement is to avoid fooling the simulator, it doesn't affect
    -- the synthesis.

    dec_is_k <= gtp_rx_k_i;
    dec_out  <= gtp_rx_data_i;
    dec_err  <= gtp_rx_enc_err_i;
  end generate gen_gtp;

-------------------------------------------------------------------------------
-- 802.3z Link Synchronization State Machine
-------------------------------------------------------------------------------

  U_SYNC_DET : old_ep_sync_detect
    port map (
      rst_n_i  => reset_synced_rxclk,
      rbclk_i  => rx_clk,
      en_i     => rx_sync_enable,
      data_i   => dec_out,
      k_i      => dec_is_k,
      err_i    => dec_err,
      synced_o => rx_synced,
      even_o   => rx_even,
      cal_i    => d_is_cal);

  -- synchronizer chain for rx_synced signal, also serving as a loss-of-sync detector
  U_sync_los : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => clk_sys_i,
      rst_n_i  => '1',
      data_i   => rx_synced,
      synced_o => rx_sync_status,
      npulse_o => rx_sync_lost_p,
      ppulse_o => open);

  synced_o    <= rx_sync_status;        -- drive the PCS outputs
  sync_lost_o <= rx_sync_lost_p;

-------------------------------------------------------------------------------
-- Calibration pattern logic
-------------------------------------------------------------------------------

  -- process checks the presence of valid calibtaion pattern and controls the
  -- state of CAL_STA bit in Receive Control Register.
  --
  -- reads: dec_out, mdio_wr_spec_cal_crst_i
  -- writes: mdio_wr_spec_rx_cal_stat_o
  --
  p_detect_cal : process(rx_clk, reset_synced_rxclk)
  begin
    if rising_edge(rx_clk) then
      if reset_synced_rxclk = '0' then
        cal_pattern_cntr <= (others => '0');
        d_is_cal         <= '0';
      else
        
        if(dec_out = c_k28_7 and dec_is_k = '1') then
          d_is_cal <= '1';
        else
          d_is_cal <= '0';
        end if;


        if(d_is_cal = '1' and mdio_wr_spec_cal_crst_i = '0') then

-- we've got c_cal_pattern_threshold valid calibration characters - indicate
-- that we're receiving a valid calibration pattern
          if(cal_pattern_cntr(cal_pattern_cntr'high) = '1') then
            mdio_wr_spec_rx_cal_stat_o <= '1';
          else
            mdio_wr_spec_rx_cal_stat_o <= '0';
            cal_pattern_cntr           <= cal_pattern_cntr + 1;
          end if;
-- we've got a non-calibration character or the pattern detection has been reset
        else
          mdio_wr_spec_rx_cal_stat_o <= '0';
          cal_pattern_cntr           <= (others => '0');
        end if;
      end if;
    end if;
  end process;


-------------------------------------------------------------------------------
-- Clock adjustment FIFO
-------------------------------------------------------------------------------  

  -- FIFO input data formatting
  fifo_wrreq <= fifo_wr_toggle and fifo_mask_write;

  fifo_in(15 downto 0) <= fifo_rx_data;
  fifo_in(16)          <= fifo_sof;
  fifo_in(17)          <= fifo_eof;
  fifo_in(18)          <= fifo_bytesel;
  fifo_in(19)          <= fifo_error;
  fifo_in(20)          <= '0';
  fifo_in(21)          <= '0';

-- Clock adjustment FIFO

  U_RX_FIFO : generic_async_fifo
    generic map (
      g_data_width             => 22,
      g_size                   => 32,
      g_with_wr_almost_full    => true,
      g_with_rd_almost_empty   => true,
      g_almost_empty_threshold => 4,
      g_almost_full_threshold  => 30)
    port map (
      rst_n_i           => fifo_clear_n,
      clk_wr_i          => rx_clk,
      d_i               => fifo_in,
      we_i              => fifo_wrreq,
      wr_empty_o        => open,
      wr_full_o         => open,
      wr_almost_empty_o => open,
      wr_almost_full_o  => fifo_almostfull,
      wr_count_o        => open,
      clk_rd_i          => clk_sys_i,
      q_o               => fifo_out,
      rd_i              => rx_rdreq,
      rd_empty_o        => fifo_empty,
      rd_full_o         => open,
      rd_almost_empty_o => open,
      rd_almost_full_o  => open,
      rd_count_o        => open);

  -- process generates the data ready signal for the RX deframer
  -- reads: rx_rdreq
  -- writers: pcs_valid_o
  p_gen_valid : process (clk_sys_i, rst_n_i)
  begin
    if rising_edge(clk_sys_i) then
      if(rst_n_i = '0') then
        pcs_valid_int <= '0';
      else
        pcs_valid_int <= rx_rdreq;
      end if;
    end if;
  end process;

  -- FIFO output data formatting
  pcs_data_o    <= fifo_out(15 downto 0);
  pcs_bytesel_o <= fifo_out(18);

  pcs_sof_o   <= fifo_out(16) and pcs_valid_int;
  pcs_eof_o   <= fifo_out(17) and pcs_valid_int;
  pcs_error_o <= fifo_out(19) and pcs_valid_int;

  pcs_valid_o <= pcs_valid_int and not (fifo_out(16) or fifo_out(17) or fifo_out(19));

  -- FIFO control signals
  rx_rdreq <= (not fifo_empty) and pcs_dreq_i;

  -- the FIFO is cleared during the reset or when the PCS is disabled
  fifo_clear_n <= '1' when (reset_synced_rxclk = '0') or (mdio_mcr_pdown_synced = '1');


  -- process postprocesses the raw 8b10b decoder output (dec_out, dec_is_k, dec_error)
  -- providing 1-bit signals indicating various 8b10b control patterns
  p_8b10b_postprocess : process(rx_clk, reset_synced_rxclk)
  begin
    if rising_edge(rx_clk) then
      
      if(reset_synced_rxclk = '0') then
        d_data             <= (others => '0');
        d_is_comma         <= '0';
        d_is_spd           <= '0';
        d_is_extend        <= '0';
        d_is_lcr           <= '0';
        d_is_epd           <= '0';
        d_is_idle          <= '0';
        d_is_k             <= '0';
        d_err              <= '0';
        d_is_sfd_char      <= '0';
        d_is_preamble_char <= '0';
        d_err              <= '0';
      else

        -- store the odd/even field information from sync detection unit (U_SYNC_DET)
        
        

        d_is_even <= rx_even;
        d_data    <= dec_out;
        d_is_k    <= dec_is_k;

        if(dec_err = '0') then
          d_err <= '0';

-- decode commas and other control characters....
          if(dec_out = c_K28_5 and dec_is_k = '1') then
            d_is_comma <= '1';
          else
            d_is_comma <= '0';
          end if;

          if(dec_out = c_k23_7 and dec_is_k = '1') then
            d_is_extend <= '1';
          else
            d_is_extend <= '0';
          end if;

          if(dec_out = c_k27_7 and dec_is_k = '1') then
            d_is_spd <= '1';
          else
            d_is_spd <= '0';
          end if;

          if(dec_out = c_K29_7 and dec_is_k = '1') then
            d_is_epd <= '1';
          else
            d_is_epd <= '0';
          end if;

          if((dec_out = c_d21_5 or dec_out = c_d2_2) and dec_is_k = '0') then
            d_is_lcr <= '1';
          else
            d_is_lcr <= '0';
          end if;

          if((dec_out = c_d5_6 or dec_out = c_d16_2) and dec_is_k = '0') then
            d_is_idle <= '1';
          else
            d_is_idle <= '0';
          end if;

          if(dec_out = c_preamble_sfd and dec_is_k = '0') then
            d_is_sfd_char <= '1';
          else
            d_is_sfd_char <= '0';
          end if;

          if(dec_out = c_preamble_char and dec_is_k = '0') then
            d_is_preamble_char <= '1';
          else
            d_is_preamble_char <= '0';
          end if;

-- invalid code received?
        else
          d_err              <= '1';
          d_is_sfd_char      <= '0';
          d_is_preamble_char <= '0';
          d_is_comma         <= '0';
          d_is_spd           <= '0';
          d_is_extend        <= '0';
          d_is_lcr           <= '0';
          d_is_epd           <= '0';
          d_is_idle          <= '0';
          d_is_k             <= '0';
        end if;
      end if;
    end if;
  end process;


-- process: RBCLK-driven RX state machine. Implements the receive logic od 802.3z compliant
-- 1000BaseX PCS.
-- reads: almost everything
-- writes: almost everything

  rx_fsm : process (rx_clk, reset_synced_rxclk)
  begin
    if rising_edge(rx_clk) then
      -- reset or PCS disabled
      if(reset_synced_rxclk = '0' or mdio_mcr_pdown_synced = '1') then
        rx_state <= RX_NOFRAME;
        rx_busy  <= '0';

        fifo_sof        <= '0';
        fifo_eof        <= '0';
        fifo_error      <= '0';
        fifo_wr_toggle  <= '0';
        fifo_mask_write <= '0';

        lcr_ready         <= '0';
        lcr_cur_val       <= (others => '0');
        lcr_prev_val      <= (others => '0');
        lcr_validity_cntr <= (others => '0');
        an_idle_cntr      <= (others => '0');
        an_idle_match_int <= '0';

        rmon_rx_overrun_p_int   <= '0';
        rmon_invalid_code_p_int <= '0';

        timestamp_stb_p_o <= '0';
      else                              -- normal PCS operation

        -- clear the autogotiation variables if the autonegotiation is disabled
        if(an_rx_en_synced = '0') then
          lcr_ready         <= '0';
          lcr_validity_cntr <= (others => '0');
          lcr_prev_val      <= (others => '0');
          an_idle_cntr      <= (others => '0');
          an_idle_match_int <= '0';
        end if;

-------------------------------------------------------------------------------
-- Main RX PCS state machine
-------------------------------------------------------------------------------          
        case rx_state is

-------------------------------------------------------------------------------
-- State NOFRAME: receiver is receiving IDLE pattern
-------------------------------------------------------------------------------            
          when RX_NOFRAME =>

            
            fifo_sof        <= '0';
            fifo_eof        <= '0';
            fifo_error      <= '0';
            fifo_bytesel    <= '0';
            fifo_mask_write <= '0';
            fifo_wr_toggle  <= '1';

            rx_busy           <= '0';
            timestamp_stb_p_o <= '0';


            if (rx_synced = '0') then
-- PCS is not synced: stay in NOFRAME state and ignore the incoming codes.
              rx_state <= RX_NOFRAME;

            elsif(d_is_comma = '1') then
-- we've got a comma character: it's probably an idle sequence or a Config_Reg value,
-- check the next code.
              rx_state <= RX_COMMA;
            elsif (d_is_spd = '1') then
-- we've got a Start-of-Packet Delimeter

              if(d_is_even = '1' and fifo_almostfull = '0') then
                preamble_cntr <= "111";
                rx_state      <= RX_SPD_PREAMBLE;
              elsif (fifo_almostfull = '1') then
                rmon_rx_overrun_p_int <= '1';
              end if;
              
            end if;


-- produce a pulse at every invalid 8b10b code.
            rmon_invalid_code_p_int <= d_err;

-------------------------------------------------------------------------------
-- State COMMA: we've received a comma character followed by something else.
-- Determine what's "something else"
-------------------------------------------------------------------------------                            

          when RX_COMMA =>
-- received a code with error (or a control code group) or a misaligned code:
-- go to the initial NOFRAME state and account the error.

            if (d_err = '1' or d_is_k = '1' or d_is_even = '1' or rx_synced = '0') then
              rmon_invalid_code_p_int <= d_err;
              rx_state                <= RX_NOFRAME;

              lcr_ready         <= '0';
              an_idle_match_int <= '0';
              an_idle_cntr      <= (others => '0');
            else

-- don't write anything to the PCS FIFO, it's a control character
              fifo_mask_write <= '0';

-- received D5.6 or D16.2 - it's an idle pattern. Simply ignore it.
              if(d_is_idle = '1') then
                rx_state  <= RX_NOFRAME;
                lcr_ready <= '0';


                if(an_idle_cntr = "11") then
                  an_idle_match_int <= '1';
                else
                  an_idle_cntr <= an_idle_cntr + 1;
                end if;


-- received D21.5 or D2.2 - it's a 802.3x autonegotiation Config_Reg (802.3 p.
-- 36.2.4.10). Begin the reception of its value.
              elsif(d_is_lcr = '1') then
                an_idle_match_int <= '0';
                an_idle_cntr      <= (others => '0');
                rx_busy           <= '1';
                rx_state          <= RX_CR3;
              else
                lcr_ready         <= '0';
                an_idle_match_int <= '0';
                an_idle_cntr      <= (others => '0');
                lcr_validity_cntr <= (others => '0');
              end if;
            end if;

-------------------------------------------------------------------------------
-- States CR3/CR4: reception of LCR register value.
-------------------------------------------------------------------------------              
          when RX_CR3 =>  -- receives the 1st byte of Config_Reg and
            -- checks if the subsequent Config_Reg
            -- values are identical.

-- an error? - abort the reception and go to NOFRAME state.
            if(d_err = '1' or d_is_k = '1' or d_is_even = '0' or rx_synced = '0') then
              rx_state                <= RX_NOFRAME;
              rmon_invalid_code_p_int <= d_err;

              -- reset the Config_Reg value and mark it as invalid
              lcr_ready         <= '0';
              lcr_validity_cntr <= (others => '0');

-- check if the autonegotiation unit has enabled the reception of LCR
            elsif (an_rx_en_synced = '1') then
              lcr_prev_val <= lcr_cur_val;

-- check for 3 subsequent Configuration sequences with identical Config_Reg value
              if(lcr_cur_val = lcr_prev_val) then
                if(lcr_validity_cntr = "10") then
-- we've got 3? Indicate that we have received valid Config_Reg.
                  lcr_ready <= '1';
                else
                  lcr_validity_cntr <= lcr_validity_cntr + 1;
                  lcr_ready         <= '0';
                end if;
              else
-- the subsequent values of Config_Reg are different?
                lcr_validity_cntr <= (others => '0');
                lcr_ready         <= '0';
              end if;

              lcr_cur_val (7 downto 0) <= d_data;
            end if;

            rx_state <= RX_CR4;

          when RX_CR4 =>                -- receives the 2nd byte of LCR

            if(d_err = '1' or d_is_k = '1' or d_is_even = '1' or rx_synced = '0') then
              -- in case of an error
              rx_state                <= RX_NOFRAME;
              rmon_invalid_code_p_int <= d_err;
            elsif (an_rx_en_synced = '1') then
              -- store the value of the LSB of the Config_Reg
              lcr_cur_val (15 downto 8) <= d_data;
            end if;

            rx_state <= RX_NOFRAME;

-------------------------------------------------------------------------------
-- State SPD_PREAMBLE: we've received an Start-Of-Packet delimeter. Check for
-- the valid preamble.
-------------------------------------------------------------------------------                            
          when RX_SPD_PREAMBLE =>

            rx_busy <= '1';

            if(d_err = '1' or rx_synced = '0') then  -- check for encoding errors.
              rx_state                <= RX_NOFRAME;
              rmon_invalid_code_p_int <= d_err;
            else

              -- keep looking for Ethernet SFD char (0xd5). If it occurs on
              -- the right position, start receiving the frame payload
              if d_is_sfd_char = '1' then
-- generate the RX timestamp pulse
                timestamp_stb_p_o <= '1';

-- we've got an SFD at proper offset from the beginning of the preamble
                if (preamble_cntr = "010") or (preamble_cntr = "001") then

-- indicate a start-of-packet condition in the RX FIFO and enable writing to
-- the FIFO.
                  fifo_sof        <= '1';
                  fifo_mask_write <= '1';
                  fifo_wr_toggle  <= '1';
                  rx_state        <= RX_PAYLOAD;
                end if;
                
              elsif (d_is_preamble_char = '1') then
                preamble_cntr <= preamble_cntr - 1;
              -- got duplicated SPD code?
              elsif (d_is_spd = '1') then
                preamble_cntr <= "111";
              end if;

              if(preamble_cntr = "000") then  -- too long preamble - abort reception
                rx_state <= RX_NOFRAME;
              end if;
            end if;

-------------------------------------------------------------------------------
-- State PAYLOAD: receives the full frame payload (including the MAC header and
-- the CRC)
-------------------------------------------------------------------------------

          when RX_PAYLOAD =>

            fifo_sof <= '0';

            -- check for errors.
            if (d_err = '1' or rx_synced = '0' or fifo_almostfull = '1') then
              rmon_invalid_code_p_int <= d_err;
              rmon_rx_overrun_p_int   <= fifo_almostfull;

              rx_state <= RX_NOFRAME;

              -- indicate an errorneous termination of the current frame in the
              -- RX FIFO
              fifo_error     <= '1';
              fifo_wr_toggle <= '1';
              
            elsif d_is_k = '1' then
              if d_is_epd = '1' then    -- got an EPD (End-of-packet delimeter)

                fifo_bytesel   <= not fifo_wr_toggle;
                fifo_wr_toggle <= not fifo_wr_toggle;

                rx_state <= RX_EXTEND;

              else
                -- any other K-character in the middle of frame (premature
                -- end)? - terminate the frame and indicate an error in the
                -- RX FIFO.

                if d_is_comma = '1' then  -- got link idle inside frame
                  rx_state <= RX_COMMA;
                else
                  rx_state <= RX_NOFRAME;
                end if;

                fifo_error     <= '1';
                fifo_wr_toggle <= '1';
              end if;
            else
              -- got a data character. Every 2 characters, write them to the
              -- 16-bit FIFO.

              fifo_wr_toggle <= not fifo_wr_toggle;

              if fifo_wr_toggle = '1' then
                fifo_rx_data(15 downto 8) <= d_data;
              else
                fifo_rx_data(7 downto 0) <= d_data;
              end if;
            end if;

-------------------------------------------------------------------------------
-- State EXTEND: receive carrier extension
-------------------------------------------------------------------------------

          when RX_EXTEND =>

            timestamp_stb_p_o <= '0';

            if d_is_extend = '1' then    -- got carrier extend. Just keep
                                         -- receiving it.
              fifo_mask_write <= '0';
              rx_state        <= RX_EXTEND;
            elsif d_is_comma = '1' then  -- got comma, real end-of-frame
              -- indicate the correct ending of the current frame in the RX FIFO
              fifo_eof        <= '1';
              fifo_mask_write <= '1';
              fifo_wr_toggle  <= '1';

              rx_state <= RX_COMMA;
            else
              -- got anything else than comma (for example, the /V/ code):

              rmon_invalid_code_p_int <= '1';
              fifo_error              <= '1';
              fifo_mask_write         <= '1';
              fifo_wr_toggle          <= '1';

              rx_state <= RX_NOFRAME;
            end if;

          when others => null;
        end case;
      end if;
    end if;
  end process;

  an_rx_val_o <= lcr_cur_val;

  U_sync_an_rx_ready : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => clk_sys_i,
      rst_n_i  => rst_n_i,
      data_i   => lcr_ready,
      synced_o => an_rx_valid_o,
      npulse_o => open,
      ppulse_o => open);

  U_sync_an_idle_match : gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => clk_sys_i,
      rst_n_i  => rst_n_i,
      data_i   => an_idle_match_int,
      synced_o => an_idle_match_o,
      npulse_o => open,
      ppulse_o => open);

  U_ext_rmon_1 : gc_extend_pulse
    generic map (
      g_width => 3)
    port map (
      clk_i      => rx_clk,
      rst_n_i    => reset_synced_rxclk,
      pulse_i    => rmon_invalid_code_p_int,
      extended_o => rmon_invalid_code_p_o);

  U_ext_rmon_2 : gc_extend_pulse
    generic map (
      g_width => 3)
    port map (
      clk_i      => rx_clk,
      rst_n_i    => reset_synced_rxclk,
      pulse_i    => rmon_rx_overrun_p_int,
      extended_o => rmon_rx_overrun_p_o);

-- drive the "RX PCS Sync Lost" event counter
  rmon_syncloss_p_o <= rx_sync_lost_p and (not mdio_mcr_pdown_i);

end behavioral;


