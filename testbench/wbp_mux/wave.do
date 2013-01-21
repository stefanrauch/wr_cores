onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /main/DUT/U_Mux/clk_sys_i
add wave -noupdate /main/DUT/U_Mux/rst_n_i

add wave -noupdate /main/DUT/U_Mux/class_core_i

add wave -divider ENDPOINT
add wave -noupdate /main/DUT/U_Mux/ep_wbs_adr_i
add wave -noupdate /main/DUT/U_Mux/ep_wbs_dat_i
add wave -noupdate /main/DUT/U_Mux/ep_wbs_sel_i
add wave -noupdate /main/DUT/U_Mux/ep_wbs_cyc_i
add wave -noupdate /main/DUT/U_Mux/ep_wbs_stb_i
add wave -noupdate /main/DUT/U_Mux/ep_wbs_ack_o
add wave -noupdate /main/DUT/U_Mux/ep_wbs_err_o
add wave -noupdate /main/DUT/U_Mux/ep_wbs_stall_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_adr_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_dat_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_sel_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_cyc_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_stb_o
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_ack_i
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_err_i
#add wave -noupdate /main/DUT/U_Mux/ep_wbm_stall_i

add wave -divider MiNIC
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_adr_i
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_dat_i
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_sel_i
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_cyc_i
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_stb_i
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_ack_o
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_err_o
#add wave -noupdate /main/DUT/U_Mux/ptp_wbs_stall_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_adr_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_dat_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_sel_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_cyc_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_stb_o
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_ack_i
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_err_i
add wave -noupdate /main/DUT/U_Mux/ptp_wbm_stall_i

add wave -divider EXT
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_adr_i
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_dat_i
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_sel_i
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_cyc_i
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_stb_i
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_ack_o
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_err_o
#add wave -noupdate /main/DUT/U_Mux/ext_wbs_stall_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_adr_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_dat_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_sel_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_cyc_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_stb_o
add wave -noupdate /main/DUT/U_Mux/ext_wbm_ack_i
add wave -noupdate /main/DUT/U_Mux/ext_wbm_err_i
add wave -noupdate /main/DUT/U_Mux/ext_wbm_stall_i

add wave -divider MUX
#add wave -noupdate /main/DUT/U_Mux/mux
#add wave -noupdate /main/DUT/U_Mux/mux_last
#add wave -noupdate /main/DUT/U_Mux/mux_extdat_reg
#add wave -noupdate /main/DUT/U_Mux/mux_ptpdat_reg
#add wave -noupdate /main/DUT/U_Mux/mux_extadr_reg
#add wave -noupdate /main/DUT/U_Mux/mux_ptpadr_reg
#add wave -noupdate /main/DUT/U_Mux/mux_extsel_reg
#add wave -noupdate /main/DUT/U_Mux/mux_ptpsel_reg
#add wave -noupdate /main/DUT/U_Mux/mux_extcyc_reg
#add wave -noupdate /main/DUT/U_Mux/mux_ptpcyc_reg
#add wave -noupdate /main/DUT/U_Mux/mux_extstb_reg
#add wave -noupdate /main/DUT/U_Mux/mux_ptpstb_reg
#add wave -noupdate /main/DUT/U_Mux/mux_pend_ext
#add wave -noupdate /main/DUT/U_Mux/mux_pend_ptp
#add wave -noupdate /main/DUT/U_Mux/force_stall
add wave -noupdate /main/DUT/U_Mux/demux
add wave -noupdate /main/DUT/U_Mux/dmux_stall_mask
add wave -noupdate /main/DUT/U_Mux/dmux_status_reg
add wave -noupdate /main/DUT/U_Mux/ep_stall_mask
add wave -noupdate /main/DUT/U_Mux/ptp_select
add wave -noupdate /main/DUT/U_Mux/ext_select
add wave -noupdate /main/DUT/U_Mux/ptp_send_status
add wave -noupdate /main/DUT/U_Mux/ext_send_status
add wave -noupdate /main/DUT/U_Mux/dmux_status_class
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {50325670 fs} 0}
configure wave -namecolwidth 350
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 fs} {262500 ns}
