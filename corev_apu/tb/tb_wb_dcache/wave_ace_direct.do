add wave -group TB -position insertpoint sim:/tb_ace_direct/*
add wave -group snoop_cache_ctrl -position insertpoint sim:/tb_ace_direct/i_dut/i_snoop_cache_ctrl/*
add wave -group req_port_0 -position insertpoint {sim:/tb_ace_direct/i_dut/master_ports[1]/i_cache_ctrl/*}
add wave -group miss_handler -position insertpoint sim:/tb_ace_direct/i_dut/i_miss_handler/*
add wave -position insertpoint sim:/tb_ace_direct/i_dut/valid_dirty_sram/gen_cut[0]/gen_mem/i_tc_sram_wrapper/i_tc_sram/sram
add wave -position insertpoint sim:/tb_ace_direct/cache_status

run -all
