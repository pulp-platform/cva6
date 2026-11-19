# CVA6 top level
if {![info exists CVA6]} {
    set CVA6 "/test/i_dut/i_cva6"
}
set Core "${CVA6}/gen_cva6/i_cva6"
set Pipeline "${Core}/i_cva6_pipeline"

add wave -group CVA6 ${CVA6}/clk_i
add wave -group CVA6 ${CVA6}/rst_ni
add wave -group CVA6 ${CVA6}/boot_addr_i
add wave -group CVA6 ${CVA6}/hart_id_i
add wave -group CVA6 ${CVA6}/irq_i
add wave -group CVA6 ${CVA6}/ipi_i
add wave -group CVA6 ${CVA6}/time_irq_i
add wave -group CVA6 ${CVA6}/debug_req_i
add wave -group CVA6 ${CVA6}/noc_req_o
add wave -group CVA6 ${CVA6}/noc_resp_i

# add wave -divider "Internal signals"

add wave -group CVA6 ${Pipeline}/priv_lvl
add wave -group CVA6 ${Pipeline}/v
add wave -group CVA6 ${Pipeline}/eret
add wave -group CVA6 ${Pipeline}/resolved_branch
add wave -group CVA6 ${Pipeline}/ex_commit
add wave -group CVA6 ${Pipeline}/pc_commit
add wave -group CVA6 ${Pipeline}/commit_ack

add wave -group CVA6 ${Pipeline}/fetch_valid_if_id
add wave -group CVA6 ${Pipeline}/fetch_ready_id_if
add wave -group CVA6 ${Pipeline}/fetch_entry_if_id

add wave -group CVA6 ${Pipeline}/issue_entry_valid_id_issue
add wave -group CVA6 ${Pipeline}/issue_instr_issue_id
add wave -group CVA6 ${Pipeline}/issue_entry_id_issue

add wave -group CVA6 ${Pipeline}/commit_ack_commit_id
add wave -group CVA6 ${Pipeline}/commit_instr_id_commit

# Frontend
set Frontend "${Pipeline}/i_frontend"

add wave -group CVA6 -group Frontend ${Frontend}/clk_i
add wave -group CVA6 -group Frontend ${Frontend}/rst_ni
# add wave -group CVA6 -group Frontend ${Frontend}/boot_addr_i
add wave -group CVA6 -group Frontend ${Frontend}/flush_bp_i
add wave -group CVA6 -group Frontend ${Frontend}/flush_i
add wave -group CVA6 -group Frontend ${Frontend}/halt_i
add wave -group CVA6 -group Frontend ${Frontend}/halt_frontend_i
add wave -group CVA6 -group Frontend ${Frontend}/set_pc_commit_i
add wave -group CVA6 -group Frontend ${Frontend}/pc_commit_i
add wave -group CVA6 -group Frontend ${Frontend}/ex_valid_i
add wave -group CVA6 -group Frontend ${Frontend}/resolved_branch_i
add wave -group CVA6 -group Frontend ${Frontend}/eret_i
# add wave -group CVA6 -group Frontend ${Frontend}/epc_i
# add wave -group CVA6 -group Frontend ${Frontend}/trap_vector_base_i
add wave -group CVA6 -group Frontend ${Frontend}/set_debug_pc_i
add wave -group CVA6 -group Frontend ${Frontend}/debug_mode_i
add wave -group CVA6 -group Frontend ${Frontend}/ypb_fetch_req_o
add wave -group CVA6 -group Frontend ${Frontend}/ypb_fetch_rsp_i
add wave -group CVA6 -group Frontend ${Frontend}/fetch_entry_o
add wave -group CVA6 -group Frontend ${Frontend}/fetch_entry_valid_o
add wave -group CVA6 -group Frontend ${Frontend}/fetch_entry_ready_i

# Cache subsystem
set Caches "${Core}/gen_cache_subsystem/i_cache_subsystem"

add wave -group CVA6 -group Caches ${Caches}/clk_i
add wave -group CVA6 -group Caches ${Caches}/icache_enable_i
add wave -group CVA6 -group Caches ${Caches}/icache_flush_i
add wave -group CVA6 -group Caches ${Caches}/icache_miss_o
add wave -group CVA6 -group Caches ${Caches}/ypb_fetch_req_i
add wave -group CVA6 -group Caches ${Caches}/ypb_fetch_rsp_o
add wave -group CVA6 -group Caches ${Caches}/dcache_enable_i
add wave -group CVA6 -group Caches ${Caches}/dcache_flush_i
add wave -group CVA6 -group Caches ${Caches}/dcache_flush_ack_o
add wave -group CVA6 -group Caches ${Caches}/dcache_miss_o
add wave -group CVA6 -group Caches ${Caches}/ypb_load_req_i
add wave -group CVA6 -group Caches ${Caches}/ypb_load_rsp_o
add wave -group CVA6 -group Caches ${Caches}/ypb_store_req_i
add wave -group CVA6 -group Caches ${Caches}/ypb_store_rsp_o
add wave -group CVA6 -group Caches ${Caches}/ypb_amo_req_i
add wave -group CVA6 -group Caches ${Caches}/ypb_amo_rsp_o
add wave -group CVA6 -group Caches ${Caches}/ypb_mmu_ptw_req_i
add wave -group CVA6 -group Caches ${Caches}/ypb_mmu_ptw_rsp_o
add wave -group CVA6 -group Caches ${Caches}/wbuffer_empty_o
add wave -group CVA6 -group Caches ${Caches}/noc_req_o
add wave -group CVA6 -group Caches ${Caches}/noc_resp_i
