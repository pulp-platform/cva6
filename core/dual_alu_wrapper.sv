module dual_alu_wrap
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type fu_data_t = logic
) (
    input  logic                clk_i,            // Clock
    input  logic                rst_ni,           // Asynchronous reset active low
    input  fusion_t             fuse_i,
    input  fu_data_t     [1:0]  fu_data_i,
    output logic [1:0][CVA6Cfg.XLEN-1:0]  result_o,
    output logic         alu_branch_res_o
);

    fu_data_t fu_data;
    logic [1:0][CVA6Cfg.XLEN-1:0] result;
    logic [1:0][  $clog2(riscv::XLEN) : 0] cpop;  // Count Population

    logic [1:0][riscv::XLEN-1:0] operand_a_bitmanip;


    always_comb begin
        fu_data = fu_data_i[0];

        if (fuse_i.rs1_into_rd) begin
            fu_data.operand_a = result[1];
        end
        if (fuse_i.rs2_into_rd) begin
            fu_data.operand_b = result[1];
        end
    end

    // alu_i module will be used with the one_cycle_data output -> FLU
    alu #(
        .CVA6Cfg( CVA6Cfg ),
        .fu_data_t ( fu_data_t )
    ) alu_i (
        .clk_i              ( clk_i             ),
        .rst_ni             ( rst_ni            ),
        .fu_data_i          ( fu_data           ),
        .result_o           ( result[0]         ),
        .alu_branch_res_o   ( /* Unconnected */ )
    );


    alu #(
        .CVA6Cfg(CVA6Cfg),
        .fu_data_t ( fu_data_t )
    ) alu2_i (
        .clk_i              ( clk_i           ),
        .rst_ni             ( rst_ni          ),
        .fu_data_i          ( fu_data_i[1]    ),
        .result_o           ( result[1]       ),
        .alu_branch_res_o   (alu_branch_res_o )
    );

    if (CVA6Cfg.RVZCB) begin : gen_bitman_cpop
        always_comb begin
            operand_a_bitmanip[0] = fu_data_i[0].operand_a;
            operand_a_bitmanip[1] = fu_data_i[1].operand_a;

            if (CVA6Cfg.IS_XLEN64 && fu_data_i[0].operation == CPOPW) begin
                operand_a_bitmanip[0] = fu_data_i[0].operand_a[31:0];
            end
            if (CVA6Cfg.IS_XLEN64 && fu_data_i[1].operation == CPOPW) begin
                operand_a_bitmanip[1] = fu_data_i[1].operand_a[31:0];
            end
        end

        popcount #(
            .INPUT_WIDTH(riscv::XLEN)
        ) i_cpop_count_1 (
            .data_i    (operand_a_bitmanip[0]),
            .popcount_o(cpop[0])
        );

        popcount #(
            .INPUT_WIDTH(riscv::XLEN)
        ) i_cpop_count_2 (
            .data_i    (operand_a_bitmanip[1]),
            .popcount_o(cpop[1])
        );
    end else begin
        assign cpop = '0;
    end

    assign result_o[0] = (fu_data_i[0].operation inside {CPOP, CPOPW}) ? {{(riscv::XLEN - ($clog2(riscv::XLEN) + 1)) {1'b0}}, cpop[0]} : result[0];
    assign result_o[1] = (fu_data_i[1].operation inside {CPOP, CPOPW}) ? {{(riscv::XLEN - ($clog2(riscv::XLEN) + 1)) {1'b0}}, cpop[1]} : result[1];

endmodule
