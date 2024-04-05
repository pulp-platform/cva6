module cache_wrap
  import ariane_pkg::*;
  import std_cache_pkg::*;
  import wt_cache_pkg::*;
#(
  parameter bit EnableEcc = 0,
  localparam int unsigned DcacheSetAss = ariane_pkg::DCACHE_SET_ASSOC,
  localparam int unsigned DcacheIdxWidth = ariane_pkg::DCACHE_INDEX_WIDTH,
  localparam int unsigned DcacheLineWidth = ariane_pkg::DCACHE_LINE_WIDTH,
  localparam int unsigned DcacheTagWidth = ariane_pkg::DCACHE_TAG_WIDTH,
  localparam int unsigned DcacheNumWords = std_cache_pkg::DCACHE_NUM_WORDS,
  localparam int unsigned DcacheByteOffs = std_cache_pkg::DCACHE_BYTE_OFFSET,
  localparam int unsigned IcacheSetAss = ariane_pkg::ICACHE_SET_ASSOC,
  localparam int unsigned IcacheTagWidth = ariane_pkg::ICACHE_TAG_WIDTH,
  localparam int unsigned IcacheUserLineWidth = ariane_pkg::ICACHE_USER_LINE_WIDTH,
  localparam int unsigned IcacheLineWidth = ariane_pkg::ICACHE_LINE_WIDTH,
  localparam int unsigned IcacheClIdxWidth = wt_cache_pkg::ICACHE_CL_IDX_WIDTH,
  localparam int unsigned IcacheNumWords = wt_cache_pkg::ICACHE_NUM_WORDS,
  localparam type cache_line_t = std_cache_pkg::cache_line_t,
  localparam type cl_be_t = std_cache_pkg::cl_be_t,
  localparam type vldrty_t = std_cache_pkg::vldrty_t,
  localparam type icache_rtrn_t = wt_cache_pkg::icache_rtrn_t
)(
  input logic clk_i,
  input logic rst_ni,
  // D$ interface
  input logic [DcacheSetAss-1:0] dcache_req_i,
  input logic [DcacheIdxWidth-1:0] dcache_addr_i,
  input logic dcache_we_i,
  input cache_line_t dcache_wdata_i,
  input cl_be_t dcache_be_i,
  output cache_line_t [DcacheSetAss-1:0] dcache_rdata_o,
  // I$ interface
  input  logic [IcacheSetAss-1:0] icache_tag_req_i,
  input  logic icache_tag_we_i,
  input  logic [IcacheClIdxWidth-1:0] icache_tag_addr_i,
  input  logic [IcacheTagWidth-1:0] icache_tag_wdata_i,
  input  logic [IcacheSetAss-1:0] icache_tag_wdata_valid_i,
  output logic [IcacheSetAss-1:0][IcacheTagWidth:0] icache_tag_rdata_o,
  input  logic [IcacheSetAss-1:0] icache_data_req_i,
  input  logic icache_data_we_i,
  input  logic [IcacheClIdxWidth-1:0] icache_data_addr_i,
  input  icache_rtrn_t icache_data_wdata_i,
  output logic [IcacheSetAss-1:0][IcacheUserLineWidth-1:0] icache_data_ruser_o,
  output logic [IcacheSetAss-1:0][IcacheLineWidth-1:0] icache_data_rdata_o
);

for (genvar i = 0; i < DcacheSetAss; i++) begin : gen_dcache_sram
  sram #(
      .DATA_WIDTH(DcacheLineWidth),
      .NUM_WORDS (DcacheNumWords),
      .ENABLE_ECC(EnableEcc),
      .ECC_GRANULARITY(32)
  ) data_sram (
      .rst_ni (rst_ni),
      .req_i  (dcache_req_i[i]),
      .we_i   (dcache_we_i),
      .addr_i (dcache_addr_i[DcacheIdxWidth-1:DcacheByteOffs]),
      .wuser_i('0),
      .wdata_i(dcache_wdata_i.data),
      .be_i   (dcache_be_i.data),
      .ruser_o(),
      .rdata_o(dcache_rdata_o[i].data),
      .error_o(/* TODO: Connect */),
      .user_error_o(/* TODO: Connect */),
      .*
  );

  sram #(
      .DATA_WIDTH(DcacheTagWidth),
      .NUM_WORDS (DcacheNumWords),
      .ENABLE_ECC(EnableEcc)
  ) tag_sram (
      .rst_ni (rst_ni),
      .req_i  (dcache_req_i[i]),
      .we_i   (dcache_we_i),
      .addr_i (dcache_addr_i[DcacheIdxWidth-1:DcacheByteOffs]),
      .wuser_i('0),
      .wdata_i(dcache_wdata_i.tag),
      .be_i   (dcache_be_i.tag),
      .ruser_o(),
      .rdata_o(dcache_rdata_o[i].tag),
      .error_o(/* TODO: Connect */),
      .user_error_o(/* TODO: Connect */),
      .*
  );

end

// ----------------
// Valid/Dirty Regs
// ----------------
vldrty_t [DcacheSetAss-1:0] dirty_wdata, dirty_rdata, be_valid_dirty_ram;

for (genvar i = 0; i < DcacheSetAss; i++) begin
  assign dirty_wdata[i] = '{dirty: dcache_wdata_i.dirty, valid: dcache_wdata_i.valid};
  assign dcache_rdata_o[i].dirty = dirty_rdata[i].dirty;
  assign dcache_rdata_o[i].valid = dirty_rdata[i].valid;
  assign be_valid_dirty_ram[i].valid = dcache_be_i.vldrty[i].valid;
  assign be_valid_dirty_ram[i].dirty = dcache_be_i.vldrty[i].dirty;
end

sram #(
    .USER_WIDTH(1),
    .DATA_WIDTH(DcacheSetAss * $bits(vldrty_t)),
    .BYTE_WIDTH(1),
    .NUM_WORDS (DcacheNumWords),
    .ENABLE_ECC(EnableEcc),
    .ECC_GRANULARITY(8) // TODO: fix to use 32
) valid_dirty_sram (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .req_i  (|dcache_req_i),
    .we_i   (dcache_we_i),
    .addr_i (dcache_addr_i[DcacheIdxWidth-1:DcacheByteOffs]),
    .wuser_i('0),
    .wdata_i(dirty_wdata),
    .be_i   (be_valid_dirty_ram),
    .ruser_o(),
    .rdata_o(dirty_rdata),
    .error_o(/* TODO: Connect */),
    .user_error_o(/* TODO: Connect */)
);

for (genvar i = 0; i < IcacheSetAss; i++) begin : gen_icache_sram
  // Tag RAM
  sram #(
      // tag + valid bit
      .DATA_WIDTH(IcacheTagWidth + 1),
      .NUM_WORDS (IcacheNumWords),
      .ENABLE_ECC (EnableEcc)
  ) tag_sram (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .req_i  (icache_tag_req_i[i]),
      .we_i   (icache_tag_we_i),
      .addr_i (icache_tag_addr_i),
      // we can always use the saved tag here since it takes a
      // couple of cycle until we write to the cache upon a miss
      .wuser_i('0),
      .wdata_i({icache_tag_wdata_valid_i[i], icache_tag_wdata_i}),
      .be_i   ('1),
      .ruser_o(),
      .rdata_o(icache_tag_rdata_o[i]),
      .error_o      (/* TODO: Connect */),
      .user_error_o (/* TODO: Connect */)
  );

  // Data RAM
  sram #(
      .USER_WIDTH(IcacheUserLineWidth),
      .DATA_WIDTH(IcacheLineWidth),
      .USER_EN   (ariane_pkg::FETCH_USER_EN),
      .NUM_WORDS (IcacheNumWords),
      .ENABLE_ECC(EnableEcc),
      .ECC_GRANULARITY(32)
  ) data_sram (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .req_i  (icache_data_req_i[i]),
      .we_i   (icache_data_we_i),
      .addr_i (icache_data_addr_i),
      .wuser_i(icache_data_wdata_i.user),
      .wdata_i(icache_data_wdata_i.data),
      .be_i   ('1),
      .ruser_o(icache_data_ruser_o[i]),
      .rdata_o(icache_data_rdata_o[i]),
      .error_o(/* TODO: Connect */),
      .user_error_o(/* TODO: Connect */)
  );
end
endmodule
