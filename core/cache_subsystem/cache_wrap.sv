module cache_wrap
  import ariane_pkg::*;
  import std_cache_pkg::*;
#(
  parameter bit EnableEcc = 0,
  localparam int unsigned DcacheSetAss = ariane_pkg::DCACHE_SET_ASSOC,
  localparam int unsigned DcacheIdxWidth = ariane_pkg::DCACHE_INDEX_WIDTH,
  localparam int unsigned DcacheLineWidth = ariane_pkg::DCACHE_LINE_WIDTH,
  localparam int unsigned DcacheTagWidth = ariane_pkg::DCACHE_TAG_WIDTH,
  localparam int unsigned DcacheNumWords = std_cache_pkg::DCACHE_NUM_WORDS,
  localparam int unsigned DcacheByteOffs = std_cache_pkg::DCACHE_BYTE_OFFSET,
  localparam type cache_line_t = std_cache_pkg::cache_line_t,
  localparam type cl_be_t = std_cache_pkg::cl_be_t,
  localparam type vldrty_t = std_cache_pkg::vldrty_t
)(
  input logic clk_i,
  input logic rst_ni,
  // D$ interface
  input logic [DcacheSetAss-1:0] dcache_req_i,
  input logic [DcacheIdxWidth-1:0] dcache_addr_i,
  input logic dcache_we_i,
  input cache_line_t dcache_wdata_i,
  input cl_be_t dcache_be_i,
  output cache_line_t [DcacheSetAss-1:0] dcache_rdata_o
);

for (genvar i = 0; i < DcacheSetAss; i++) begin : sram_block
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
endmodule
