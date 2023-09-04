//------------------------------------------------------------------------------
// interface to the dcache request / response ports
//------------------------------------------------------------------------------
interface dcache_intf (
    input logic clk
);
    import ariane_pkg::*; 

    // request / response between CPU core and dcache 
    dcache_req_i_t req;
    dcache_req_o_t resp;

    // this should be hooked up to (cache_ctrl.gnt_i && (cache_ctrl.state==IDLE))
    // to detect when a write request is accepted
    logic          wr_gnt;

endinterface


//------------------------------------------------------------------------------
// interface to the dcache management ports
//------------------------------------------------------------------------------
interface dcache_mgmt_intf (
    input logic clk
);

    logic dcache_enable;        // from CSR
    logic dcache_flush;         // high until acknowledged
    logic dcache_flushing;      // flushing started
    logic dcache_flush_ack;     // send a single cycle acknowledge signal when the cache is flushed
    logic dcache_miss;          // we missed on a ld/st
    logic wbuffer_empty;        // statically set to 1, as there is no wbuffer in this cache system

    modport dut (
        input  dcache_enable, dcache_flush,
        output dcache_flushing, dcache_flush_ack, dcache_miss, wbuffer_empty
    );

    modport driver (
        output dcache_enable, dcache_flush,
        input  dcache_flushing, dcache_flush_ack, dcache_miss, wbuffer_empty
    );

endinterface

//------------------------------------------------------------------------------
// interface to probe cache internal SRAMs
//------------------------------------------------------------------------------
interface dcache_sram_if (input logic clk);
    import ariane_pkg::*;
    import std_cache_pkg::*;

    // interface for probing into sram

    typedef logic [4*DCACHE_DIRTY_WIDTH-1:0] vld_t;
    typedef logic [63:0]                     data_t;
    typedef logic [63:0]                     tag_t;
    typedef data_t                           data_sram_t [DCACHE_NUM_WORDS-1:0];
    typedef tag_t                            tag_sram_t  [DCACHE_NUM_WORDS-1:0];
    typedef vld_t                            vld_sram_t  [DCACHE_NUM_WORDS-1:0];

    data_sram_t                                       data_sram [1:0][DCACHE_SET_ASSOC-1:0];
    tag_sram_t                                        tag_sram       [DCACHE_SET_ASSOC-1:0];
    vld_sram_t                                        vld_sram;
    logic                                             vld_req;
    logic                                             vld_we;
    logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] vld_index;


endinterface

//------------------------------------------------------------------------------
// interface to probe internal cache grant signals
//------------------------------------------------------------------------------
interface dcache_gnt_if (input logic clk);
    logic [4:0] gnt;
    logic [4:0] rd_gnt;
    logic [2:0] bypass_gnt;
    logic [2:0] miss_gnt;
endinterface
