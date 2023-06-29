//------------------------------------------------------------------------------
// interface to probe SRAM
//------------------------------------------------------------------------------
interface sram_intf #(
    parameter int unsigned DCACHE_SET_ASSOC = 0,
    parameter int unsigned DATA_WIDTH       = 0,
    parameter int unsigned NUM_WORDS        = 0
);
    // interface for probing into sram
    typedef logic [$clog2(NUM_WORDS)-1:0] addr_t;
    typedef logic [1:0][DATA_WIDTH-1:0]   data_t;

    // SRAM is too large for probing as a single object, probe one set at a time
    addr_t addr [DCACHE_SET_ASSOC];
    data_t data [DCACHE_SET_ASSOC];

endinterface

