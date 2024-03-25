//------------------------------------------------------------------------------
// interface to the icache request / response ports
//------------------------------------------------------------------------------
interface icache_intf (
    input logic clk
);
    // request / response between CPU core and dcache
    ariane_pkg::icache_dreq_i_t req;
    ariane_pkg::icache_dreq_o_t resp;

endinterface
