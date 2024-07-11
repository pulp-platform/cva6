//------------------------------------------------------------------------------
// interface to the icache request / response ports
//------------------------------------------------------------------------------
interface icache_intf (
    input logic clk
);
    // request / response between CPU core and dcache
    ariane_pkg::icache_dreq_t req;
    ariane_pkg::icache_drsp_t resp;

endinterface
