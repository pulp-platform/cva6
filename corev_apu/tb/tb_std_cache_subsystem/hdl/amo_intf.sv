// interface to the AMO request / response ports
interface amo_intf (
    input logic clk
);
    import ariane_pkg::*; 

    // request / response
    amo_req_t  req;
    amo_resp_t resp;

    // this should be hooked up to the miss_handler instance
    // to detect when a request is accepted
    logic  gnt;

endinterface
