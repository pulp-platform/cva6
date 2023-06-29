// Description: test utilities for the standard Ariane cache subsystem.
// main package definition
package tb_std_cache_subsystem_pkg;
    import ariane_pkg::*;
    import snoop_test::*;
    import std_cache_pkg::*;

    // definitions for dcache request and response
    typedef enum {WR_REQ, RD_REQ, RD_RESP, WR_RESP, EVICT, READBACK} dcache_trans_t;

    // definitions for dcache management transactions
    typedef enum {FLUSH_REQ} dcache_mgmt_trans_t;

    // definitions for amo request and response
    typedef enum {AMO_WR_REQ, AMO_RD_REQ, AMO_RD_RESP} amo_trans_t;

    // enum for snoop type to get better view in waveform
    typedef enum logic [3:0] {
        READ_ONCE             = snoop_pkg::READ_ONCE,
        READ_SHARED           = snoop_pkg::READ_SHARED,
        READ_CLEAN            = snoop_pkg::READ_CLEAN,
        READ_NOT_SHARED_DIRTY = snoop_pkg::READ_NOT_SHARED_DIRTY,
        READ_UNIQUE           = snoop_pkg::READ_UNIQUE,
        CLEAN_SHARED          = snoop_pkg::CLEAN_SHARED,
        CLEAN_INVALID         = snoop_pkg::CLEAN_INVALID,
        CLEAN_UNIQUE          = snoop_pkg::CLEAN_UNIQUE,
        MAKE_INVALID          = snoop_pkg::MAKE_INVALID,
        DVM_COMPLETE          = snoop_pkg::DVM_COMPLETE,
        DVM_MESSAGE           = snoop_pkg::DVM_MESSAGE
    } acsnoop_enum_t;


    //--------------------------------------------------------------------------
    // Helper functions
    //--------------------------------------------------------------------------

    // define min and max functions
    let max(a,b) = (a > b) ? a : b;
    let min(a,b) = (a < b) ? a : b;

    // get tag from address
    function automatic logic [DCACHE_TAG_WIDTH-1:0] addr2tag (input logic[63:0] addr);
        return addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH];
    endfunction

    // get index from address
    function automatic logic [DCACHE_INDEX_WIDTH-1:0] addr2index (input logic[63:0] addr);
        return addr[DCACHE_INDEX_WIDTH-1:0];
    endfunction

    // get mem_idx from address
    function automatic logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] addr2mem_idx (input logic[63:0] addr);
        return addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET];
    endfunction

    // get address from index and tag
    function automatic logic [63:0] tag_index2addr (
        input logic [DCACHE_TAG_WIDTH-1:0]   tag,
        input logic [DCACHE_INDEX_WIDTH-1:0] index
    );
        return {tag, index};
    endfunction

    // update part of cache line with <data> at <offset>
    function automatic void update_cache_line (
        inout logic [DCACHE_LINE_WIDTH-1:0] cache_line,
        input riscv::xlen_t                 data,
        input int unsigned                  offset // in units of data width
    );
        logic [riscv::XLEN-1:0]       data_mask;
        logic [DCACHE_LINE_WIDTH-1:0] line_mask;

        data_mask = '1;
        line_mask = data_mask; // zero-extend

        cache_line = ((line_mask & data) << (offset * riscv::XLEN)) | (cache_line & ~(line_mask << (offset * riscv::XLEN)));
    endfunction


    function automatic logic [63:0] get_rand_addr_from_cfg(ariane_cfg_t cfg);
        logic [63:0] start_addr, end_addr;
        logic [31:0] addr_msb, addr_lsb;
        int region;

        region = $urandom_range(2);
        case (region)
            0 : begin
                start_addr = cfg.ExecuteRegionAddrBase[0];
                end_addr   = cfg.ExecuteRegionAddrBase[0] + cfg.ExecuteRegionLength[0];
            end
            1 : begin
                start_addr = cfg.CachedRegionAddrBase[0];
                end_addr   = cfg.CachedRegionAddrBase[0] + cfg.CachedRegionLength[0];
            end
            2 : begin
                start_addr = cfg.SharedRegionAddrBase[0];
                end_addr   = cfg.SharedRegionAddrBase[0] + cfg.SharedRegionLength[0];
            end
        endcase

        a_lsb_zero : assert (start_addr[31:0] === 0 && end_addr[31:0] === 0) else
            $error("expected 32 LSB zeros for address range");

        addr_msb = $urandom_range((end_addr-1) >> 32, start_addr>>32);
        addr_lsb = $urandom;

        return {addr_msb, addr_lsb};

    endfunction


    //--------------------------------------------------------------------------
    // AMO request class
    //--------------------------------------------------------------------------
    class amo_req;
        amo_t        op;
        logic [63:0] addr; // address
        logic [63:0] data; // data as layouted in the register

        function string print_me();
            return $sformatf("type %0s, address 0x%16h, data 0x%16h", op.name(), addr, data);
        endfunction
    endclass


    //--------------------------------------------------------------------------
    // AMO response class
    //--------------------------------------------------------------------------
    class amo_resp;
        amo_t        op;
        logic [63:0] data;
    endclass


    //--------------------------------------------------------------------------
    // Driver for the AMO interface
    //--------------------------------------------------------------------------
    class amo_driver;

        virtual amo_intf vif;
        string name;
        int verbosity;
        ariane_cfg_t cfg;

        function new (virtual amo_intf vif, ariane_cfg_t cfg, string name="amo_driver");
            this.vif = vif;
            vif.req = '0;
            this.name=name;
            verbosity = 1;
            this.cfg = cfg;
        endfunction

        // request
        task req (
            input logic [63:0] data         = '0,
            input logic [63:0] addr         = '0,
            input amo_t        op           = AMO_ADD,
            input bit          rand_data    = 0,
            input bit          rand_addr    = 0,
            input bit          rand_op      = 0,
            input bit          check_result = 1'b0,
            input logic [63:0] exp_result   = '0
        );
            logic [63:0] addr_int;
            logic [63:0] data_int;
            amo_t        op_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (rand_data) begin
                data_int = {$urandom, $urandom};
            end else begin
                data_int = data;
            end

            if (rand_op) begin
                if ($urandom_range(3) > 2) begin
                    // increase chance for AMO_LR
                    op_int = AMO_LR;
                end else begin
                    op_int = amo_t'($urandom_range(AMO_MINU, AMO_LR)); // avoid sending AMO_NONE and unsupported AMO_CAS1,AMO_CAS2
                end
            end else begin
                op_int = op;
            end

            if (verbosity > 0) begin
                $display("%t ns %s sending AMO request %s to address 0x%8h with data 0x%8h", $time, name, op_int.name(), addr_int, data_int);
            end

            #0;
            vif.req.req       = 1;        // this request is valid
            vif.req.amo_op    = op_int;   // atomic memory operation to perform
            vif.req.size      = 2'b11;    // 2'b10 --> word operation, 2'b11 --> double word operation
            vif.req.operand_a = addr_int; // address
            vif.req.operand_b = data_int; // address

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.ack);

            if (verbosity > 0) begin
                $display("%t ns %s got ack for AMO request %s to address 0x%8h", $time, name, op.name(), addr_int);
            end

            if (check_result) begin
                a_rd_check : assert (vif.resp.result == exp_result) else
                    $error("%s : data mismatch. Expected 0x%16h, got 0x%16h", name, exp_result, vif.resp.result);
            end

            #0;
            vif.req.req    = 1'b0;

        endtask

    endclass


    //--------------------------------------------------------------------------
    // Monitor for the AMO interface
    //--------------------------------------------------------------------------
    class amo_monitor;

        mailbox #(amo_req)  req_mbox;
        mailbox #(amo_resp) resp_mbox;

        virtual amo_intf    vif;

        string              name;
        int                 verbosity;

        function new (virtual amo_intf vif, string name="amo_monitor");
            this.vif  = vif;
            this.name = name;
            verbosity = 1;
        endfunction

        // get read requests and responses
        local task mon;
            $display("%t ns %s monitoring AMO requests and responses", $time, name);
            forever begin
                if (vif.req.req && vif.gnt) begin // got read request
                    amo_t    op;
                    amo_req  req;
                    amo_resp resp;

                    req = new();
                    req.op   = vif.req.amo_op;
                    req.addr = vif.req.operand_a;
                    req.data = vif.req.operand_b;
                    op       = vif.req.amo_op; // remember op

                    if (verbosity > 0) begin
                        $display("%t ns %s got AMO request : %s", $time, name, req.print_me());
                    end
                    req_mbox.put(req);

                    // wait for result
                    do begin
                        @(posedge vif.clk);
                    end while (!vif.resp.ack);

                    resp      = new();
                    resp.op   = op;
                    resp.data = vif.resp.result;

                    #0; // add zero delay here to make sure read response is repoerted after read request if it gets served immediately
                    if (verbosity > 0) begin
                        $display("%t ns %s got AMO response with data 0x%8h", $time, name, resp.data);
                    end
                    resp_mbox.put(resp);

                end else begin
                    @(posedge vif.clk);
                end
            end
        endtask

        task monitor;
            mon();
        endtask

    endclass


    //--------------------------------------------------------------------------
    // dcache request
    //--------------------------------------------------------------------------
    class dcache_req;
        dcache_trans_t                       trans_type;
        logic [DCACHE_INDEX_WIDTH-1:0]       address_index;
        logic [DCACHE_TAG_WIDTH-1:0]         address_tag;
        riscv::xlen_t                        data;
        // help variables
        int                                  port_idx;
        int                                  prio;
        bit                                  update_cache;
        bit                                  insert_readback;
        bit                                  r_dirty;
        bit                                  r_shared;
        int                                  data_offset; // data offset into cache line
        logic [DCACHE_LINE_WIDTH-1:0]        cache_line;  // for carrying an entire cache line from read response
        logic [$clog2(DCACHE_SET_ASSOC)-1:0] target_way;
        logic                                target_way_valid;

        function new();
            this.target_way_valid = 1'b0;
        endfunction

        task set_data_offset;
            data_offset = address_index[3];
        endtask

        task add_to_cache_line (
            input riscv::xlen_t d
        );
            cache_line = {d, cache_line} >> (riscv::XLEN);

        endtask

        function logic [63:0] get_addr ();
            return tag_index2addr(.tag(this.address_tag), .index(this.address_index));
        endfunction


        function string print_me();
            if ((trans_type == WR_REQ) || (trans_type == RD_RESP)) begin
                return $sformatf("type %0s, tag 0x%11h, index 0x%3h, data 0x%16h",trans_type.name(), address_tag, address_index, data);
            end else if (trans_type == READBACK) begin
                return $sformatf("type %0s, tag 0x%11h, index 0x%3h, data 0x%16h_%16h",trans_type.name(), address_tag, address_index, cache_line[127:64], cache_line[63:0]);
            end else begin
                return $sformatf("type %0s, tag 0x%11h, index 0x%3h",trans_type.name(), address_tag, address_index);
            end
        endfunction

    endclass


    //--------------------------------------------------------------------------
    // dcache response
    //--------------------------------------------------------------------------
    class dcache_resp;
        dcache_trans_t trans_type;
        riscv::xlen_t  data;
    endclass


    //--------------------------------------------------------------------------
    // Driver for the CPU / data cache interface
    //--------------------------------------------------------------------------
    class dcache_driver;

        virtual dcache_intf vif;
        ariane_cfg_t cfg;
        string name;
        int verbosity;

        function new (virtual dcache_intf vif, ariane_cfg_t cfg, string name="dcache_driver");
            this.vif              = vif;
            vif.req               = '0;
            vif.req.address_tag   = $urandom;
            vif.req.address_index = $urandom;
            this.cfg              = cfg;
            this.name             = name;
            verbosity             = 0;
        endfunction

        // read request
        task rd (
            input logic [63:0] addr      = '0,
            input bit          rand_addr = 0
        );
            logic [63:0] addr_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (verbosity > 0) begin
                $display("%t ns %s sending read request for address 0x%8h", $time, name, addr_int);
            end

            #0;
            vif.req.data_req      = 1'b1;
            vif.req.data_we       = 1'b0;
            vif.req.data_be       = '1;
            vif.req.data_size     = 2'b11;
            vif.req.address_index = addr2index(addr_int);

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.data_gnt);

            fork
                // send tag while allowing a new read to start
                begin
                    if (verbosity > 0) begin
                        $display("%t ns %s got grant for read address 0x%8h, sending tag 0x%6h", $time, name, addr_int, addr2tag(addr_int));
                    end

                    #0;
                    vif.req.data_req    = 1'b0;
                    #0; // one more zero delay to "win" over an earlier read that sets tag_valid to 0
                    vif.req.tag_valid   = 1'b1;
                    vif.req.address_tag = addr2tag(addr_int);

                    @(posedge vif.clk);
                    #0;
                    vif.req.tag_valid = '0;
                end
                begin
                    ;
                end
            join_any
        endtask

        // read request, wait for read data
        task rd_wait (
            input logic [63:0] addr         = '0,
            input bit          rand_addr    = 0,
            input bit          check_result = 1'b0,
            input logic [63:0] exp_result   = '0
        );
            logic [63:0] addr_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (verbosity > 0) begin
                $display("%t ns %s sending read request for address 0x%8h", $time, name, addr_int);
            end

            #0;
            vif.req.data_req      = 1'b1;
            vif.req.data_we       = 1'b0;
            vif.req.data_be       = '1;
            vif.req.data_size     = 2'b11;
            vif.req.address_index = addr2index(addr_int);

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.data_gnt);

            if (verbosity > 0) begin
                $display("%t ns %s got grant for read address 0x%8h, sending tag 0x%6h", $time, name, addr_int, addr2tag(addr_int));
            end

            #0;
            vif.req.data_req    = 1'b0;
            #0; // one more zero delay to "win" over an earlier read that sets tag_valid to 0
            vif.req.tag_valid   = 1'b1;
            vif.req.address_tag = addr2tag(addr_int);

            do begin
                @(posedge vif.clk);
                #0;
                vif.req.tag_valid = '0;
            end while (!vif.resp.data_rvalid);

            if (check_result) begin
                a_rd_check : assert (vif.resp.data_rdata == exp_result) else
                    $error("%s : data mismatch. Expected 0x%16h, got 0x%16h", name, exp_result, vif.resp.data_rdata);
            end


        endtask

        // write request
        task wr (
            input logic [63:0] data      = 0,
            input logic [63:0] addr      = '0,
            input bit          rand_data = 0,
            input bit          rand_addr = 0
        );
            logic [63:0] addr_int;
            logic [63:0] data_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (rand_data) begin
                data_int = {$urandom,$urandom};
            end else begin
                data_int = data;
            end
            if (verbosity > 0) begin
                $display("%t ns %s sending write request for address 0x%8h with data 0x%8h", $time, name, addr_int, data_int);
            end

            #0;
            vif.req.data_req      = 1'b1;
            vif.req.data_we       = 1'b1;
            vif.req.data_be       = '1;
            vif.req.data_size     = 2'b11;
            vif.req.data_wdata    = data_int;
            vif.req.address_index = addr2index(addr_int);
            vif.req.address_tag   = addr2tag(addr_int);
            vif.req.tag_valid     = 1'b1;

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.data_gnt);

            #0;
            vif.req.data_req    = 1'b0;
            vif.req.data_we     = 1'b0;
            vif.req.tag_valid   = 1'b0;

        endtask

    endclass


    //--------------------------------------------------------------------------
    // Monitor for the LSU / data cache interface
    //--------------------------------------------------------------------------
    class dcache_monitor;

        mailbox #(dcache_req)  req_mbox;
        mailbox #(dcache_resp) resp_mbox;

        virtual dcache_intf    vif;

        string                 name;
        int                    verbosity;
        int                    port_idx;

        function new (virtual dcache_intf vif, int port_idx=0, string name="dcache_monitor");
            this.vif       = vif;
            this.name      = name;
            this.port_idx  = port_idx;
            verbosity = 0;
        endfunction

        // get read requests
        local task mon_rd_req;
            dcache_req rd_req;
            $display("%t ns %s monitoring read requests", $time, name);
            forever begin
                if (vif.req.data_req && !vif.req.data_we) begin // got read request

                    while (!vif.resp.data_gnt) begin
                        @(posedge vif.clk);
                    end
                    if (verbosity > 0) begin
                        $display("%t ns %s got request for read", $time, name);
                    end

                    rd_req = new();
                    rd_req.trans_type      = RD_REQ;
                    rd_req.address_index = vif.req.address_index;
                    rd_req.port_idx      = port_idx;

                    @(posedge vif.clk);
                    while (!vif.req.tag_valid) begin
                        @(posedge vif.clk);
                    end

                    rd_req.address_tag = vif.req.address_tag;
                    rd_req.set_data_offset();
                    if (verbosity > 0) begin
                        $display("%t ns %s got request for read tag 0x%6h, index 0x%3h", $time, name, rd_req.address_tag, rd_req.address_index);
                    end
                    req_mbox.put(rd_req);

                end else begin
                    @(posedge vif.clk);
                end
            end
        endtask

        // get read responses
        local task mon_rd_resp;
            dcache_resp rd_resp;
            $display("%t ns %s monitoring read responses", $time, name);
            forever begin
                if (vif.resp.data_rvalid) begin // got read request
                    rd_resp = new();
                    rd_resp.trans_type = RD_RESP;
                    rd_resp.data = vif.resp.data_rdata;
                    #0; // add zero delay here to make sure read response is repoerted after read request if it gets served immediately
                    if (verbosity > 0) begin
                        $display("%t ns %s got read response with data 0x%8h", $time, name, rd_resp.data);
                    end
                    resp_mbox.put(rd_resp);
                end
                @(posedge vif.clk);
            end
        endtask

        // get write requests
        local task mon_wr_req;
            dcache_req  wr_req;
            dcache_resp wr_resp;
            $display("%t ns %s monitoring write requests", $time, name);
            forever begin
                if (vif.req.data_req && vif.req.data_we && vif.wr_gnt) begin // got write request
                    wr_req = new();
                    wr_req.trans_type      = WR_REQ;
                    wr_req.address_index = vif.req.address_index;
                    wr_req.address_tag   = vif.req.address_tag;
                    wr_req.data          = vif.req.data_wdata;
                    wr_req.port_idx      = port_idx;
                    wr_req.set_data_offset();

                    if (verbosity > 0) begin
                        $display("%t ns %s got request for write tag 0x%6h, index 0x%3h, data 0x%8h", $time, name, wr_req.address_tag, wr_req.address_index, wr_req.data);
                    end
                    req_mbox.put(wr_req);

                    while (!vif.resp.data_gnt) begin
                        @(posedge vif.clk);
                    end
                    wr_resp = new();
                    wr_resp.trans_type = WR_RESP;
                    resp_mbox.put(wr_resp);
                end
                @(posedge vif.clk);
            end
        endtask

        task monitor;
            fork
                mon_rd_req();
                mon_rd_resp();
                mon_wr_req();
            join
        endtask

    endclass



    //--------------------------------------------------------------------------
    // dcache management transaction
    //--------------------------------------------------------------------------
    class dcache_mgmt_trans;
        dcache_mgmt_trans_t            trans_type;

        function string print_me();
            return $sformatf("type %0s",trans_type.name());
        endfunction

    endclass


    //--------------------------------------------------------------------------
    // Driver for the dcache management interface
    //--------------------------------------------------------------------------
    class dcache_mgmt_driver;

        virtual dcache_mgmt_intf vif;
        string name;
        int verbosity;

        function new (virtual dcache_mgmt_intf vif, string name="dcache_driver");
            this.vif = vif;
            vif.dcache_enable = 1'b1;
            vif.dcache_flush  = 1'b0;
            this.name=name;
            verbosity = 0;
        endfunction

        // flush
        task flush ();

            #0;
            vif.dcache_flush = 1'b1;

            if (verbosity > 0) begin
                $display("%t ns %s requesting flush", $time, name);
            end

            do begin
                @(posedge vif.clk);
            end while (!vif.dcache_flush_ack);
            #0;
            vif.dcache_flush = 1'b0;

            if (verbosity > 0) begin
                $display("%t ns %s flush done", $time, name);
            end
        endtask
    endclass


    //--------------------------------------------------------------------------
    // Monitor for the dcache management interface
    //--------------------------------------------------------------------------
    class dcache_mgmt_monitor;

        mailbox #(dcache_mgmt_trans) mbox;

        virtual dcache_mgmt_intf     vif;

        string                       name;
        int                          verbosity;

        function new (virtual dcache_mgmt_intf vif, string name="dcache_mgmt_monitor");
            this.vif  = vif;
            this.name = name;
            verbosity = 0;
        endfunction

        // get flush requests
        local task mon_flush;
            dcache_mgmt_trans trans;
            $display("%t ns %s monitoring flush requests", $time, name);
            forever begin
                if (vif.dcache_flush) begin // got flush request
                    trans = new();
                    trans.trans_type = FLUSH_REQ;
                    if (verbosity > 0) begin
                        $display("%t ns %s got flush request", $time, name);
                    end
                    mbox.put(trans);

                    // wait for ack
                    while (!vif.dcache_flush_ack) begin
                        @(posedge vif.clk);
                    end
                end
                @(posedge vif.clk);
            end
        endtask

        task monitor;
            mon_flush();
        endtask

    endclass



    //--------------------------------------------------------------------------
    // scoreboard
    //--------------------------------------------------------------------------
    class std_cache_scoreboard #(
        parameter int unsigned AXI_ADDR_WIDTH = 0,
        parameter int unsigned AXI_DATA_WIDTH = 0,
        parameter int unsigned AXI_ID_WIDTH   = 0,
        parameter int unsigned AXI_USER_WIDTH = 0
    );

        typedef ace_test::ace_driver #(
            .AW(AXI_ADDR_WIDTH), .DW(AXI_DATA_WIDTH), .IW(AXI_ID_WIDTH), .UW(AXI_USER_WIDTH)
        ) ace_driver_t;

        typedef snoop_test::snoop_driver #(
            .AW(AXI_ADDR_WIDTH), .DW(AXI_DATA_WIDTH)
        ) snoop_driver_t;

        typedef ace_driver_t::ax_ace_beat_t   ax_ace_beat_t;
        typedef ace_driver_t::w_beat_t        w_beat_t;
        typedef ace_driver_t::b_beat_t        b_beat_t;
        typedef ace_driver_t::r_ace_beat_t    r_ace_beat_t;

        typedef snoop_driver_t::ace_ac_beat_t ace_ac_beat_t;
        typedef snoop_driver_t::ace_cd_beat_t ace_cd_beat_t;
        typedef snoop_driver_t::ace_cr_beat_t ace_cr_beat_t;

        mailbox #(dcache_req)    dcache_req_mbox_prio;
        mailbox #(dcache_req)    dcache_req_mbox_prio_tmp;
        mailbox #(dcache_req)    dcache_req_mbox  [2:0];
        mailbox #(dcache_resp)   dcache_resp_mbox [2:0];

        mailbox #(dcache_req)    req_to_cache_update;

        mailbox #(dcache_req)    req_to_cache_check;
        mailbox #(ace_ac_beat_t) snoop_to_cache_update;

        mailbox #(amo_req)       amo_req_mbox;
        mailbox #(amo_resp)      amo_resp_mbox;

        mailbox #(dcache_mgmt_trans) mgmt_mbox;

        // ACE mailboxes
        mailbox aw_mbx = new, w_mbx = new, b_mbx = new, ar_mbx = new, r_mbx = new;

        // Snoop mailboxes
        mailbox ac_mbx = new, ac_mbx_int = new, cd_mbx = new, cr_mbx = new;

        virtual dcache_sram_if sram_vif;
        virtual dcache_gnt_if  gnt_vif;

        string       name;
        ariane_cfg_t ArianeCfg;

        // Cache model
        cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status;
        logic                              [DCACHE_SET_ASSOC-1:0] lfsr;

        int cache_msg_timeout =  1000;
        int snoop_msg_timeout =  1000;
        int amo_msg_timeout   = 10000;

        function new (
            virtual dcache_sram_if sram_vif,
            virtual dcache_gnt_if  gnt_vif,
            ariane_cfg_t               cfg,
            string                     name="std_cache_scoreboard"
        );
            this.sram_vif             = sram_vif;
            this.gnt_vif              = gnt_vif;
            this.name                 = name;
            this.ArianeCfg            = cfg;

            this.dcache_req_mbox_prio = new();
            this.dcache_req_mbox_prio_tmp = new();
            this.mgmt_mbox            = new();

            cache_status              = '0;
            lfsr                      = '0;

            req_to_cache_update = new();
            req_to_cache_check = new();
            snoop_to_cache_update = new();

        endfunction

        function void set_cache_msg_timeout(int t);
            cache_msg_timeout = t;
        endfunction

        function void set_snoop_msg_timeout(int t);
            snoop_msg_timeout = t;
        endfunction

        function void set_amo_msg_timeout(int t);
            amo_msg_timeout = t;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // cache check functions
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit isHit (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic bit isDirty (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].dirty && cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic bit isShared (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].shared && cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic int getHitWay (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return i;
            end
            $error("No hit way found");
            return -1;
        endfunction

        function automatic bit isCleanUnique (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b1011 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b10 || ar.ax_domain == 2'b01))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadShared (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0001 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadOnce (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0000 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadUnique (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0111 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadNoSnoop (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0000 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b00 || ar.ax_domain == 2'b11))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteBack (input ax_ace_beat_t aw);
            if (aw.ax_snoop == 3'b011 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b00 || aw.ax_domain == 2'b01 || aw.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteUnique ( input ax_ace_beat_t aw );
            if (aw.ax_snoop == 3'b000 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b01 || aw.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteNoSnoop( input ax_ace_beat_t aw );
            if (aw.ax_snoop == 3'b000 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b00 || aw.ax_domain == 2'b11))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit mustEvict (input logic [63:0] addr);
            logic valid = 1'b1;
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                valid = valid & cache_status[addr2mem_idx(addr)][i].valid;
            end
            if (!isHit(addr) && valid == 1'b1 && cache_status[addr2mem_idx(addr)][lfsr[$clog2(DCACHE_SET_ASSOC)-1:0]].dirty == 1'b1)
                return 1'b1;
            else
                return 1'b0;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // calculate next lfsr
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic[7:0] nextLfsr (input logic[7:0] n);
            logic tmp;
            tmp = !(n[7] ^ n[3] ^ n[2] ^ n[1]);
            return {n[6:0], tmp};
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get target way and update lfsr
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic [$clog2(DCACHE_SET_ASSOC)-1:0] get_way_from_lfsr (
            inout logic [7:0] lfsr
        );
            logic [$clog2(DCACHE_SET_ASSOC)-1:0] result;

            result = lfsr[$clog2(DCACHE_SET_ASSOC)-1:0];
            lfsr       = nextLfsr(lfsr);

            return result;
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get target way from cache_status
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic get_way_from_cache (
            input  logic [63:0]                         addr,
            output logic [$clog2(DCACHE_SET_ASSOC)-1:0] way
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic [DCACHE_SET_ASSOC-1:0]                      valid_v;

            mem_idx_v = addr2mem_idx(addr);
            for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                valid_v[i] = cache_status[mem_idx_v][i].valid;
            end
            way = one_hot_to_bin(get_victim_cl(~valid_v));
            return !(&valid_v);
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check target way in cache_status
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic check_way_from_cache (
            input logic [63:0]                         addr,
            input logic [$clog2(DCACHE_SET_ASSOC)-1:0] way
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            mem_idx_v = addr2mem_idx(addr);
            return cache_status[mem_idx_v][way].valid;
        endfunction



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check cache contents against real memory
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit checkCache (
            input logic [63:0]                         addr,
            input logic [$clog2(DCACHE_SET_ASSOC)-1:0] way,
            input string                               origin = ""
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic [DCACHE_INDEX_WIDTH-1:0]                    idx_v;
            logic [DCACHE_TAG_WIDTH-1:0]                      tag_v;
            bit                                               OK;

            OK        = 1'b1;
            mem_idx_v = addr2mem_idx(addr);
            idx_v     = addr2index(addr);
            tag_v     = addr2tag(addr);

            // check the target way
            if (cache_status[mem_idx_v][way].dirty != sram_vif.vld_sram[mem_idx_v][8*way]) begin
                OK = 1'b0;
                $error("%s: Cache mismatch index %h tag %h way %h - dirty bit: expected %d, actual %d", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].dirty, sram_vif.vld_sram[mem_idx_v][8*way]);
            end
            if (cache_status[mem_idx_v][way].valid != sram_vif.vld_sram[mem_idx_v][8*way+1]) begin
                OK = 1'b0;
                $error("%s: Cache mismatch index %h tag %h way %h - valid bit: expected %d, actual %d", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].valid, sram_vif.vld_sram[mem_idx_v][8*way+1]);
            end
            if (cache_status[mem_idx_v][way].shared != sram_vif.vld_sram[mem_idx_v][8*way+2]) begin
                OK = 1'b0;
                $error("%s: Cache mismatch index %h tag %h way %h - shared bit: expected %d, actual %d", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].shared, sram_vif.vld_sram[mem_idx_v][8*way+2]);
            end


            // check tags and data for valid entries
            for (int w=0;w<DCACHE_SET_ASSOC; w++) begin
                if (cache_status[mem_idx_v][w].valid) begin

                    if (cache_status[mem_idx_v][w].tag != sram_vif.tag_sram[w][mem_idx_v][47:0]) begin
                        OK = 1'b0;
                        $error("%s: Cache mismatch index %h tag %h way %0h - tag: expected %h, actual %h", {name,".",origin}, idx_v, tag_v, w, cache_status[mem_idx_v][w].tag, sram_vif.tag_sram[w][mem_idx_v][47:0]);
                    end

                    if (cache_status[mem_idx_v][w].data != {sram_vif.data_sram[1][w][mem_idx_v], sram_vif.data_sram[0][w][mem_idx_v]}) begin
                        OK = 1'b0;
                        $error("%s: Cache mismatch index %h tag %h way %h - data: expected 0x%16h_%16h, actual 0x%16h_%16h", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].data[127:64], cache_status[mem_idx_v][way].data[63:0], sram_vif.data_sram[1][way][mem_idx_v], sram_vif.data_sram[0][way][mem_idx_v]);
                    end

                end else if (sram_vif.vld_sram[mem_idx_v][8*w+1]) begin
                    OK = 1'b0;
                    $error("%s: Cache mismatch index %h tag %h way %0h - valid: expected %h, actual %h", {name,".",origin}, idx_v, tag_v, w, cache_status[mem_idx_v][w].valid, sram_vif.vld_sram[mem_idx_v][8*w+1]);
                end
            end
            return OK;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Get expected CR response from current cache contents
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic ace_cr_beat_t GetCRResp (
            input ace_ac_beat_t req
        );
            ace_cr_beat_t resp;
            resp         = new();
            resp.cr_resp = '0;

            if (req.ac_snoop != snoop_pkg::CLEAN_INVALID &&
                req.ac_snoop != snoop_pkg::READ_ONCE &&
                req.ac_snoop != snoop_pkg::READ_UNIQUE &&
                req.ac_snoop != snoop_pkg::READ_SHARED) begin
                resp.cr_resp.error = 1'b1;
            end

            if (isDirty(req.ac_addr) && (req.ac_snoop == snoop_pkg::READ_UNIQUE || req.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.passDirty = 1'b1;
            end

            if (isHit(req.ac_addr) && (req.ac_snoop != snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.dataTransfer = 1'b1;
            end

            if (isDirty(req.ac_addr) && (req.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.dataTransfer = 1'b1;
            end

            return resp;

        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check CR response
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit checkCRResp (
            input ace_ac_beat_t req,
            input ace_cr_beat_t exp,
            input ace_cr_beat_t resp
        );
            bit OK;
            OK = 1'b1;

            if (exp.cr_resp.error != resp.cr_resp.error) begin
                $error("%s: CR.resp.error mismatch: expected %h, actual %h", name, exp.cr_resp.error, resp.cr_resp.error);

                OK = 1'b0;
            end

            if (isShared(req.ac_addr) != resp.cr_resp.isShared && resp.cr_resp.error == 1'b0 && req.ac_snoop != snoop_pkg::CLEAN_INVALID) begin
                $error("%s: CR.resp.isShared mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, isShared(req.ac_addr), resp.cr_resp.isShared);
                OK = 1'b0;
            end

            if(exp.cr_resp.passDirty != resp.cr_resp.passDirty && resp.cr_resp.error == 1'b0) begin
                $error("%s: CR.resp.passDirty mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, exp.cr_resp.passDirty, resp.cr_resp.passDirty);
                OK = 1'b0;
            end

            if(exp.cr_resp.dataTransfer != resp.cr_resp.dataTransfer && resp.cr_resp.error == 1'b0) begin
                $error("%s: CR.resp.dataTransfer mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, exp.cr_resp.dataTransfer, resp.cr_resp.dataTransfer);
                OK = 1'b0;
            end

        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // update cache model contents when receiving snoop
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task update_cache_from_snoop;
            // helper variables
            logic [DCACHE_SET_ASSOC-1:0]                      valid_v;
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic                                             hit_v;
            bit                                               CheckOK;
            ace_ac_beat_t                                     ac;
            logic [$clog2(DCACHE_SET_ASSOC)-1:0]              hit_way;
            int                                               cnt;

            forever begin
                snoop_to_cache_update.get(ac);

                mem_idx_v = addr2mem_idx(ac.ac_addr);
                hit_v     = 1'b0;

                // look for the right tag
                for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                    if (cache_status[mem_idx_v][i].valid && cache_status[mem_idx_v][i].tag == addr2tag(ac.ac_addr)) begin
                        hit_way = i;
                        hit_v   = 1'b1;
                        break;
                    end
                end

                // actual cache update takes 3 more cycles (with grant for some)
                cnt = 0;
                // 1. wait for grant to read cache
                while (!gnt_vif.gnt[1]) begin
                    $display("%t ns %s: skipping cycle without grant for snoop", $time, name);
                    @(posedge sram_vif.clk); // skip cycles without grant
                    cnt++;
                    if (cnt > 1000) begin
                        $error("%t timeout while waiting for grant for snoop update", $time);
                        break;
                    end
                end
                @(posedge sram_vif.clk);

                // 2. wait for FSM
                @(posedge sram_vif.clk);

                // 3. wait for grant to write cache, if required
                if (hit_v && (ac.ac_snoop == snoop_pkg::READ_SHARED ||
                              ac.ac_snoop == snoop_pkg::READ_UNIQUE ||
                              ac.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                    while (!gnt_vif.gnt[1]) begin
                        $display("%t ns %s: skipping cycle without grant for snoop", $time, name);
                        @(posedge sram_vif.clk); // skip cycles without grant
                        cnt++;
                        if (cnt > 1000) begin
                            $error("%t timeout while waiting for grant for snoop update", $time);
                            break;
                        end
                    end
                end
                @(posedge sram_vif.clk);
                $display("%t ns %s updating cache status from snoop", $time, name);

                if (hit_v) begin
                    case (ac.ac_snoop)
                        snoop_pkg::READ_SHARED: begin
                            $display("Update mem [%0d][%0d] from READ_SHARED", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b1;
                        end
                        snoop_pkg::READ_UNIQUE: begin
                            $display("Update mem [%0d][%0d] from READ_UNIQUE", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b0;
                            cache_status[mem_idx_v][hit_way].valid = 1'b0;
                            cache_status[mem_idx_v][hit_way].dirty = 1'b0;
                        end
                        snoop_pkg::CLEAN_INVALID: begin
                            $display("Update mem [%0d][%0d] from CLEAN_INVALID", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b0;
                            cache_status[mem_idx_v][hit_way].valid = 1'b0;
                            cache_status[mem_idx_v][hit_way].dirty = 1'b0;
                        end
                        snoop_pkg::READ_ONCE: begin
                            $display("Update mem [%0d][%0d] from READ_ONCE", mem_idx_v, hit_way);
                        end
                        default: begin
                            $error("%s: unexpected snoop type %0d", name, ac.ac_snoop);
                        end
                    endcase
                    if (cache_status[mem_idx_v][hit_way].valid) begin
                        $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %0d, shared : %0d, tag : 0x%6h, data : 0x%16h_%16h", mem_idx_v, hit_way,
                            cache_status[mem_idx_v][hit_way].valid,
                            cache_status[mem_idx_v][hit_way].dirty,
                            cache_status[mem_idx_v][hit_way].shared,
                            cache_status[mem_idx_v][hit_way].tag,
                            cache_status[mem_idx_v][hit_way].data[127:64],
                            cache_status[mem_idx_v][hit_way].data[63:0]);
                    end else begin
                        $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %0d, shared : %0d, tag : 0x%6h", mem_idx_v, hit_way,
                            cache_status[mem_idx_v][hit_way].valid,
                            cache_status[mem_idx_v][hit_way].dirty,
                            cache_status[mem_idx_v][hit_way].shared,
                            cache_status[mem_idx_v][hit_way].tag);
                    end
                    CheckOK = checkCache(ac.ac_addr, hit_way, "update_cache_from_snoop");
                end else begin
                    $display("No hit for addr %8h", ac.ac_addr);
                end

            end
        endtask



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // update cache model contents when receiving dcache request
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic update_cache_from_req;

            forever begin
                dcache_req   req_t;
                req_to_cache_update.get(req_t);

                fork
                    begin
                        // declare variables here to get sepa
                        logic [DCACHE_SET_ASSOC-1:0]                      valid_v;
                        logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
                        logic [63:0]                                      addr_v;
                        bit                                               CheckOK;
                        logic [$clog2(DCACHE_SET_ASSOC)-1:0]              target_way;
                        bit                                               hit;
                        dcache_req                                        req;

                        req    = new req_t;

                        addr_v    = tag_index2addr(.tag(req.address_tag), .index(req.address_index));
                        mem_idx_v = addr2mem_idx(addr_v);
                        for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                            valid_v[i] = cache_status[mem_idx_v][i].valid;
                        end
                        hit = isHit(addr_v);

                        // check that cache access is granted if needed
                        if (req.update_cache) begin
                            int cnt = 0;
                            while (!gnt_vif.gnt[req.prio]) begin
                                $display("%t ns %s skipping cycle without grant for dcache req : %s", $time, name, req.print_me());
                                @(posedge sram_vif.clk); // skip cycles without grant
                                cnt++;
                                if (cnt > cache_msg_timeout) begin
                                    $error("%s : Timeout while waiting for grant for dcache req : %s", name, req.print_me());
                                    break;
                                end
                            end
                            $display("%t ns %s got grant for dcache req : %s", $time, name, req.print_me());
                            @(posedge sram_vif.clk);
                            $display("%t ns %s updating cache status from dcache req : %s", $time, name, req.print_me());
                        end else begin
                            $display("%t ns %s no cache update expected for dcache req : %s", $time, name, req.print_me());
                        end


                        if (hit) begin
                            // cache hit
                            $display("Cache hit");
                            target_way = getHitWay(addr_v);
                            if (req.trans_type == WR_REQ) begin
                                cache_status[mem_idx_v][target_way].dirty  = 1'b1;
                                cache_status[mem_idx_v][target_way].shared = 1'b0;
                                update_cache_line(cache_status[mem_idx_v][target_way].data, req.data, req.data_offset);
                            end
                        end else begin
                            // cache miss
                            $display("Cache miss");
                            if (&valid_v) begin
                                // all ways occupied
                                $display("No empty way");

                                if (req.target_way_valid) begin
                                    target_way = req.target_way;
                                end else begin
                                    target_way = get_way_from_lfsr(lfsr);
                                end

                                if (req.trans_type == EVICT) begin
                                    $display("Evict");
                                    cache_status[mem_idx_v][target_way].valid  = 1'b0;
                                    cache_status[mem_idx_v][target_way].dirty  = 1'b0;
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                end else  if (req.trans_type == WR_REQ) begin
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].dirty  = 1'b1;
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                    update_cache_line(cache_status[mem_idx_v][target_way].data, req.data, req.data_offset);
                                end else  if (req.trans_type == READBACK || req.trans_type == RD_RESP) begin
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty;
                                    cache_status[mem_idx_v][target_way].shared = req.r_shared;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                end else begin
                                    $error("Didn't expect trans_type %s", req.trans_type.name());
                                end
                            end else begin
                                // there is an empty way
                                if (req.target_way_valid) begin
                                    $display("Using target way from request");
                                    target_way = req.target_way;
                                end else begin
                                    $display("Empty way found");
                                    target_way = one_hot_to_bin(get_victim_cl(~valid_v));
                                end
                                cache_status[mem_idx_v][target_way].tag   = req.address_tag;
                                cache_status[mem_idx_v][target_way].valid = 1'b1;
                                if (req.trans_type == WR_REQ) begin
                                    cache_status[mem_idx_v][target_way].dirty  = 1'b1;
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                    update_cache_line(cache_status[mem_idx_v][target_way].data, req.data, req.data_offset);
                                end else  if (req.trans_type == READBACK || req.trans_type == RD_RESP) begin
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty;
                                    cache_status[mem_idx_v][target_way].shared = req.r_shared;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                end else begin
                                    $error("Didn't expect trans_type %s", req.trans_type.name());
                                end
                            end
                        end

                        if (hit && req.trans_type == RD_REQ) begin
                            assert (req.update_cache == 0) else $error("Didn't expect update for a read hit");
                        end else begin
                            assert (req.update_cache == 1) else $error("Expected cache update for a write or miss");
                        end
                            $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %0d, shared : %0d, tag : 0x%6h, data : 0x%16h_%16h", mem_idx_v, target_way,
                                cache_status[mem_idx_v][target_way].valid,
                                cache_status[mem_idx_v][target_way].dirty,
                                cache_status[mem_idx_v][target_way].shared,
                                cache_status[mem_idx_v][target_way].tag,
                                cache_status[mem_idx_v][target_way].data[127:64],
                                cache_status[mem_idx_v][target_way].data[63:0]
                            );

                            CheckOK = checkCache(addr_v, target_way, "update_cache_from_req");

                    end

               join_none

            end
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving snoop requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_snoop;
            forever begin
                ace_ac_beat_t  ac;
                bit            timeout = 0;
                acsnoop_enum_t e;

                // wait for snoop request
                ac = new();
                ac_mbx.get(ac);
                e = acsnoop_enum_t'(ac.ac_snoop);
                $display("%t ns %s.check_snoop: Got snoop request %0s", $time, name, e.name());
                a_empty_ac : assert (ac_mbx.num() == 0) else $error ("%S.check_snoop : AC mailbox not empty", name);

                fork
                    begin
                        fork
                            begin
                                bit           CheckOK;
                                ace_cr_beat_t cr, cr_exp;

                                if (is_inside_cacheable_regions(ArianeCfg, ac.ac_addr)) begin
                                    snoop_to_cache_update.put(ac);
                                end

                                // wait to prepare expected response until last cycle before cache is updated by snoop
                                // 1. wait for grant to read cache
                                while (!gnt_vif.gnt[1]) begin
                                    $display("%t ns %s.check_snoop: skipping cycle without grant for snoop", $time, name);
                                    @(posedge sram_vif.clk); // skip cycles without grant
                                end
                                @(posedge sram_vif.clk);
                                // 2. wait for FSM
                                @(posedge sram_vif.clk);

                                cr_exp = GetCRResp(ac);
                                $display("%t ns %s.check_snoop: Got expected response PassDirty : %1b, DataTransfer : %1b, Error : %1b for address %16h", $time, name, cr_exp.cr_resp.passDirty, cr_exp.cr_resp.dataTransfer, cr_exp.cr_resp.error, ac.ac_addr);

                                // wait for the response
                                cr_mbx.get(cr);
                                $display("%t ns %s.check_snoop: Got snoop response 0b%5b (WasUnique : %1b, isShared : %1b, PassDirty : %1b, Error : %1b, DataTransfer : %1b)", $time, name, cr.cr_resp, cr.cr_resp[4],cr.cr_resp[3],cr.cr_resp[2],cr.cr_resp[1],cr.cr_resp[0]);
                                a_empty_cr : assert (cr_mbx.num() == 0) else $error ("%S.check_snoop : CR mailbox not empty", name);

                                CheckOK = checkCRResp(.req(ac), .exp(cr_exp), .resp(cr));

                                // send snoop to do_hit()
                                ac_mbx_int.put(ac);

                                // expect the data
                                $display("%t ns %s.check_snoop: CD mailbox size : %0d", $time, name, cd_mbx.num());

                                if (cr_exp.cr_resp.dataTransfer) begin
                                    ace_cd_beat_t cd;
                                    cd = new();
                                    cd.cd_last = 1'b0;
                                    while (!cd.cd_last) begin
                                        cd_mbx.get(cd);
                                        $display("%t ns %s.check_snoop: Got snoop data 0x%16h, last = %0d", $time, name, cd.cd_data,cd.cd_last);
                                    end
                                end
                                // check that no unexpected CD response has been generated
                                a_empty_cd : assert (cd_mbx.num() == 0) else $error ("%S.check_snoop : CD mailbox not empty", name);
                            end
                        join
                    end

                    // timeout
                    begin
                        automatic int cnt = 0;
                        while (cnt < snoop_msg_timeout) begin
                            @(posedge sram_vif.clk);
                            cnt++;
                        end
                        timeout = 1;
                    end

                join_any

                if (timeout) $error("%s.check_snoop : Timeout", name);

            end // forever
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get cache requests in prio order
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic get_cache_msg;
            $display("%t ns %s retreiving dcache messages", $time, name);
            forever begin
                for (int i=0; i<=2; i++) begin
                    dcache_req msg;
                    if (dcache_req_mbox[i].try_get(msg)) begin
                        dcache_req msg_t;
                        msg_t = new msg;
                        dcache_req_mbox_prio_tmp.put(msg_t);
                    end
                end
                @(posedge sram_vif.clk);
            end
        endtask

        local task automatic get_cache_msg_tmp;
            dcache_req msg;
            forever begin
                dcache_req_mbox_prio_tmp.get(msg);
                fork
                    begin
                        dcache_req msg_t;
                        msg_t = new msg;
                        dcache_req_mbox_prio.put(msg_t);
                        check_cache_msg();
                    end
                    begin
                        @(posedge sram_vif.clk);
                    end
                join_any
            end
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // subtask for hit case
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic do_hit (input dcache_req msg);
            logic [63:0]  addr_v;

            $display("%t ns %s started hit task for message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));
            if (msg.trans_type == WR_REQ) begin
                ace_ac_beat_t ac = new();
                msg.update_cache = 1'b1;

                // empty snoop mailbox
                while (ac_mbx_int.try_get(ac));

                // Add one additional cycle before checking cache status, mimicing cache ctrl FSM.
                @(posedge sram_vif.clk);

                if (isShared(addr_v)) begin
                    ax_ace_beat_t ar_beat     = new();
                    r_ace_beat_t  r_beat      = new();
                    r_ace_beat_t  r_beat_peek = new();

                    // wait for AR beat
                    ar_mbx.get(ar_beat);
                    $display("%t ns %s.do_hit: got AR beat for message : %s", $time, name, msg.print_me());
                    if (!isCleanUnique(ar_beat))
                        $error("%s Error CLEAN_UNIQUE expected for message : %s", name, msg.print_me());
                    a_empty_ar : assert (ar_mbx.num() == 0) else $error ("%S.do_hit : AR mailbox not empty", name);

                    // wait for R beat
                    while (!r_beat.r_last) begin
                        r_mbx.peek(r_beat_peek);
                        if (r_beat_peek.r_id == ar_beat.ax_id) begin
                            // this is our response
                            r_mbx.get(r_beat);
                            $display("%t ns %s.do_hit: got R beat with last = %0d for message : %s", $time, name, r_beat.r_last, msg.print_me());
                        end else begin
                            @(posedge sram_vif.clk);
                        end
                    end
                    a_empty_r : assert (r_mbx.num() == 0) else $error ("%S.do_hit : R mailbox not empty", name);

                    msg.insert_readback = 1'b1;

                    // check if a ReadShared has arrived during writing
                    while (ac_mbx_int.try_get(ac)) begin
                        if (ac.ac_snoop == snoop_pkg::READ_SHARED && ac.ac_addr == addr_v) begin
                            $display("%t ns %s Got matching ReadShared during hit + write shared, calling hit routine for message : %s", $time, name, msg.print_me());
                            do_hit(msg);
                        end
                    end

                end

            end


            if (!isHit(addr_v)) begin
                $display("%t ns %s Cache status changed from hit to miss, calling miss routine for message : %s", $time, name, msg.print_me());
                do_miss(msg);
            end
        endtask

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // subtask for miss case
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic do_miss (input dcache_req msg);
            dcache_req    readback_msg;
            logic [63:0]  addr_v;

            $display("%t ns %s started miss task for message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));

            if (!msg.insert_readback) begin // if cache was changed due to insert readback then the original port will do the
                msg.prio = 0; // miss has highest prio
            end
            msg.update_cache = 1'b1;

            fork
                begin
                    ax_ace_beat_t aw_beat = new();
                    b_beat_t      b_beat  = new();
                    w_beat_t      w_beat  = new();

                    // monitor if eviction is necessary
                    dcache_req evict_msg;
                    while (!mustEvict(addr_v)) begin
                        // check if target way gets taken, then update to new way
                        if (msg.target_way_valid && check_way_from_cache(msg.get_addr(), msg.target_way)) begin
                            msg.target_way_valid = get_way_from_cache(msg.get_addr(), msg.target_way);
                            if (msg.target_way_valid) begin
                                $display("%t ns %s target way got occupied, update way to %d for message : %s", $time, name, msg.target_way, msg.print_me());
                            end else begin
                                $display("%t ns %s all ways got occupied, invalidate target way for message : %s", $time, name, msg.print_me());
                            end
                        end
                        @(posedge sram_vif.clk);
                    end

                    // get target way to evict msg
                    evict_msg                  = new msg;
                    evict_msg.trans_type       = EVICT;
                    evict_msg.target_way       = get_way_from_lfsr(lfsr);
                    evict_msg.target_way_valid = 1'b1;

                    // copy target way to msg
                    msg.target_way       = evict_msg.target_way;
                    msg.target_way_valid = 1'b1;

                    $display("%t ns %s Eviction needed, wait for eviction AW beat for message : %s", $time, name, msg.print_me());
                    aw_mbx.get(aw_beat);
                    if (!isWriteBack(aw_beat))
                        $error("%s.do_miss : WRITEBACK request expected after eviction for message : %s", name, msg.print_me());
                    a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.do_miss : AW mailbox not empty", name);

                    $display("%t ns %s inserting a new dcache message :%s", $time, name, msg.print_me());
                    req_to_cache_update.put(evict_msg);

                    // wait for W beat
                    while (!w_beat.w_last) begin
                        w_mbx.get(w_beat);
                        $display("%t ns %s.do_miss : got W beat with last = %0d for message %s", $time, name, w_beat.w_last, msg.print_me());
                    end
                    a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.do_miss : W mailbox not empty", name);

                    // wait for B beat
                    b_mbx.get(b_beat);
                    $display("%t ns %s.do_miss : got B beat for message %s", $time, name, msg.print_me());
                    a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.do_miss : B mailbox not empty", name);

                    wait (0); // avoid exiting fork

                end

                begin

                    ax_ace_beat_t ar_beat     = new();
                    r_ace_beat_t  r_beat      = new();
                    r_ace_beat_t  r_beat_peek = new();
                    int           r_cnt       = 0;

                    // wait for miss FSM before getting target way
                    repeat (2)
                        @(posedge sram_vif.clk);
                    msg.target_way_valid = get_way_from_cache(msg.get_addr(), msg.target_way);

                    if (msg.target_way_valid) begin
                        $display("%t ns %s.do_miss: set target way to %d for message : %s", $time, name, msg.target_way, msg.print_me());
                    end else begin
                        $display("%t ns %s.do_miss: all ways occupied for message : %s", $time, name, msg.print_me());
                    end


                    $display("%t ns %s wait for AR beat for message : %s", $time, name, msg.print_me());
                    // wait for AR beat
                    ar_mbx.get(ar_beat);
                    $display("%t ns %s.do_miss: got AR beat for message : %s", $time, name, msg.print_me());
                    a_empty_ar : assert (ar_mbx.num() == 0) else $error ("%S.do_miss : AR mailbox not empty", name);

                    if (msg.trans_type == WR_REQ) begin
                        if (is_inside_shareable_regions(ArianeCfg, msg.get_addr())) begin
                            if (!isReadUnique(ar_beat)) begin
                                $error("%s.do_miss : READ_UNIQUE request expected for message : %s", name, msg.print_me());
                            end
                        end else begin
                            if (!isReadNoSnoop(ar_beat)) begin
                                $error("%s.do_miss : READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                        end
                    end else begin // RD_REQ
                        if (is_inside_shareable_regions(ArianeCfg, msg.get_addr())) begin
                            if (!isReadShared(ar_beat)) begin
                                $error("%s.do_miss : READ_SHARED request expected for message : %s", name, msg.print_me());
                            end
                        end else begin
                            if (!isReadNoSnoop(ar_beat)) begin
                                $error("%s.do_miss : READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                        end
                    end

                    // wait for R beat
                    while (!r_beat.r_last) begin
                        r_mbx.peek(r_beat_peek);
                        if (r_beat_peek.r_id == ar_beat.ax_id) begin
                            // this is our response
                            r_mbx.get(r_beat);
                            msg.add_to_cache_line(r_beat.r_data);
                            $display("%t ns %s.do_miss: got R beat with last = %0d for message : %s", $time, name, r_beat.r_last, msg.print_me());
                            if (msg.trans_type == RD_REQ) begin
                                if (r_cnt == msg.data_offset) begin
                                    $display("%t ns %s.do_miss: got R beat with valid data, changing type from RD_REQ to RD_RESP for message : %s", $time, name, msg.print_me());
                                    msg.trans_type = RD_RESP;
                                    msg.data       = r_beat.r_data;
                                end
                            end
                            r_cnt++;
                        end else begin
                            @(posedge sram_vif.clk);
                        end
                    end
                    a_empty_r : assert (r_mbx.num() == 0) else $error ("%S.do_miss : R mailbox not empty", name);

                    msg.r_dirty  = r_beat.r_resp[2];
                    msg.r_shared = r_beat.r_resp[3];

                    if (msg.insert_readback) begin
                        // write readback data to cache
                        readback_msg = new();
                        readback_msg.prio          = 0;            // this will be written by miss handler
                        readback_msg.port_idx      = msg.port_idx; // keep port that caused the readback for logging reasons
                        readback_msg.trans_type    = READBACK;
                        readback_msg.address_tag   = msg.address_tag;   // keep tag
                        readback_msg.address_index = msg.address_index; // keep address
                        readback_msg.update_cache  = 1'b1;
                        readback_msg.r_dirty       = r_beat.r_resp[2];
                        readback_msg.r_shared      = r_beat.r_resp[3];

                        $display("%t ns %s inserting a new dcache message : %s", $time, name, readback_msg.print_me());

                        req_to_cache_update.put(readback_msg);
                    end

                end

                begin
                    // check if hit status changes, could be result of miss handler writeback
                    // in that case stop waiting for an AR beat
                    $display("%t ns %s monitoring hit status for message : %s", $time, name, msg.print_me());

                    while (!isHit(addr_v) || !gnt_vif.rd_gnt[msg.port_idx + 2]) begin
                        @(posedge sram_vif.clk);
                    end

                    // status changed to hit, revert changes in priority
                    msg.prio = msg.port_idx + 2;
                    if (msg.trans_type == WR_REQ) begin
                        msg.update_cache = 1'b1;
                    end else begin
                        msg.update_cache = 1'b0;
                    end

                    $display("%t ns %s Cache status changed from miss to hit, abort waiting for AR for message : %s", $time, name, msg.print_me());
                end
            join_any
            disable fork;

            assert (ar_mbx.num() == 0) else $error("AR mailbox not empty");
            assert (r_mbx.num() == 0) else $error("R mailbox not empty");

            if (isHit(addr_v)) begin
                $display("%t ns %s Cache status changed from miss to hit, calling hit routine for message : %s", $time, name, msg.print_me());
                do_hit(msg);
            end

        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving dcache requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_cache_msg;
            dcache_req    msg;
            dcache_req    readback_msg;
            logic [63:0]  addr_v;
            bit           CheckOK;
            bit timeout = 0;

            dcache_req_mbox_prio.get(msg);

            // default
            msg.prio         = msg.port_idx + 2;
            msg.update_cache = 1'b0;

            $display("%t ns %s got dcache message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));

            fork
                begin
                    // bypass
                    if (!is_inside_cacheable_regions(ArianeCfg, addr_v)) begin
                        if (msg.trans_type == WR_REQ) begin
                            b_beat_t b_beat = new();
                            w_beat_t w_beat = new();
                            if (is_inside_shareable_regions(ArianeCfg, addr_v)) begin
                                ax_ace_beat_t aw_beat = new();
                                aw_mbx.get(aw_beat);
                                if (!isWriteUnique(aw_beat))
                                    $error("%s.check_cache_msg : WRITE_UNIQUE request expected for message : %s", name, msg.print_me());
                            end else begin
                                ax_ace_beat_t aw_beat = new();
                                // wait for grant before checking AW, a snoop transaction may be active
                                while (!gnt_vif.bypass_gnt[msg.port_idx]) begin
                                    @(posedge sram_vif.clk);
                                end
                                aw_mbx.get(aw_beat);
                                if (!isWriteNoSnoop(aw_beat))
                                    $error("%s.check_cache_msg : WRITE_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                            a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.check_cache_msg : AW mailbox not empty", name);

                            // wait for W beat
                            while (!w_beat.w_last) begin
                                w_mbx.get(w_beat);
                                $display("%t ns %s.check_cache_msg: got W beat with last = %0d for message : %s", $time, name, w_beat.w_last, msg.print_me());
                            end
                            a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.check_cache_msg : W mailbox not empty", name);

                            // wait for B beat
                            b_mbx.get(b_beat);
                            a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.check_cache_msg : B mailbox not empty", name);
                        end else begin
                            ax_ace_beat_t ar_beat     = new();
                            r_ace_beat_t  r_beat      = new();
                            r_ace_beat_t  r_beat_peek = new();

                            ar_mbx.get(ar_beat);
                            $display("%t ns %s.check_cache_msg: got AR beat for message : %s", $time, name, msg.print_me());
                            if (is_inside_shareable_regions(ArianeCfg, addr_v)) begin
                                if (!isReadOnce(ar_beat))
                                    $error("%s.check_cache_msg : READ_ONCE request expected for message : %s", name, msg.print_me());
                            end else begin
                                if (!isReadNoSnoop(ar_beat))
                                    $error("%s.check_cache_msg : READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                            a_empty_ar : assert (ar_mbx.num() == 0) else $error ("%S.check_cache_msg : AR mailbox not empty", name);

                            // wait for R beat
                            while (!r_beat.r_last) begin
                                r_mbx.peek(r_beat_peek);
                                if (r_beat_peek.r_id == ar_beat.ax_id) begin
                                    // this is our response
                                    r_mbx.get(r_beat);
                                    $display("%t ns %s.check_cache_msg: got R beat with last = %0d for message : %s", $time, name, r_beat.r_last, msg.print_me());
                                end else begin
                                    @(posedge sram_vif.clk);
                                end
                            end
                            a_empty_r : assert (r_mbx.num() == 0) else $error ("%S.check_cache_msg : R mailbox not empty", name);

                            msg.r_dirty  = r_beat.r_resp[2];
                            msg.r_shared = r_beat.r_resp[3];

                        end
                    end
                    // cacheable
                    else begin

                        if (isHit(addr_v)) begin
                            do_hit(msg);
                        end else begin
                            do_miss(msg);
                        end

                    end

                    if (is_inside_cacheable_regions(ArianeCfg, addr_v)) begin
                        // send to cache update
                        $display("%t ns %s Sending message to cache update : %s", $time, name, msg.print_me());
                        req_to_cache_update.put(msg);
                    end

                end

                // timeout
                begin
                    automatic int cnt;
                    cnt = cache_msg_timeout;
                    while (cnt > 0) begin
                        cnt--;
                        @(posedge sram_vif.clk);
                    end
                    timeout = 1;
                end

            join_any

            if (timeout) $error("%s : Timeout in check_cache_msg for message : %s", name, msg.print_me());

        endtask



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when flushing cache
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic flush_cache;
            $display("%t ns %s.flush_cache: Flushing started", $time, name);

            for (int w = 0; w < DCACHE_NUM_WORDS; w++) begin
                int w_cnt = 0;
                for (int l = 0; l < DCACHE_SET_ASSOC; l++) begin
                    if (cache_status[w][l].valid && cache_status[w][l].dirty) begin
                        fork
                            begin
                                automatic int ll = l;
                                automatic int ww = w;
                                // expect write back of dirty data
                                ax_ace_beat_t aw_beat = new();
                                b_beat_t      b_beat  = new();
                                w_beat_t      w_beat  = new();

                                // wait for AW beat
                                aw_mbx.get(aw_beat);
                                $display("%t ns %s.flush_cache: got AW beat for cache[%0d][%0d]", $time, name, ww, ll);
                                if (!isWriteBack(aw_beat))
                                    $error("%s.flush_cache : WRITEBACK request expected after eviction of cache[%0d][%0d]", name, ww, ll);
                                a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.flush_cache : AW mailbox not empty", name);

                                // wait for W beat
                                while (!w_beat.w_last) begin
                                    w_mbx.get(w_beat);
                                    $display("%t ns %s.flush_cache: got W beat with last = %0d for cache[%0d][%0d]", $time, name, w_beat.w_last, ww, ll);
                                end
                                a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.flush_cache : W mailbox not empty", name);

                                // wait for B beat
                                b_mbx.get(b_beat);
                                $display("%t ns %s.flush_cache: got B beat for cache[%0d][%0d]", $time, name, ww, ll);
                                a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.flush_cache : B mailbox not empty", name);
                            end
                            begin
                                // expect clear of cache entry
                                automatic int ll  = l;
                                automatic int ww  = w;
                                automatic int cnt = 0;
                                while (!gnt_vif.gnt[0]) begin
                                    $display("%t ns %s.flush_cache : skipping cycle without grant for flush of cache entry [%0d][%0d]", $time, name, ww, ll);
                                    @(posedge sram_vif.clk); // skip cycles without grant
                                    cnt++;
                                    if (cnt > 1000) begin
                                        $error("%s.flush_cache : timeout while waiting for grant for flush of cache entry [%0d][%0d]", name, ww, ll);
                                        break;
                                    end
                                end
                                @(posedge sram_vif.clk);

                                // clear entry in cache model
                                $display("%t ns %s.flush_cache: Flushing cache entry [%0d][%0d]", $time, name, ww, ll);
                                cache_status[ww][ll] = '0;
                            end
                        join_any
                    end
                end // l

                // expect clear of cache entry
                while (!gnt_vif.gnt[0]) begin
                    $display("%t ns %s.flush_cache : skipping cycle without grant for clear of cache set [%0d]", $time, name, w);
                    @(posedge sram_vif.clk); // skip cycles without grant
                    w_cnt++;
                    if (w_cnt > 1000) begin
                        $error("%s.flush_cache : timeout while waiting for grant for clear of cache set [%0d]", name, w);
                        break;
                    end
                end
                @(posedge sram_vif.clk);

                // clear entry in cache model
                $display("%t ns %s.flush_cache: Clear cache set [%0d]", $time, name, w);
                cache_status[w] = '0;

            end // w

        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving AMO requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_amo_msg;

            forever begin
                amo_req       msg;
                bit timeout = 0;

                amo_req_mbox.get(msg);
                $display("%t ns %s.check_amo_msg: Got amo message %s", $time, name, msg.print_me());

                fork
                    begin
                        flush_cache();
                        if (msg.op != AMO_LR) begin
                            ax_ace_beat_t aw_beat     = new();
                            b_beat_t      b_beat      = new();
                            w_beat_t      w_beat      = new();
                            r_ace_beat_t  r_beat      = new();
                            r_ace_beat_t  r_beat_peek = new();

                            aw_mbx.get(aw_beat);
                            $display("%t ns %s.check_amo_msg: got AW beat for message %s", $time, name, msg.print_me());

                            if (is_inside_shareable_regions(ArianeCfg, msg.addr)) begin
                                if (!isWriteUnique(aw_beat))
                                    $error("%s.check_amo_msg : WRITE_UNIQUE request expected for message %s", name, msg.print_me());
                            end else begin
                                if (!isWriteNoSnoop(aw_beat))
                                    $error("%s.check_amo_msg : WRITE_NO_SNOOP request expected for message %s", name, msg.print_me());
                            end
                            a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.check_amo_msg : AW mailbox not empty", name);

                            // wait for W beat
                            while (!w_beat.w_last) begin
                                w_mbx.get(w_beat);
                                $display("%t ns %s.check_amo_msg: got W beat with last = %0d for message %s", $time, name, w_beat.w_last, msg.print_me());
                            end
                            a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.check_amo_msg : W mailbox not empty", name);

                            // wait for B beat
                            b_mbx.get(b_beat);
                            $display("%t ns %s.check_amo_msg: got B beat for message %s", $time, name, msg.print_me());
                            a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.check_amo_msg : B mailbox not empty", name);

                            if (msg.op != AMO_SC) begin // AMO_SC has no data response, only OK/ not OK decoded from B beat
                                // wait for R beat
                                while (!r_beat.r_last) begin
                                    r_mbx.peek(r_beat_peek);
                                    if (r_beat_peek.r_id == aw_beat.ax_id) begin
                                        // this is our response
                                        r_mbx.get(r_beat);
                                        $display("%t ns %s.check_amo_msg: got R beat with last = %0d for message %s", $time, name, r_beat.r_last, msg.print_me());
                                    end else begin
                                        @(posedge sram_vif.clk);
                                    end
                                end
                                a_empty_r : assert (r_mbx.num() == 0) else $error ("%S.check_amo_msg : R mailbox not empty", name);
                            end

                        end else begin
                            ax_ace_beat_t ar_beat     = new();
                            r_ace_beat_t  r_beat      = new();
                            r_ace_beat_t  r_beat_peek = new();

                            ar_mbx.get(ar_beat);
                            $display("%t ns %s.check_amo_msg: got AR beat for message %s", $time, name, msg.print_me());
                            if (is_inside_shareable_regions(ArianeCfg, msg.addr)) begin
                                if (!isReadOnce(ar_beat))
                                    $error("%s.check_amo_msg : READ_ONCE request expected for message %s",name, msg.print_me());
                            end else begin
                                if (!isReadNoSnoop(ar_beat))
                                    $error("%s.check_amo_msg : READ_NO_SNOOP request expected for message %s", name, msg.print_me());
                            end
                            a_empty_ar : assert (ar_mbx.num() == 0) else $error ("%S.check_amo_msg : AR mailbox not empty", name);

                            // wait for R beat
                            while (!r_beat.r_last) begin
                                r_mbx.peek(r_beat_peek);
                                if (r_beat_peek.r_id == ar_beat.ax_id) begin
                                    // this is our response
                                    r_mbx.get(r_beat);
                                    $display("%t ns %s.check_amo_msg: got R beat with last = %0d for message %s", $time, name, r_beat.r_last, msg.print_me());
                                end else begin
                                    @(posedge sram_vif.clk);
                                end
                            end
                            a_empty_r : assert (r_mbx.num() == 0) else $error ("%S.check_amo_msg : R mailbox not empty", name);

                        end

                    end

                    // timeout
                    begin
                        automatic int cnt = 0;
                        while (cnt < amo_msg_timeout) begin
                            @(posedge sram_vif.clk);
                            cnt++;
                        end
                        timeout = 1;
                    end

                join_any
                if (timeout) $error("%s.check_amo_msg : Timeout for message %s", name, msg.print_me());
            end

        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Handle management transactions (currently only flush implemented)
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_mgmt_trans;

            forever begin
                dcache_mgmt_trans trans;
                bit timeout = 0;

                mgmt_mbox.get(trans);
                $display("%t ns %s.check_mgmt_trans: Got management transaction %s", $time, name, trans.print_me());

                fork
                    begin
                        if (trans.trans_type == FLUSH_REQ) begin
                            flush_cache();
                        end
                    end

                    // timeout
                    begin
                        repeat (10000) @(posedge sram_vif.clk);
                        timeout = 1;
                    end

                join_any
                if (timeout) $error("%s.check_mgmt_trans : Timeout for transaction %s", name, trans.print_me());
            end

        endtask

        task run;
            fork
                get_cache_msg();
                get_cache_msg_tmp();
                check_snoop();
                check_amo_msg();
                check_mgmt_trans();
                update_cache_from_req();
                update_cache_from_snoop();
            join
        endtask

    endclass


    //--------------------------------------------------------------------------
    // dcache checker
    //--------------------------------------------------------------------------
    class std_dcache_checker #(
        parameter int unsigned NB_CORES        = 2,
        parameter int unsigned SRAM_DATA_WIDTH = 0,
        parameter int unsigned SRAM_NUM_WORDS  = 0
    );

        virtual dcache_sram_if                                                 dc_sram_vif [NB_CORES];
        virtual sram_intf #(DCACHE_SET_ASSOC, SRAM_DATA_WIDTH, SRAM_NUM_WORDS) sram_vif    [NB_CORES];

        string       name;
        ariane_cfg_t ArianeCfg;

        function new (
            virtual sram_intf #(DCACHE_SET_ASSOC, SRAM_DATA_WIDTH, SRAM_NUM_WORDS) sram_vif    [NB_CORES],
            virtual dcache_sram_if                                                 dc_sram_vif [NB_CORES],
            ariane_cfg_t                                                           cfg,
            string                                                                 name="std_dcache_checker"
        );
            this.name      = name;
            this.ArianeCfg = cfg;
            for (int c = 0; c < NB_CORES; c++) begin
                this.sram_vif[c]    = sram_vif[c];
                this.dc_sram_vif[c] = dc_sram_vif[c];
            end
        endfunction

        // check the cache contents vs main memory and other caches on every write to the cache status
        local task automatic mon_dcache;
            $display("%t ns %s monitoring dcache", $time, name);
            for (int c=0; c < NB_CORES; c++) begin
                fork
                    automatic int cc = c;

                    begin
                        forever begin
                            if (dc_sram_vif[cc].vld_req && dc_sram_vif[cc].vld_we) begin
                                logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] index;
                                index = dc_sram_vif[cc].vld_index;
                                $display("%t ns %s.monitor: Saw write to cache %0d, index 0x%3h",$time, name, cc, index);

                                @(posedge dc_sram_vif[cc].clk);
                                #0;

                                for (int cw=0; cw<DCACHE_SET_ASSOC; cw++) begin
                                    logic                         cc_valid, cc_dirty, cc_shared;
                                    logic [DCACHE_TAG_WIDTH:0]    cc_tag;
                                    logic [DCACHE_LINE_WIDTH-1:0] cc_data;
                                    cc_dirty  = dc_sram_vif[cc].vld_sram[index][8*cw];
                                    cc_valid  = dc_sram_vif[cc].vld_sram[index][8*cw+1];
                                    cc_shared = dc_sram_vif[cc].vld_sram[index][8*cw+2];
                                    cc_tag    = dc_sram_vif[cc].tag_sram[cw][index][DCACHE_TAG_WIDTH-1:0];
                                    cc_data   = {dc_sram_vif[cc].data_sram[1][cw][index], dc_sram_vif[cc].data_sram[0][cw][index]};
                                    if (cc_valid) begin
                                        logic any_dirty;
                                        any_dirty = cc_dirty;
                                        // check entries in other caches
                                        for (int oc=0; oc < NB_CORES; oc++) begin
                                            if (oc != cc) begin
                                                for (int ow=0; ow<DCACHE_SET_ASSOC; ow++) begin
                                                    logic                         oc_valid, oc_dirty, oc_shared;
                                                    logic [DCACHE_TAG_WIDTH:0]    oc_tag;
                                                    logic [DCACHE_LINE_WIDTH-1:0] oc_data;
                                                    oc_dirty  = dc_sram_vif[oc].vld_sram[index][8*ow];
                                                    oc_valid  = dc_sram_vif[oc].vld_sram[index][8*ow+1];
                                                    oc_shared = dc_sram_vif[oc].vld_sram[index][8*ow+2];
                                                    oc_tag    = dc_sram_vif[oc].tag_sram[ow][index][DCACHE_TAG_WIDTH-1:0];
                                                    oc_data   = {dc_sram_vif[oc].data_sram[1][ow][index], dc_sram_vif[oc].data_sram[0][ow][index]};
                                                    if (oc_valid && (oc_tag == cc_tag)) begin
                                                        any_dirty = any_dirty | oc_dirty;
                                                        $display("%t ns %s.monitor: Cache match for index 0x%3h, tag 0x%16h between way %0d in core %0d and way %0d in core %0d",$time, name, index, cc_tag, cw, cc, ow, oc);

                                                        // check that data matches
                                                        a_data : assert (cc_data == oc_data) else
                                                            $error("%s: Cache data mismatch for index %h, tag %h - core %0d, way %0d = 0x%16h_%16h, core %0d, way %0d = 0x%16h_%16h", name, index, cc_tag, cc, cw, cc_data[127:64], cc_data[63:0], oc, ow, oc_data[127:64], oc_data[63:0]);

                                                        // If data is present in both caches they should be marked shared.
                                                        // This will also implicitly check that a unique data is not present in
                                                        // any other chache.
                                                        a_cc_shared : assert (cc_shared) else
                                                            $error("%s.monitor: Expected shared = 1 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 0", name, index, cc_tag, cw, cc);
                                                        a_oc_shared : assert (oc_shared) else
                                                            $error("%s.monitor: Expected shared = 1 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 0", name, index, oc_tag, ow, oc);

                                                        // only one core could have the data marked as dirty
                                                        if (cc_dirty) begin
                                                            a_oc_clean : assert (!oc_dirty) else
                                                                $error("%s.monitor: Expected dirty = 0 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 1", name, index, oc_tag, ow, oc);
                                                        end


                                                    end
                                                end
                                            end
                                        end

                                        // check that data matches SRAM for globally clean entries
                                        if (!any_dirty) begin
                                            logic [63:0] addr;
                                            addr                  = {cc_tag, index};
                                            sram_vif[cc].addr[cw] = (addr - (ArianeCfg.ExecuteRegionAddrBase[3] >> DCACHE_BYTE_OFFSET)) << 1;
                                            #0
                                            a_mem_data : assert (cc_data == sram_vif[cc].data[cw]) else
                                                $error("%s: Cache vs Memory data mismatch for index %h, tag %h - core %0d, way %0d = 0x%16h_%16h, Memory[0x%16h] = 0x%16h_%16h", name, index, cc_tag, cc, cw, cc_data[127:64], cc_data[63:0], sram_vif[cc].addr[cw], sram_vif[cc].data[cw][1], sram_vif[cc].data[cw][0]);
                                        end
                                    end
                                end
                            end else begin
                                @(posedge dc_sram_vif[cc].clk);
                            end
                        end
                    end

                join_none
            end
            wait fork;
        endtask


        task monitor;
            mon_dcache();
        endtask

    endclass


endpackage
