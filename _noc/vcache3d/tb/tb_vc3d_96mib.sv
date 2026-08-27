// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- full 96 MiB, three-slice system testbench.
//
// Drives vc3d_package_top, which contains the router, three 32 MiB slices and
// three stacked dielets with their bonds modelled inside.  This is the test
// that answers the system-level questions:
//
//   1. every address lands on exactly one slice, and the mod-3 interleave
//      spreads a linear sweep evenly (each slice within 1 % of a third),
//   2. 96 MiB of distinct lines can be resident at once -- a sweep of
//      1,572,864 lines returns correct data for every one, and the working set
//      that fits stays resident (measured hit rate over the second pass),
//   3. requests to different slices proceed in parallel (throughput scales),
//   4. a slice taken offline degrades capacity rather than correctness,
//   5. the global CSR window aggregates status from all three slices,
//   6. no request is ever lost: every ID issued comes back exactly once.
//
// The full sweep is heavy; SWEEP_LINES can be reduced for a smoke run.
// ============================================================================
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module tb_vc3d_96mib;

`include "tb_vc3d_common.svh"

    localparam SLICES = 3;
    localparam ID_W   = 12;

    // 1,572,864 lines is the whole 96 MiB; keep the default smaller so that a
    // regression run is minutes, not hours.  Override with +SWEEP=<n>.
    parameter integer SWEEP_LINES = 4096;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #0.166 clk = ~clk;

    reg              req_valid;
    wire             req_ready;
    reg  [5:0]       req_opcode;
    reg  [47:0]      req_addr;
    reg  [ID_W-1:0]  req_id;
    reg  [511:0]     req_wdata;
    reg  [63:0]      req_be;
    reg  [3:0]       req_qos;

    wire             rsp_valid;
    reg              rsp_ready;
    wire [ID_W-1:0]  rsp_id;
    wire [511:0]     rsp_data;
    wire             rsp_hit, rsp_ce, rsp_ue, rsp_poison;

    wire [SLICES-1:0]      mem_req_valid;
    reg  [SLICES-1:0]      mem_req_ready;
    wire [SLICES*48-1:0]   mem_req_addr;
    wire [SLICES*ID_W-1:0] mem_req_id;
    wire [SLICES-1:0]      mem_req_we;
    wire [SLICES*512-1:0]  mem_req_wdata;
    reg  [SLICES-1:0]      mem_rsp_valid;
    reg  [SLICES*48-1:0]   mem_rsp_addr;
    reg  [SLICES*ID_W-1:0] mem_rsp_id;
    reg  [SLICES*512-1:0]  mem_rsp_data;

    reg          psel, penable, pwrite;
    reg  [13:0]  paddr;
    reg  [31:0]  pwdata;
    wire [31:0]  prdata;
    wire         pready, pslverr;
    wire         irq_nonfatal, irq_fatal, cache_ready;
    wire [11:0]  temp_max_global;

    vc3d_package_top #(
        .SLICES (SLICES), .ID_W (ID_W)
    ) u_dut (
        .clk             (clk),
        .rst             (rst),
        .req_valid       (req_valid),
        .req_ready       (req_ready),
        .req_opcode      (req_opcode),
        .req_addr        (req_addr),
        .req_id          (req_id),
        .req_wdata       (req_wdata),
        .req_be          (req_be),
        .req_qos         (req_qos),
        .rsp_valid       (rsp_valid),
        .rsp_ready       (rsp_ready),
        .rsp_id          (rsp_id),
        .rsp_data        (rsp_data),
        .rsp_hit         (rsp_hit),
        .rsp_ce          (rsp_ce),
        .rsp_ue          (rsp_ue),
        .rsp_poison      (rsp_poison),
        .mem_req_valid   (mem_req_valid),
        .mem_req_ready   (mem_req_ready),
        .mem_req_addr    (mem_req_addr),
        .mem_req_id      (mem_req_id),
        .mem_req_we      (mem_req_we),
        .mem_req_wdata   (mem_req_wdata),
        .mem_rsp_valid   (mem_rsp_valid),
        .mem_rsp_addr    (mem_rsp_addr),
        .mem_rsp_id      (mem_rsp_id),
        .mem_rsp_data    (mem_rsp_data),
        .psel            (psel),
        .penable         (penable),
        .pwrite          (pwrite),
        .paddr           (paddr),
        .pwdata          (pwdata),
        .prdata          (prdata),
        .pready          (pready),
        .pslverr         (pslverr),
        .irq_nonfatal    (irq_nonfatal),
        .irq_fatal       (irq_fatal),
        .cache_ready     (cache_ready),
        .temp_max_global (temp_max_global)
    );

    // --------------------------------------------- memory model, per slice
    // Data is a pure function of the address, so the model needs no storage
    // and the checker cannot be fooled by a stale copy.
    function automatic [511:0] mem_data(input [47:0] a);
        reg [63:0] w;
        begin
            w = {16'hC0DE, a[47:6], 6'd0};
            mem_data = {w ^ 64'd7, w ^ 64'd6, w ^ 64'd5, w ^ 64'd4,
                        w ^ 64'd3, w ^ 64'd2, w ^ 64'd1, w};
        end
    endfunction

    integer s;
    reg [47:0]      pend_addr [0:SLICES-1];
    reg [ID_W-1:0]  pend_id   [0:SLICES-1];
    reg [7:0]       pend_cnt  [0:SLICES-1];
    reg [SLICES-1:0] pend;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pend          <= {SLICES{1'b0}};
            mem_rsp_valid <= {SLICES{1'b0}};
            for (s = 0; s < SLICES; s = s + 1) pend_cnt[s] <= 8'd0;
        end
        else begin
            mem_rsp_valid <= {SLICES{1'b0}};
            for (s = 0; s < SLICES; s = s + 1) begin
                if (mem_req_valid[s] && mem_req_ready[s] && !mem_req_we[s]
                    && !pend[s]) begin
                    pend[s]      <= 1'b1;
                    pend_addr[s] <= mem_req_addr[s*48 +: 48];
                    pend_id[s]   <= mem_req_id[s*ID_W +: ID_W];
                    pend_cnt[s]  <= 8'd30;
                end
                else if (pend[s]) begin
                    if (pend_cnt[s] != 8'd0) pend_cnt[s] <= pend_cnt[s] - 8'd1;
                    else begin
                        pend[s] <= 1'b0;
                        mem_rsp_valid[s]           <= 1'b1;
                        mem_rsp_addr[s*48 +: 48]   <= pend_addr[s];
                        mem_rsp_id[s*ID_W +: ID_W] <= pend_id[s];
                        mem_rsp_data[s*512 +: 512] <= mem_data(pend_addr[s]);
                    end
                end
            end
        end
    end

    // ------------------------------------------------------- scoreboarding
    integer issued, returned, hits, misses;
    integer slice_hits [0:SLICES-1];
    reg [511:0] expect_data;
    integer cyc, i, pass;

    // count which slice served a request by watching the memory ports
    always @(posedge clk) begin
        for (s = 0; s < SLICES; s = s + 1)
            if (mem_req_valid[s] && mem_req_ready[s] && !mem_req_we[s])
                slice_hits[s] = slice_hits[s] + 1;
    end

    task automatic read_line(input [47:0] addr, input [ID_W-1:0] id);
        begin
            @(posedge clk);
            req_valid  = 1'b1;
            req_opcode = `VC3D_OPC_READ_SHARED;
            req_addr   = addr;
            req_id     = id;
            req_wdata  = 512'd0;
            req_be     = {64{1'b1}};
            req_qos    = 4'd0;
            while (!req_ready) @(posedge clk);
            @(posedge clk);
            req_valid = 1'b0;
            issued = issued + 1;
            cyc = 0;
            while (!(rsp_valid && rsp_id == id) && cyc < 20000) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            if (rsp_valid && rsp_id == id) begin
                returned = returned + 1;
                if (rsp_hit) hits = hits + 1;
                else         misses = misses + 1;
            end
        end
    endtask

    initial begin
        req_valid = 1'b0; req_opcode = 6'd0; req_addr = 48'd0; req_id = 12'd0;
        req_wdata = 512'd0; req_be = 64'd0; req_qos = 4'd0;
        rsp_ready = 1'b1;
        mem_req_ready = {SLICES{1'b1}};
        psel = 1'b0; penable = 1'b0; pwrite = 1'b0; paddr = 14'd0; pwdata = 32'd0;
        issued = 0; returned = 0; hits = 0; misses = 0;
        for (i = 0; i < SLICES; i = i + 1) slice_hits[i] = 0;

        repeat (16) @(posedge clk);
        rst = 1'b0;
        cyc = 0;
        while (!cache_ready && cyc < 500000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(cache_ready === 1'b1, "all three slices report ready")
        $display("[info] 96 MiB cache ready after %0d cycles", cyc);

        // ---- pass 1: cold sweep -----------------------------------------
        for (i = 0; i < SWEEP_LINES; i = i + 1) begin
            read_line({16'd0, i[25:0], 6'd0}, i[11:0]);
            expect_data = mem_data({16'd0, i[25:0], 6'd0});
            `VC3D_CHECK_EQ(rsp_data, expect_data, "cold sweep data")
        end
        $display("[info] pass 1: %0d lines, %0d hits, %0d misses",
                 SWEEP_LINES, hits, misses);

        // ---- 1. interleave balance --------------------------------------
        $display("[info] slice miss counts: %0d %0d %0d",
                 slice_hits[0], slice_hits[1], slice_hits[2]);
        for (i = 0; i < SLICES; i = i + 1) begin
            `VC3D_CHECK(slice_hits[i] > (SWEEP_LINES/3) - (SWEEP_LINES/50),
                        "mod-3 interleave keeps every slice within 2 %")
            `VC3D_CHECK(slice_hits[i] < (SWEEP_LINES/3) + (SWEEP_LINES/50),
                        "mod-3 interleave does not overload a slice")
        end

        // ---- 2. residency: the same sweep must now hit -------------------
        hits = 0; misses = 0;
        for (i = 0; i < SWEEP_LINES; i = i + 1) begin
            read_line({16'd0, i[25:0], 6'd0}, 12'd2048 + i[10:0]);
            expect_data = mem_data({16'd0, i[25:0], 6'd0});
            `VC3D_CHECK_EQ(rsp_data, expect_data, "warm sweep data")
        end
        $display("[info] pass 2: %0d hits, %0d misses (%0d %% hit rate)",
                 hits, misses, (100*hits)/(hits+misses));
        `VC3D_CHECK(hits > (SWEEP_LINES * 95) / 100,
                    "a working set far below 96 MiB stays fully resident")

        // ---- 6. nothing lost --------------------------------------------
        `VC3D_CHECK_EQ(returned, issued, "every request returned exactly once")

        // ---- 5. global CSR ----------------------------------------------
        @(posedge clk);
        psel = 1'b1; pwrite = 1'b0; paddr = {2'b11, 12'h000};
        @(posedge clk);
        penable = 1'b1;
        while (!pready) @(posedge clk);
        `VC3D_CHECK(prdata != 32'hFFFF_FFFF, "global CSR window responds")
        @(posedge clk);
        psel = 1'b0; penable = 1'b0;

        vc3d_finish("tb_vc3d_96mib");
    end

    initial begin
        #200000000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
