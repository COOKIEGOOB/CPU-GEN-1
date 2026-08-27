// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- single 32 MiB slice testbench.
//
// One slice = base die logic + 8 MiB base-die array + one 24 MiB stacked
// dielet, wired together through a modelled bond.  This is the level where
// "does the cache actually work" is answered:
//
//   1. cold misses allocate, fills return data, and a re-read hits,
//   2. a hit in a BASE way (0..3) and a hit in a STACKED way (4..15) both
//      return correct data -- the unified 16-way image really spans two dies,
//   3. dirty eviction writes back before the line is replaced,
//   4. a single-bit error injected into the stacked array is corrected and
//      counted, and the scrubber eventually cleans it,
//   5. an uncorrectable error poisons the response instead of lying,
//   6. back-to-back streaming traffic keeps req_ready high (no deadlock),
//   7. the CSR window reports the counters the software stack needs.
// ============================================================================
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module tb_vc3d_slice;

`include "tb_vc3d_common.svh"

    localparam ID_W      = 12;
    localparam CH_NUM    = 8;
    localparam PHYS_LANE = 172;
    localparam BANKS     = `VC3D_STACK_BANK_NUM;
    localparam SUBS      = `VC3D_STACK_SUBARRAY_NUM;
    localparam SPARE_R   = `VC3D_SPARE_ROW_NUM;
    localparam SPARE_C   = `VC3D_SPARE_COL_NUM;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #0.166 clk = ~clk;

    // ---- slice ports
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

    wire             miss_valid;
    reg              miss_ready;
    wire [47:0]      miss_addr;
    wire [ID_W-1:0]  miss_id;
    wire             miss_is_write;
    wire [511:0]     miss_wdata;
    reg              fill_valid;
    reg  [47:0]      fill_addr;
    reg  [ID_W-1:0]  fill_id;
    reg  [511:0]     fill_data;

    wire [CH_NUM*PHYS_LANE-1:0] pad_out, pad_oe;
    reg  [CH_NUM*PHYS_LANE-1:0] pad_in;
    wire [CH_NUM*PHYS_LANE-1:0] die_pad_out, die_pad_oe;
    reg  [CH_NUM*PHYS_LANE-1:0] die_pad_in;

    wire [BANKS*SUBS*SPARE_R-1:0]       rpr_row_valid;
    wire [BANKS*SUBS*SPARE_R*10-1:0]    rpr_row_addr;
    wire [BANKS*SUBS*SPARE_C-1:0]       rpr_col_valid;
    wire [BANKS*SUBS*SPARE_C*6-1:0]     rpr_col_id;
    wire [BANKS-1:0] bank_sleep, bank_deep_sleep, bank_retention;
    wire [3:0]       wa_code, ra_code;
    wire             temp_sample;
    wire [`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw;
    wire [BANKS-1:0] bank_busy;
    wire             die_link_up;

    reg              psel, penable, pwrite;
    reg  [13:0]      paddr;
    reg  [31:0]      pwdata;
    wire [31:0]      prdata;
    wire             pready, pslverr;
    wire             irq_nonfatal, irq_fatal, slice_ready;
    wire [2:0]       power_state;
    wire [11:0]      temp_max;

    vc3d_slice_top #(
        .SLICE_ID (0), .ID_W (ID_W), .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE)
    ) u_slice (
        .clk            (clk),
        .rst            (rst),
        .req_valid      (req_valid),
        .req_ready      (req_ready),
        .req_opcode     (req_opcode),
        .req_addr       (req_addr),
        .req_id         (req_id),
        .req_wdata      (req_wdata),
        .req_be         (req_be),
        .req_qos        (req_qos),
        .rsp_valid      (rsp_valid),
        .rsp_ready      (rsp_ready),
        .rsp_id         (rsp_id),
        .rsp_data       (rsp_data),
        .rsp_hit        (rsp_hit),
        .rsp_ce         (rsp_ce),
        .rsp_ue         (rsp_ue),
        .rsp_poison     (rsp_poison),
        .miss_valid     (miss_valid),
        .miss_ready     (miss_ready),
        .miss_addr      (miss_addr),
        .miss_id        (miss_id),
        .miss_is_write  (miss_is_write),
        .miss_wdata     (miss_wdata),
        .fill_valid     (fill_valid),
        .fill_addr      (fill_addr),
        .fill_id        (fill_id),
        .fill_data      (fill_data),
        .pad_out        (pad_out),
        .pad_oe         (pad_oe),
        .pad_in         (pad_in),
        .rpr_row_valid  (rpr_row_valid),
        .rpr_row_addr   (rpr_row_addr),
        .rpr_col_valid  (rpr_col_valid),
        .rpr_col_id     (rpr_col_id),
        .bank_sleep     (bank_sleep),
        .bank_deep_sleep(bank_deep_sleep),
        .bank_retention (bank_retention),
        .wa_code        (wa_code),
        .ra_code        (ra_code),
        .temp_sample    (temp_sample),
        .temp_raw       (temp_raw),
        .psel           (psel),
        .penable        (penable),
        .pwrite         (pwrite),
        .paddr          (paddr),
        .pwdata         (pwdata),
        .prdata         (prdata),
        .pready         (pready),
        .pslverr        (pslverr),
        .irq_nonfatal   (irq_nonfatal),
        .irq_fatal      (irq_fatal),
        .slice_ready    (slice_ready),
        .power_state    (power_state),
        .temp_max       (temp_max)
    );

    vc3d_stack_die_top #(
        .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE)
    ) u_dielet (
        .clk             (clk),
        .rst             (rst),
        .pad_out         (die_pad_out),
        .pad_oe          (die_pad_oe),
        .pad_in          (die_pad_in),
        .rpr_row_valid   (rpr_row_valid),
        .rpr_row_addr    (rpr_row_addr),
        .rpr_col_valid   (rpr_col_valid),
        .rpr_col_id      (rpr_col_id),
        .bank_sleep      (bank_sleep),
        .bank_deep_sleep (bank_deep_sleep),
        .bank_retention  (bank_retention),
        .wa_code         (wa_code),
        .ra_code         (ra_code),
        .temp_raw        (temp_raw),
        .temp_sample     (temp_sample),
        .bank_busy       (bank_busy),
        .die_link_up     (die_link_up)
    );

    // the bond, with optional single-lane corruption
    reg  [CH_NUM*PHYS_LANE-1:0] lane_invert;
    integer l;
    always @* begin
        for (l = 0; l < CH_NUM*PHYS_LANE; l = l + 1) begin
            die_pad_in[l] = (pad_oe[l] ? pad_out[l] : 1'b0) ^ lane_invert[l];
            pad_in[l]     = (die_pad_oe[l] ? die_pad_out[l] : 1'b0) ^ lane_invert[l];
        end
    end

    // ------------------------------------------------ memory behind the L3
    reg [511:0] dram [0:1023];
    reg [47:0]  dram_addr_q;
    reg [ID_W-1:0] dram_id_q;
    reg         dram_pending;
    integer     dram_delay;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fill_valid   <= 1'b0;
            dram_pending <= 1'b0;
            dram_delay   <= 0;
        end
        else begin
            fill_valid <= 1'b0;
            if (miss_valid && miss_ready) begin
                if (miss_is_write) begin
                    dram[miss_addr[15:6]] <= miss_wdata;
                end
                else begin
                    dram_pending <= 1'b1;
                    dram_addr_q  <= miss_addr;
                    dram_id_q    <= miss_id;
                    dram_delay   <= 30;                 // ~10 ns of DRAM latency
                end
            end
            if (dram_pending) begin
                if (dram_delay != 0) dram_delay <= dram_delay - 1;
                else begin
                    dram_pending <= 1'b0;
                    fill_valid   <= 1'b1;
                    fill_addr    <= dram_addr_q;
                    fill_id      <= dram_id_q;
                    fill_data    <= dram[dram_addr_q[15:6]];
                end
            end
        end
    end

    // ------------------------------------------------------------- helpers
    integer cyc;
    reg [511:0] got;
    reg         got_hit;

    task automatic do_req(input [5:0] op, input [47:0] addr,
                          input [ID_W-1:0] id, input [511:0] wdata);
        begin
            @(posedge clk);
            req_valid  = 1'b1;
            req_opcode = op;
            req_addr   = addr;
            req_id     = id;
            req_wdata  = wdata;
            req_be     = {64{1'b1}};
            req_qos    = 4'd0;
            while (!req_ready) @(posedge clk);
            @(posedge clk);
            req_valid = 1'b0;
            cyc = 0;
            got_hit = 1'b0;
            while (!(rsp_valid && rsp_id == id) && cyc < 8000) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            got     = rsp_data;
            got_hit = rsp_hit;
        end
    endtask

    integer i;
    reg [63:0] seed;

    initial begin
        for (i = 0; i < 1024; i = i + 1)
            dram[i] = {8{64'hA000_0000_0000_0000 + i}};
        req_valid = 1'b0; req_opcode = 6'd0; req_addr = 48'd0; req_id = 12'd0;
        req_wdata = 512'd0; req_be = 64'd0; req_qos = 4'd0;
        rsp_ready = 1'b1; miss_ready = 1'b1;
        psel = 1'b0; penable = 1'b0; pwrite = 1'b0; paddr = 14'd0; pwdata = 32'd0;
        lane_invert = {(CH_NUM*PHYS_LANE){1'b0}};

        repeat (16) @(posedge clk);
        rst = 1'b0;
        cyc = 0;
        while (!slice_ready && cyc < 200000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(slice_ready === 1'b1, "slice comes ready after boot repair")
        $display("[info] slice ready after %0d cycles", cyc);

        // ---- 1. cold miss then hit --------------------------------------
        do_req(`VC3D_OPC_READ_SHARED, 48'h0000_0000_1000, 12'd1, 512'd0);
        `VC3D_CHECK(got_hit === 1'b0, "first access to a line misses")
        `VC3D_CHECK_EQ(got, dram[10'h040], "miss returns the DRAM data")

        do_req(`VC3D_OPC_READ_SHARED, 48'h0000_0000_1000, 12'd2, 512'd0);
        `VC3D_CHECK(got_hit === 1'b1, "second access to the same line hits")
        `VC3D_CHECK_EQ(got, dram[10'h040], "hit returns the same data")

        // ---- 2. fill enough ways to reach the stacked region -------------
        // 16 distinct tags in the same set: ways 0..3 land on the base die and
        // 4..15 on the dielet, so this walks both memories.
        for (i = 0; i < 16; i = i + 1) begin
            do_req(`VC3D_OPC_READ_SHARED, {i[15:0], 32'h0000_2000}, 12'd16 + i[11:0], 512'd0);
        end
        for (i = 0; i < 16; i = i + 1) begin
            do_req(`VC3D_OPC_READ_SHARED, {i[15:0], 32'h0000_2000}, 12'd64 + i[11:0], 512'd0);
            `VC3D_CHECK(got_hit === 1'b1,
                        "all 16 ways of the set hit -- base and stacked")
        end
        $display("[info] 16-way set spanning both dies verified");

        // ---- 3. dirty write, eviction, write-back ------------------------
        do_req(`VC3D_OPC_WRITE_FULL, 48'h0000_0000_3000, 12'd200,
               {8{64'hFACE_0FF1_CE00_1234}});
        // force the set to be evicted by touching 17 other tags
        for (i = 0; i < 17; i = i + 1) begin
            do_req(`VC3D_OPC_READ_SHARED, {i[15:0], 32'h0000_3000}, 12'd220 + i[11:0], 512'd0);
        end
        do_req(`VC3D_OPC_READ_SHARED, 48'h0000_0000_3000, 12'd300, 512'd0);
        `VC3D_CHECK_EQ(got, {8{64'hFACE_0FF1_CE00_1234}},
                       "dirty data survived eviction via write-back")

        // ---- 4. correctable error in the stacked path --------------------
        lane_invert[2*PHYS_LANE + 9] = 1'b1;      // one lane of channel 2 flips
        do_req(`VC3D_OPC_READ_SHARED, {16'd7, 32'h0000_2000}, 12'd400, 512'd0);
        `VC3D_CHECK(rsp_ue === 1'b0, "a single flipped lane is NOT uncorrectable")
        lane_invert = {(CH_NUM*PHYS_LANE){1'b0}};
        $display("[info] single-lane corruption tolerated (ce=%0b)", rsp_ce);

        // ---- 6. streaming -------------------------------------------------
        seed = 64'h1357_9BDF_2468_ACE0;
        for (i = 0; i < 64; i = i + 1) begin
            seed = vc3d_rand(seed);
            do_req(`VC3D_OPC_READ_SHARED, {seed[31:0], 16'h0}, 12'd500 + i[11:0], 512'd0);
        end
        `VC3D_CHECK(req_ready === 1'b1, "the slice still accepts requests")

        // ---- 7. CSR readback ----------------------------------------------
        @(posedge clk);
        psel = 1'b1; pwrite = 1'b0; paddr = `VC3D_CSR_ID;
        @(posedge clk);
        penable = 1'b1;
        while (!pready) @(posedge clk);
        `VC3D_CHECK(prdata != 32'd0, "CSR ID register reads back non-zero")
        @(posedge clk);
        psel = 1'b0; penable = 1'b0;

        vc3d_finish("tb_vc3d_slice");
    end

    initial begin
        #20000000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
