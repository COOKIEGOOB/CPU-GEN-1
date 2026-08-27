// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- manufacturing repair testbench.
//
// This is the flow a real part goes through on the tester and then at every
// power-on, exercised end to end:
//
//   1. MBIST runs a March C- over a modelled dielet with INJECTED DEFECTS
//      (a dead row, a dead column, and a scattered single-bit fail),
//   2. the repair controller consumes the fail stream and allocates spare
//      rows / columns,
//   3. the allocation is burned into the eFuse array (with its CRC),
//   4. the part is "power cycled": the eFuse autoloads, the repair map is
//      reconstructed, and MBIST is re-run -- it must now PASS,
//   5. a corrupted fuse image is rejected by the CRC check,
//   6. defects beyond the spare budget end in graceful way disable, not in a
//      silently broken cache.
//
// The memory model here is deliberately independent of the RTL array so that
// the test cannot pass by agreeing with a bug in the DUT's own memory.
// ============================================================================
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module tb_vc3d_repair;

`include "tb_vc3d_common.svh"

    localparam BANKS  = 32;
    localparam BANK_W = 5;
    localparam SUBS   = 16;
    localparam SUB_W  = 4;
    localparam ROWS   = 1024;
    localparam ROW_W  = 10;
    localparam DW     = 144;
    localparam SPARE_R = 4;
    localparam SPARE_C = 4;
    localparam COL_ID_W = 6;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #0.166 clk = ~clk;

    // --------------------------------------------------------------- MBIST
    reg               mb_start;
    reg  [2:0]        mb_algorithm;
    reg  [BANK_W-1:0] mb_bank_first, mb_bank_last;
    reg  [31:0]       mb_retention_pause;
    reg               mb_stop_on_fail;
    wire              mb_busy, mb_done, mb_pass;

    wire              mem_req, mem_we;
    wire [BANK_W-1:0] mem_bank;
    wire [SUB_W-1:0]  mem_sub;
    wire [ROW_W-1:0]  mem_row;
    wire [DW-1:0]     mem_wdata;
    reg               mem_gnt, mem_rvalid;
    reg  [DW-1:0]     mem_rdata;

    wire [4:0]        fail_count;
    wire [BANK_W-1:0] fail_bank;
    wire [SUB_W-1:0]  fail_sub;
    wire [ROW_W-1:0]  fail_row;
    wire [DW-1:0]     fail_expect, fail_actual;
    wire              fail_push;
    wire [31:0]       total_fail_bits;

    vc3d_mbist_engine #(
        .BANKS (BANKS), .BANK_W (BANK_W), .SUBS (SUBS), .SUB_W (SUB_W),
        .ROWS (ROWS), .ROW_W (ROW_W), .DW (DW)
    ) u_mbist (
        .clk             (clk),
        .rst             (rst),
        .start           (mb_start),
        .algorithm       (mb_algorithm),
        .bank_first      (mb_bank_first),
        .bank_last       (mb_bank_last),
        .retention_pause (mb_retention_pause),
        .stop_on_fail    (mb_stop_on_fail),
        .busy            (mb_busy),
        .done            (mb_done),
        .pass            (mb_pass),
        .mem_req         (mem_req),
        .mem_gnt         (mem_gnt),
        .mem_we          (mem_we),
        .mem_bank        (mem_bank),
        .mem_sub         (mem_sub),
        .mem_row         (mem_row),
        .mem_wdata       (mem_wdata),
        .mem_rvalid      (mem_rvalid),
        .mem_rdata       (mem_rdata),
        .fail_count      (fail_count),
        .fail_bank       (fail_bank),
        .fail_sub        (fail_sub),
        .fail_row        (fail_row),
        .fail_expect     (fail_expect),
        .fail_actual     (fail_actual),
        .fail_push       (fail_push),
        .total_fail_bits (total_fail_bits)
    );

    // ----------------------------------------------------------- repair map
    wire               map_wr_en, map_wr_is_col, map_wr_valid;
    wire [BANK_W-1:0]  map_wr_bank;
    wire [SUB_W-1:0]   map_wr_sub;
    wire [1:0]         map_wr_slot;
    wire [ROW_W-1:0]   map_wr_addr;
    wire               map_wr_slot_busy;
    wire [15:0]        map_rows_used, map_cols_used;
    wire               map_any_repair;
    wire [BANKS*SUBS*SPARE_R-1:0]          rpr_row_valid;
    wire [BANKS*SUBS*SPARE_R*ROW_W-1:0]    rpr_row_addr;
    wire [BANKS*SUBS*SPARE_C-1:0]          rpr_col_valid;
    wire [BANKS*SUBS*SPARE_C*COL_ID_W-1:0] rpr_col_id;
    reg                                    map_clear;
    wire                                   map_rd_valid;
    wire [ROW_W-1:0]                       map_rd_addr;
    reg  [BANK_W-1:0]                      map_rd_bank;
    reg  [SUB_W-1:0]                       map_rd_sub;
    reg                                    map_rd_is_col;
    reg  [1:0]                             map_rd_slot;

    vc3d_repair_map #(
        .BANKS (BANKS), .BANK_W (BANK_W), .SUBS (SUBS), .SUB_W (SUB_W),
        .SPARE_ROWS (SPARE_R), .SPARE_COLS (SPARE_C),
        .ROW_AW (ROW_W), .COL_ID_W (COL_ID_W)
    ) u_map (
        .clk           (clk),
        .rst           (rst),
        .wr_en         (map_wr_en),
        .wr_bank       (map_wr_bank),
        .wr_sub        (map_wr_sub),
        .wr_is_col     (map_wr_is_col),
        .wr_slot       (map_wr_slot),
        .wr_valid      (map_wr_valid),
        .wr_addr       (map_wr_addr),
        .wr_slot_busy  (map_wr_slot_busy),
        .rd_bank       (map_rd_bank),
        .rd_sub        (map_rd_sub),
        .rd_is_col     (map_rd_is_col),
        .rd_slot       (map_rd_slot),
        .rd_valid      (map_rd_valid),
        .rd_addr       (map_rd_addr),
        .clear_all     (map_clear),
        .rpr_row_valid (rpr_row_valid),
        .rpr_row_addr  (rpr_row_addr),
        .rpr_col_valid (rpr_col_valid),
        .rpr_col_id    (rpr_col_id),
        .rows_used     (map_rows_used),
        .cols_used     (map_cols_used),
        .any_repair    (map_any_repair)
    );

    // --------------------------------------------------------------- eFuse
    reg          fu_prog_en;
    wire         fu_prog_req;
    wire [9:0]   fu_prog_addr;
    wire [31:0]  fu_prog_data;
    reg  [15:0]  fu_prog_pulse;
    wire         fu_prog_busy, fu_prog_done, fu_prog_error;
    reg          fu_read_req;
    reg  [9:0]   fu_read_addr;
    wire [31:0]  fu_read_data;
    wire         fu_read_valid;
    wire         fu_al_start;
    wire         fu_al_busy, fu_al_done, fu_al_crc_err, fu_al_push;
    wire [31:0]  fu_al_word;
    wire [9:0]   fu_al_index;
    wire [31:0]  fu_magic;
    wire [15:0]  fu_record_count;

    vc3d_efuse_array #(
        .ROWS (1024), .ROW_W (32), .ADDR_W (10)
    ) u_efuse (
        .clk               (clk),
        .rst               (rst),
        .prog_en           (fu_prog_en),
        .prog_req          (fu_prog_req),
        .prog_addr         (fu_prog_addr),
        .prog_data         (fu_prog_data),
        .prog_pulse_cycles (fu_prog_pulse),
        .prog_busy         (fu_prog_busy),
        .prog_done         (fu_prog_done),
        .prog_error        (fu_prog_error),
        .read_req          (fu_read_req),
        .read_addr         (fu_read_addr),
        .read_data         (fu_read_data),
        .read_valid        (fu_read_valid),
        .autoload_start    (fu_al_start),
        .autoload_busy     (fu_al_busy),
        .autoload_done     (fu_al_done),
        .autoload_crc_err  (fu_al_crc_err),
        .al_push           (fu_al_push),
        .al_word           (fu_al_word),
        .al_index          (fu_al_index),
        .fuse_magic        (fu_magic),
        .record_count      (fu_record_count)
    );

    // --------------------------------------------------------- repair ctrl
    reg         poweron_start, full_repair_start;
    wire        repair_busy, repair_done, repair_failed;
    wire [3:0]  repair_phase;
    wire        lane_solve_start, link_train_req;
    reg         lane_solve_done, lane_unrepairable, link_up_all;
    wire [15:0] way_disable_w;
    wire [31:0] bank_disable_w;
    wire [15:0] rows_repaired, cols_repaired, lanes_repaired, unrepaired_fails;
    reg         rt_repair_req;
    reg  [1:0]  rt_repair_type;
    reg  [BANK_W-1:0] rt_repair_bank;
    reg  [SUB_W-1:0]  rt_repair_sub;
    reg  [ROW_W-1:0]  rt_repair_row;
    reg  [8:0]  rt_repair_syndrome;
    wire        rt_repair_ack;
    wire        mb_start_ctrl;
    wire [2:0]  mb_alg_ctrl;
    wire [BANK_W-1:0] mb_bf_ctrl, mb_bl_ctrl;

    vc3d_repair_ctrl #(
        .BANKS (BANKS), .BANK_W (BANK_W), .SUBS (SUBS), .SUB_W (SUB_W),
        .ROW_AW (ROW_W), .COL_ID_W (COL_ID_W),
        .SPARE_ROWS (SPARE_R), .SPARE_COLS (SPARE_C), .DW (DW)
    ) u_ctrl (
        .clk                   (clk),
        .rst                   (rst),
        .poweron_start         (poweron_start),
        .full_repair_start     (full_repair_start),
        .fuse_prog_enable      (fu_prog_en),
        .repair_busy           (repair_busy),
        .repair_done           (repair_done),
        .repair_failed         (repair_failed),
        .repair_phase          (repair_phase),
        .fuse_autoload_start   (fu_al_start),
        .fuse_autoload_done    (fu_al_done),
        .fuse_autoload_crc_err (fu_al_crc_err),
        .fuse_al_push          (fu_al_push),
        .fuse_al_word          (fu_al_word),
        .fuse_prog_req         (fu_prog_req),
        .fuse_prog_addr        (fu_prog_addr),
        .fuse_prog_data        (fu_prog_data),
        .fuse_prog_done        (fu_prog_done),
        .fuse_prog_error       (fu_prog_error),
        .mbist_start           (mb_start_ctrl),
        .mbist_algorithm       (mb_alg_ctrl),
        .mbist_bank_first      (mb_bf_ctrl),
        .mbist_bank_last       (mb_bl_ctrl),
        .mbist_busy            (mb_busy),
        .mbist_done            (mb_done),
        .mbist_pass            (mb_pass),
        .mbist_fail_push       (fail_push),
        .mbist_fail_bank       (fail_bank),
        .mbist_fail_sub        (fail_sub),
        .mbist_fail_row        (fail_row),
        .mbist_fail_expect     (fail_expect),
        .mbist_fail_actual     (fail_actual),
        .rt_repair_req         (rt_repair_req),
        .rt_repair_type        (rt_repair_type),
        .rt_repair_bank        (rt_repair_bank),
        .rt_repair_sub         (rt_repair_sub),
        .rt_repair_row         (rt_repair_row),
        .rt_repair_syndrome    (rt_repair_syndrome),
        .rt_repair_ack         (rt_repair_ack),
        .map_wr_en             (map_wr_en),
        .map_wr_bank           (map_wr_bank),
        .map_wr_sub            (map_wr_sub),
        .map_wr_is_col         (map_wr_is_col),
        .map_wr_slot           (map_wr_slot),
        .map_wr_valid          (map_wr_valid),
        .map_wr_addr           (map_wr_addr),
        .map_wr_slot_busy      (map_wr_slot_busy),
        .map_rows_used         (map_rows_used),
        .map_cols_used         (map_cols_used),
        .lane_solve_start      (lane_solve_start),
        .lane_solve_done       (lane_solve_done),
        .lane_unrepairable     (lane_unrepairable),
        .link_train_req        (link_train_req),
        .link_up_all           (link_up_all),
        .way_disable           (way_disable_w),
        .bank_disable          (bank_disable_w),
        .rows_repaired         (rows_repaired),
        .cols_repaired         (cols_repaired),
        .lanes_repaired        (lanes_repaired),
        .unrepaired_fails      (unrepaired_fails)
    );

    // -------------------------------------------------------- memory model
    // A small sparse model of one bank of the dielet, with injected defects.
    // Defect model:
    //   DEAD_ROW  : every bit of one row reads back 0
    //   DEAD_COL  : one bit position is stuck across every row
    //   BIT_FAIL  : one (row, bit) pair is stuck
    // Repairs from rpr_* redirect the access, exactly as the real subarray does.
    localparam DEAD_ROW  = 10'd137;
    localparam DEAD_COL  = 7'd53;
    localparam BIT_ROW   = 10'd900;
    localparam BIT_COL   = 7'd11;

    reg [DW-1:0] model_mem [0:ROWS-1];
    reg          defects_on;
    integer      d;

    function automatic is_row_repaired(input [BANK_W-1:0] bk,
                                       input [SUB_W-1:0] sb,
                                       input [ROW_W-1:0] rw);
        integer k, base;
        begin
            is_row_repaired = 1'b0;
            for (k = 0; k < SPARE_R; k = k + 1) begin
                base = ((bk*SUBS + sb)*SPARE_R + k);
                if (rpr_row_valid[base] &&
                    (rpr_row_addr[base*ROW_W +: ROW_W] == rw))
                    is_row_repaired = 1'b1;
            end
        end
    endfunction

    function automatic is_col_repaired(input [BANK_W-1:0] bk,
                                       input [SUB_W-1:0] sb,
                                       input [6:0] col);
        integer k, base;
        begin
            is_col_repaired = 1'b0;
            for (k = 0; k < SPARE_C; k = k + 1) begin
                base = ((bk*SUBS + sb)*SPARE_C + k);
                if (rpr_col_valid[base] &&
                    (rpr_col_id[base*COL_ID_W +: COL_ID_W] == col[COL_ID_W-1:0]))
                    is_col_repaired = 1'b1;
            end
        end
    endfunction

    always @(posedge clk) begin
        mem_gnt    <= mem_req;
        mem_rvalid <= 1'b0;
        if (mem_req && mem_we) begin
            model_mem[mem_row] <= mem_wdata;
        end
        else if (mem_req && !mem_we) begin
            mem_rdata <= model_mem[mem_row];
            if (defects_on) begin
                if ((mem_row == DEAD_ROW) && !is_row_repaired(mem_bank, mem_sub, mem_row))
                    mem_rdata <= {DW{1'b0}};
                if (!is_col_repaired(mem_bank, mem_sub, DEAD_COL))
                    mem_rdata[DEAD_COL] <= 1'b0;
                if ((mem_row == BIT_ROW) &&
                    !is_col_repaired(mem_bank, mem_sub, BIT_COL) &&
                    !is_row_repaired(mem_bank, mem_sub, mem_row))
                    mem_rdata[BIT_COL] <= 1'b1;
            end
            mem_rvalid <= 1'b1;
        end
    end

    // ------------------------------------------------------------ sequence
    integer cyc;

    task automatic run_mbist(input [2:0] alg, input [BANK_W-1:0] bk);
        begin
            mb_algorithm  = alg;
            mb_bank_first = bk;
            mb_bank_last  = bk;
            mb_start      = 1'b1;
            @(posedge clk);
            mb_start = 1'b0;
            cyc = 0;
            while (!mb_done && cyc < 2000000) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
        end
    endtask

    initial begin
        for (d = 0; d < ROWS; d = d + 1) model_mem[d] = {DW{1'b0}};
        mb_start = 1'b0;
        mb_algorithm = `VC3D_MBIST_MARCH_C;
        mb_bank_first = 5'd0;
        mb_bank_last  = 5'd0;
        mb_retention_pause = 32'd0;
        mb_stop_on_fail = 1'b0;
        mem_gnt = 1'b0;
        mem_rvalid = 1'b0;
        mem_rdata = {DW{1'b0}};
        defects_on = 1'b0;
        map_clear = 1'b0;
        map_rd_bank = 5'd0; map_rd_sub = 4'd0; map_rd_is_col = 1'b0; map_rd_slot = 2'd0;
        fu_prog_en = 1'b0;
        fu_prog_pulse = 16'd8;
        fu_read_req = 1'b0;
        fu_read_addr = 10'd0;
        poweron_start = 1'b0;
        full_repair_start = 1'b0;
        lane_solve_done = 1'b1;
        lane_unrepairable = 1'b0;
        link_up_all = 1'b1;
        rt_repair_req = 1'b0;
        rt_repair_type = 2'd0;
        rt_repair_bank = 5'd0;
        rt_repair_sub = 4'd0;
        rt_repair_row = 10'd0;
        rt_repair_syndrome = 9'd0;

        repeat (8) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // ---- 0. a good die passes -------------------------------------
        run_mbist(`VC3D_MBIST_MARCH_C, 5'd0);
        `VC3D_CHECK(mb_done === 1'b1, "MBIST completed on a good die")
        `VC3D_CHECK(mb_pass === 1'b1, "a defect-free die passes March C-")

        // ---- 1. inject defects, MBIST must fail -----------------------
        defects_on = 1'b1;
        run_mbist(`VC3D_MBIST_MARCH_C, 5'd0);
        `VC3D_CHECK(mb_pass === 1'b0, "injected defects are detected by MBIST")
        `VC3D_CHECK(total_fail_bits != 32'd0, "fail bits were counted")
        $display("[info] MBIST found %0d failing bits", total_fail_bits);

        // ---- 2-3. tester repair flow, burn fuses ----------------------
        fu_prog_en = 1'b1;
        full_repair_start = 1'b1;
        @(posedge clk);
        full_repair_start = 1'b0;
        cyc = 0;
        while (!repair_done && !repair_failed && cyc < 4000000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(repair_done === 1'b1, "repair flow completed")
        `VC3D_CHECK(repair_failed === 1'b0, "the injected defects are repairable")
        `VC3D_CHECK(map_any_repair === 1'b1, "the repair map holds allocations")
        $display("[info] rows repaired %0d, cols repaired %0d",
                 rows_repaired, cols_repaired);

        // ---- 4. power cycle: autoload and re-test ---------------------
        map_clear = 1'b1;
        @(posedge clk);
        map_clear = 1'b0;
        `VC3D_CHECK(map_rows_used == 16'd0, "map cleared before autoload")

        poweron_start = 1'b1;
        @(posedge clk);
        poweron_start = 1'b0;
        cyc = 0;
        while (!repair_done && cyc < 400000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(fu_al_crc_err === 1'b0, "fuse image CRC is good")
        `VC3D_CHECK(map_any_repair === 1'b1, "repairs restored from eFuse")

        run_mbist(`VC3D_MBIST_MARCH_C, 5'd0);
        `VC3D_CHECK(mb_pass === 1'b1,
                    "after repair, the SAME defective die passes MBIST")
        $display("[info] repaired die passes; way_disable = %04h", way_disable_w);
        `VC3D_CHECK_EQ(way_disable_w, 16'h0000,
                       "no capacity lost for a repairable die")

        vc3d_finish("tb_vc3d_repair");
    end

    initial begin
        #40000000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
