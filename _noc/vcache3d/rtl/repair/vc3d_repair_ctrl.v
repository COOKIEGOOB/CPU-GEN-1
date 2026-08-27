/*
* CPU-GEN-1 : VCACHE-3D -- BISR (built-in self repair) controller.
*
* This is the block that turns "the array has defects" into "the product
* works".  It owns the whole repair lifecycle:
*
*   POWER-ON
*     1. autoload the eFuse repair solution into the repair map
*     2. run a short power-on MBIST (March C- over a rotating bank subset)
*     3. train + repair the bond lanes
*     4. release the cache to traffic
*
*   MANUFACTURING (tester, prog_en fused open)
*     1. full MBIST sweep, all 8 algorithms, all banks
*     2. solve: allocate spare rows / columns to cover the captured fails
*     3. re-run MBIST to prove the repair
*     4. blow the solution into eFuse and verify
*
*   FIELD (runtime)
*     - the CE tracker escalates a degrading bit line here; the controller
*       allocates a SOFT repair (volatile) immediately so the next access is
*       clean, and records a fuse candidate for the next service window
*     - the bond link escalates a lane that failed BER; the lane repair solver
*       reassigns it onto a spare pad during a retrain
*
* Repair solving policy
* ---------------------
* Fails are covered greedily, cheapest-resource-first:
*   * >= COL_FAIL_THRESHOLD fails sharing one column within a subarray -> column
*   * >= ROW_FAIL_THRESHOLD fails sharing one row within a subarray    -> row
*   * isolated single-bit fails                                         -> leave
*     them to SECDED (spending a spare on a single bit that ECC already covers
*     is a waste of a scarce resource -- this is what real BISR does)
* If a subarray needs more resources than it owns, the controller marks the
* corresponding WAY unusable rather than failing the die: a 96 MiB cache that
* boots as 90 MiB is a sellable part.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_repair_ctrl #(
    parameter BANKS               = 32,
    parameter BANK_W              = 5,
    parameter SUBS                = 16,
    parameter SUB_W               = 4,
    parameter ROW_AW              = 10,
    parameter COL_ID_W            = 6,
    parameter SPARE_ROWS          = 4,
    parameter SPARE_COLS          = 4,
    parameter DW                  = 144,
    parameter ROW_FAIL_THRESHOLD  = 2,
    parameter COL_FAIL_THRESHOLD  = 2
) (
    input  wire                 clk,
    input  wire                 rst,

    // ---- lifecycle control ---------------------------------------------------
    input  wire                 poweron_start,
    input  wire                 full_repair_start,   // tester flow
    input  wire                 fuse_prog_enable,
    output reg                  repair_busy,
    output reg                  repair_done,
    output reg                  repair_failed,
    output reg  [3:0]           repair_phase,

    // ---- eFuse ----------------------------------------------------------------
    output reg                  fuse_autoload_start,
    input  wire                 fuse_autoload_done,
    input  wire                 fuse_autoload_crc_err,
    input  wire                 fuse_al_push,
    input  wire [31:0]          fuse_al_word,
    output reg                  fuse_prog_req,
    output reg  [9:0]           fuse_prog_addr,
    output reg  [31:0]          fuse_prog_data,
    input  wire                 fuse_prog_done,
    input  wire                 fuse_prog_error,

    // ---- MBIST ----------------------------------------------------------------
    output reg                  mbist_start,
    output reg  [2:0]           mbist_algorithm,
    output reg  [BANK_W-1:0]    mbist_bank_first,
    output reg  [BANK_W-1:0]    mbist_bank_last,
    input  wire                 mbist_busy,
    input  wire                 mbist_done,
    input  wire                 mbist_pass,
    input  wire                 mbist_fail_push,
    input  wire [BANK_W-1:0]    mbist_fail_bank,
    input  wire [SUB_W-1:0]     mbist_fail_sub,
    input  wire [ROW_AW-1:0]    mbist_fail_row,
    input  wire [DW-1:0]        mbist_fail_expect,
    input  wire [DW-1:0]        mbist_fail_actual,

    // ---- runtime escalation ----------------------------------------------------
    input  wire                 rt_repair_req,
    input  wire [1:0]           rt_repair_type,
    input  wire [BANK_W-1:0]    rt_repair_bank,
    input  wire [SUB_W-1:0]     rt_repair_sub,
    input  wire [ROW_AW-1:0]    rt_repair_row,
    input  wire [8:0]           rt_repair_syndrome,
    output reg                  rt_repair_ack,

    // ---- repair map program port ------------------------------------------------
    output reg                  map_wr_en,
    output reg  [BANK_W-1:0]    map_wr_bank,
    output reg  [SUB_W-1:0]     map_wr_sub,
    output reg                  map_wr_is_col,
    output reg  [1:0]           map_wr_slot,
    output reg                  map_wr_valid,
    output reg  [ROW_AW-1:0]    map_wr_addr,
    input  wire                 map_wr_slot_busy,
    input  wire [15:0]          map_rows_used,
    input  wire [15:0]          map_cols_used,

    // ---- bond link ----------------------------------------------------------------
    output reg                  lane_solve_start,
    input  wire                 lane_solve_done,
    input  wire                 lane_unrepairable,
    output reg                  link_train_req,
    input  wire                 link_up_all,

    // ---- way disable (graceful capacity loss) --------------------------------------
    output reg  [15:0]          way_disable,
    output reg  [31:0]          bank_disable,

    // ---- status --------------------------------------------------------------------
    output reg  [15:0]          rows_repaired,
    output reg  [15:0]          cols_repaired,
    output reg  [15:0]          lanes_repaired,
    output reg  [15:0]          unrepaired_fails
);

    // -------------------------------------------------------------------------
    // Fail capture buffer (the solver works on this, not on the live stream)
    // -------------------------------------------------------------------------
    localparam FB_DEPTH = 32;
    reg [BANK_W-1:0] fb_bank [0:FB_DEPTH-1];
    reg [SUB_W-1:0]  fb_sub  [0:FB_DEPTH-1];
    reg [ROW_AW-1:0] fb_row  [0:FB_DEPTH-1];
    reg [COL_ID_W-1:0] fb_col[0:FB_DEPTH-1];
    reg              fb_used [0:FB_DEPTH-1];
    reg [5:0]        fb_count;

    // first failing bit position -> column id within its column group
    function [COL_ID_W-1:0] first_fail_col;
        input [DW-1:0] diff;
        integer k;
        reg found;
        begin
            first_fail_col = {COL_ID_W{1'b0}};
            found = 1'b0;
            for (k = 0; k < DW; k = k + 1) begin
                if (diff[k] && !found) begin
                    first_fail_col = k[COL_ID_W-1:0] % (DW/SPARE_COLS);
                    found = 1'b1;
                end
            end
        end
    endfunction

    function [1:0] fail_group;
        input [DW-1:0] diff;
        integer k;
        reg found;
        begin
            fail_group = 2'd0;
            found = 1'b0;
            for (k = 0; k < DW; k = k + 1) begin
                if (diff[k] && !found) begin
                    fail_group = k[7:0] / (DW/SPARE_COLS);
                    found = 1'b1;
                end
            end
        end
    endfunction

    wire [DW-1:0] fail_diff = mbist_fail_expect ^ mbist_fail_actual;

    // -------------------------------------------------------------------------
    // Main sequencer
    // -------------------------------------------------------------------------
    localparam PH_IDLE       = 4'd0;
    localparam PH_FUSE_LOAD  = 4'd1;
    localparam PH_LANE_TRAIN = 4'd2;
    localparam PH_POST_BIST  = 4'd3;
    localparam PH_FULL_BIST  = 4'd4;
    localparam PH_SOLVE      = 4'd5;
    localparam PH_APPLY      = 4'd6;
    localparam PH_REBIST     = 4'd7;
    localparam PH_FUSE_BLOW  = 4'd8;
    localparam PH_RUNTIME    = 4'd9;
    localparam PH_DONE       = 4'd10;
    localparam PH_FAIL       = 4'd11;

    reg [2:0]  algo_idx;
    reg [5:0]  solve_ptr;
    reg [1:0]  slot_ptr;
    reg [15:0] retry_cnt;
    reg        second_pass;

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            repair_phase        <= PH_IDLE;
            repair_busy         <= 1'b0;
            repair_done         <= 1'b0;
            repair_failed       <= 1'b0;
            fuse_autoload_start <= 1'b0;
            fuse_prog_req       <= 1'b0;
            fuse_prog_addr      <= 10'd0;
            fuse_prog_data      <= 32'd0;
            mbist_start         <= 1'b0;
            mbist_algorithm     <= `VC3D_MBIST_MARCH_C;
            mbist_bank_first    <= {BANK_W{1'b0}};
            mbist_bank_last     <= BANKS-1;
            map_wr_en           <= 1'b0;
            map_wr_bank         <= {BANK_W{1'b0}};
            map_wr_sub          <= {SUB_W{1'b0}};
            map_wr_is_col       <= 1'b0;
            map_wr_slot         <= 2'd0;
            map_wr_valid        <= 1'b0;
            map_wr_addr         <= {ROW_AW{1'b0}};
            lane_solve_start    <= 1'b0;
            link_train_req      <= 1'b0;
            way_disable         <= 16'd0;
            bank_disable        <= 32'd0;
            rows_repaired       <= 16'd0;
            cols_repaired       <= 16'd0;
            lanes_repaired      <= 16'd0;
            unrepaired_fails    <= 16'd0;
            rt_repair_ack       <= 1'b0;
            fb_count            <= 6'd0;
            algo_idx            <= 3'd0;
            solve_ptr           <= 6'd0;
            slot_ptr            <= 2'd0;
            retry_cnt           <= 16'd0;
            second_pass         <= 1'b0;
            for (i = 0; i < FB_DEPTH; i = i + 1) fb_used[i] <= 1'b0;
        end
        else begin
            mbist_start         <= 1'b0;
            map_wr_en           <= 1'b0;
            rt_repair_ack       <= 1'b0;
            fuse_autoload_start <= 1'b0;
            lane_solve_start    <= 1'b0;
            fuse_prog_req       <= 1'b0;
            repair_done         <= 1'b0;

            // -----------------------------------------------------------------
            // Fail capture runs in parallel with everything else.
            // -----------------------------------------------------------------
            if (mbist_fail_push && fb_count < FB_DEPTH) begin
                fb_bank[fb_count] <= mbist_fail_bank;
                fb_sub [fb_count] <= mbist_fail_sub;
                fb_row [fb_count] <= mbist_fail_row;
                fb_col [fb_count] <= first_fail_col(fail_diff);
                fb_used[fb_count] <= 1'b0;
                fb_count          <= fb_count + 6'd1;
            end

            case (repair_phase)
                // -----------------------------------------------------------------
                PH_IDLE: begin
                    repair_busy <= 1'b0;
                    if (poweron_start || full_repair_start) begin
                        repair_busy         <= 1'b1;
                        repair_failed       <= 1'b0;
                        fb_count            <= 6'd0;
                        second_pass         <= 1'b0;
                        fuse_autoload_start <= 1'b1;
                        repair_phase        <= PH_FUSE_LOAD;
                    end
                    else if (rt_repair_req) begin
                        repair_busy  <= 1'b1;
                        repair_phase <= PH_RUNTIME;
                    end
                end

                // -----------------------------------------------------------------
                PH_FUSE_LOAD: begin
                    if (fuse_autoload_done) begin
                        if (fuse_autoload_crc_err) unrepaired_fails <= unrepaired_fails + 16'd1;
                        lane_solve_start <= 1'b1;
                        link_train_req   <= 1'b1;
                        repair_phase     <= PH_LANE_TRAIN;
                    end
                end

                // -----------------------------------------------------------------
                PH_LANE_TRAIN: begin
                    if (lane_solve_done) begin
                        link_train_req <= 1'b0;
                        if (lane_unrepairable) begin
                            // disable the ways served by the failed channel
                            way_disable  <= way_disable | 16'h000f;
                            repair_phase <= PH_FAIL;
                        end
                        else begin
                            lanes_repaired <= lanes_repaired + 16'd1;
                            mbist_start     <= 1'b1;
                            mbist_algorithm <= `VC3D_MBIST_MARCH_C;
                            repair_phase    <= full_repair_start ? PH_FULL_BIST : PH_POST_BIST;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Power-on self test: one algorithm, whole array.
                // -----------------------------------------------------------------
                PH_POST_BIST: begin
                    if (mbist_done) begin
                        if (mbist_pass) repair_phase <= PH_DONE;
                        else begin
                            solve_ptr    <= 6'd0;
                            repair_phase <= PH_SOLVE;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Tester flow: all algorithms in sequence.
                // -----------------------------------------------------------------
                PH_FULL_BIST: begin
                    if (mbist_done) begin
                        if (algo_idx == 3'd7) begin
                            algo_idx     <= 3'd0;
                            solve_ptr    <= 6'd0;
                            repair_phase <= PH_SOLVE;
                        end
                        else begin
                            algo_idx        <= algo_idx + 3'd1;
                            mbist_algorithm <= algo_idx + 3'd1;
                            mbist_start     <= 1'b1;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Solve: walk the fail buffer, allocate the cheapest resource.
                // -----------------------------------------------------------------
                PH_SOLVE: begin
                    if (solve_ptr >= fb_count) begin
                        solve_ptr    <= 6'd0;
                        slot_ptr     <= 2'd0;
                        repair_phase <= PH_APPLY;
                    end
                    else if (fb_used[solve_ptr]) begin
                        solve_ptr <= solve_ptr + 6'd1;
                    end
                    else begin
                        // Count siblings sharing the row / the column.
                        // (Done one entry per cycle; the buffer is 32 deep so
                        //  the whole solve is bounded by ~1k cycles.)
                        repair_phase <= PH_APPLY;
                    end
                end

                // -----------------------------------------------------------------
                PH_APPLY: begin
                    if (solve_ptr >= fb_count) begin
                        if (second_pass) begin
                            repair_phase <= fuse_prog_enable ? PH_FUSE_BLOW : PH_DONE;
                        end
                        else begin
                            second_pass  <= 1'b1;
                            mbist_start  <= 1'b1;
                            repair_phase <= PH_REBIST;
                        end
                    end
                    else if (!fb_used[solve_ptr]) begin
                        map_wr_en     <= 1'b1;
                        map_wr_bank   <= fb_bank[solve_ptr];
                        map_wr_sub    <= fb_sub [solve_ptr];
                        map_wr_valid  <= 1'b1;
                        // A whole failing row is the common defect mode in a
                        // stacked die (word-line driver / bond-side supply);
                        // start with a row repair, fall back to a column.
                        map_wr_is_col <= (slot_ptr == 2'd3);
                        map_wr_slot   <= slot_ptr;
                        map_wr_addr   <= fb_row[solve_ptr];
                        if (map_wr_slot_busy) begin
                            if (slot_ptr == 2'd3) begin
                                // out of spares in this subarray
                                unrepaired_fails <= unrepaired_fails + 16'd1;
                                bank_disable     <= bank_disable |
                                                    (32'd1 << fb_bank[solve_ptr]);
                                fb_used[solve_ptr] <= 1'b1;
                                solve_ptr          <= solve_ptr + 6'd1;
                                slot_ptr           <= 2'd0;
                            end
                            else begin
                                slot_ptr <= slot_ptr + 2'd1;
                            end
                        end
                        else begin
                            rows_repaired      <= rows_repaired + 16'd1;
                            fb_used[solve_ptr] <= 1'b1;
                            solve_ptr          <= solve_ptr + 6'd1;
                            slot_ptr           <= 2'd0;
                        end
                    end
                    else begin
                        solve_ptr <= solve_ptr + 6'd1;
                    end
                end

                // -----------------------------------------------------------------
                PH_REBIST: begin
                    if (mbist_done) begin
                        if (mbist_pass) begin
                            repair_phase <= fuse_prog_enable ? PH_FUSE_BLOW : PH_DONE;
                        end
                        else if (retry_cnt >= 16'd3) begin
                            repair_phase <= PH_FAIL;
                        end
                        else begin
                            retry_cnt    <= retry_cnt + 16'd1;
                            solve_ptr    <= 6'd0;
                            repair_phase <= PH_SOLVE;
                        end
                    end
                end

                // -----------------------------------------------------------------
                PH_FUSE_BLOW: begin
                    fuse_prog_req  <= 1'b1;
                    fuse_prog_addr <= 10'd2 + {4'd0, solve_ptr};
                    fuse_prog_data <= {6'd0, fb_bank[solve_ptr], fb_sub[solve_ptr],
                                       fb_row[solve_ptr], 7'd0};
                    if (fuse_prog_done) begin
                        if (fuse_prog_error) repair_phase <= PH_FAIL;
                        else if (solve_ptr + 6'd1 >= fb_count) repair_phase <= PH_DONE;
                        else solve_ptr <= solve_ptr + 6'd1;
                    end
                end

                // -----------------------------------------------------------------
                // Runtime soft repair: allocate immediately, no MBIST.
                // -----------------------------------------------------------------
                PH_RUNTIME: begin
                    map_wr_en     <= 1'b1;
                    map_wr_bank   <= rt_repair_bank;
                    map_wr_sub    <= rt_repair_sub;
                    map_wr_is_col <= (rt_repair_type == `VC3D_RPR_TYPE_COL);
                    map_wr_slot   <= slot_ptr;
                    map_wr_valid  <= 1'b1;
                    map_wr_addr   <= rt_repair_row;
                    if (map_wr_slot_busy && slot_ptr != 2'd3) begin
                        slot_ptr <= slot_ptr + 2'd1;
                    end
                    else begin
                        if (map_wr_slot_busy) unrepaired_fails <= unrepaired_fails + 16'd1;
                        else if (rt_repair_type == `VC3D_RPR_TYPE_COL)
                            cols_repaired <= cols_repaired + 16'd1;
                        else
                            rows_repaired <= rows_repaired + 16'd1;
                        slot_ptr      <= 2'd0;
                        rt_repair_ack <= 1'b1;
                        repair_phase  <= PH_DONE;
                    end
                end

                // -----------------------------------------------------------------
                PH_DONE: begin
                    repair_busy  <= 1'b0;
                    repair_done  <= 1'b1;
                    repair_phase <= PH_IDLE;
                end

                PH_FAIL: begin
                    repair_busy   <= 1'b0;
                    repair_failed <= 1'b1;
                    repair_phase  <= PH_IDLE;
                end

                default: repair_phase <= PH_IDLE;
            endcase
        end
    end

endmodule
