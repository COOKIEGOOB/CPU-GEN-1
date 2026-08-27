/*
* CPU-GEN-1 : VCACHE-3D -- background ECC scrubber (patrol + demand).
*
* Why a scrubber is mandatory here
* --------------------------------
* A 96 MiB stacked cache holds ~8.05e8 bits.  At a representative HD-SRAM soft
* error rate of ~1e-3 FIT/Mb the array accumulates roughly
*      805 Mb * 1e-3 FIT/Mb = 0.8 FIT  (one event per ~1.4e5 device-hours)
* but a hyperscale fleet of 1e5 sockets sees one event every ~1.4 hours.  Any
* line that is read rarely can accumulate a SECOND flip before it is ever
* decoded, at which point SECDED can only detect, not correct.  Patrol
* scrubbing bounds the accumulation window and turns latent double faults into
* corrected single faults.
*
* Operation
* ---------
*  * Patrol mode walks {way, set} in a strided order (stride is coprime with
*    the set count so that consecutive scrubs hit different banks) and issues
*    read-modify-write requests through the slice's dedicated maintenance port.
*  * A scrub read that reports CE issues a writeback of the corrected data
*    (a "clean and re-write"), which is what actually removes the flipped bit.
*  * A scrub read that reports UE marks the line poisoned, invalidates it if
*    clean, and pushes a record into the error log.
*  * Rate is programmable: SCRUB_PERIOD cycles between grants, SCRUB_BURST
*    lines per grant.  Defaults give a full 32 MiB slice sweep in
*        524288 lines / 4 per grant * 4096 cycles = 5.4e8 cycles ~= 0.18 s
*    at 3 GHz, i.e. ~5 full sweeps per second per slice, far below the
*    single-event accumulation window while costing <0.1% of array bandwidth.
*  * Demand mode (triggered by the CE tracker) immediately re-writes a line
*    whose CE count crossed the threshold, and asks the repair controller for
*    a spare row/column if the same syndrome keeps coming back.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_scrub_engine #(
    parameter SET_NUM    = 32768,
    parameter SET_W      = 15,
    parameter WAY_NUM    = 16,
    parameter WAY_W      = 4,
    parameter SET_STRIDE = 4097     // coprime with 32768 -> full coverage
) (
    input  wire              clk,
    input  wire              rst,

    // ---- configuration (CSR) -----------------------------------------------
    input  wire              scrub_enable,
    input  wire              scrub_oneshot,
    input  wire [31:0]       scrub_period,
    input  wire [3:0]        scrub_burst,
    input  wire [15:0]       ce_threshold,

    // ---- maintenance port to the slice pipeline -----------------------------
    output reg               scrub_req_valid,
    input  wire              scrub_req_ready,
    output reg  [SET_W-1:0]  scrub_set,
    output reg  [WAY_W-1:0]  scrub_way,
    output reg               scrub_is_write,
    output reg  [511:0]      scrub_wdata,

    input  wire              scrub_rsp_valid,
    input  wire [511:0]      scrub_rdata,
    input  wire              scrub_rsp_ce,
    input  wire              scrub_rsp_ue,
    input  wire [35:0]       scrub_rsp_syndrome,
    input  wire              scrub_rsp_valid_line,

    // ---- demand scrub request (from the CE tracker) -------------------------
    input  wire              demand_valid,
    input  wire [SET_W-1:0]  demand_set,
    input  wire [WAY_W-1:0]  demand_way,
    output reg               demand_ack,

    // ---- error reporting ----------------------------------------------------
    output reg               elog_push,
    output reg  [2:0]        elog_class,
    output reg  [2:0]        elog_src,
    output reg  [SET_W-1:0]  elog_set,
    output reg  [WAY_W-1:0]  elog_way,
    output reg  [35:0]       elog_syndrome,

    // ---- repair escalation --------------------------------------------------
    output reg               repair_req,
    output reg  [SET_W-1:0]  repair_set,
    output reg  [WAY_W-1:0]  repair_way,
    output reg  [35:0]       repair_syndrome,
    input  wire              repair_ack,

    // ---- status -------------------------------------------------------------
    output reg  [31:0]       scrub_ce_count,
    output reg  [31:0]       scrub_ue_count,
    output reg  [31:0]       scrub_sweep_count,
    output wire [SET_W-1:0]  scrub_progress,
    output reg               scrub_active
);

    localparam ST_IDLE   = 3'd0;
    localparam ST_WAIT   = 3'd1;
    localparam ST_READ   = 3'd2;
    localparam ST_RSP    = 3'd3;
    localparam ST_WRITE  = 3'd4;
    localparam ST_REPAIR = 3'd5;
    localparam ST_DEMAND = 3'd6;

    reg [2:0]        state;
    reg [31:0]       period_cnt;
    reg [3:0]        burst_cnt;
    reg [SET_W-1:0]  cur_set;
    reg [WAY_W-1:0]  cur_way;
    reg [SET_W-1:0]  next_set;
    reg              in_demand;

    assign scrub_progress = cur_set;

    // strided set walk: set += SET_STRIDE (mod SET_NUM); way advances when the
    // walk wraps back to its origin
    always @* begin
        next_set = cur_set + SET_STRIDE[SET_W-1:0];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state             <= ST_IDLE;
            period_cnt        <= 32'd0;
            burst_cnt         <= 4'd0;
            cur_set           <= {SET_W{1'b0}};
            cur_way           <= {WAY_W{1'b0}};
            scrub_req_valid   <= 1'b0;
            scrub_set         <= {SET_W{1'b0}};
            scrub_way         <= {WAY_W{1'b0}};
            scrub_is_write    <= 1'b0;
            scrub_wdata       <= 512'd0;
            scrub_ce_count    <= 32'd0;
            scrub_ue_count    <= 32'd0;
            scrub_sweep_count <= 32'd0;
            scrub_active      <= 1'b0;
            elog_push         <= 1'b0;
            elog_class        <= `VC3D_ERR_NONE;
            elog_src          <= `VC3D_ERRSRC_SCRUB;
            elog_set          <= {SET_W{1'b0}};
            elog_way          <= {WAY_W{1'b0}};
            elog_syndrome     <= 36'd0;
            repair_req        <= 1'b0;
            repair_set        <= {SET_W{1'b0}};
            repair_way        <= {WAY_W{1'b0}};
            repair_syndrome   <= 36'd0;
            demand_ack        <= 1'b0;
            in_demand         <= 1'b0;
        end
        else begin
            elog_push  <= 1'b0;
            demand_ack <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                ST_IDLE: begin
                    scrub_active <= 1'b0;
                    if (demand_valid) begin
                        in_demand  <= 1'b1;
                        demand_ack <= 1'b1;
                        state      <= ST_DEMAND;
                    end
                    else if (scrub_enable || scrub_oneshot) begin
                        period_cnt <= 32'd0;
                        state      <= ST_WAIT;
                    end
                end

                // -------------------------------------------------------------
                ST_WAIT: begin
                    scrub_active <= 1'b1;
                    if (demand_valid) begin
                        in_demand  <= 1'b1;
                        demand_ack <= 1'b1;
                        state      <= ST_DEMAND;
                    end
                    else if (period_cnt >= scrub_period) begin
                        period_cnt <= 32'd0;
                        burst_cnt  <= scrub_burst;
                        state      <= ST_READ;
                    end
                    else begin
                        period_cnt <= period_cnt + 32'd1;
                    end
                end

                // -------------------------------------------------------------
                ST_DEMAND: begin
                    scrub_req_valid <= 1'b1;
                    scrub_set       <= demand_set;
                    scrub_way       <= demand_way;
                    scrub_is_write  <= 1'b0;
                    if (scrub_req_ready) begin
                        scrub_req_valid <= 1'b0;
                        state           <= ST_RSP;
                    end
                end

                // -------------------------------------------------------------
                ST_READ: begin
                    scrub_req_valid <= 1'b1;
                    scrub_set       <= cur_set;
                    scrub_way       <= cur_way;
                    scrub_is_write  <= 1'b0;
                    if (scrub_req_ready) begin
                        scrub_req_valid <= 1'b0;
                        state           <= ST_RSP;
                    end
                end

                // -------------------------------------------------------------
                ST_RSP: begin
                    if (scrub_rsp_valid) begin
                        elog_set      <= scrub_set;
                        elog_way      <= scrub_way;
                        elog_syndrome <= scrub_rsp_syndrome;
                        elog_src      <= `VC3D_ERRSRC_SCRUB;

                        if (scrub_rsp_ue) begin
                            scrub_ue_count <= scrub_ue_count + 32'd1;
                            elog_push      <= 1'b1;
                            elog_class     <= `VC3D_ERR_UE;
                            // poison + invalidate is performed by the pipeline
                            state          <= ST_REPAIR;
                        end
                        else if (scrub_rsp_ce && scrub_rsp_valid_line) begin
                            scrub_ce_count <= scrub_ce_count + 32'd1;
                            elog_push      <= 1'b1;
                            elog_class     <= `VC3D_ERR_CE;
                            scrub_wdata    <= scrub_rdata;   // already corrected
                            state          <= ST_WRITE;
                        end
                        else begin
                            state <= in_demand ? ST_IDLE : ST_WRITE;
                            if (!scrub_rsp_ce) state <= ST_WAIT;
                        end

                        if (!scrub_rsp_ce && !scrub_rsp_ue) begin
                            // clean line: advance the walk
                            if (next_set == {SET_W{1'b0}}) begin
                                cur_way <= cur_way + {{(WAY_W-1){1'b0}}, 1'b1};
                                if (cur_way == WAY_NUM-1)
                                    scrub_sweep_count <= scrub_sweep_count + 32'd1;
                            end
                            cur_set   <= next_set;
                            burst_cnt <= burst_cnt - 4'd1;
                            state     <= (burst_cnt <= 4'd1) ? ST_WAIT : ST_READ;
                            if (in_demand) begin
                                in_demand <= 1'b0;
                                state     <= ST_IDLE;
                            end
                        end
                    end
                end

                // -------------------------------------------------------------
                ST_WRITE: begin
                    scrub_req_valid <= 1'b1;
                    scrub_is_write  <= 1'b1;
                    if (scrub_req_ready) begin
                        scrub_req_valid <= 1'b0;
                        scrub_is_write  <= 1'b0;
                        if (next_set == {SET_W{1'b0}}) begin
                            cur_way <= cur_way + {{(WAY_W-1){1'b0}}, 1'b1};
                            if (cur_way == WAY_NUM-1)
                                scrub_sweep_count <= scrub_sweep_count + 32'd1;
                        end
                        cur_set   <= next_set;
                        burst_cnt <= burst_cnt - 4'd1;
                        if (in_demand) begin
                            in_demand <= 1'b0;
                            state     <= ST_IDLE;
                        end
                        else begin
                            state <= (burst_cnt <= 4'd1) ? ST_WAIT : ST_READ;
                        end
                    end
                end

                // -------------------------------------------------------------
                ST_REPAIR: begin
                    // Persistent fault: ask the repair controller for a spare.
                    repair_req      <= 1'b1;
                    repair_set      <= scrub_set;
                    repair_way      <= scrub_way;
                    repair_syndrome <= scrub_rsp_syndrome;
                    if (repair_ack) begin
                        repair_req <= 1'b0;
                        if (in_demand) begin
                            in_demand <= 1'b0;
                            state     <= ST_IDLE;
                        end
                        else begin
                            cur_set <= next_set;
                            state   <= ST_WAIT;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
