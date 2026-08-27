/*
* CPU-GEN-1 : VCACHE-3D -- RAS error log (first-error + FIFO history).
*
* Software-visible error record, modelled on the ACPI/APEI expectations of a
* server memory-side cache:
*
*   * FIRST-ERROR registers latch the first CE and the first UE after each
*     clear, with full address/way/syndrome context, and never change until
*     software clears them (the "first fault wins" rule -- a later avalanche
*     must not overwrite the root cause).
*   * A 32-deep FIFO records subsequent events with an overflow sticky bit.
*   * Saturating CE/UE counters per source (data, tag, state, SF, bond).
*   * A UE raises `fatal_o` when the poisoned line is unrecoverable (dirty and
*     not backed by memory); otherwise `nonfatal_o` is raised and the line is
*     poison-forwarded so the consumer can contain the failure.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_error_log #(
    parameter DEPTH   = 32,
    parameter DEPTH_W = 5,
    parameter ADDR_W  = 48
) (
    input  wire               clk,
    input  wire               rst,

    // ---- event input --------------------------------------------------------
    input  wire               push,
    input  wire [2:0]         ev_class,
    input  wire [2:0]         ev_src,
    input  wire [ADDR_W-1:0]  ev_addr,
    input  wire [3:0]         ev_way,
    input  wire [35:0]        ev_syndrome,
    input  wire               ev_dirty,

    // ---- software interface -------------------------------------------------
    input  wire               clear_first,
    input  wire               clear_counts,
    input  wire               pop,
    output wire               fifo_empty,
    output wire [DEPTH_W:0]   fifo_level,
    output wire [63:0]        fifo_rd_lo,
    output wire [63:0]        fifo_rd_hi,

    output reg  [63:0]        first_ce_lo,
    output reg  [63:0]        first_ce_hi,
    output reg  [63:0]        first_ue_lo,
    output reg  [63:0]        first_ue_hi,
    output reg                first_ce_valid,
    output reg                first_ue_valid,
    output reg                overflow,

    output reg  [31:0]        ce_count,
    output reg  [31:0]        ue_count,
    output reg  [31:0]        poison_count,
    output reg  [31:0]        link_err_count,

    // ---- interrupts ---------------------------------------------------------
    output reg                nonfatal_o,
    output reg                fatal_o
);

    // record packing: {syndrome[35:0], way[3:0], src[2:0], class[3:0]} | addr
    wire [63:0] rec_lo = {16'd0, ev_addr};
    wire [63:0] rec_hi = {ev_dirty, 20'd0, ev_syndrome, ev_way[3:0], ev_src};

    reg [63:0] fifo_lo [0:DEPTH-1];
    reg [63:0] fifo_hi [0:DEPTH-1];
    reg [DEPTH_W:0] wptr, rptr;

    wire full  = ((wptr[DEPTH_W-1:0] == rptr[DEPTH_W-1:0]) &&
                  (wptr[DEPTH_W] != rptr[DEPTH_W]));
    assign fifo_empty = (wptr == rptr);
    assign fifo_level = wptr - rptr;
    assign fifo_rd_lo = fifo_lo[rptr[DEPTH_W-1:0]];
    assign fifo_rd_hi = fifo_hi[rptr[DEPTH_W-1:0]];

    wire is_ce     = (ev_class == `VC3D_ERR_CE) || (ev_class == `VC3D_ERR_CHECKBIT_CE);
    wire is_ue     = (ev_class == `VC3D_ERR_UE) || (ev_class == `VC3D_ERR_TAG_UE);
    wire is_poison = (ev_class == `VC3D_ERR_POISON);
    wire is_link   = (ev_class == `VC3D_ERR_LINK_CRC) || (ev_class == `VC3D_ERR_LINK_LANE);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wptr <= {(DEPTH_W+1){1'b0}};
            rptr <= {(DEPTH_W+1){1'b0}};
            overflow <= 1'b0;
        end
        else begin
            if (push) begin
                if (!full) begin
                    fifo_lo[wptr[DEPTH_W-1:0]] <= rec_lo;
                    fifo_hi[wptr[DEPTH_W-1:0]] <= rec_hi;
                    wptr <= wptr + 1'b1;
                end
                else begin
                    overflow <= 1'b1;
                end
            end
            if (pop && !fifo_empty) rptr <= rptr + 1'b1;
            if (clear_first) overflow <= 1'b0;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            first_ce_valid <= 1'b0;
            first_ue_valid <= 1'b0;
            first_ce_lo <= 64'd0; first_ce_hi <= 64'd0;
            first_ue_lo <= 64'd0; first_ue_hi <= 64'd0;
        end
        else if (clear_first) begin
            first_ce_valid <= 1'b0;
            first_ue_valid <= 1'b0;
        end
        else begin
            if (push && is_ce && !first_ce_valid) begin
                first_ce_valid <= 1'b1;
                first_ce_lo    <= rec_lo;
                first_ce_hi    <= rec_hi;
            end
            if (push && is_ue && !first_ue_valid) begin
                first_ue_valid <= 1'b1;
                first_ue_lo    <= rec_lo;
                first_ue_hi    <= rec_hi;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ce_count <= 32'd0; ue_count <= 32'd0;
            poison_count <= 32'd0; link_err_count <= 32'd0;
            nonfatal_o <= 1'b0; fatal_o <= 1'b0;
        end
        else begin
            nonfatal_o <= 1'b0;
            fatal_o    <= 1'b0;
            if (clear_counts) begin
                ce_count <= 32'd0; ue_count <= 32'd0;
                poison_count <= 32'd0; link_err_count <= 32'd0;
            end
            else if (push) begin
                if (is_ce     && ce_count       != 32'hffffffff) ce_count       <= ce_count + 1'b1;
                if (is_ue     && ue_count       != 32'hffffffff) ue_count       <= ue_count + 1'b1;
                if (is_poison && poison_count   != 32'hffffffff) poison_count   <= poison_count + 1'b1;
                if (is_link   && link_err_count != 32'hffffffff) link_err_count <= link_err_count + 1'b1;
            end
            if (push && is_ue) begin
                // A dirty line with no memory copy is unrecoverable -> fatal.
                fatal_o    <= ev_dirty;
                nonfatal_o <= ~ev_dirty;
            end
            else if (push && (is_ce || is_link || is_poison)) begin
                nonfatal_o <= 1'b1;
            end
        end
    end

endmodule
