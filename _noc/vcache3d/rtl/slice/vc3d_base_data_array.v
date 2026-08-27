/*
* CPU-GEN-1 : VCACHE-3D -- base-die data array (the un-stacked 8 MiB).
*
* Ways 0..3 of every set live here, on the base die, next to the pipeline.
* Ways 4..15 live on the stacked dielet.  That split is the heart of the
* design:
*
*   * the base-die ways give the cache a LOW-LATENCY region -- a hit in ways
*     0..3 does not pay the bond round trip at all, so the cache's effective
*     latency is far better than a uniformly stacked array;
*   * the insertion policy places new lines in the base ways and demotes them
*     into the stacked ways on eviction pressure, so the hot working set
*     naturally lives in the fast region (this is a NUCA policy, and it is
*     what makes 96 MiB usable rather than merely large);
*   * the stacked die can be power-collapsed entirely and the cache still
*     works at 8 MiB per slice.
*
* Storage is 576 bits per line (512 data + 64 ECC/poison/metadata), matching
* the stacked-array word format exactly so that a line can migrate between the
* two regions without re-encoding.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_base_data_array #(
    parameter SETS  = 32768,
    parameter SET_W = 15,
    parameter WAYS  = 4,
    parameter WAY_W = 2,
    parameter DW    = 576
) (
    input  wire              clk,
    input  wire              rst,

    input  wire              ce,
    input  wire              we,
    input  wire [SET_W-1:0]  set_idx,
    input  wire [WAY_W-1:0]  way_idx,
    input  wire [DW-1:0]     wdata,
    output wire [DW-1:0]     rdata,
    output wire              rvalid,

    input  wire [WAYS-1:0]   way_sleep,
    output wire [WAYS-1:0]   way_busy
);

    // -------------------------------------------------------------------------
    // Set banking.  A 32768 x 576 macro cannot be read inside a 333 ps cycle
    // (285 ps access + wire + setup + skew).  Four 8192-deep banks per way
    // access in 190 ps and close with margin, and banking on the low set bits
    // lets four different sets be in flight in the same way.
    // -------------------------------------------------------------------------
    localparam BANKS   = 4;
    localparam BANK_W  = 2;
    localparam ROW_W   = SET_W - BANK_W;

    wire [BANK_W-1:0] bank_sel = set_idx[BANK_W-1:0];
    wire [ROW_W-1:0]  row_addr = set_idx[SET_W-1:BANK_W];

    // the macros carry an extra output register (OUT_REG), so the mux select
    // has to be delayed by two cycles, not one
    reg [BANK_W-1:0] bank_sel_q, bank_sel_q2;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bank_sel_q  <= {BANK_W{1'b0}};
            bank_sel_q2 <= {BANK_W{1'b0}};
        end
        else begin
            bank_sel_q  <= bank_sel;
            bank_sel_q2 <= bank_sel_q;
        end
    end

    wire [DW-1:0]   way_rdata  [0:WAYS-1];
    wire [DW-1:0]   bank_rdata [0:WAYS-1][0:BANKS-1];
    wire [WAYS-1:0] way_rvalid;
    wire [WAYS-1:0] way_ce;
    wire [BANKS-1:0] bank_rvalid [0:WAYS-1];

    genvar w, b;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            assign way_ce[w]   = ce && (way_idx == w[WAY_W-1:0]) && !way_sleep[w];
            assign way_busy[w] = way_ce[w];

            for (b = 0; b < BANKS; b = b + 1) begin : g_bank
                wire bsel = way_ce[w] && (bank_sel == b[BANK_W-1:0]);

                vc3d_sram_sp #(
                    .AW       (ROW_W),
                    .DW       (DW),
                    .USE_MASK (0),
                    .OUT_REG  (1)
                ) u_ram (
                    .clk    (clk),
                    .rst    (rst),
                    .ce     (bsel),
                    .we     (we),
                    .addr   (row_addr),
                    .wdata  (wdata),
                    .wmask  ({DW{1'b1}}),
                    .rdata  (bank_rdata[w][b]),
                    .rvalid (bank_rvalid[w][b])
                );
            end

            assign way_rdata[w]  = bank_rdata[w][bank_sel_q2];
            assign way_rvalid[w] = |bank_rvalid[w];
        end
    endgenerate

    reg [WAY_W-1:0] way_q, way_q2;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            way_q  <= {WAY_W{1'b0}};
            way_q2 <= {WAY_W{1'b0}};
        end
        else begin
            way_q  <= way_idx;
            way_q2 <= way_q;
        end
    end

    reg [DW-1:0] mux;
    integer i;
    always @* begin
        mux = {DW{1'b0}};
        for (i = 0; i < WAYS; i = i + 1)
            if (way_q2 == i[WAY_W-1:0]) mux = way_rdata[i];
    end

    assign rdata  = mux;
    assign rvalid = |way_rvalid;

endmodule
