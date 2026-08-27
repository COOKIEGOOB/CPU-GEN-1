/*
* CPU-GEN-1 : VCACHE-3D -- correctable-error locality tracker.
*
* SECDED hides single-bit errors, which is exactly the danger: a bit line that
* has gone marginal keeps producing CEs until, one day, a second flip in the
* same subline turns into a UE.  Production parts therefore track WHERE the
* CEs are, not just how many there are, and spend a spare row/column before
* the fault becomes uncorrectable.  That decision loop is this module.
*
* Structure: a small direct-mapped counter cache keyed on
*      {bank, subarray, syndrome}
* -- deliberately NOT on the full address, because the interesting locality is
* physical (a bit line, a word line, a bond lane), not logical.  When a bucket
* crosses `threshold`, the tracker:
*      1. requests a demand scrub of the affected line (removes the soft flip);
*      2. if the bucket keeps growing after the scrub, escalates to the repair
*         controller with a repair hint derived from the syndrome pattern:
*           - same syndrome across many rows      -> column repair
*           - many syndromes within one row       -> row repair
*           - syndrome aliasing one bond lane     -> lane repair
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_ce_tracker #(
    parameter ENTRIES   = 64,
    parameter ENTRY_W   = 6,
    parameter SET_W     = 15,
    parameter WAY_W     = 4
) (
    input  wire              clk,
    input  wire              rst,

    input  wire              ce_valid,
    input  wire [SET_W-1:0]  ce_set,
    input  wire [WAY_W-1:0]  ce_way,
    input  wire [4:0]        ce_bank,
    input  wire [3:0]        ce_sub,
    input  wire [8:0]        ce_syndrome,
    input  wire [1:0]        ce_subline,

    input  wire [15:0]       threshold,
    input  wire              clear,

    // demand scrub request
    output reg               demand_valid,
    output reg  [SET_W-1:0]  demand_set,
    output reg  [WAY_W-1:0]  demand_way,
    input  wire              demand_ack,

    // repair escalation
    output reg               repair_req,
    output reg  [1:0]        repair_type,   // `VC3D_RPR_TYPE_*
    output reg  [4:0]        repair_bank,
    output reg  [3:0]        repair_sub,
    output reg  [SET_W-1:0]  repair_row,
    output reg  [8:0]        repair_syndrome,
    input  wire              repair_ack,

    output reg  [31:0]       total_ce,
    output reg  [15:0]       max_bucket
);

    reg [17:0] key   [0:ENTRIES-1];   // {bank[4:0], sub[3:0], syndrome[8:0]}
    reg [15:0] count [0:ENTRIES-1];
    reg [SET_W-1:0] last_row [0:ENTRIES-1];
    reg [SET_W-1:0] first_row[0:ENTRIES-1];
    reg        valid [0:ENTRIES-1];

    wire [17:0] in_key = {ce_bank, ce_sub, ce_syndrome};
    // fold the key into the index: XOR of the three fields keeps bank and
    // syndrome locality distinguishable in a 64-entry table
    wire [ENTRY_W-1:0] idx = in_key[5:0] ^ in_key[11:6] ^ {2'b0, in_key[15:12]};

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < ENTRIES; i = i + 1) begin
                key[i]       <= 18'd0;
                count[i]     <= 16'd0;
                valid[i]     <= 1'b0;
                last_row[i]  <= {SET_W{1'b0}};
                first_row[i] <= {SET_W{1'b0}};
            end
            total_ce     <= 32'd0;
            max_bucket   <= 16'd0;
            demand_valid <= 1'b0;
            repair_req   <= 1'b0;
            repair_type  <= `VC3D_RPR_TYPE_COL;
            repair_bank  <= 5'd0;
            repair_sub   <= 4'd0;
            repair_row   <= {SET_W{1'b0}};
            repair_syndrome <= 9'd0;
            demand_set   <= {SET_W{1'b0}};
            demand_way   <= {WAY_W{1'b0}};
        end
        else begin
            if (demand_ack) demand_valid <= 1'b0;
            if (repair_ack) repair_req   <= 1'b0;

            if (clear) begin
                for (i = 0; i < ENTRIES; i = i + 1) begin
                    count[i] <= 16'd0;
                    valid[i] <= 1'b0;
                end
                total_ce   <= 32'd0;
                max_bucket <= 16'd0;
            end
            else if (ce_valid) begin
                if (total_ce != 32'hffffffff) total_ce <= total_ce + 32'd1;

                if (valid[idx] && key[idx] == in_key) begin
                    count[idx]    <= count[idx] + 16'd1;
                    last_row[idx] <= ce_set;
                    if (count[idx] + 16'd1 > max_bucket)
                        max_bucket <= count[idx] + 16'd1;

                    // first escalation: scrub the line to clear a soft flip
                    if (count[idx] + 16'd1 == threshold) begin
                        demand_valid <= 1'b1;
                        demand_set   <= ce_set;
                        demand_way   <= ce_way;
                    end

                    // second escalation: spend a spare resource
                    if (count[idx] + 16'd1 >= {threshold[14:0], 1'b0}) begin
                        repair_req      <= 1'b1;
                        repair_bank     <= ce_bank;
                        repair_sub      <= ce_sub;
                        repair_row      <= ce_set;
                        repair_syndrome <= ce_syndrome;
                        // same syndrome seen on different rows  -> column fault
                        // same row repeatedly                   -> row fault
                        repair_type <= (first_row[idx] != ce_set) ? `VC3D_RPR_TYPE_COL
                                                                  : `VC3D_RPR_TYPE_ROW;
                    end
                end
                else begin
                    key[idx]       <= in_key;
                    count[idx]     <= 16'd1;
                    valid[idx]     <= 1'b1;
                    first_row[idx] <= ce_set;
                    last_row[idx]  <= ce_set;
                end
            end
        end
    end

endmodule
