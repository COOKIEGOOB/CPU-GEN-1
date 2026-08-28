/*
* CPU-GEN-1 : VCACHE-3D -- macro-sliced bank wrapper.
*
* A bank is the unit of independent access, power gating and thermal reporting
* on the dielet.  This wrapper is structurally identical to the generated bank
* module in vc3d_stack_bank_array.v, but the per-subarray storage is built
* from divided-local-bitline macro slices (vc3d_stack_macro_subarray) instead
* of the legacy flat 1024 x 148 macro.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_stack_macro_bank #(
    parameter BANK_ID    = 0,
    parameter SUBS       = 16,
    parameter SUB_W      = 4,
    parameter AW         = 10,
    parameter DW         = 144,
    parameter SPARE_ROWS = 4,
    parameter SPARE_COLS = 4,
    parameter COL_ID_W   = 6
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     ce,
    input  wire                     we,
    input  wire [SUB_W-1:0]         sub_sel,
    input  wire [AW-1:0]            row_addr,
    input  wire [DW-1:0]            wdata,
    input  wire [DW-1:0]            wbit_mask,
    output wire [DW-1:0]            rdata,
    output wire                     rvalid,
    input  wire [SUBS*SPARE_ROWS-1:0]       rpr_row_valid,
    input  wire [SUBS*SPARE_ROWS*AW-1:0]    rpr_row_addr,
    input  wire [SUBS*SPARE_COLS-1:0]       rpr_col_valid,
    input  wire [SUBS*SPARE_COLS*COL_ID_W-1:0] rpr_col_id,
    input  wire                     sleep,
    input  wire                     deep_sleep,
    input  wire                     retention,
    input  wire [3:0]               wa_code,
    input  wire [3:0]               ra_code,
    output wire [SUBS-1:0]          sub_spare_row_hit,
    output wire [SUBS-1:0]          sub_access_blocked,
    output wire                     bank_busy
);

    wire [SUBS-1:0] sub_ce;
    wire [SUBS-1:0] sub_rvalid;
    wire [DW-1:0]   sub_rdata [0:SUBS-1];

    generate
        for (genvar s = 0; s < SUBS; s = s + 1) begin : g_sub
            assign sub_ce[s] = ce & (sub_sel == s[SUB_W-1:0]);

            vc3d_stack_macro_subarray #(
                .DEPTH       (1 << AW),
                .AW          (AW),
                .DW          (DW),
                .SPARE_ROWS  (SPARE_ROWS),
                .SPARE_COLS  (SPARE_COLS),
                .COL_GROUP_W (DW / SPARE_COLS),
                .COL_ID_W    (COL_ID_W),
                .MACRO_SLICES (`VC3D_STACK_MACRO_SLICES),
                .SLICE_DEPTH (`VC3D_STACK_MACRO_DEPTH),
                .SLICE_AW    (`VC3D_STACK_MACRO_AW),
                .SLICE_ACCESS(`VC3D_STACK_MACRO_ACCESS_PS)
            ) u_sub (
                .clk               (clk),
                .rst               (rst),
                .ce                (sub_ce[s]),
                .we                (we),
                .addr              (row_addr),
                .wdata             (wdata),
                .wbit_mask         (wbit_mask),
                .rdata             (sub_rdata[s]),
                .rvalid            (sub_rvalid[s]),
                .rpr_row_valid     (rpr_row_valid[s*SPARE_ROWS +: SPARE_ROWS]),
                .rpr_row_addr      (rpr_row_addr[s*SPARE_ROWS*AW +: SPARE_ROWS*AW]),
                .rpr_col_valid     (rpr_col_valid[s*SPARE_COLS +: SPARE_COLS]),
                .rpr_col_id        (rpr_col_id[s*SPARE_COLS*COL_ID_W +: SPARE_COLS*COL_ID_W]),
                .sleep             (sleep),
                .deep_sleep        (deep_sleep),
                .retention         (retention),
                .wa_code           (wa_code),
                .ra_code           (ra_code),
                .spare_row_hit     (sub_spare_row_hit[s]),
                .spare_row_hit_vec (),
                .access_blocked    (sub_access_blocked[s])
            );
        end
    endgenerate

    reg [SUB_W-1:0] sub_sel_q, sub_sel_q2;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sub_sel_q  <= {SUB_W{1'b0}};
            sub_sel_q2 <= {SUB_W{1'b0}};
        end
        else begin
            sub_sel_q  <= sub_sel;
            sub_sel_q2 <= sub_sel_q;
        end
    end

    reg [DW-1:0] rdata_mux;
    always @* begin
        rdata_mux = {DW{1'b0}};
        case (sub_sel_q2)
            4'd0:  rdata_mux = sub_rdata[0];
            4'd1:  rdata_mux = sub_rdata[1];
            4'd2:  rdata_mux = sub_rdata[2];
            4'd3:  rdata_mux = sub_rdata[3];
            4'd4:  rdata_mux = sub_rdata[4];
            4'd5:  rdata_mux = sub_rdata[5];
            4'd6:  rdata_mux = sub_rdata[6];
            4'd7:  rdata_mux = sub_rdata[7];
            4'd8:  rdata_mux = sub_rdata[8];
            4'd9:  rdata_mux = sub_rdata[9];
            4'd10: rdata_mux = sub_rdata[10];
            4'd11: rdata_mux = sub_rdata[11];
            4'd12: rdata_mux = sub_rdata[12];
            4'd13: rdata_mux = sub_rdata[13];
            4'd14: rdata_mux = sub_rdata[14];
            4'd15: rdata_mux = sub_rdata[15];
            default: rdata_mux = {DW{1'b0}};
        endcase
    end

    assign rdata  = rdata_mux;
    assign rvalid = |sub_rvalid;
    assign bank_busy = ce;

endmodule
