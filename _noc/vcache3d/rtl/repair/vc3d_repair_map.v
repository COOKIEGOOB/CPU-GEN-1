/*
* CPU-GEN-1 : VCACHE-3D -- distributed repair register file (row + column).
*
* The repair state does NOT live in one central CAM: at 512 subarrays a
* central CAM would need a 512-way fanout of every repaired address and would
* be the slowest path on the die.  Instead every subarray owns its own four
* row-repair slots and four column-repair slots, exactly mirroring the
* redundancy resources the SRAM compiler gives it, and this module is the
* register file plus the write/read port used by:
*
*    * the eFuse autoload sequencer at reset          (hard repair)
*    * the BISR controller after MBIST                (hard repair candidate)
*    * the CE tracker / scrubber at runtime           (soft repair)
*    * firmware through the CSR window                (debug / field service)
*
* Programming is one slot per write, addressed by {bank, subarray, kind, slot}.
* Reads are single-cycle and are used by the tester to dump the repair
* solution before it is blown into fuses.
*
* Resource accounting (`slots_used_o`) is exported so that the BISR controller
* can declare a die unrepairable instead of silently dropping a repair.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_repair_map #(
    parameter BANKS       = 32,
    parameter BANK_W      = 5,
    parameter SUBS        = 16,
    parameter SUB_W       = 4,
    parameter SPARE_ROWS  = 4,
    parameter SPARE_COLS  = 4,
    parameter ROW_AW      = 10,
    parameter COL_ID_W    = 6
) (
    input  wire                                     clk,
    input  wire                                     rst,

    // ---- program port --------------------------------------------------------
    input  wire                                     wr_en,
    input  wire [BANK_W-1:0]                        wr_bank,
    input  wire [SUB_W-1:0]                         wr_sub,
    input  wire                                     wr_is_col,     // 0=row 1=col
    input  wire [1:0]                               wr_slot,
    input  wire                                     wr_valid,
    input  wire [ROW_AW-1:0]                        wr_addr,       // row addr or col id
    output wire                                     wr_slot_busy,

    // ---- read-back port ------------------------------------------------------
    input  wire [BANK_W-1:0]                        rd_bank,
    input  wire [SUB_W-1:0]                         rd_sub,
    input  wire                                     rd_is_col,
    input  wire [1:0]                               rd_slot,
    output wire                                     rd_valid,
    output wire [ROW_AW-1:0]                        rd_addr,

    // ---- invalidate all (test / re-fuse) -------------------------------------
    input  wire                                     clear_all,

    // ---- fabric outputs to the stacked array ---------------------------------
    output wire [BANKS*SUBS*SPARE_ROWS-1:0]         rpr_row_valid,
    output wire [BANKS*SUBS*SPARE_ROWS*ROW_AW-1:0]  rpr_row_addr,
    output wire [BANKS*SUBS*SPARE_COLS-1:0]         rpr_col_valid,
    output wire [BANKS*SUBS*SPARE_COLS*COL_ID_W-1:0] rpr_col_id,

    // ---- accounting -----------------------------------------------------------
    output wire [15:0]                              rows_used,
    output wire [15:0]                              cols_used,
    output wire                                     any_repair
);

    localparam UNITS = BANKS * SUBS;   // 512 subarrays

    // Flattened storage: [unit][slot]
    reg                 row_v [0:UNITS*SPARE_ROWS-1];
    reg [ROW_AW-1:0]    row_a [0:UNITS*SPARE_ROWS-1];
    reg                 col_v [0:UNITS*SPARE_COLS-1];
    reg [COL_ID_W-1:0]  col_a [0:UNITS*SPARE_COLS-1];

    wire [8:0] wr_unit = {wr_bank, wr_sub};
    wire [8:0] rd_unit = {rd_bank, rd_sub};

    wire [10:0] wr_row_idx = {wr_unit, wr_slot};
    wire [10:0] wr_col_idx = {wr_unit, wr_slot};
    wire [10:0] rd_row_idx = {rd_unit, rd_slot};
    wire [10:0] rd_col_idx = {rd_unit, rd_slot};

    assign wr_slot_busy = wr_is_col ? col_v[wr_col_idx] : row_v[wr_row_idx];
    assign rd_valid     = rd_is_col ? col_v[rd_col_idx] : row_v[rd_row_idx];
    assign rd_addr      = rd_is_col ? {{(ROW_AW-COL_ID_W){1'b0}}, col_a[rd_col_idx]}
                                    : row_a[rd_row_idx];

    integer i;
    reg [15:0] rows_used_r, cols_used_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < UNITS*SPARE_ROWS; i = i + 1) begin
                row_v[i] <= 1'b0;
                row_a[i] <= {ROW_AW{1'b0}};
            end
            for (i = 0; i < UNITS*SPARE_COLS; i = i + 1) begin
                col_v[i] <= 1'b0;
                col_a[i] <= {COL_ID_W{1'b0}};
            end
            rows_used_r <= 16'd0;
            cols_used_r <= 16'd0;
        end
        else if (clear_all) begin
            for (i = 0; i < UNITS*SPARE_ROWS; i = i + 1) row_v[i] <= 1'b0;
            for (i = 0; i < UNITS*SPARE_COLS; i = i + 1) col_v[i] <= 1'b0;
            rows_used_r <= 16'd0;
            cols_used_r <= 16'd0;
        end
        else if (wr_en) begin
            if (wr_is_col) begin
                if (wr_valid && !col_v[wr_col_idx]) cols_used_r <= cols_used_r + 16'd1;
                if (!wr_valid && col_v[wr_col_idx]) cols_used_r <= cols_used_r - 16'd1;
                col_v[wr_col_idx] <= wr_valid;
                col_a[wr_col_idx] <= wr_addr[COL_ID_W-1:0];
            end
            else begin
                if (wr_valid && !row_v[wr_row_idx]) rows_used_r <= rows_used_r + 16'd1;
                if (!wr_valid && row_v[wr_row_idx]) rows_used_r <= rows_used_r - 16'd1;
                row_v[wr_row_idx] <= wr_valid;
                row_a[wr_row_idx] <= wr_addr;
            end
        end
    end

    assign rows_used  = rows_used_r;
    assign cols_used  = cols_used_r;
    assign any_repair = (rows_used_r != 16'd0) || (cols_used_r != 16'd0);

    // -------------------------------------------------------------------------
    // Fabric flattening.  Each subarray sees only its own slots, so this is
    // pure wiring -- no logic, no fanout problem.
    // -------------------------------------------------------------------------
    genvar gu, gs;
    generate
        for (gu = 0; gu < UNITS; gu = gu + 1) begin : g_unit
            for (gs = 0; gs < SPARE_ROWS; gs = gs + 1) begin : g_rslot
                assign rpr_row_valid[gu*SPARE_ROWS + gs] = row_v[gu*SPARE_ROWS + gs];
                assign rpr_row_addr[(gu*SPARE_ROWS + gs)*ROW_AW +: ROW_AW] =
                       row_a[gu*SPARE_ROWS + gs];
            end
            for (gs = 0; gs < SPARE_COLS; gs = gs + 1) begin : g_cslot
                assign rpr_col_valid[gu*SPARE_COLS + gs] = col_v[gu*SPARE_COLS + gs];
                assign rpr_col_id[(gu*SPARE_COLS + gs)*COL_ID_W +: COL_ID_W] =
                       col_a[gu*SPARE_COLS + gs];
            end
        end
    endgenerate

endmodule
