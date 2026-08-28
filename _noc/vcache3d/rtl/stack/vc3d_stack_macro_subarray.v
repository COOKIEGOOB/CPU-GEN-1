/*
* CPU-GEN-1 : VCACHE-3D -- macro-sliced stacked subarray (dielet frequency path).
*
* Drop-in replacement for vc3d_stack_subarray with the same repair, power and
* assist interface, but the physical storage is banked into `VC3D_STACK_MACRO_
* SLICES` direct SRAM macro slices with divided local bitlines.
*
*    slice 0 : rows [  0..255]   (256 x 148, ~260 ps access)
*    slice 1 : rows [256..511]
*    slice 2 : rows [512..767]
*    slice 3 : rows [768..1023]
*
* The slice decode (addr[9:8]) is registered because the macro-cell output is
* registered (OUT_REG), exactly like the production compiler cell.  Write data
* is steered to the selected slice; read data is muxed from the selected slice
* through the same column-redundancy inverse remap as the legacy subarray.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_stack_macro_subarray #(
    parameter DEPTH        = 1024,
    parameter AW           = 10,
    parameter DW           = 144,
    parameter SPARE_ROWS   = 4,
    parameter SPARE_COLS   = 4,
    parameter COL_GROUP_W  = 36,          // DW / SPARE_COLS
    parameter COL_ID_W     = 6,           // ceil(log2(COL_GROUP_W))
    parameter MACRO_SLICES = `VC3D_STACK_MACRO_SLICES,
    parameter SLICE_DEPTH  = `VC3D_STACK_MACRO_DEPTH,
    parameter SLICE_AW     = `VC3D_STACK_MACRO_AW,
    parameter SLICE_ACCESS = `VC3D_STACK_MACRO_ACCESS_PS
) (
    input  wire                       clk,
    input  wire                       rst,

    // functional port ---------------------------------------------------------
    input  wire                       ce,
    input  wire                       we,
    input  wire [AW-1:0]              addr,
    input  wire [DW-1:0]              wdata,
    input  wire [DW-1:0]              wbit_mask,
    output wire [DW-1:0]              rdata,
    output wire                       rvalid,

    // row-repair interface ----------------------------------------------------
    input  wire [SPARE_ROWS-1:0]      rpr_row_valid,
    input  wire [SPARE_ROWS*AW-1:0]   rpr_row_addr,

    // column-repair interface -------------------------------------------------
    input  wire [SPARE_COLS-1:0]      rpr_col_valid,
    input  wire [SPARE_COLS*COL_ID_W-1:0] rpr_col_id,

    // power / assist ----------------------------------------------------------
    input  wire                       sleep,
    input  wire                       deep_sleep,
    input  wire                       retention,
    input  wire [3:0]                 wa_code,
    input  wire [3:0]                 ra_code,

    // status ------------------------------------------------------------------
    output wire                       spare_row_hit,
    output wire [SPARE_ROWS-1:0]      spare_row_hit_vec,
    output wire                       access_blocked
);

    localparam PHYS_W = DW + SPARE_COLS;
    localparam SLICE_SEL_W = $clog2(MACRO_SLICES);

    // -------------------------------------------------------------------------
    // Row redundancy CAM (unchanged from the legacy subarray).
    // -------------------------------------------------------------------------
    wire [SPARE_ROWS-1:0] row_match;
    genvar gr;
    generate
        for (gr = 0; gr < SPARE_ROWS; gr = gr + 1) begin : g_row_cam
            assign row_match[gr] = rpr_row_valid[gr] &&
                                   (addr == rpr_row_addr[gr*AW +: AW]);
        end
    endgenerate

    assign spare_row_hit_vec = row_match;
    assign spare_row_hit     = |row_match;

    reg [SPARE_ROWS-1:0] spare_sel_oh;
    integer sr;
    always @* begin
        spare_sel_oh = {SPARE_ROWS{1'b0}};
        for (sr = SPARE_ROWS-1; sr >= 0; sr = sr - 1) begin
            if (row_match[sr]) begin
                spare_sel_oh = {SPARE_ROWS{1'b0}};
                spare_sel_oh[sr] = 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Column redundancy: identical to the legacy subarray.
    // -------------------------------------------------------------------------
    wire [PHYS_W-1:0] phys_wdata;
    wire [PHYS_W-1:0] phys_wmask;
    wire [PHYS_W-1:0] phys_rdata;
    wire [DW-1:0]     logical_rdata;

    genvar gg, gb;
    generate
        for (gg = 0; gg < SPARE_COLS; gg = gg + 1) begin : g_col_group
            wire                 grp_rpr   = rpr_col_valid[gg];
            wire [COL_ID_W-1:0]  grp_colid = rpr_col_id[gg*COL_ID_W +: COL_ID_W];

            for (gb = 0; gb < COL_GROUP_W + 1; gb = gb + 1) begin : g_col_bit
                localparam integer PBIT = gg*(COL_GROUP_W+1) + gb;

                if (gb == 0) begin : g_first
                    assign phys_wdata[PBIT] = wdata[gg*COL_GROUP_W + 0];
                    assign phys_wmask[PBIT] = wbit_mask[gg*COL_GROUP_W + 0];
                end
                else if (gb == COL_GROUP_W) begin : g_spare
                    assign phys_wdata[PBIT] = wdata[gg*COL_GROUP_W + COL_GROUP_W-1];
                    assign phys_wmask[PBIT] = grp_rpr &
                                              wbit_mask[gg*COL_GROUP_W + COL_GROUP_W-1];
                end
                else begin : g_shift
                    wire shifted = grp_rpr && (gb > grp_colid);
                    assign phys_wdata[PBIT] = shifted ? wdata[gg*COL_GROUP_W + gb - 1]
                                                      : wdata[gg*COL_GROUP_W + gb];
                    assign phys_wmask[PBIT] = shifted ? wbit_mask[gg*COL_GROUP_W + gb - 1]
                                                      : wbit_mask[gg*COL_GROUP_W + gb];
                end
            end

            for (gb = 0; gb < COL_GROUP_W; gb = gb + 1) begin : g_col_rd
                wire rd_shift = grp_rpr && (gb >= grp_colid);
                assign logical_rdata[gg*COL_GROUP_W + gb] =
                    rd_shift ? phys_rdata[gg*(COL_GROUP_W+1) + gb + 1]
                             : phys_rdata[gg*(COL_GROUP_W+1) + gb];
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Divided local bitline macro-slice decode.
    // -------------------------------------------------------------------------
    wire array_off  = deep_sleep;
    wire array_hold = sleep | retention;
    wire do_access  = ce & ~array_off & ~array_hold;
    wire do_write   = do_access & we;
    wire do_read    = do_access & ~we;

    assign access_blocked = ce & (array_off | array_hold);

    localparam SLICE_ADDR_W = AW - SLICE_SEL_W;   // 10 - 2 = 8

    wire [SLICE_SEL_W-1:0] slice_sel = addr[AW-1 -: SLICE_SEL_W];
    wire [SLICE_ADDR_W-1:0] slice_row = addr[SLICE_ADDR_W-1:0];

    reg [SLICE_SEL_W-1:0] slice_sel_q;
    always @(posedge clk or posedge rst) begin
        if (rst) slice_sel_q <= {SLICE_SEL_W{1'b0}};
        else     slice_sel_q <= slice_sel;
    end

    // -------------------------------------------------------------------------
    // Direct SRAM macro slices with divided local bitlines.
    // -------------------------------------------------------------------------
    wire [PHYS_W-1:0] cell_rdata [0:MACRO_SLICES-1];
    wire              cell_rvalid [0:MACRO_SLICES-1];

    genvar gm;
    generate
        for (gm = 0; gm < MACRO_SLICES; gm = gm + 1) begin : g_macro
            wire              cell_ce;
            wire              cell_we;
            wire [SLICE_AW-1:0] cell_addr;
            wire              bl_sel;

            assign cell_ce   = do_access & (slice_sel == gm[SLICE_SEL_W-1:0]);
            assign cell_we   = do_write & (slice_sel == gm[SLICE_SEL_W-1:0]);
            assign cell_addr = slice_row;
            assign bl_sel    = 1'b1;   // divided local bitline, active for all macros

            vc3d_stack_macro_cell #(
                .SLICE_ID  (gm),
                .DEPTH     (SLICE_DEPTH),
                .AW        (SLICE_AW),
                .DW        (PHYS_W),
                .ACCESS_PS (SLICE_ACCESS),
                .OUT_REG   (1)
            ) u_cell (
                .clk          (clk),
                .rst          (rst),
                .ce           (cell_ce),
                .we           (cell_we),
                .addr         (cell_addr),
                .wdata        (phys_wdata),
                .wbit_mask    (phys_wmask),
                .rdata        (cell_rdata[gm]),
                .rvalid       (cell_rvalid[gm]),
                .local_bl_sel (bl_sel),
                .cell_active  (),
                .access_count (),
                .stall_count  ()
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Slice read mux (select registered to match the OUT_REG cell), then merge
    // any spare-row hit into the response.
    // -------------------------------------------------------------------------
    reg [PHYS_W-1:0] cell_mux;
    always @* begin
        cell_mux = {PHYS_W{1'b0}};
        case (slice_sel_q)
            2'd0: cell_mux = cell_rdata[0];
            2'd1: cell_mux = cell_rdata[1];
            2'd2: cell_mux = cell_rdata[2];
            2'd3: cell_mux = cell_rdata[3];
            default: cell_mux = {PHYS_W{1'b0}};
        endcase
    end

    reg [PHYS_W-1:0] spare_mem[0:SPARE_ROWS-1];
    reg [PHYS_W-1:0] spare_rd;
    reg [SLICE_SEL_W-1:0] spare_slice_q;
    reg              spare_hit_q;
    reg              spare_hit_sel_q;

    reg [PHYS_W-1:0] rd_q;
    reg              rd_valid_q;
    reg [SLICE_SEL_W-1:0] rd_slice_q;

    integer si, bi;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_q          <= {PHYS_W{1'b0}};
            rd_valid_q    <= 1'b0;
            rd_slice_q    <= {SLICE_SEL_W{1'b0}};
            spare_hit_q   <= 1'b0;
            spare_hit_sel_q <= 1'b0;
            spare_slice_q <= {SLICE_SEL_W{1'b0}};
            spare_rd      <= {PHYS_W{1'b0}};
        end
        else begin
            rd_q        <= spare_hit ? spare_rd : cell_mux;
            rd_valid_q  <= do_read;
            rd_slice_q  <= slice_sel;
            spare_hit_q <= spare_hit;
            for (si = 0; si < SPARE_ROWS; si = si + 1) begin
                if (spare_sel_oh[si]) begin
                    spare_rd       <= spare_mem[si];
                    spare_slice_q  <= slice_sel;
                end
            end
            spare_hit_sel_q <= spare_hit;

            if (do_write) begin
                if (spare_row_hit) begin
                    for (si = 0; si < SPARE_ROWS; si = si + 1) begin
                        if (spare_sel_oh[si]) begin
                            for (bi = 0; bi < PHYS_W; bi = bi + 1)
                                if (phys_wmask[bi]) spare_mem[si][bi] <= phys_wdata[bi];
                        end
                    end
                end
            end
        end
    end

    // The selected macro cell is the normal storage path; spare rows are
    // additionally mirrored into the appropriate slice on write and read back
    // on a spare-row hit.  The physical rdata is the normal mux (spare read
    // takes priority in the register above).
    assign phys_rdata = rd_q;
    assign rdata      = logical_rdata;
    assign rvalid     = rd_valid_q;

endmodule
