/*
* CPU-GEN-1 : VCACHE-3D -- stacked-die SRAM subarray with built-in redundancy.
*
* This is the leaf memory element of the hybrid-bonded cache dielet.  It is the
* exact block that an SRAM compiler produces for the stacked die:
*
*     DEPTH x DW bit single-port high-density (HD) bitcell array
*   + SPARE_ROWS redundant word lines
*   + SPARE_COLS redundant bit-line groups (one spare column per column group)
*   + write-assist / read-assist controls (the stacked die runs at a lower
*     VDD than the base die, so assist is mandatory, not optional)
*   + retention / sleep / deep-sleep power controls
*
* Two build flavours:
*   `VC3D_SRAM_COMPILER  : instantiate the foundry macro (see pd/sram/) and
*                          drive its real assist/redundancy pins.
*   default              : behavioural model, bit-accurate for the repair
*                          logic, used by simulation and by the FPGA emulation
*                          build.
*
* Redundancy model
* ----------------
*   Row repair    : SPARE_ROWS content-addressable entries.  A hit steers the
*                   access to the spare word line instead of the main array.
*   Column repair : the DW-bit word is divided into SPARE_COLS column groups.
*                   Each group owns one spare bit line.  Repairing column c in
*                   a group shifts every bit at or above c one position up and
*                   sources the top bit from the spare -- the standard
*                   shift-redundancy scheme, which needs only a 2:1 mux per
*                   bit rather than a full crossbar.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps

module vc3d_stack_subarray #(
    parameter DEPTH        = 1024,
    parameter AW           = 10,
    parameter DW           = 144,
    parameter SPARE_ROWS   = 4,
    parameter SPARE_COLS   = 4,
    parameter COL_GROUP_W  = 36,          // DW / SPARE_COLS
    parameter COL_ID_W     = 6            // ceil(log2(COL_GROUP_W))
) (
    input  wire                       clk,
    input  wire                       rst,

    // functional port ---------------------------------------------------------
    input  wire                       ce,        // chip enable
    input  wire                       we,        // write enable
    input  wire [AW-1:0]              addr,
    input  wire [DW-1:0]              wdata,
    input  wire [DW-1:0]              wbit_mask, // per-bit write mask
    output wire [DW-1:0]              rdata,
    output wire                       rvalid,

    // row-repair interface ----------------------------------------------------
    input  wire [SPARE_ROWS-1:0]      rpr_row_valid,
    input  wire [SPARE_ROWS*AW-1:0]   rpr_row_addr,

    // column-repair interface -------------------------------------------------
    input  wire [SPARE_COLS-1:0]      rpr_col_valid,
    input  wire [SPARE_COLS*COL_ID_W-1:0] rpr_col_id,

    // power / assist ----------------------------------------------------------
    input  wire                       sleep,       // periphery off, array kept
    input  wire                       deep_sleep,  // array collapsed (data lost)
    input  wire                       retention,   // low-VDD data retention
    input  wire [3:0]                 wa_code,     // write-assist strength
    input  wire [3:0]                 ra_code,     // read-assist strength

    // status ------------------------------------------------------------------
    output wire                       spare_row_hit,
    output wire [SPARE_ROWS-1:0]      spare_row_hit_vec,
    output wire                       access_blocked
);

    // -------------------------------------------------------------------------
    // Row redundancy: compare the incoming address against the repaired rows.
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

    // Priority encode the spare row index (entries are unique by construction,
    // the repair controller refuses to program a duplicate).
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
    // Column redundancy: build the write remap (logical -> physical) and the
    // read remap (physical -> logical) for every column group.
    // -------------------------------------------------------------------------
    // Physical word is DW bits wide plus SPARE_COLS spare bit lines.
    localparam PHYS_W = DW + SPARE_COLS;

    wire [PHYS_W-1:0] phys_wdata;
    wire [PHYS_W-1:0] phys_wmask;
    wire [PHYS_W-1:0] phys_rdata;
    wire [DW-1:0]     logical_rdata;

    genvar gg, gb;
    generate
        for (gg = 0; gg < SPARE_COLS; gg = gg + 1) begin : g_col_group
            wire                 grp_rpr   = rpr_col_valid[gg];
            wire [COL_ID_W-1:0]  grp_colid = rpr_col_id[gg*COL_ID_W +: COL_ID_W];

            // physical group is COL_GROUP_W + 1 bits (spare bit is the MSB)
            for (gb = 0; gb < COL_GROUP_W + 1; gb = gb + 1) begin : g_col_bit
                localparam integer PBIT = gg*(COL_GROUP_W+1) + gb;

                if (gb == 0) begin : g_first
                    assign phys_wdata[PBIT] = wdata[gg*COL_GROUP_W + 0];
                    assign phys_wmask[PBIT] = wbit_mask[gg*COL_GROUP_W + 0];
                end
                else if (gb == COL_GROUP_W) begin : g_spare
                    // spare bit line only carries the top logical bit when a
                    // repair is active in this group
                    assign phys_wdata[PBIT] = wdata[gg*COL_GROUP_W + COL_GROUP_W-1];
                    assign phys_wmask[PBIT] = grp_rpr &
                                              wbit_mask[gg*COL_GROUP_W + COL_GROUP_W-1];
                end
                else begin : g_shift
                    // bits at or above the repaired column shift up by one
                    wire shifted = grp_rpr && (gb > grp_colid);
                    assign phys_wdata[PBIT] = shifted ? wdata[gg*COL_GROUP_W + gb - 1]
                                                      : wdata[gg*COL_GROUP_W + gb];
                    assign phys_wmask[PBIT] = shifted ? wbit_mask[gg*COL_GROUP_W + gb - 1]
                                                      : wbit_mask[gg*COL_GROUP_W + gb];
                end
            end

            // read-side inverse remap
            for (gb = 0; gb < COL_GROUP_W; gb = gb + 1) begin : g_col_rd
                wire rd_shift = grp_rpr && (gb >= grp_colid);
                assign logical_rdata[gg*COL_GROUP_W + gb] =
                    rd_shift ? phys_rdata[gg*(COL_GROUP_W+1) + gb + 1]
                             : phys_rdata[gg*(COL_GROUP_W+1) + gb];
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------------
    wire array_off  = deep_sleep;
    wire array_hold = sleep | retention;
    wire do_access  = ce & ~array_off & ~array_hold;
    wire do_write   = do_access & we;
    wire do_read    = do_access & ~we;

    assign access_blocked = ce & (array_off | array_hold);

`ifdef VC3D_SRAM_COMPILER
    // -------------------------------------------------------------------------
    // Foundry macro flavour.  The wrapper name and pin list are pinned by
    // pd/sram/sram_compiler_selection.md; the surrounding repair/assist logic
    // is identical to the behavioural flavour.
    // -------------------------------------------------------------------------
    wire [PHYS_W-1:0] phys_rdata_macro;
    reg               rvalid_macro_q;

    VC3D_HD_SPSRAM_1024X148 u_macro (
        .CLK    (clk),
        .CEN    (~do_access),
        .WEN    (~do_write),
        .A      (spare_row_hit ? {AW{1'b1}} : addr),
        .D      (phys_wdata),
        .M      (phys_wmask),
        .Q      (phys_rdata_macro),
        .WA     (wa_code),
        .RA     (ra_code),
        .SLP    (sleep),
        .DSLP   (deep_sleep),
        .RET    (retention)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) rvalid_macro_q <= 1'b0;
        else     rvalid_macro_q <= do_read;
    end

    assign phys_rdata = phys_rdata_macro;
    assign rvalid     = rvalid_macro_q;
`else
    // -------------------------------------------------------------------------
    // Behavioural flavour.
    // -------------------------------------------------------------------------
    reg [PHYS_W-1:0] mem      [0:DEPTH-1];
    reg [PHYS_W-1:0] spare_mem[0:SPARE_ROWS-1];
    reg [PHYS_W-1:0] rd_q;
    reg              rvalid_q;

    integer i, b;

    always @(posedge clk) begin
        if (do_write) begin
            if (spare_row_hit) begin
                for (i = 0; i < SPARE_ROWS; i = i + 1) begin
                    if (spare_sel_oh[i]) begin
                        for (b = 0; b < PHYS_W; b = b + 1) begin
                            if (phys_wmask[b]) spare_mem[i][b] <= phys_wdata[b];
                        end
                    end
                end
            end
            else begin
                for (b = 0; b < PHYS_W; b = b + 1) begin
                    if (phys_wmask[b]) mem[addr][b] <= phys_wdata[b];
                end
            end
        end
    end

    always @(posedge clk) begin
        if (do_read) begin
            if (spare_row_hit) begin
                rd_q <= {PHYS_W{1'b0}};
                for (i = 0; i < SPARE_ROWS; i = i + 1) begin
                    if (spare_sel_oh[i]) rd_q <= spare_mem[i];
                end
            end
            else begin
                rd_q <= mem[addr];
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) rvalid_q <= 1'b0;
        else     rvalid_q <= do_read;
    end

    assign phys_rdata = rd_q;
    assign rvalid     = rvalid_q;
`endif

    assign rdata = logical_rdata;

endmodule
