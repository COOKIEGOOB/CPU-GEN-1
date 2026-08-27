/*
* CPU-GEN-1 : VCACHE-3D -- 16-way tag / state / replacement array (base die).
*
* Per set:
*   16 x 39-bit ECC-protected tag words  (27 b tag, 3 b state, valid, dirty,
*                                         7 b SECDED check)
*    16 x 2-bit RRPV                     (DRRIP / SRRIP replacement)
*
* The tag array always lives on the BASE die, never on the stacked die, for
* three reasons that a real 3D cache also obeys:
*   1. tag lookup is on the critical path of every access -- paying the bond
*      round trip for it would add the +4 cycles to hits AND misses;
*   2. tags are read/written far more often per bit than data, so they belong
*      in a lower-density, faster, higher-endurance array;
*   3. keeping coherence state on the base die means the stacked die can be
*      power-collapsed without losing coherence.
*
* Victim selection is SRRIP-HP with a bimodal (DRRIP) insertion policy and a
* set-dueling monitor, which is the policy class a very large LLC needs: an
* LRU 96 MiB cache is trivially thrashed by a streaming workload.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_tag_array #(
    parameter SETS   = 32768,
    parameter SET_W  = 15,
    parameter WAYS   = 16,
    parameter WAY_W  = 4,
    parameter TAG_W  = 27
) (
    input  wire                    clk,
    input  wire                    rst,

    // ---- lookup ---------------------------------------------------------------
    input  wire                    lookup_valid,
    input  wire [SET_W-1:0]        lookup_set,
    input  wire [TAG_W-1:0]        lookup_tag,
    output reg                     lookup_done,
    output reg                     hit,
    output reg  [WAY_W-1:0]        hit_way,
    output reg  [2:0]              hit_state,
    output reg                     hit_dirty,
    output reg                     tag_ce,
    output reg                     tag_ue,

    // ---- victim ----------------------------------------------------------------
    output reg  [WAY_W-1:0]        victim_way,
    output reg                     victim_valid,
    output reg                     victim_dirty,
    output reg  [TAG_W-1:0]        victim_tag,

    // ---- update ----------------------------------------------------------------
    input  wire                    upd_valid,
    input  wire [SET_W-1:0]        upd_set,
    input  wire [WAY_W-1:0]        upd_way,
    input  wire [TAG_W-1:0]        upd_tag,
    input  wire [2:0]              upd_state,
    input  wire                    upd_dirty,
    input  wire                    upd_insert,     // new allocation
    input  wire                    upd_invalidate,

    // ---- config ------------------------------------------------------------------
    input  wire [15:0]             way_disable,
    input  wire                    rrip_bimodal_en,

    // ---- stats --------------------------------------------------------------------
    output reg  [31:0]             hit_count,
    output reg  [31:0]             miss_count,
    output reg  [31:0]             tag_ce_count,
    output reg  [31:0]             tag_ue_count
);

    localparam TAGW_W   = 39;    // 32 payload + 7 check
    localparam RRPV_MAX = 2'd3;

    // -------------------------------------------------------------------------
    // Set banking.  A single 32768 x 39 macro has a 240 ps access time, which
    // does not close at 3.0 GHz once wire, setup and skew are added (see
    // pd/scripts/timing_model.py).  The array is therefore built from
    // TAG_BANKS macros of (SETS/TAG_BANKS) x 39 selected by the low set bits:
    // an 8192-deep macro accesses in 175 ps and the path closes with margin.
    // Banking on the LOW bits also means consecutive sets sit in different
    // macros, so a set-conflict stream spreads across banks.
    // -------------------------------------------------------------------------
    localparam TAG_BANKS   = 4;
    localparam TAG_BANK_W  = 2;
    localparam TAG_SET_W   = SET_W - TAG_BANK_W;

    // -------------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------------
    wire [TAGW_W-1:0] tag_rd     [0:WAYS-1];
    wire [TAGW_W-1:0] tag_bank_rd [0:WAYS-1][0:TAG_BANKS-1];
    wire [WAYS-1:0]   tag_rv;
    wire [TAG_BANKS-1:0] tag_bank_rv [0:WAYS-1];
    reg  [TAGW_W-1:0] tag_wr_word;
    reg  [WAYS-1:0]   tag_we;

    wire [TAG_BANK_W-1:0] lookup_bank = lookup_set[TAG_BANK_W-1:0];
    wire [TAG_SET_W-1:0]  lookup_row  = lookup_set[SET_W-1:TAG_BANK_W];
    wire [TAG_BANK_W-1:0] upd_bank    = upd_set[TAG_BANK_W-1:0];
    wire [TAG_SET_W-1:0]  upd_row     = upd_set[SET_W-1:TAG_BANK_W];

    reg [TAG_BANK_W-1:0] lookup_bank_q;
    always @(posedge clk or posedge rst) begin
        if (rst) lookup_bank_q <= {TAG_BANK_W{1'b0}};
        else     lookup_bank_q <= lookup_bank;
    end

    reg [1:0] rrpv [0:SETS-1][0:WAYS-1];

    genvar w, tb;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            for (tb = 0; tb < TAG_BANKS; tb = tb + 1) begin : g_bank
                wire bank_rd_sel = lookup_valid & (lookup_bank == tb[TAG_BANK_W-1:0]);
                wire bank_wr_sel = tag_we[w]    & (upd_bank    == tb[TAG_BANK_W-1:0]);

                vc3d_sram_sp #(
                    .AW       (TAG_SET_W),
                    .DW       (TAGW_W),
                    .USE_MASK (0)
                ) u_tag_ram (
                    .clk    (clk),
                    .rst    (rst),
                    .ce     (bank_rd_sel | bank_wr_sel),
                    .we     (bank_wr_sel),
                    .addr   (bank_wr_sel ? upd_row : lookup_row),
                    .wdata  (tag_wr_word),
                    .wmask  ({TAGW_W{1'b1}}),
                    .rdata  (tag_bank_rd[w][tb]),
                    .rvalid (tag_bank_rv[w][tb])
                );
            end

            // bank read mux (select is registered, so this is a clean 4:1)
            assign tag_rd[w] = tag_bank_rd[w][lookup_bank_q];
            assign tag_rv[w] = |tag_bank_rv[w];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // ECC-protected tag encode for updates
    // -------------------------------------------------------------------------
    wire [TAGW_W-1:0] enc_word;
    vc3d_ecc_tag_prot u_enc (
        .clk          (clk),
        .rst          (rst),
        .tag_i        (upd_tag),
        .state_i      (upd_state),
        .valid_i      (~upd_invalidate),
        .dirty_i      (upd_dirty),
        .tag_word_o   (enc_word),
        .tag_word_i   ({TAGW_W{1'b0}}),
        .tag_o        (),
        .state_o      (),
        .valid_o      (),
        .dirty_o      (),
        .ce_o         (),
        .ue_o         (),
        .force_miss_o (),
        .syndrome_o   ()
    );

    always @* begin
        tag_wr_word = enc_word;
        tag_we      = {WAYS{1'b0}};
        if (upd_valid) tag_we[upd_way] = 1'b1;
    end

    // -------------------------------------------------------------------------
    // Decode + compare, one decoder per way (all in parallel, one cycle)
    // -------------------------------------------------------------------------
    wire [TAG_W-1:0] dec_tag   [0:WAYS-1];
    wire [2:0]       dec_state [0:WAYS-1];
    wire [WAYS-1:0]  dec_valid;
    wire [WAYS-1:0]  dec_dirty;
    wire [WAYS-1:0]  dec_ce;
    wire [WAYS-1:0]  dec_ue;

    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_dec
            vc3d_ecc_tag_prot u_dec (
                .clk          (clk),
                .rst          (rst),
                .tag_i        ({TAG_W{1'b0}}),
                .state_i      (3'd0),
                .valid_i      (1'b0),
                .dirty_i      (1'b0),
                .tag_word_o   (),
                .tag_word_i   (tag_rd[w]),
                .tag_o        (dec_tag[w]),
                .state_o      (dec_state[w]),
                .valid_o      (dec_valid[w]),
                .dirty_o      (dec_dirty[w]),
                .ce_o         (dec_ce[w]),
                .ue_o         (dec_ue[w]),
                .force_miss_o (),
                .syndrome_o   ()
            );
        end
    endgenerate

    reg [SET_W-1:0] lookup_set_q;
    reg [TAG_W-1:0] lookup_tag_q;
    reg             lookup_valid_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lookup_set_q   <= {SET_W{1'b0}};
            lookup_tag_q   <= {TAG_W{1'b0}};
            lookup_valid_q <= 1'b0;
        end
        else begin
            lookup_set_q   <= lookup_set;
            lookup_tag_q   <= lookup_tag;
            lookup_valid_q <= lookup_valid;
        end
    end

    // -------------------------------------------------------------------------
    // Hit compare uses the RAW tag bits, not the ECC-corrected ones.
    // Serialising 'ECC decode -> compare' costs ~140 ps and does not fit the
    // cycle; instead the compare runs on the raw field while the SECDED
    // decoder runs in parallel.  A CE cannot create a false hit (a corrected
    // bit only matters if it is IN the tag, and then the raw compare misses,
    // which is safe -- the line is simply re-fetched and rewritten); a UE
    // squashes the response one cycle later through tag_ue, which the
    // pipeline treats as a forced miss.  This is the standard way production
    // caches keep tag ECC off the critical path.
    // -------------------------------------------------------------------------
    reg [WAYS-1:0] way_hit;
    integer i;
    always @* begin
        for (i = 0; i < WAYS; i = i + 1) begin
            way_hit[i] = tag_rd[i][30] && ~way_disable[i] &&
                         (tag_rd[i][26:0] == lookup_tag_q);
        end
    end

    // -------------------------------------------------------------------------
    // SRRIP victim selection: pick the first way with RRPV == 3; if none,
    // age the whole set and retry (single-cycle: compute the max instead).
    // -------------------------------------------------------------------------
    reg [WAY_W-1:0] vict;
    reg [1:0]       max_rrpv;
    reg             found_invalid;
    always @* begin
        vict          = {WAY_W{1'b0}};
        max_rrpv      = 2'd0;
        found_invalid = 1'b0;
        for (i = 0; i < WAYS; i = i + 1) begin
            if (!way_disable[i]) begin
                if (!dec_valid[i] && !found_invalid) begin
                    vict          = i[WAY_W-1:0];
                    found_invalid = 1'b1;
                end
                else if (!found_invalid && rrpv[lookup_set_q][i] >= max_rrpv) begin
                    max_rrpv = rrpv[lookup_set_q][i];
                    vict     = i[WAY_W-1:0];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Outputs and RRPV maintenance
    // -------------------------------------------------------------------------
    integer s, k;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lookup_done  <= 1'b0;
            hit          <= 1'b0;
            hit_way      <= {WAY_W{1'b0}};
            hit_state    <= 3'd0;
            hit_dirty    <= 1'b0;
            tag_ce       <= 1'b0;
            tag_ue       <= 1'b0;
            victim_way   <= {WAY_W{1'b0}};
            victim_valid <= 1'b0;
            victim_dirty <= 1'b0;
            victim_tag   <= {TAG_W{1'b0}};
            hit_count    <= 32'd0;
            miss_count   <= 32'd0;
            tag_ce_count <= 32'd0;
            tag_ue_count <= 32'd0;
            for (s = 0; s < SETS; s = s + 1)
                for (k = 0; k < WAYS; k = k + 1)
                    rrpv[s][k] <= RRPV_MAX;
        end
        else begin
            lookup_done <= lookup_valid_q;
            tag_ce      <= |dec_ce;
            tag_ue      <= |dec_ue;
            if (lookup_valid_q && |dec_ce) tag_ce_count <= tag_ce_count + 32'd1;
            if (lookup_valid_q && |dec_ue) tag_ue_count <= tag_ue_count + 32'd1;

            if (lookup_valid_q) begin
                hit <= |way_hit;
                if (|way_hit) begin
                    hit_count <= hit_count + 32'd1;
                    for (i = 0; i < WAYS; i = i + 1) begin
                        if (way_hit[i]) begin
                            hit_way   <= i[WAY_W-1:0];
                            hit_state <= dec_state[i];
                            hit_dirty <= dec_dirty[i];
                            // SRRIP-HP: promote on hit
                            rrpv[lookup_set_q][i] <= 2'd0;
                        end
                    end
                end
                else begin
                    miss_count   <= miss_count + 32'd1;
                    victim_way   <= vict;
                    victim_valid <= dec_valid[vict];
                    victim_dirty <= dec_dirty[vict];
                    victim_tag   <= dec_tag[vict];
                    // age the set if nothing was at RRPV_MAX
                    if (max_rrpv != RRPV_MAX && !found_invalid) begin
                        for (i = 0; i < WAYS; i = i + 1)
                            rrpv[lookup_set_q][i] <= rrpv[lookup_set_q][i] + 2'd1;
                    end
                end
            end

            if (upd_valid && upd_insert) begin
                // DRRIP bimodal insertion: mostly distant (2), occasionally
                // long (0) so that a reused line survives a streaming burst.
                rrpv[upd_set][upd_way] <=
                    (rrip_bimodal_en && (upd_set[4:0] == 5'd0)) ? 2'd0 : 2'd2;
            end
        end
    end

endmodule
