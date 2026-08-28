/*
* CPU-GEN-1 : VCACHE-3D -- 32 MiB slice pipeline (tag, way steering, ECC, fill).
*
* Stages
*   S0  accept request, hash to set, allocate MSHR if this is a miss candidate
*   S1  tag SRAM read (16 ways, ECC protected)
*   S2  tag decode + compare + victim select (SRRIP)
*   S3  data access dispatch:
*         way < 4   -> base-die array, data back at S5
*         way >= 4  -> stacked array via the bond link, data back at S3+4+
*   S4+ ECC decode, CE correction, poison propagation
*   S6  response
*
* The two return paths are deliberately NOT merged into one fixed-latency
* pipeline: forcing base-die hits to wait for the stacked latency would throw
* away the whole benefit of the base/stack split.  Responses therefore carry a
* tag and may complete out of order; the router restores per-requester order.
*
* Maintenance port: the scrubber and BISR share a low-priority port that only
* wins when the demand pipeline is idle, except for UE handling, which is
* always highest priority.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_slice_pipeline #(
    parameter SET_W   = 15,
    parameter TAG_W   = 27,
    parameter WAY_W   = 4,
    parameter WAYS    = 16,
    parameter BASE_WAYS = 4,
    parameter ID_W    = 12
) (
    input  wire               clk,
    input  wire               rst,

    // ---- demand request ------------------------------------------------------
    input  wire               req_valid,
    output wire               req_ready,
    input  wire [5:0]         req_opcode,
    input  wire [47:0]        req_addr,
    input  wire [ID_W-1:0]    req_id,
    input  wire [511:0]       req_wdata,
    input  wire [63:0]        req_be,
    input  wire [3:0]         req_qos,

    // ---- response --------------------------------------------------------------
    output reg                rsp_valid,
    input  wire               rsp_ready,
    output reg  [ID_W-1:0]    rsp_id,
    output reg  [511:0]       rsp_data,
    output reg                rsp_hit,
    output reg                rsp_ce,
    output reg                rsp_ue,
    output reg                rsp_poison,

    // ---- speculative/parallel-ECC data return ---------------------------------
    output reg                rsp_spec,
    output reg                rsp_parity_ok,
    output reg                replay_valid,
    output reg  [ID_W-1:0]    replay_id,
    output reg  [47:0]        replay_addr,
    output reg  [511:0]       replay_data,
    output reg                replay_ce,
    output reg                replay_ue,
    output reg                replay_poison,

    // ---- miss / fill interface to the next level --------------------------------
    output reg                miss_valid,
    input  wire               miss_ready,
    output reg  [47:0]        miss_addr,
    output reg  [ID_W-1:0]    miss_id,
    output reg                miss_is_write,
    output reg  [511:0]       miss_wdata,

    input  wire               fill_valid,
    input  wire [47:0]        fill_addr,
    input  wire [ID_W-1:0]    fill_id,
    input  wire [511:0]       fill_data,

    // ---- tag array --------------------------------------------------------------
    output reg                tag_lookup_valid,
    output reg  [SET_W-1:0]   tag_lookup_set,
    output reg  [TAG_W-1:0]   tag_lookup_tag,
    output reg  [3:0]         tag_lookup_qos,
    output reg                tag_lookup_prefetch,
    input  wire               tag_lookup_done,
    input  wire               tag_hit,
    input  wire [WAY_W-1:0]   tag_hit_way,
    input  wire [2:0]         tag_hit_state,
    input  wire               tag_hit_dirty,
    input  wire               tag_ce,
    input  wire               tag_ue,
    input  wire [WAY_W-1:0]   tag_victim_way,
    input  wire               tag_victim_valid,
    input  wire               tag_victim_dirty,
    input  wire [TAG_W-1:0]   tag_victim_tag,
    output reg                tag_upd_valid,
    output reg  [SET_W-1:0]   tag_upd_set,
    output reg  [WAY_W-1:0]   tag_upd_way,
    output reg  [TAG_W-1:0]   tag_upd_tag,
    output reg  [2:0]         tag_upd_state,
    output reg                tag_upd_dirty,
    output reg                tag_upd_insert,
    output reg                tag_upd_invalidate,

    // ---- base-die data array -------------------------------------------------------
    output reg                base_ce,
    output reg                base_we,
    output reg  [SET_W-1:0]   base_set,
    output reg  [1:0]         base_way,
    output reg  [575:0]       base_wdata,
    input  wire [575:0]       base_rdata,
    input  wire               base_rvalid,

    // ---- stacked array (through vc3d_stack_ctrl) --------------------------------------
    output reg                stk_req_valid,
    input  wire               stk_req_ready,
    output reg                stk_req_we,
    output reg  [SET_W-1:0]   stk_req_set,
    output reg  [WAY_W-1:0]   stk_req_way,
    output reg  [575:0]       stk_req_wdata,
    output reg  [7:0]         stk_req_tag,
    input  wire               stk_rsp_valid,
    input  wire [575:0]       stk_rsp_rdata,
    input  wire [7:0]         stk_rsp_tag,
    input  wire               stk_rsp_link_err,

    // ---- maintenance (scrub / BISR) ----------------------------------------------------
    input  wire               mnt_req_valid,
    output reg                mnt_req_ready,
    input  wire [SET_W-1:0]   mnt_set,
    input  wire [WAY_W-1:0]   mnt_way,
    input  wire               mnt_is_write,
    input  wire [511:0]       mnt_wdata,
    output reg                mnt_rsp_valid,
    output reg  [511:0]       mnt_rdata,
    output reg                mnt_rsp_ce,
    output reg                mnt_rsp_ue,
    output reg  [35:0]        mnt_rsp_syndrome,
    output reg                mnt_rsp_line_valid,

    // ---- error reporting -------------------------------------------------------------------
    output reg                err_push,
    output reg  [2:0]         err_class,
    output reg  [2:0]         err_src,
    output reg  [47:0]        err_addr,
    output reg  [3:0]         err_way,
    output reg  [35:0]        err_syndrome,
    output reg                err_dirty,

    // ---- config / status ---------------------------------------------------------------------
    input  wire [15:0]        way_disable,
    input  wire               stack_enable,
    output reg  [31:0]        base_hit_count,
    output reg  [31:0]        stack_hit_count,
    output reg  [31:0]        fill_count,
    output reg  [31:0]        writeback_count,
    output wire               pipe_busy
);

    // -------------------------------------------------------------------------
    // Address decode
    // -------------------------------------------------------------------------
    wire [SET_W-1:0] req_set = req_addr[SET_W+5:6];
    wire             req_is_direct = (req_opcode == `VC3D_OPC_DIRECT_READ) ||
                                     (req_opcode == `VC3D_OPC_DIRECT_WRITE);
    wire [TAG_W-1:0] req_tag = req_addr[47:SET_W+6];

    // -------------------------------------------------------------------------
    // S0/S1 : accept + tag lookup
    // -------------------------------------------------------------------------
    reg              s1_valid;
    reg [ID_W-1:0]   s1_id;
    reg [47:0]       s1_addr;
    reg [5:0]        s1_opcode;
    reg [511:0]      s1_wdata;
    reg              s1_is_mnt;
    reg [WAY_W-1:0]  s1_mnt_way;

    wire mnt_win = mnt_req_valid && !req_valid;

    assign req_ready = !s1_valid || tag_lookup_done;
    assign pipe_busy = s1_valid;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s1_valid         <= 1'b0;
            s1_id            <= {ID_W{1'b0}};
            s1_addr          <= 48'd0;
            s1_opcode        <= 6'd0;
            s1_wdata         <= 512'd0;
            s1_is_mnt        <= 1'b0;
            s1_mnt_way       <= {WAY_W{1'b0}};
            tag_lookup_valid <= 1'b0;
            tag_lookup_set   <= {SET_W{1'b0}};
            tag_lookup_tag   <= {TAG_W{1'b0}};
            tag_lookup_qos   <= 4'd0;
            tag_lookup_prefetch <= 1'b0;
            mnt_req_ready    <= 1'b0;
        end
        else begin
            tag_lookup_valid <= 1'b0;
            mnt_req_ready    <= 1'b0;

            if (req_valid && req_ready) begin
                s1_valid         <= 1'b1;
                s1_id            <= req_id;
                s1_addr          <= req_addr;
                s1_opcode        <= req_opcode;
                s1_wdata         <= req_wdata;
                // A DIRECT access carries its way in addr[9:6] and skips the
                // tag lookup: the HN-F adapter (rtl/top/hnf_vcache3d_adapter.v)
                // has already resolved the way, so re-running the tag compare
                // would only add latency and could disagree with the HN-F.
                // It reuses the maintenance path, which is exactly a directed
                // (set, way) access with no allocation and no miss.
                s1_is_mnt        <= req_is_direct;
                s1_mnt_way       <= req_addr[9:6];
                tag_lookup_valid <= ~req_is_direct;
                tag_lookup_set   <= req_set;
                tag_lookup_tag   <= req_tag;
                tag_lookup_qos   <= req_qos;
                tag_lookup_prefetch <= (req_opcode == `VC3D_OPC_PREFETCH);
            end
            else if (mnt_win) begin
                s1_valid      <= 1'b1;
                s1_is_mnt     <= 1'b1;
                s1_mnt_way    <= mnt_way;
                s1_addr       <= {17'd0, mnt_set, 6'd0};
                s1_opcode     <= mnt_is_write ? `VC3D_OPC_SCRUB_WRITE : `VC3D_OPC_SCRUB_READ;
                s1_wdata      <= mnt_wdata;
                mnt_req_ready <= 1'b1;
            end
            else if (tag_lookup_done || s1_is_mnt) begin
                s1_valid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // S2/S3 : way steering
    // -------------------------------------------------------------------------
    wire [WAY_W-1:0] access_way = s1_is_mnt ? s1_mnt_way : tag_hit_way;
    wire             way_is_base = (access_way < BASE_WAYS);

    wire [575:0] coded_wdata;
    vc3d_ecc_line_enc u_enc (
        .line_i    (s1_wdata),
        .poison_i  (1'b0),
        .written_i (4'hf),
        .seq_i     (2'd0),
        .coded_o   (coded_wdata)
    );

    reg              s3_valid;
    reg              s3_is_mnt;
    reg              s3_from_stack;
    reg [ID_W-1:0]   s3_id;
    reg [47:0]       s3_addr;
    reg [WAY_W-1:0]  s3_way;
    reg              s3_hit;
    reg              s3_dirty;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            base_ce        <= 1'b0;
            base_we        <= 1'b0;
            base_set       <= {SET_W{1'b0}};
            base_way       <= 2'd0;
            base_wdata     <= 576'd0;
            stk_req_valid  <= 1'b0;
            stk_req_we     <= 1'b0;
            stk_req_set    <= {SET_W{1'b0}};
            stk_req_way    <= {WAY_W{1'b0}};
            stk_req_wdata  <= 576'd0;
            stk_req_tag    <= 8'd0;
            s3_valid       <= 1'b0;
            s3_is_mnt      <= 1'b0;
            s3_from_stack  <= 1'b0;
            s3_id          <= {ID_W{1'b0}};
            s3_addr        <= 48'd0;
            s3_way         <= {WAY_W{1'b0}};
            s3_hit         <= 1'b0;
            s3_dirty       <= 1'b0;
            base_hit_count  <= 32'd0;
            stack_hit_count <= 32'd0;
        end
        else begin
            base_ce       <= 1'b0;
            base_we       <= 1'b0;
            if (stk_req_ready) stk_req_valid <= 1'b0;
            s3_valid      <= 1'b0;

            if (s1_valid && (tag_lookup_done || s1_is_mnt)) begin
                s3_valid      <= 1'b1;
                s3_is_mnt     <= s1_is_mnt;
                s3_id         <= s1_id;
                s3_addr       <= s1_addr;
                s3_way        <= access_way;
                s3_hit        <= s1_is_mnt ? 1'b1 : tag_hit;
                s3_dirty      <= tag_hit_dirty;
                s3_from_stack <= ~way_is_base;

                if ((s1_is_mnt || tag_hit) && way_is_base) begin
                    base_ce    <= 1'b1;
                    base_we    <= (s1_opcode == `VC3D_OPC_WRITE_FULL) ||
                                  (s1_opcode == `VC3D_OPC_SCRUB_WRITE) ||
                                  (s1_opcode == `VC3D_OPC_DIRECT_WRITE);
                    base_set   <= s1_addr[SET_W+5:6];
                    base_way   <= access_way[1:0];
                    base_wdata <= coded_wdata;
                    if (tag_hit) base_hit_count <= base_hit_count + 32'd1;
                end
                else if ((s1_is_mnt || tag_hit) && !way_is_base && stack_enable) begin
                    stk_req_valid <= 1'b1;
                    stk_req_we    <= (s1_opcode == `VC3D_OPC_WRITE_FULL) ||
                                     (s1_opcode == `VC3D_OPC_SCRUB_WRITE) ||
                                     (s1_opcode == `VC3D_OPC_DIRECT_WRITE);
                    stk_req_set   <= s1_addr[SET_W+5:6];
                    stk_req_way   <= access_way;
                    stk_req_wdata <= coded_wdata;
                    stk_req_tag   <= {4'd0, access_way};
                    if (tag_hit) stack_hit_count <= stack_hit_count + 32'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // S4..S6 : ECC decode of whichever array answered
    // -------------------------------------------------------------------------
    wire [575:0] coded_rd = base_rvalid ? base_rdata : stk_rsp_rdata;
    wire         rd_valid = base_rvalid | stk_rsp_valid;

    // Two-stage decoder: syndrome in one cycle, correction in the next, so
    // that the ECC does not set the frequency of the whole cache.
    wire [511:0] dec_line;
    wire         dec_ce, dec_ue, dec_poison, dec_valid;
    wire [3:0]   dec_ce_vec, dec_ue_vec;
    wire [35:0]  dec_syndrome;
    wire [3:0]   dec_written;

    vc3d_ecc_line_dec_pipe u_dec (
        .clk        (clk),
        .rst        (rst),
        .in_valid   (rd_valid),
        .coded_i    (coded_rd),
        .out_valid  (dec_valid),
        .line_o     (dec_line),
        .ce_o       (dec_ce),
        .ue_o       (dec_ue),
        .ce_vec_o   (dec_ce_vec),
        .ue_vec_o   (dec_ue_vec),
        .syndrome_o (dec_syndrome),
        .poison_o   (dec_poison),
        .written_o  (dec_written)
    );

    // -------------------------------------------------------------------------
    // Speculative / parallel-ECC data path.
    //
    // The raw line is returned speculatively at the S6 mux stage together with
    // a one-cycle parity/SECDED syndrome flag.  The full four-way SECDED
    // corrector is still built on the base die and runs in parallel; it only
    // stalls/replays an access whose syndrome is non-zero (well under one in a
    // million accesses on healthy silicon).  This pulls the stacked latency
    // down from S7+S8 serialisation to an S6 fast return.
    // -------------------------------------------------------------------------
    wire [35:0] fast_syn;
    wire [3:0]  fast_poison;
    genvar fg;
    generate
        for (fg = 0; fg < 4; fg = fg + 1) begin : g_fast_syn
            wire [143:0] fword = coded_rd[fg*144 +: 144];
            vc3d_secded_syn_128 u_fsyn (
                .data_i     (fword[127:0]),
                .check_i    (fword[136:128]),
                .syndrome_o (fast_syn[fg*9 +: 9])
            );
            assign fast_poison[fg] = fword[137];
        end
    endgenerate

    // Full SECDED syndrome is zero (and no poison / link error) => the raw
    // line itself is safe to return speculatively.  This is the common case.
    wire fast_ok = `VC3D_SPEC_RETURN_ENABLE &&
                   (|fast_syn == 1'b0) && (|fast_poison == 1'b0) &&
                   (~stk_rsp_link_err);

    // Extract the raw 512-bit data payload from the 576-bit coded format
    // (128 data bits per subline, ECC/poison/metadata interleaved every 144).
    wire [511:0] raw_line = {coded_rd[559 -: 128],
                             coded_rd[415 -: 128],
                             coded_rd[271 -: 128],
                             coded_rd[127 -: 128]};

    // the S3 attributes must be delayed to meet the decoded data
    reg              s5_valid, s5_is_mnt, s5_hit, s5_dirty, s5_from_stack, s5_link_err;
    reg              s5_fast_ok;
    reg [ID_W-1:0]   s5_id;
    reg [47:0]       s5_addr;
    reg [WAY_W-1:0]  s5_way;
    reg [511:0]      s5_raw_line;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s5_valid <= 1'b0; s5_is_mnt <= 1'b0; s5_hit <= 1'b0; s5_dirty <= 1'b0;
            s5_from_stack <= 1'b0; s5_link_err <= 1'b0; s5_fast_ok <= 1'b0;
            s5_id <= {ID_W{1'b0}}; s5_addr <= 48'd0; s5_way <= {WAY_W{1'b0}};
            s5_raw_line <= 512'd0;
        end
        else begin
            s5_valid      <= rd_valid;
            s5_is_mnt     <= s3_is_mnt;
            s5_hit        <= s3_hit;
            s5_dirty      <= s3_dirty;
            s5_from_stack <= s3_from_stack;
            s5_link_err   <= stk_rsp_link_err;
            s5_fast_ok    <= fast_ok;
            s5_id         <= s3_id;
            s5_addr       <= s3_addr;
            s5_way        <= s3_way;
            s5_raw_line   <= raw_line;
        end
    end

    reg fast_rsp_sent;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rsp_valid          <= 1'b0;
            rsp_id             <= {ID_W{1'b0}};
            rsp_data           <= 512'd0;
            rsp_hit            <= 1'b0;
            rsp_ce             <= 1'b0;
            rsp_ue             <= 1'b0;
            rsp_poison         <= 1'b0;
            rsp_spec           <= 1'b0;
            rsp_parity_ok      <= 1'b0;
            replay_valid       <= 1'b0;
            replay_id          <= {ID_W{1'b0}};
            replay_addr        <= 48'd0;
            replay_data        <= 512'd0;
            replay_ce          <= 1'b0;
            replay_ue          <= 1'b0;
            replay_poison      <= 1'b0;
            mnt_rsp_valid      <= 1'b0;
            mnt_rdata          <= 512'd0;
            mnt_rsp_ce         <= 1'b0;
            mnt_rsp_ue         <= 1'b0;
            mnt_rsp_syndrome   <= 36'd0;
            mnt_rsp_line_valid <= 1'b0;
            err_push           <= 1'b0;
            err_class          <= `VC3D_ERR_NONE;
            err_src            <= `VC3D_ERRSRC_BASE_DATA;
            err_addr           <= 48'd0;
            err_way            <= 4'd0;
            err_syndrome       <= 36'd0;
            err_dirty          <= 1'b0;
            fast_rsp_sent      <= 1'b0;
        end
        else begin
            rsp_valid     <= 1'b0;
            rsp_spec      <= 1'b0;
            rsp_parity_ok <= 1'b0;
            replay_valid  <= 1'b0;
            mnt_rsp_valid <= 1'b0;
            err_push      <= 1'b0;

            // ---------- speculative S6 return ---------------------------------
            // Raw data + a 1-cycle syndrome/parity flag.  Only the (overwhelming)
            // zero-syndrome case takes this path; anything else waits for the
            // full parallel SECDED corrector and triggers a replay below.
            if (s5_valid && !s5_is_mnt && s5_fast_ok && !fast_rsp_sent) begin
                rsp_valid      <= 1'b1;
                rsp_spec       <= 1'b1;
                rsp_parity_ok  <= 1'b1;
                rsp_id         <= s5_id;
                rsp_data       <= s5_raw_line;
                rsp_hit        <= s5_hit;
                rsp_ce         <= 1'b0;
                rsp_ue         <= 1'b0;
                rsp_poison     <= 1'b0;
                fast_rsp_sent  <= 1'b1;
            end

            // ---------- full parallel-SECDED path (only on non-zero syndrome) --
            if (dec_valid) begin
                if (s5_is_mnt) begin
                    mnt_rsp_valid      <= 1'b1;
                    mnt_rdata          <= dec_line;
                    mnt_rsp_ce         <= dec_ce;
                    mnt_rsp_ue         <= dec_ue;
                    mnt_rsp_syndrome   <= dec_syndrome;
                    mnt_rsp_line_valid <= |dec_written;
                end
                else if (fast_rsp_sent) begin
                    // The speculative response was already returned; the
                    // parallel corrector now publishes the corrected line only
                    // when it found a real error (rare).  Otherwise it just
                    // closes the speculative window.
                    fast_rsp_sent <= 1'b0;
                    if (dec_ce || dec_ue || s5_link_err) begin
                        replay_valid  <= 1'b1;
                        replay_id     <= s5_id;
                        replay_addr   <= s5_addr;
                        replay_data   <= dec_line;
                        replay_ce     <= dec_ce;
                        replay_ue     <= dec_ue;
                        replay_poison <= dec_poison;
                    end
                end
                else begin
                    rsp_valid  <= 1'b1;
                    rsp_id     <= s5_id;
                    rsp_data   <= dec_line;
                    rsp_hit    <= s5_hit;
                    rsp_ce     <= dec_ce;
                    rsp_ue     <= dec_ue;
                    rsp_poison <= dec_poison;
                end

                if (dec_ce || dec_ue || s5_link_err) begin
                    err_push     <= 1'b1;
                    err_class    <= s5_link_err ? `VC3D_ERR_LINK_CRC :
                                    dec_ue      ? `VC3D_ERR_UE       :
                                                  `VC3D_ERR_CE;
                    err_src      <= s5_from_stack ? `VC3D_ERRSRC_STACK_DATA
                                                  : `VC3D_ERRSRC_BASE_DATA;
                    err_addr     <= s5_addr;
                    err_way      <= s5_way;
                    err_syndrome <= dec_syndrome;
                    err_dirty    <= s5_dirty;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Miss / fill handling
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            miss_valid         <= 1'b0;
            miss_addr          <= 48'd0;
            miss_id            <= {ID_W{1'b0}};
            miss_is_write      <= 1'b0;
            miss_wdata         <= 512'd0;
            tag_upd_valid      <= 1'b0;
            tag_upd_set        <= {SET_W{1'b0}};
            tag_upd_way        <= {WAY_W{1'b0}};
            tag_upd_tag        <= {TAG_W{1'b0}};
            tag_upd_state      <= 3'd0;
            tag_upd_dirty      <= 1'b0;
            tag_upd_insert     <= 1'b0;
            tag_upd_invalidate <= 1'b0;
            fill_count         <= 32'd0;
            writeback_count    <= 32'd0;
        end
        else begin
            tag_upd_valid  <= 1'b0;
            tag_upd_insert <= 1'b0;
            if (miss_ready) miss_valid <= 1'b0;

            // miss: request the line, and write back the victim if dirty
            if (s1_valid && tag_lookup_done && !tag_hit && !s1_is_mnt) begin
                miss_valid    <= 1'b1;
                miss_addr     <= s1_addr;
                miss_id       <= s1_id;
                miss_is_write <= 1'b0;
                if (tag_victim_valid && tag_victim_dirty) begin
                    writeback_count <= writeback_count + 32'd1;
                end
            end

            // fill: allocate into the victim way
            if (fill_valid) begin
                fill_count         <= fill_count + 32'd1;
                tag_upd_valid      <= 1'b1;
                tag_upd_set        <= fill_addr[SET_W+5:6];
                tag_upd_way        <= tag_victim_way;
                tag_upd_tag        <= fill_addr[47:SET_W+6];
                tag_upd_state      <= 3'd1;
                tag_upd_dirty      <= 1'b0;
                tag_upd_insert     <= 1'b1;
                tag_upd_invalidate <= 1'b0;
            end
        end
    end

endmodule
