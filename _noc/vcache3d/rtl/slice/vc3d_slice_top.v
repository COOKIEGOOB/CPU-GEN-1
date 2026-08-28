/*
* CPU-GEN-1 : VCACHE-3D -- 32 MiB slice top level (base-die side).
*
* One slice = 8 MiB base-die array + 24 MiB stacked dielet, presented as a
* single unified 32 MiB, 16-way, 64 B-line cache with:
*
*     * SECDED ECC on data (4 x 128+9 per line) and on tags (32+7)
*     * background patrol scrubbing + CE locality tracking
*     * BISR: eFuse autoload, MBIST, spare row/column allocation, lane repair
*     * hybrid-bond link with training, per-lane repair, CRC retry
*     * thermal sensing, graded throttling, per-bank power gating, DVFS
*     * a full CSR window and 64 performance counters
*
* Three of these, address-interleaved by vc3d_interleave_router, make the
* 96 MiB system in rtl/top/vc3d_l3_96mib_top.v.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_slice_top #(
    parameter SLICE_ID  = 0,
    parameter SET_W     = 15,
    parameter TAG_W     = 27,
    parameter WAY_W     = 4,
    parameter WAYS      = 16,
    parameter BASE_WAYS = 4,
    parameter ID_W      = 12,
    parameter CH_NUM    = 8,
    parameter PHYS_LANE = 172,
    parameter DDR       = `VC3D_BOND_DDR_ENABLE
) (
    input  wire                       clk,
    input  wire                       rst,

    // ---- request / response ---------------------------------------------------
    input  wire                       req_valid,
    output wire                       req_ready,
    input  wire [5:0]                 req_opcode,
    input  wire [47:0]                req_addr,
    input  wire [ID_W-1:0]            req_id,
    input  wire [511:0]               req_wdata,
    input  wire [63:0]                req_be,
    input  wire [3:0]                 req_qos,

    output wire                       rsp_valid,
    input  wire                       rsp_ready,
    output wire [ID_W-1:0]            rsp_id,
    output wire [511:0]               rsp_data,
    output wire                       rsp_hit,
    output wire                       rsp_ce,
    output wire                       rsp_ue,
    output wire                       rsp_poison,
    output wire                       rsp_spec,
    output wire                       rsp_parity_ok,
    output wire                       replay_valid,
    output wire [ID_W-1:0]            replay_id,
    output wire [47:0]                replay_addr,
    output wire [511:0]               replay_data,
    output wire                       replay_ce,
    output wire                       replay_ue,
    output wire                       replay_poison,

    // ---- next level (memory / SN-F) ---------------------------------------------
    output wire                       miss_valid,
    input  wire                       miss_ready,
    output wire [47:0]                miss_addr,
    output wire [ID_W-1:0]            miss_id,
    output wire                       miss_is_write,
    output wire [511:0]               miss_wdata,
    input  wire                       fill_valid,
    input  wire [47:0]                fill_addr,
    input  wire [ID_W-1:0]            fill_id,
    input  wire [511:0]               fill_data,

    // ---- hybrid-bond pads to the stacked die ---------------------------------------
    output wire [CH_NUM*PHYS_LANE-1:0] pad_out,
    output wire [CH_NUM*PHYS_LANE-1:0] pad_oe,
    input  wire [CH_NUM*PHYS_LANE-1:0] pad_in,

    // ---- repair fabric to the stacked die ---------------------------------------------
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM-1:0] rpr_row_valid,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM*10-1:0] rpr_row_addr,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM-1:0] rpr_col_valid,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM*6-1:0] rpr_col_id,

    // ---- power / thermal to the stacked die ---------------------------------------------
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_sleep,
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_deep_sleep,
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_retention,
    output wire [3:0]                      wa_code,
    output wire [3:0]                      ra_code,
    output wire                            temp_sample,
    input  wire [`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw,

    // ---- CSR (APB) ------------------------------------------------------------------------
    input  wire                       psel,
    input  wire                       penable,
    input  wire                       pwrite,
    input  wire [13:0]                paddr,
    input  wire [31:0]                pwdata,
    output wire [31:0]                prdata,
    output wire                       pready,
    output wire                       pslverr,

    // ---- interrupts / status ------------------------------------------------------------------
    output wire                       irq_nonfatal,
    output wire                       irq_fatal,
    output wire                       slice_ready,
    output wire [2:0]                 power_state,
    output wire [11:0]                temp_max
);

    localparam BANKS = `VC3D_STACK_BANK_NUM;
    localparam SUBS  = `VC3D_STACK_SUBARRAY_NUM;

    // =========================================================================
    // Interconnect wires
    // =========================================================================
    // tag
    wire              tag_lookup_valid, tag_lookup_done, tag_hit, tag_ce, tag_ue;
    wire [SET_W-1:0]  tag_lookup_set;
    wire [TAG_W-1:0]  tag_lookup_tag;
    wire [3:0]        tag_lookup_qos;
    wire              tag_lookup_prefetch;
    wire              tag_victim_tier;
    wire [WAY_W-1:0]  tag_hit_way, tag_victim_way;
    wire [2:0]        tag_hit_state;
    wire              tag_hit_dirty, tag_victim_valid, tag_victim_dirty;
    wire [TAG_W-1:0]  tag_victim_tag;
    wire              tag_upd_valid, tag_upd_dirty, tag_upd_insert, tag_upd_invalidate;
    wire [SET_W-1:0]  tag_upd_set;
    wire [WAY_W-1:0]  tag_upd_way;
    wire [TAG_W-1:0]  tag_upd_tag;
    wire [2:0]        tag_upd_state;
    wire [31:0]       tag_hit_count, tag_miss_count, tag_ce_count, tag_ue_count;

    // base array
    wire              base_ce, base_we, base_rvalid;
    wire [SET_W-1:0]  base_set;
    wire [1:0]        base_way;
    wire [575:0]      base_wdata, base_rdata;

    // stack
    wire              stk_req_valid, stk_req_ready, stk_req_we;
    wire [SET_W-1:0]  stk_req_set;
    wire [WAY_W-1:0]  stk_req_way;
    wire [575:0]      stk_req_wdata, stk_rsp_rdata;
    wire [7:0]        stk_req_tag, stk_rsp_tag;
    wire              stk_rsp_valid, stk_rsp_link_err;

    // bond
    wire [CH_NUM-1:0]           btx_valid, btx_ready, brx_valid, brx_crc_err;
    wire [CH_NUM*4-1:0]         btx_cmd, brx_cmd;
    wire [CH_NUM*144-1:0]       btx_payload, brx_payload;
    wire                        link_up_all, lane_solve_done, lane_unrepairable;
    wire [15:0]                 link_way_mask, dead_lane_total;
    wire [CH_NUM-1:0]           link_fatal;

    // maintenance / scrub
    wire              mnt_req_valid, mnt_req_ready, mnt_is_write;
    wire [SET_W-1:0]  mnt_set;
    wire [WAY_W-1:0]  mnt_way;
    wire [511:0]      mnt_wdata, mnt_rdata;
    wire              mnt_rsp_valid, mnt_rsp_ce, mnt_rsp_ue, mnt_rsp_line_valid;
    wire [35:0]       mnt_rsp_syndrome;

    // errors
    wire              err_push, err_dirty;
    wire [2:0]        err_class, err_src;
    wire [47:0]       err_addr;
    wire [3:0]        err_way;
    wire [35:0]       err_syndrome;
    wire              scrub_elog_push;
    wire [2:0]        scrub_elog_class, scrub_elog_src;
    wire [SET_W-1:0]  scrub_elog_set;
    wire [WAY_W-1:0]  scrub_elog_way;
    wire [35:0]       scrub_elog_syndrome;

    // repair
    wire              map_wr_en, map_wr_is_col, map_wr_valid, map_wr_slot_busy;
    wire [4:0]        map_wr_bank;
    wire [3:0]        map_wr_sub;
    wire [1:0]        map_wr_slot;
    wire [9:0]        map_wr_addr;
    wire [15:0]       map_rows_used, map_cols_used;
    wire              mbist_start, mbist_busy, mbist_done, mbist_pass, mbist_fail_push;
    wire [2:0]        mbist_algorithm;
    wire [4:0]        mbist_bank_first, mbist_bank_last, mbist_fail_bank;
    wire [3:0]        mbist_fail_sub;
    wire [9:0]        mbist_fail_row;
    wire [143:0]      mbist_fail_expect, mbist_fail_actual;
    wire              mnt_mem_req, mnt_mem_gnt, mnt_mem_we, mnt_mem_rvalid;
    wire [4:0]        mnt_mem_bank;
    wire [3:0]        mnt_mem_sub;
    wire [9:0]        mnt_mem_row;
    wire [143:0]      mnt_mem_wdata, mnt_mem_rdata;
    wire              fuse_autoload_start, fuse_autoload_done, fuse_autoload_crc_err;
    wire              fuse_al_push, fuse_prog_req, fuse_prog_done, fuse_prog_error;
    wire [31:0]       fuse_al_word, fuse_prog_data;
    wire [9:0]        fuse_prog_addr;
    wire              lane_solve_start, link_train_req;
    wire [15:0]       way_disable_repair;
    wire [31:0]       bank_disable;
    wire              repair_busy, repair_done, repair_failed;
    wire [3:0]        repair_phase;
    wire [15:0]       rows_repaired, cols_repaired, lanes_repaired, unrepaired_fails;

    // ce tracker
    wire              ce_demand_valid, ce_demand_ack, ce_repair_req, ce_repair_ack;
    wire [SET_W-1:0]  ce_demand_set, ce_repair_row;
    wire [WAY_W-1:0]  ce_demand_way;
    wire [1:0]        ce_repair_type;
    wire [4:0]        ce_repair_bank;
    wire [3:0]        ce_repair_sub;
    wire [8:0]        ce_repair_syndrome;
    wire [31:0]       total_ce;
    wire [15:0]       max_bucket;

    // scrub
    wire              scrub_repair_req, scrub_repair_ack;
    wire [SET_W-1:0]  scrub_repair_set;
    wire [WAY_W-1:0]  scrub_repair_way;
    wire [35:0]       scrub_repair_syndrome;
    wire [31:0]       scrub_ce_count, scrub_ue_count, scrub_sweep_count;
    wire [SET_W-1:0]  scrub_progress;
    wire              scrub_active;

    // power / thermal
    wire [2:0]        throttle_level;
    wire              throttle_active, thermal_critical;
    wire [BANKS-1:0]  bank_sleep_hint;
    wire [11:0]       temp_avg;
    wire [3:0]        temp_max_id;
    wire [31:0]       throttle_events, throttle_cycles;
    wire [2:0]        logic_level, array_level, dvfs_power_state;
    wire [3:0]        logic_clk_div, array_clk_div;
    wire [7:0]        logic_vid, array_vid;
    wire              array_retention, array_wake_busy;
    wire [31:0]       retention_entries, wake_stall_cycles;
    wire [BANKS-1:0]  pg_sleep, pg_deep_sleep, pg_retention;
    wire [15:0]       banks_powered;
    wire [31:0]       gate_events;

    // csr
    wire              csr_scrub_enable, csr_scrub_oneshot, csr_ecc_enable;
    wire [31:0]       csr_scrub_period;
    wire [3:0]        csr_scrub_burst;
    wire [15:0]       csr_ce_threshold, csr_way_disable;
    wire              csr_stack_enable, csr_throttle_enable, csr_retention_enable;
    wire [11:0]       csr_throttle_threshold, csr_critical_threshold;
    wire              csr_mbist_start, csr_repair_start, csr_poweron_start;
    wire [2:0]        csr_mbist_algo;
    wire              csr_fuse_prog_enable, csr_elog_pop, csr_clear_first, csr_clear_counts;
    wire [2:0]        csr_dvfs_level;
    wire              csr_dvfs_valid;
    wire [5:0]        csr_perf_sel;
    wire [47:0]       perf_value;

    // error log
    wire [63:0]       elog_rd_lo, elog_rd_hi, first_ce_lo, first_ce_hi, first_ue_lo, first_ue_hi;
    wire              elog_empty, elog_overflow, first_ce_valid, first_ue_valid;
    wire [5:0]        elog_level;
    wire [31:0]       elog_ce_count, elog_ue_count, elog_poison_count, elog_link_err_count;

    // =========================================================================
    // Pipeline
    // =========================================================================
    vc3d_slice_pipeline #(
        .SET_W (SET_W), .TAG_W (TAG_W), .WAY_W (WAY_W),
        .WAYS (WAYS), .BASE_WAYS (BASE_WAYS), .ID_W (ID_W)
    ) u_pipe (
        .clk (clk), .rst (rst),
        .req_valid (req_valid), .req_ready (req_ready), .req_opcode (req_opcode),
        .req_addr (req_addr), .req_id (req_id), .req_wdata (req_wdata),
        .req_be (req_be), .req_qos (req_qos),
        .rsp_valid (rsp_valid), .rsp_ready (rsp_ready), .rsp_id (rsp_id),
        .rsp_data (rsp_data), .rsp_hit (rsp_hit), .rsp_ce (rsp_ce),
        .rsp_ue (rsp_ue), .rsp_poison (rsp_poison),
        .rsp_spec (rsp_spec), .rsp_parity_ok (rsp_parity_ok),
        .replay_valid (replay_valid), .replay_id (replay_id),
        .replay_addr (replay_addr), .replay_data (replay_data),
        .replay_ce (replay_ce), .replay_ue (replay_ue),
        .replay_poison (replay_poison),
        .miss_valid (miss_valid), .miss_ready (miss_ready), .miss_addr (miss_addr),
        .miss_id (miss_id), .miss_is_write (miss_is_write), .miss_wdata (miss_wdata),
        .fill_valid (fill_valid), .fill_addr (fill_addr), .fill_id (fill_id),
        .fill_data (fill_data),
        .tag_lookup_valid (tag_lookup_valid), .tag_lookup_set (tag_lookup_set),
        .tag_lookup_tag (tag_lookup_tag), .tag_lookup_qos (tag_lookup_qos),
        .tag_lookup_prefetch (tag_lookup_prefetch),
        .tag_lookup_done (tag_lookup_done),
        .tag_hit (tag_hit), .tag_hit_way (tag_hit_way), .tag_hit_state (tag_hit_state),
        .tag_hit_dirty (tag_hit_dirty), .tag_ce (tag_ce), .tag_ue (tag_ue),
        .tag_victim_way (tag_victim_way), .tag_victim_valid (tag_victim_valid),
        .tag_victim_dirty (tag_victim_dirty), .tag_victim_tag (tag_victim_tag),
        .tag_upd_valid (tag_upd_valid), .tag_upd_set (tag_upd_set),
        .tag_upd_way (tag_upd_way), .tag_upd_tag (tag_upd_tag),
        .tag_upd_state (tag_upd_state), .tag_upd_dirty (tag_upd_dirty),
        .tag_upd_insert (tag_upd_insert), .tag_upd_invalidate (tag_upd_invalidate),
        .base_ce (base_ce), .base_we (base_we), .base_set (base_set),
        .base_way (base_way), .base_wdata (base_wdata), .base_rdata (base_rdata),
        .base_rvalid (base_rvalid),
        .stk_req_valid (stk_req_valid), .stk_req_ready (stk_req_ready),
        .stk_req_we (stk_req_we), .stk_req_set (stk_req_set), .stk_req_way (stk_req_way),
        .stk_req_wdata (stk_req_wdata), .stk_req_tag (stk_req_tag),
        .stk_rsp_valid (stk_rsp_valid), .stk_rsp_rdata (stk_rsp_rdata),
        .stk_rsp_tag (stk_rsp_tag), .stk_rsp_link_err (stk_rsp_link_err),
        .mnt_req_valid (mnt_req_valid), .mnt_req_ready (mnt_req_ready),
        .mnt_set (mnt_set), .mnt_way (mnt_way), .mnt_is_write (mnt_is_write),
        .mnt_wdata (mnt_wdata), .mnt_rsp_valid (mnt_rsp_valid), .mnt_rdata (mnt_rdata),
        .mnt_rsp_ce (mnt_rsp_ce), .mnt_rsp_ue (mnt_rsp_ue),
        .mnt_rsp_syndrome (mnt_rsp_syndrome), .mnt_rsp_line_valid (mnt_rsp_line_valid),
        .err_push (err_push), .err_class (err_class), .err_src (err_src),
        .err_addr (err_addr), .err_way (err_way), .err_syndrome (err_syndrome),
        .err_dirty (err_dirty),
        .way_disable (csr_way_disable | way_disable_repair | ~link_way_mask),
        .stack_enable (csr_stack_enable & link_up_all & ~array_retention),
        .base_hit_count (), .stack_hit_count (), .fill_count (), .writeback_count (),
        .pipe_busy ()
    );

    // =========================================================================
    // Tag array
    // =========================================================================
    vc3d_tag_array #(
        .SETS (1 << SET_W), .SET_W (SET_W), .WAYS (WAYS), .WAY_W (WAY_W),
        .TAG_W (TAG_W), .BASE_WAYS (BASE_WAYS),
        .TIER_AWARE (`VC3D_TIER_AWARE_REPL_ENABLE)
    ) u_tag (
        .clk (clk), .rst (rst),
        .lookup_valid (tag_lookup_valid), .lookup_set (tag_lookup_set),
        .lookup_tag (tag_lookup_tag), .lookup_qos (tag_lookup_qos),
        .lookup_prefetch (tag_lookup_prefetch), .stack_enable (link_up_all),
        .lookup_done (tag_lookup_done),
        .hit (tag_hit), .hit_way (tag_hit_way), .hit_state (tag_hit_state),
        .hit_dirty (tag_hit_dirty), .tag_ce (tag_ce), .tag_ue (tag_ue),
        .victim_way (tag_victim_way), .victim_valid (tag_victim_valid),
        .victim_dirty (tag_victim_dirty), .victim_tag (tag_victim_tag),
        .victim_tier (tag_victim_tier),
        .upd_valid (tag_upd_valid), .upd_set (tag_upd_set), .upd_way (tag_upd_way),
        .upd_tag (tag_upd_tag), .upd_state (tag_upd_state), .upd_dirty (tag_upd_dirty),
        .upd_insert (tag_upd_insert), .upd_invalidate (tag_upd_invalidate),
        .way_disable (csr_way_disable | way_disable_repair), .rrip_bimodal_en (1'b1),
        .hot_qos_threshold (`VC3D_TIER_HOT_QOS_THRESHOLD),
        .hit_count (tag_hit_count), .miss_count (tag_miss_count),
        .tag_ce_count (tag_ce_count), .tag_ue_count (tag_ue_count)
    );

    // =========================================================================
    // Base-die data array
    // =========================================================================
    vc3d_base_data_array #(
        .SETS (1 << SET_W), .SET_W (SET_W), .WAYS (BASE_WAYS), .WAY_W (2), .DW (576)
    ) u_base_data (
        .clk (clk), .rst (rst),
        .ce (base_ce), .we (base_we), .set_idx (base_set), .way_idx (base_way),
        .wdata (base_wdata), .rdata (base_rdata), .rvalid (base_rvalid),
        .way_sleep (4'd0), .way_busy ()
    );

    // =========================================================================
    // Stacked-array controller + bond link (base-die side)
    // =========================================================================
    vc3d_stack_ctrl #(
        .SET_W (SET_W), .WAY_W (WAY_W), .CH_NUM (CH_NUM), .DDR (DDR)
    ) u_stack_ctrl (
        .clk (clk), .rst (rst),
        .req_valid (stk_req_valid), .req_ready (stk_req_ready), .req_we (stk_req_we),
        .req_set (stk_req_set), .req_way (stk_req_way), .req_wdata (stk_req_wdata),
        .req_tag (stk_req_tag),
        .rsp_valid (stk_rsp_valid), .rsp_rdata (stk_rsp_rdata), .rsp_tag (stk_rsp_tag),
        .rsp_link_err (stk_rsp_link_err),
        .tx_valid (btx_valid), .tx_ready (btx_ready), .tx_cmd (btx_cmd),
        .tx_payload (btx_payload),
        .rx_valid (brx_valid), .rx_cmd (brx_cmd), .rx_payload (brx_payload),
        .rx_crc_err (brx_crc_err),
        .link_up_all (link_up_all), .way_mask (link_way_mask),
        .mnt_req (mnt_mem_req), .mnt_gnt (mnt_mem_gnt), .mnt_we (mnt_mem_we),
        .mnt_bank (mnt_mem_bank), .mnt_sub (mnt_mem_sub), .mnt_row (mnt_mem_row),
        .mnt_wdata (mnt_mem_wdata), .mnt_rvalid (mnt_mem_rvalid), .mnt_rdata (mnt_mem_rdata),
        .stack_read_count (), .stack_write_count (), .stack_retry_count (),
        .outstanding ()
    );

    vc3d_bond_link #(
        .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE), .DDR (DDR)
    ) u_bond (
        .clk (clk), .rst (rst),
        .tx_valid (btx_valid), .tx_ready (btx_ready), .tx_cmd (btx_cmd),
        .tx_payload (btx_payload),
        .rx_valid (brx_valid), .rx_cmd (brx_cmd), .rx_payload (brx_payload),
        .rx_crc_err (brx_crc_err),
        .pad_out (pad_out), .pad_oe (pad_oe), .pad_in (pad_in),
        .link_enable (csr_stack_enable), .train_req (link_train_req),
        .link_state (), .link_up (), .link_up_all (link_up_all),
        .link_fatal (link_fatal), .way_mask (link_way_mask),
        .crc_err_count (), .retrain_count (),
        .lane_solve_start (lane_solve_start), .lane_solve_done (lane_solve_done),
        .lane_unrepairable (lane_unrepairable), .dead_lane_total (dead_lane_total)
    );

    // =========================================================================
    // RAS: scrub, CE tracking, error log
    // =========================================================================
    vc3d_scrub_engine #(
        .SET_NUM (1 << SET_W), .SET_W (SET_W), .WAY_NUM (WAYS), .WAY_W (WAY_W)
    ) u_scrub (
        .clk (clk), .rst (rst),
        .scrub_enable (csr_scrub_enable), .scrub_oneshot (csr_scrub_oneshot),
        .scrub_period (csr_scrub_period), .scrub_burst (csr_scrub_burst),
        .ce_threshold (csr_ce_threshold),
        .scrub_req_valid (mnt_req_valid), .scrub_req_ready (mnt_req_ready),
        .scrub_set (mnt_set), .scrub_way (mnt_way), .scrub_is_write (mnt_is_write),
        .scrub_wdata (mnt_wdata),
        .scrub_rsp_valid (mnt_rsp_valid), .scrub_rdata (mnt_rdata),
        .scrub_rsp_ce (mnt_rsp_ce), .scrub_rsp_ue (mnt_rsp_ue),
        .scrub_rsp_syndrome (mnt_rsp_syndrome), .scrub_rsp_valid_line (mnt_rsp_line_valid),
        .demand_valid (ce_demand_valid), .demand_set (ce_demand_set),
        .demand_way (ce_demand_way), .demand_ack (ce_demand_ack),
        .elog_push (scrub_elog_push), .elog_class (scrub_elog_class),
        .elog_src (scrub_elog_src), .elog_set (scrub_elog_set),
        .elog_way (scrub_elog_way), .elog_syndrome (scrub_elog_syndrome),
        .repair_req (scrub_repair_req), .repair_set (scrub_repair_set),
        .repair_way (scrub_repair_way), .repair_syndrome (scrub_repair_syndrome),
        .repair_ack (scrub_repair_ack),
        .scrub_ce_count (scrub_ce_count), .scrub_ue_count (scrub_ue_count),
        .scrub_sweep_count (scrub_sweep_count), .scrub_progress (scrub_progress),
        .scrub_active (scrub_active)
    );

    vc3d_ce_tracker #(
        .SET_W (SET_W), .WAY_W (WAY_W)
    ) u_ce_track (
        .clk (clk), .rst (rst),
        .ce_valid (err_push && (err_class == `VC3D_ERR_CE)),
        .ce_set (err_addr[SET_W+5:6]), .ce_way (err_way),
        .ce_bank (err_addr[10:6] ^ {1'b0, err_way}), .ce_sub (err_addr[14:11]),
        .ce_syndrome (err_syndrome[8:0]), .ce_subline (2'd0),
        .threshold (csr_ce_threshold), .clear (csr_clear_counts),
        .demand_valid (ce_demand_valid), .demand_set (ce_demand_set),
        .demand_way (ce_demand_way), .demand_ack (ce_demand_ack),
        .repair_req (ce_repair_req), .repair_type (ce_repair_type),
        .repair_bank (ce_repair_bank), .repair_sub (ce_repair_sub),
        .repair_row (ce_repair_row), .repair_syndrome (ce_repair_syndrome),
        .repair_ack (ce_repair_ack),
        .total_ce (total_ce), .max_bucket (max_bucket)
    );

    vc3d_error_log #(
        .DEPTH (`VC3D_SCRUB_LOG_DEPTH), .DEPTH_W (5), .ADDR_W (48)
    ) u_elog (
        .clk (clk), .rst (rst),
        .push (err_push | scrub_elog_push),
        .ev_class (err_push ? err_class : scrub_elog_class),
        .ev_src (err_push ? err_src : scrub_elog_src),
        .ev_addr (err_push ? err_addr : {17'd0, scrub_elog_set, 6'd0}),
        .ev_way (err_push ? err_way : scrub_elog_way),
        .ev_syndrome (err_push ? err_syndrome : scrub_elog_syndrome),
        .ev_dirty (err_dirty),
        .clear_first (csr_clear_first), .clear_counts (csr_clear_counts),
        .pop (csr_elog_pop),
        .fifo_empty (elog_empty), .fifo_level (elog_level),
        .fifo_rd_lo (elog_rd_lo), .fifo_rd_hi (elog_rd_hi),
        .first_ce_lo (first_ce_lo), .first_ce_hi (first_ce_hi),
        .first_ue_lo (first_ue_lo), .first_ue_hi (first_ue_hi),
        .first_ce_valid (first_ce_valid), .first_ue_valid (first_ue_valid),
        .overflow (elog_overflow),
        .ce_count (elog_ce_count), .ue_count (elog_ue_count),
        .poison_count (elog_poison_count), .link_err_count (elog_link_err_count),
        .nonfatal_o (irq_nonfatal), .fatal_o (irq_fatal)
    );

    // =========================================================================
    // Repair subsystem
    // =========================================================================
    vc3d_repair_map #(
        .BANKS (BANKS), .SUBS (SUBS)
    ) u_map (
        .clk (clk), .rst (rst),
        .wr_en (map_wr_en), .wr_bank (map_wr_bank), .wr_sub (map_wr_sub),
        .wr_is_col (map_wr_is_col), .wr_slot (map_wr_slot), .wr_valid (map_wr_valid),
        .wr_addr (map_wr_addr), .wr_slot_busy (map_wr_slot_busy),
        .rd_bank (5'd0), .rd_sub (4'd0), .rd_is_col (1'b0), .rd_slot (2'd0),
        .rd_valid (), .rd_addr (), .clear_all (1'b0),
        .rpr_row_valid (rpr_row_valid), .rpr_row_addr (rpr_row_addr),
        .rpr_col_valid (rpr_col_valid), .rpr_col_id (rpr_col_id),
        .rows_used (map_rows_used), .cols_used (map_cols_used), .any_repair ()
    );

    vc3d_mbist_engine #(
        .BANKS (BANKS), .SUBS (SUBS)
    ) u_mbist (
        .clk (clk), .rst (rst),
        .start (mbist_start | csr_mbist_start), .algorithm (mbist_algorithm),
        .bank_first (mbist_bank_first), .bank_last (mbist_bank_last),
        .retention_pause (32'd10000), .stop_on_fail (1'b0),
        .busy (mbist_busy), .done (mbist_done), .pass (mbist_pass),
        .mem_req (mnt_mem_req), .mem_gnt (mnt_mem_gnt), .mem_we (mnt_mem_we),
        .mem_bank (mnt_mem_bank), .mem_sub (mnt_mem_sub), .mem_row (mnt_mem_row),
        .mem_wdata (mnt_mem_wdata), .mem_rvalid (mnt_mem_rvalid), .mem_rdata (mnt_mem_rdata),
        .fail_count (), .fail_bank (mbist_fail_bank), .fail_sub (mbist_fail_sub),
        .fail_row (mbist_fail_row), .fail_expect (mbist_fail_expect),
        .fail_actual (mbist_fail_actual), .fail_push (mbist_fail_push),
        .total_fail_bits ()
    );

    vc3d_efuse_array u_fuse (
        .clk (clk), .rst (rst),
        .prog_en (csr_fuse_prog_enable), .prog_req (fuse_prog_req),
        .prog_addr (fuse_prog_addr), .prog_data (fuse_prog_data),
        .prog_pulse_cycles (16'd30000),
        .prog_busy (), .prog_done (fuse_prog_done), .prog_error (fuse_prog_error),
        .read_req (1'b0), .read_addr (10'd0), .read_data (), .read_valid (),
        .autoload_start (fuse_autoload_start), .autoload_busy (),
        .autoload_done (fuse_autoload_done), .autoload_crc_err (fuse_autoload_crc_err),
        .al_push (fuse_al_push), .al_word (fuse_al_word), .al_index (),
        .fuse_magic (), .record_count ()
    );

    vc3d_repair_ctrl #(
        .BANKS (BANKS), .SUBS (SUBS)
    ) u_repair (
        .clk (clk), .rst (rst),
        .poweron_start (csr_poweron_start), .full_repair_start (csr_repair_start),
        .fuse_prog_enable (csr_fuse_prog_enable),
        .repair_busy (repair_busy), .repair_done (repair_done),
        .repair_failed (repair_failed), .repair_phase (repair_phase),
        .fuse_autoload_start (fuse_autoload_start), .fuse_autoload_done (fuse_autoload_done),
        .fuse_autoload_crc_err (fuse_autoload_crc_err), .fuse_al_push (fuse_al_push),
        .fuse_al_word (fuse_al_word), .fuse_prog_req (fuse_prog_req),
        .fuse_prog_addr (fuse_prog_addr), .fuse_prog_data (fuse_prog_data),
        .fuse_prog_done (fuse_prog_done), .fuse_prog_error (fuse_prog_error),
        .mbist_start (mbist_start), .mbist_algorithm (mbist_algorithm),
        .mbist_bank_first (mbist_bank_first), .mbist_bank_last (mbist_bank_last),
        .mbist_busy (mbist_busy), .mbist_done (mbist_done), .mbist_pass (mbist_pass),
        .mbist_fail_push (mbist_fail_push), .mbist_fail_bank (mbist_fail_bank),
        .mbist_fail_sub (mbist_fail_sub), .mbist_fail_row (mbist_fail_row),
        .mbist_fail_expect (mbist_fail_expect), .mbist_fail_actual (mbist_fail_actual),
        .rt_repair_req (ce_repair_req | scrub_repair_req),
        .rt_repair_type (ce_repair_type), .rt_repair_bank (ce_repair_bank),
        .rt_repair_sub (ce_repair_sub), .rt_repair_row (ce_repair_row[9:0]),
        .rt_repair_syndrome (ce_repair_syndrome), .rt_repair_ack (ce_repair_ack),
        .map_wr_en (map_wr_en), .map_wr_bank (map_wr_bank), .map_wr_sub (map_wr_sub),
        .map_wr_is_col (map_wr_is_col), .map_wr_slot (map_wr_slot),
        .map_wr_valid (map_wr_valid), .map_wr_addr (map_wr_addr),
        .map_wr_slot_busy (map_wr_slot_busy), .map_rows_used (map_rows_used),
        .map_cols_used (map_cols_used),
        .lane_solve_start (lane_solve_start), .lane_solve_done (lane_solve_done),
        .lane_unrepairable (lane_unrepairable), .link_train_req (link_train_req),
        .link_up_all (link_up_all),
        .way_disable (way_disable_repair), .bank_disable (bank_disable),
        .rows_repaired (rows_repaired), .cols_repaired (cols_repaired),
        .lanes_repaired (lanes_repaired), .unrepaired_fails (unrepaired_fails)
    );

    assign scrub_repair_ack = ce_repair_ack;

    // =========================================================================
    // Thermal / power
    // =========================================================================
    vc3d_thermal_throttle #(
        .SENSORS (`VC3D_TEMP_SENSOR_NUM), .BANKS (BANKS)
    ) u_thermal (
        .clk (clk), .rst (rst),
        .temp_raw (temp_raw), .base_die_temp (12'd500), .temp_sample (temp_sample),
        .throttle_threshold (csr_throttle_threshold),
        .critical_threshold (csr_critical_threshold),
        .throttle_enable (csr_throttle_enable),
        .temp_max (temp_max), .temp_max_id (temp_max_id), .temp_avg (temp_avg),
        .throttle_level (throttle_level), .throttle_active (throttle_active),
        .critical (thermal_critical), .bank_sleep_hint (bank_sleep_hint),
        .throttle_events (throttle_events), .throttle_cycles (throttle_cycles)
    );

    vc3d_dvfs_ctrl u_dvfs (
        .clk (clk), .rst (rst),
        .access_active (req_valid | stk_req_valid),
        .throttle_level (throttle_level), .thermal_critical (thermal_critical),
        .fw_level_req (csr_dvfs_level), .fw_level_valid (csr_dvfs_valid),
        .retention_enable (csr_retention_enable),
        .logic_level (logic_level), .array_level (array_level),
        .logic_clk_div (logic_clk_div), .array_clk_div (array_clk_div),
        .logic_vid (logic_vid), .array_vid (array_vid),
        .array_retention (array_retention), .array_wake_busy (array_wake_busy),
        .power_state (dvfs_power_state), .retention_entries (retention_entries),
        .wake_stall_cycles (wake_stall_cycles)
    );

    vc3d_power_gate #(
        .BANKS (BANKS)
    ) u_pg (
        .clk (clk), .rst (rst),
        .bank_on_req (~(bank_sleep_hint | bank_disable[BANKS-1:0])),
        .bank_ret_req ({BANKS{array_retention}}),
        .bank_sleep (pg_sleep), .bank_deep_sleep (pg_deep_sleep),
        .bank_retention (pg_retention), .header_weak (), .header_strong (),
        .seq_busy (), .gate_events (gate_events), .banks_powered (banks_powered)
    );

    assign bank_sleep      = pg_sleep;
    assign bank_deep_sleep = pg_deep_sleep;
    assign bank_retention  = pg_retention;
    assign wa_code         = {2'b0, array_level[2:1]};
    assign ra_code         = {2'b0, array_level[2:1]};
    assign power_state     = dvfs_power_state;
    assign slice_ready     = link_up_all & ~repair_busy & ~array_wake_busy;

    // =========================================================================
    // CSR + performance counters
    // =========================================================================
    vc3d_perf_counters u_perf (
        .clk (clk), .rst (rst),
        .sel (csr_perf_sel), .value (perf_value),
        .ev_req            (req_valid & req_ready),
        .ev_hit            (rsp_valid & rsp_hit),
        .ev_miss           (miss_valid),
        .ev_base_hit       (base_rvalid),
        .ev_stack_hit      (stk_rsp_valid),
        .ev_fill           (fill_valid),
        .ev_writeback      (miss_valid & miss_is_write),
        .ev_ce             (err_push & (err_class == `VC3D_ERR_CE)),
        .ev_ue             (err_push & (err_class == `VC3D_ERR_UE)),
        .ev_scrub_read     (mnt_req_valid & mnt_req_ready & ~mnt_is_write),
        .ev_scrub_write    (mnt_req_valid & mnt_req_ready &  mnt_is_write),
        .ev_link_crc       (|brx_crc_err),
        .ev_link_retrain   (link_train_req),
        .ev_throttle       (throttle_active),
        .ev_retention      (array_retention),
        .ev_repair         (map_wr_en)
    );

    vc3d_csr_apb #(
        .SLICE_ID (SLICE_ID)
    ) u_csr (
        .clk (clk), .rst (rst),
        .psel (psel), .penable (penable), .pwrite (pwrite), .paddr (paddr),
        .pwdata (pwdata), .prdata (prdata), .pready (pready), .pslverr (pslverr),
        .o_ecc_enable (csr_ecc_enable),
        .o_scrub_enable (csr_scrub_enable), .o_scrub_oneshot (csr_scrub_oneshot),
        .o_scrub_period (csr_scrub_period), .o_scrub_burst (csr_scrub_burst),
        .o_ce_threshold (csr_ce_threshold), .o_way_disable (csr_way_disable),
        .o_stack_enable (csr_stack_enable), .o_throttle_enable (csr_throttle_enable),
        .o_retention_enable (csr_retention_enable),
        .o_throttle_threshold (csr_throttle_threshold),
        .o_critical_threshold (csr_critical_threshold),
        .o_mbist_start (csr_mbist_start), .o_mbist_algo (csr_mbist_algo),
        .o_repair_start (csr_repair_start), .o_poweron_start (csr_poweron_start),
        .o_fuse_prog_enable (csr_fuse_prog_enable),
        .o_elog_pop (csr_elog_pop), .o_clear_first (csr_clear_first),
        .o_clear_counts (csr_clear_counts),
        .o_dvfs_level (csr_dvfs_level), .o_dvfs_valid (csr_dvfs_valid),
        .o_perf_sel (csr_perf_sel),
        .i_perf_value (perf_value),
        .i_hit_count (tag_hit_count), .i_miss_count (tag_miss_count),
        .i_ce_count (elog_ce_count), .i_ue_count (elog_ue_count),
        .i_scrub_progress ({1'b0, scrub_progress}), .i_scrub_active (scrub_active),
        .i_scrub_sweeps (scrub_sweep_count),
        .i_elog_lo (elog_rd_lo), .i_elog_hi (elog_rd_hi),
        .i_elog_level ({2'd0, elog_level}), .i_elog_overflow (elog_overflow),
        .i_first_ce_lo (first_ce_lo), .i_first_ue_lo (first_ue_lo),
        .i_repair_phase (repair_phase), .i_repair_busy (repair_busy),
        .i_repair_failed (repair_failed),
        .i_rows_repaired (rows_repaired), .i_cols_repaired (cols_repaired),
        .i_lanes_repaired (lanes_repaired), .i_unrepaired (unrepaired_fails),
        .i_mbist_busy (mbist_busy), .i_mbist_done (mbist_done), .i_mbist_pass (mbist_pass),
        .i_link_up (link_up_all), .i_link_fatal (|link_fatal),
        .i_dead_lanes (dead_lane_total),
        .i_temp_max (temp_max), .i_temp_avg (temp_avg), .i_temp_max_id (temp_max_id),
        .i_throttle_level (throttle_level), .i_throttle_active (throttle_active),
        .i_power_state (dvfs_power_state), .i_banks_powered (banks_powered),
        .i_logic_vid (logic_vid), .i_array_vid (array_vid)
    );

endmodule
