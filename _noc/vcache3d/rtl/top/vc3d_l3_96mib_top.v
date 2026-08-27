/*
* CPU-GEN-1 : VCACHE-3D -- 96 MiB three-slice stacked L3 top level.
*
*      +--------------------------------------------------------------+
*      |                  vc3d_l3_96mib_top (BASE DIE)                |
*      |  +--------------------+   +----------------------------+     |
*      |  | interleave router  |-->| slice 0  (32 MiB)          |==== bond ==> cache die 0
*      |  |  mod-3 exact hash  |-->| slice 1  (32 MiB)          |==== bond ==> cache die 1
*      |  |  reorder + QoS     |-->| slice 2  (32 MiB)          |==== bond ==> cache die 2
*      |  +--------------------+   +----------------------------+     |
*      |  CSR aggregator, RAS aggregator, thermal aggregator          |
*      +--------------------------------------------------------------+
*
* Aggregate figures (see docs/PPA_REPORT.md for the derivation):
*      capacity            96 MiB (3 x 32 MiB, 16-way, 64 B lines)
*      base-die portion    24 MiB (fast region, no bond latency)
*      stacked portion     72 MiB
*      peak read bandwidth 3 slices x 64 B/cycle = 192 B/cycle
*      bond bandwidth      3 x 144 B/cycle each way
*      ECC                 SECDED per 128 b subline + SECDED on tags
*      repair              4 spare rows + 4 spare columns per subarray,
*                          8 spare lanes per bond channel, eFuse persisted
*
* Firmware view: three 4 KiB CSR windows at paddr[13:12] = slice id, plus a
* global window at paddr[13:12] = 3 that aggregates status and broadcasts
* control writes to all slices.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_l3_96mib_top #(
    parameter SLICES    = 3,
    parameter ID_W      = 12,
    parameter SET_W     = 15,
    parameter CH_NUM    = 8,
    parameter PHYS_LANE = 172
) (
    input  wire                            clk,
    input  wire                            rst,

    // ---- SoC cache port ---------------------------------------------------------
    input  wire                            req_valid,
    output wire                            req_ready,
    input  wire [5:0]                      req_opcode,
    input  wire [47:0]                     req_addr,
    input  wire [ID_W-1:0]                 req_id,
    input  wire [511:0]                    req_wdata,
    input  wire [63:0]                     req_be,
    input  wire [3:0]                      req_qos,

    output wire                            rsp_valid,
    input  wire                            rsp_ready,
    output wire [ID_W-1:0]                 rsp_id,
    output wire [511:0]                    rsp_data,
    output wire                            rsp_hit,
    output wire                            rsp_ce,
    output wire                            rsp_ue,
    output wire                            rsp_poison,

    // ---- memory side (one port per slice, they are independent streams) ---------
    output wire [SLICES-1:0]               mem_req_valid,
    input  wire [SLICES-1:0]               mem_req_ready,
    output wire [SLICES*48-1:0]            mem_req_addr,
    output wire [SLICES*ID_W-1:0]          mem_req_id,
    output wire [SLICES-1:0]               mem_req_we,
    output wire [SLICES*512-1:0]           mem_req_wdata,
    input  wire [SLICES-1:0]               mem_rsp_valid,
    input  wire [SLICES*48-1:0]            mem_rsp_addr,
    input  wire [SLICES*ID_W-1:0]          mem_rsp_id,
    input  wire [SLICES*512-1:0]           mem_rsp_data,

    // ---- hybrid-bond pads, one field per slice ------------------------------------
    output wire [SLICES*CH_NUM*PHYS_LANE-1:0] pad_out,
    output wire [SLICES*CH_NUM*PHYS_LANE-1:0] pad_oe,
    input  wire [SLICES*CH_NUM*PHYS_LANE-1:0] pad_in,

    // ---- repair / power / thermal fabric to the three cache dies --------------------
    output wire [SLICES*`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM-1:0]    rpr_row_valid,
    output wire [SLICES*`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM*10-1:0] rpr_row_addr,
    output wire [SLICES*`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM-1:0]    rpr_col_valid,
    output wire [SLICES*`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM*6-1:0]  rpr_col_id,
    output wire [SLICES*`VC3D_STACK_BANK_NUM-1:0] bank_sleep,
    output wire [SLICES*`VC3D_STACK_BANK_NUM-1:0] bank_deep_sleep,
    output wire [SLICES*`VC3D_STACK_BANK_NUM-1:0] bank_retention,
    output wire [SLICES*4-1:0]                    wa_code,
    output wire [SLICES*4-1:0]                    ra_code,
    output wire [SLICES-1:0]                      temp_sample,
    input  wire [SLICES*`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw,

    // ---- CSR --------------------------------------------------------------------------
    input  wire                            psel,
    input  wire                            penable,
    input  wire                            pwrite,
    input  wire [13:0]                     paddr,
    input  wire [31:0]                     pwdata,
    output reg  [31:0]                     prdata,
    output wire                            pready,
    output wire                            pslverr,

    // ---- aggregate status ----------------------------------------------------------------
    output wire                            irq_nonfatal,
    output wire                            irq_fatal,
    output wire                            cache_ready,
    output wire [11:0]                     temp_max_global,
    output wire [SLICES-1:0]               slice_ready_vec
);

    localparam BANKS = `VC3D_STACK_BANK_NUM;
    localparam SUBS  = `VC3D_STACK_SUBARRAY_NUM;
    localparam RROW  = BANKS*SUBS*`VC3D_SPARE_ROW_NUM;
    localparam RRA   = RROW*10;
    localparam RCOL  = BANKS*SUBS*`VC3D_SPARE_COL_NUM;
    localparam RCI   = RCOL*6;
    localparam TEMPW = `VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH;

    // -------------------------------------------------------------------------
    // Router <-> slice wiring
    // -------------------------------------------------------------------------
    wire [SLICES-1:0]      s_req_valid, s_req_ready;
    wire [SLICES*6-1:0]    s_req_opcode;
    wire [SLICES*48-1:0]   s_req_addr;
    wire [SLICES*ID_W-1:0] s_req_id;
    wire [SLICES*512-1:0]  s_req_wdata;
    wire [SLICES*64-1:0]   s_req_be;
    wire [SLICES*4-1:0]    s_req_qos;
    wire [SLICES-1:0]      s_rsp_valid, s_rsp_ready;
    wire [SLICES*ID_W-1:0] s_rsp_id;
    wire [SLICES*512-1:0]  s_rsp_data;
    wire [SLICES-1:0]      s_rsp_hit, s_rsp_ce, s_rsp_ue, s_rsp_poison;

    wire [SLICES-1:0]      s_irq_nonfatal, s_irq_fatal;
    wire [SLICES*32-1:0]   s_prdata;
    wire [SLICES-1:0]      s_pready, s_pslverr;
    wire [SLICES*12-1:0]   s_temp_max;
    wire [SLICES*3-1:0]    s_power_state;

    reg  [SLICES-1:0]      slice_enable_r;
    reg  [1:0]             interleave_mode_r;

    // -------------------------------------------------------------------------
    // Router
    // -------------------------------------------------------------------------
    vc3d_interleave_router #(
        .SLICES (SLICES), .ID_W (ID_W), .SET_W (SET_W)
    ) u_router (
        .clk (clk), .rst (rst),
        .req_valid (req_valid), .req_ready (req_ready), .req_opcode (req_opcode),
        .req_addr (req_addr), .req_id (req_id), .req_wdata (req_wdata),
        .req_be (req_be), .req_qos (req_qos),
        .rsp_valid (rsp_valid), .rsp_ready (rsp_ready), .rsp_id (rsp_id),
        .rsp_data (rsp_data), .rsp_hit (rsp_hit), .rsp_ce (rsp_ce),
        .rsp_ue (rsp_ue), .rsp_poison (rsp_poison),
        .s_req_valid (s_req_valid), .s_req_ready (s_req_ready),
        .s_req_opcode (s_req_opcode), .s_req_addr (s_req_addr), .s_req_id (s_req_id),
        .s_req_wdata (s_req_wdata), .s_req_be (s_req_be), .s_req_qos (s_req_qos),
        .s_rsp_valid (s_rsp_valid), .s_rsp_ready (s_rsp_ready), .s_rsp_id (s_rsp_id),
        .s_rsp_data (s_rsp_data), .s_rsp_hit (s_rsp_hit), .s_rsp_ce (s_rsp_ce),
        .s_rsp_ue (s_rsp_ue), .s_rsp_poison (s_rsp_poison),
        .interleave_mode (interleave_mode_r), .slice_enable (slice_enable_r),
        .strict_order (1'b0),
        .route_count_0 (), .route_count_1 (), .route_count_2 (),
        .reorder_stalls (), .slice_backpressure ()
    );

    // -------------------------------------------------------------------------
    // Three slices
    // -------------------------------------------------------------------------
    genvar s;
    generate
        for (s = 0; s < SLICES; s = s + 1) begin : g_slice
            wire slice_psel = psel && (paddr[13:12] == s[1:0] || paddr[13:12] == 2'd3);

            vc3d_slice_top #(
                .SLICE_ID (s), .SET_W (SET_W), .ID_W (ID_W),
                .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE)
            ) u_slice (
                .clk (clk), .rst (rst),
                .req_valid   (s_req_valid[s]),
                .req_ready   (s_req_ready[s]),
                .req_opcode  (s_req_opcode[s*6 +: 6]),
                .req_addr    (s_req_addr[s*48 +: 48]),
                .req_id      (s_req_id[s*ID_W +: ID_W]),
                .req_wdata   (s_req_wdata[s*512 +: 512]),
                .req_be      (s_req_be[s*64 +: 64]),
                .req_qos     (s_req_qos[s*4 +: 4]),
                .rsp_valid   (s_rsp_valid[s]),
                .rsp_ready   (s_rsp_ready[s]),
                .rsp_id      (s_rsp_id[s*ID_W +: ID_W]),
                .rsp_data    (s_rsp_data[s*512 +: 512]),
                .rsp_hit     (s_rsp_hit[s]),
                .rsp_ce      (s_rsp_ce[s]),
                .rsp_ue      (s_rsp_ue[s]),
                .rsp_poison  (s_rsp_poison[s]),
                .miss_valid  (mem_req_valid[s]),
                .miss_ready  (mem_req_ready[s]),
                .miss_addr   (mem_req_addr[s*48 +: 48]),
                .miss_id     (mem_req_id[s*ID_W +: ID_W]),
                .miss_is_write (mem_req_we[s]),
                .miss_wdata  (mem_req_wdata[s*512 +: 512]),
                .fill_valid  (mem_rsp_valid[s]),
                .fill_addr   (mem_rsp_addr[s*48 +: 48]),
                .fill_id     (mem_rsp_id[s*ID_W +: ID_W]),
                .fill_data   (mem_rsp_data[s*512 +: 512]),
                .pad_out     (pad_out[s*CH_NUM*PHYS_LANE +: CH_NUM*PHYS_LANE]),
                .pad_oe      (pad_oe[s*CH_NUM*PHYS_LANE +: CH_NUM*PHYS_LANE]),
                .pad_in      (pad_in[s*CH_NUM*PHYS_LANE +: CH_NUM*PHYS_LANE]),
                .rpr_row_valid (rpr_row_valid[s*RROW +: RROW]),
                .rpr_row_addr  (rpr_row_addr[s*RRA +: RRA]),
                .rpr_col_valid (rpr_col_valid[s*RCOL +: RCOL]),
                .rpr_col_id    (rpr_col_id[s*RCI +: RCI]),
                .bank_sleep      (bank_sleep[s*BANKS +: BANKS]),
                .bank_deep_sleep (bank_deep_sleep[s*BANKS +: BANKS]),
                .bank_retention  (bank_retention[s*BANKS +: BANKS]),
                .wa_code     (wa_code[s*4 +: 4]),
                .ra_code     (ra_code[s*4 +: 4]),
                .temp_sample (temp_sample[s]),
                .temp_raw    (temp_raw[s*TEMPW +: TEMPW]),
                .psel (slice_psel), .penable (penable), .pwrite (pwrite),
                .paddr ({2'd0, paddr[11:0]}), .pwdata (pwdata),
                .prdata (s_prdata[s*32 +: 32]), .pready (s_pready[s]),
                .pslverr (s_pslverr[s]),
                .irq_nonfatal (s_irq_nonfatal[s]),
                .irq_fatal    (s_irq_fatal[s]),
                .slice_ready  (slice_ready_vec[s]),
                .power_state  (s_power_state[s*3 +: 3]),
                .temp_max     (s_temp_max[s*12 +: 12])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // CSR aggregation
    // -------------------------------------------------------------------------
    localparam GLOBAL_ID       = 14'h3000;
    localparam GLOBAL_STATUS   = 14'h3004;
    localparam GLOBAL_SLICE_EN = 14'h3008;
    localparam GLOBAL_ILV      = 14'h300c;
    localparam GLOBAL_TEMP     = 14'h3010;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            slice_enable_r    <= {SLICES{1'b1}};
            interleave_mode_r <= 2'd0;
        end
        else if (psel && penable && pwrite) begin
            case (paddr)
                GLOBAL_SLICE_EN: slice_enable_r    <= pwdata[SLICES-1:0];
                GLOBAL_ILV:      interleave_mode_r <= pwdata[1:0];
                default: ;
            endcase
        end
    end

    integer k;
    always @* begin
        prdata = 32'd0;
        case (paddr)
            GLOBAL_ID:       prdata = 32'h5633_0060;    // "V3D" + 96 MiB
            GLOBAL_STATUS:   prdata = {24'd0, 4'd0,
                                       cache_ready, slice_ready_vec};
            GLOBAL_SLICE_EN: prdata = {29'd0, slice_enable_r};
            GLOBAL_ILV:      prdata = {30'd0, interleave_mode_r};
            GLOBAL_TEMP:     prdata = {20'd0, temp_max_global};
            default: begin
                prdata = 32'd0;
                for (k = 0; k < SLICES; k = k + 1)
                    if (paddr[13:12] == k[1:0]) prdata = s_prdata[k*32 +: 32];
            end
        endcase
    end

    assign pready       = &s_pready;
    assign pslverr      = |s_pslverr;
    assign irq_nonfatal = |s_irq_nonfatal;
    assign irq_fatal    = |s_irq_fatal;
    assign cache_ready  = &(slice_ready_vec | ~slice_enable_r);

    reg [11:0] tmax;
    integer t;
    always @* begin
        tmax = 12'd0;
        for (t = 0; t < SLICES; t = t + 1)
            if (s_temp_max[t*12 +: 12] > tmax) tmax = s_temp_max[t*12 +: 12];
    end
    assign temp_max_global = tmax;

endmodule
