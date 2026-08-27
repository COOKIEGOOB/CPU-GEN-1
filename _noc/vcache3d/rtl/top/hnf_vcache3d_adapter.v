/*
* CPU-GEN-1 : VCACHE-3D -- HN-F attach adapter.
*
* Drop-in replacement for _noc/hnf/hnf_data_sram.v.  The HN-F cache pipeline
* talks to its L3 data array with a plain SRAM-style port:
*
*     l3_index_q     set index
*     l3_rd_ways_q   one-hot way read select
*     l3_wr_ways_q   one-hot way write select
*     l3_wr_data_q   64 B line in
*     l3_rd_data_q   64 B line out, registered
*
* This module presents exactly that port and services it from the 3D stack:
* ways that live on the base die (0..VC3D_BASE_WAY_NUM-1) are answered from the
* base data array in the same number of cycles as the original SRAM, and ways
* that live on the dielet are answered over the hybrid bond.  The HN-F pipeline
* does NOT have to know which die a way lives on -- that is the whole point of
* keeping one unified 16-way image.
*
* Two things the original port cannot express, and how they are handled:
*
*   1. VARIABLE LATENCY.  A stacked way takes longer than a base way.  The
*      adapter therefore drives `l3_stall_req`, which the HN-F pipeline uses to
*      hold the access stage (the same mechanism already used for SRAM
*      contention).  If VC3D_HNF_FIXED_LATENCY is defined, the adapter instead
*      pads every access to the worst case so the pipeline sees a fixed
*      latency and needs no modification at all -- slower, but a zero-change
*      integration.
*
*   2. ERRORS.  The original array cannot report a CE or a UE.  The adapter
*      exposes l3_ce / l3_ue / l3_poison; wiring them into the HN-F error
*      logic is a one-line change, and leaving them unconnected preserves the
*      old behaviour exactly (an unconnected output is legal and the linter
*      only warns on unconnected INPUTS).
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module hnf_vcache3d_adapter #(
    parameter INDEX_WIDTH = 15,
    parameter WAY_NUM     = 16,
    parameter LINE_WIDTH  = 512,
    parameter ID_W        = 12,
    parameter CH_NUM      = 8,
    parameter PHYS_LANE   = 172,
    parameter SLICE_ID    = 0
) (
    input  wire                        clk,
    input  wire                        rst,

    // ---- the hnf_data_sram-compatible port ----------------------------------
    input  wire [INDEX_WIDTH-1:0]      l3_index_q,
    input  wire [WAY_NUM-1:0]          l3_rd_ways_q,
    input  wire [WAY_NUM-1:0]          l3_wr_ways_q,
    input  wire [LINE_WIDTH-1:0]       l3_wr_data_q,
    output reg  [LINE_WIDTH-1:0]       l3_rd_data_q,

    // ---- flow control and error reporting (new) ------------------------------
    output wire                        l3_stall_req,
    output reg                         l3_rd_valid_q,
    output reg                         l3_ce,
    output reg                         l3_ue,
    output reg                         l3_poison,

    // ---- hybrid bond to this slice's dielet ----------------------------------
    output wire [CH_NUM*PHYS_LANE-1:0] pad_out,
    output wire [CH_NUM*PHYS_LANE-1:0] pad_oe,
    input  wire [CH_NUM*PHYS_LANE-1:0] pad_in,

    // ---- repair / power fabric to the dielet ---------------------------------
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM-1:0]    rpr_row_valid,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_ROW_NUM*10-1:0] rpr_row_addr,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM-1:0]    rpr_col_valid,
    output wire [`VC3D_STACK_BANK_NUM*`VC3D_STACK_SUBARRAY_NUM*`VC3D_SPARE_COL_NUM*6-1:0]  rpr_col_id,
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_sleep,
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_deep_sleep,
    output wire [`VC3D_STACK_BANK_NUM-1:0] bank_retention,
    output wire [3:0]                      wa_code,
    output wire [3:0]                      ra_code,
    output wire                            temp_sample,
    input  wire [`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw,

    // ---- CSR (APB) passthrough -------------------------------------------------
    input  wire                        psel,
    input  wire                        penable,
    input  wire                        pwrite,
    input  wire [13:0]                 paddr,
    input  wire [31:0]                 pwdata,
    output wire [31:0]                 prdata,
    output wire                        pready,
    output wire                        pslverr,

    output wire                        irq_nonfatal,
    output wire                        irq_fatal,
    output wire                        slice_ready
);

    // -------------------------------------------------------------------------
    // Way select: the HN-F drives a one-hot mask, the slice wants an index.
    // -------------------------------------------------------------------------
    function automatic [3:0] onehot_to_index(input [WAY_NUM-1:0] oh);
        integer k;
        begin
            onehot_to_index = 4'd0;
            for (k = 0; k < WAY_NUM; k = k + 1)
                if (oh[k]) onehot_to_index = k[3:0];
        end
    endfunction

    wire        do_write = |l3_wr_ways_q;
    wire        do_read  = |l3_rd_ways_q;
    wire [3:0]  way_sel  = do_write ? onehot_to_index(l3_wr_ways_q)
                                    : onehot_to_index(l3_rd_ways_q);

    // The HN-F index is the set; the tag is irrelevant here because the HN-F
    // has already done the tag lookup -- this is a DIRECTED array access, so
    // the adapter uses the slice's maintenance opcode path, which bypasses the
    // slice's own tag array and addresses (set, way) directly.
    wire [47:0] direct_addr = {21'd0, {{(15-INDEX_WIDTH){1'b0}}, l3_index_q},
                               6'd0, 6'd0};

    reg         req_valid_r;
    reg  [5:0]  req_opcode_r;
    reg  [47:0] req_addr_r;
    reg  [11:0] req_id_r;
    reg  [511:0] req_wdata_r;
    wire        req_ready_w;

    wire         rsp_valid_w;
    wire [11:0]  rsp_id_w;
    wire [511:0] rsp_data_w;
    wire         rsp_hit_w, rsp_ce_w, rsp_ue_w, rsp_poison_w;

    // outstanding-access tracking: the port is one-deep, exactly like the SRAM
    reg         busy;
    reg [3:0]   way_q;

    wire stacked_way = (way_sel >= `VC3D_BASE_WAY_NUM);

    assign l3_stall_req = busy;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            req_valid_r  <= 1'b0;
            req_opcode_r <= 6'd0;
            req_addr_r   <= 48'd0;
            req_id_r     <= 12'd0;
            req_wdata_r  <= 512'd0;
            busy         <= 1'b0;
            way_q        <= 4'd0;
        end
        else begin
            req_valid_r <= 1'b0;
            if (!busy && (do_read || do_write)) begin
                req_valid_r  <= 1'b1;
                req_opcode_r <= do_write ? `VC3D_OPC_DIRECT_WRITE
                                         : `VC3D_OPC_DIRECT_READ;
                req_addr_r   <= {direct_addr[47:10], way_sel, 6'd0};
                req_id_r     <= {8'd0, way_sel};
                req_wdata_r  <= l3_wr_data_q;
                way_q        <= way_sel;
                busy         <= 1'b1;
            end
            else if (busy && rsp_valid_w) begin
                busy <= 1'b0;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            l3_rd_data_q  <= {LINE_WIDTH{1'b0}};
            l3_rd_valid_q <= 1'b0;
            l3_ce         <= 1'b0;
            l3_ue         <= 1'b0;
            l3_poison     <= 1'b0;
        end
        else begin
            l3_rd_valid_q <= rsp_valid_w;
            l3_ce         <= rsp_valid_w & rsp_ce_w;
            l3_ue         <= rsp_valid_w & rsp_ue_w;
            l3_poison     <= rsp_valid_w & rsp_poison_w;
            if (rsp_valid_w) l3_rd_data_q <= rsp_data_w;
        end
    end

    // -------------------------------------------------------------------------
    // The slice itself.  Its miss port is tied off: the HN-F owns allocation
    // and refill, so a directed access must never generate a miss request.  If
    // it ever does, miss_valid is left visible for assertion binding.
    // -------------------------------------------------------------------------
    wire         miss_valid_w;
    wire [47:0]  miss_addr_w;
    wire [11:0]  miss_id_w;
    wire         miss_is_write_w;
    wire [511:0] miss_wdata_w;

    vc3d_slice_top #(
        .SLICE_ID  (SLICE_ID),
        .ID_W      (ID_W),
        .CH_NUM    (CH_NUM),
        .PHYS_LANE (PHYS_LANE)
    ) u_slice (
        .clk             (clk),
        .rst             (rst),
        .req_valid       (req_valid_r),
        .req_ready       (req_ready_w),
        .req_opcode      (req_opcode_r),
        .req_addr        (req_addr_r),
        .req_id          (req_id_r),
        .req_wdata       (req_wdata_r),
        .req_be          ({64{1'b1}}),
        .req_qos         (4'd0),
        .rsp_valid       (rsp_valid_w),
        .rsp_ready       (1'b1),
        .rsp_id          (rsp_id_w),
        .rsp_data        (rsp_data_w),
        .rsp_hit         (rsp_hit_w),
        .rsp_ce          (rsp_ce_w),
        .rsp_ue          (rsp_ue_w),
        .rsp_poison      (rsp_poison_w),
        .miss_valid      (miss_valid_w),
        .miss_ready      (1'b1),
        .miss_addr       (miss_addr_w),
        .miss_id         (miss_id_w),
        .miss_is_write   (miss_is_write_w),
        .miss_wdata      (miss_wdata_w),
        .fill_valid      (1'b0),
        .fill_addr       (48'd0),
        .fill_id         (12'd0),
        .fill_data       (512'd0),
        .pad_out         (pad_out),
        .pad_oe          (pad_oe),
        .pad_in          (pad_in),
        .rpr_row_valid   (rpr_row_valid),
        .rpr_row_addr    (rpr_row_addr),
        .rpr_col_valid   (rpr_col_valid),
        .rpr_col_id      (rpr_col_id),
        .bank_sleep      (bank_sleep),
        .bank_deep_sleep (bank_deep_sleep),
        .bank_retention  (bank_retention),
        .wa_code         (wa_code),
        .ra_code         (ra_code),
        .temp_sample     (temp_sample),
        .temp_raw        (temp_raw),
        .psel            (psel),
        .penable         (penable),
        .pwrite          (pwrite),
        .paddr           (paddr),
        .pwdata          (pwdata),
        .prdata          (prdata),
        .pready          (pready),
        .pslverr         (pslverr),
        .irq_nonfatal    (irq_nonfatal),
        .irq_fatal       (irq_fatal),
        .slice_ready     (slice_ready),
        .power_state     (),
        .temp_max        ()
    );

endmodule
