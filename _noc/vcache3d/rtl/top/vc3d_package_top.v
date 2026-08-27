/*
* CPU-GEN-1 : VCACHE-3D -- PACKAGE-level top: base die + three bonded dielets.
*
* This module exists so that simulation and formal see the SAME hierarchy the
* package actually has:
*
*     vc3d_package_top
*       +-- vc3d_l3_96mib_top      (base die, ~1 mm below the lid)
*       +-- vc3d_stack_die_top x3  (three cache dielets, hybrid bonded)
*             ^ connected ONLY through the bond pad arrays -- there is no
*               back-door wiring, so anything the RTL can do here, the silicon
*               can do through the bond field.
*
* The pad connection is deliberately written as an explicit crossing (base
* pad_out -> die pad_in and vice versa) with an optional per-pad delay model
* (`VC3D_BOND_DELAY_MODEL) so that bond RC and the deskew logic can be
* exercised in simulation. See pd/package/ for the bump/pad map that this
* connectivity is generated from.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_package_top #(
    parameter SLICES    = 3,
    parameter ID_W      = 12,
    parameter CH_NUM    = 8,
    parameter PHYS_LANE = 172
) (
    input  wire                    clk,
    input  wire                    rst,

    input  wire                    req_valid,
    output wire                    req_ready,
    input  wire [5:0]              req_opcode,
    input  wire [47:0]             req_addr,
    input  wire [ID_W-1:0]         req_id,
    input  wire [511:0]            req_wdata,
    input  wire [63:0]             req_be,
    input  wire [3:0]              req_qos,

    output wire                    rsp_valid,
    input  wire                    rsp_ready,
    output wire [ID_W-1:0]         rsp_id,
    output wire [511:0]            rsp_data,
    output wire                    rsp_hit,
    output wire                    rsp_ce,
    output wire                    rsp_ue,
    output wire                    rsp_poison,

    output wire [SLICES-1:0]       mem_req_valid,
    input  wire [SLICES-1:0]       mem_req_ready,
    output wire [SLICES*48-1:0]    mem_req_addr,
    output wire [SLICES*ID_W-1:0]  mem_req_id,
    output wire [SLICES-1:0]       mem_req_we,
    output wire [SLICES*512-1:0]   mem_req_wdata,
    input  wire [SLICES-1:0]       mem_rsp_valid,
    input  wire [SLICES*48-1:0]    mem_rsp_addr,
    input  wire [SLICES*ID_W-1:0]  mem_rsp_id,
    input  wire [SLICES*512-1:0]   mem_rsp_data,

    input  wire                    psel,
    input  wire                    penable,
    input  wire                    pwrite,
    input  wire [13:0]             paddr,
    input  wire [31:0]             pwdata,
    output wire [31:0]             prdata,
    output wire                    pready,
    output wire                    pslverr,

    output wire                    irq_nonfatal,
    output wire                    irq_fatal,
    output wire                    cache_ready,
    output wire [11:0]             temp_max_global
);

    localparam BANKS = `VC3D_STACK_BANK_NUM;
    localparam SUBS  = `VC3D_STACK_SUBARRAY_NUM;
    localparam RROW  = BANKS*SUBS*`VC3D_SPARE_ROW_NUM;
    localparam RRA   = RROW*10;
    localparam RCOL  = BANKS*SUBS*`VC3D_SPARE_COL_NUM;
    localparam RCI   = RCOL*6;
    localparam TEMPW = `VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH;
    localparam PADW  = CH_NUM*PHYS_LANE;

    wire [SLICES*PADW-1:0] base_pad_out, base_pad_oe, base_pad_in;
    wire [SLICES*PADW-1:0] die_pad_out,  die_pad_oe,  die_pad_in;

    wire [SLICES*RROW-1:0] rpr_row_valid;
    wire [SLICES*RRA-1:0]  rpr_row_addr;
    wire [SLICES*RCOL-1:0] rpr_col_valid;
    wire [SLICES*RCI-1:0]  rpr_col_id;
    wire [SLICES*BANKS-1:0] bank_sleep, bank_deep_sleep, bank_retention;
    wire [SLICES*4-1:0]    wa_code, ra_code;
    wire [SLICES-1:0]      temp_sample;
    wire [SLICES*TEMPW-1:0] temp_raw;
    wire [SLICES-1:0]      slice_ready_vec;

    // -------------------------------------------------------------------------
    // Base die
    // -------------------------------------------------------------------------
    vc3d_l3_96mib_top #(
        .SLICES (SLICES), .ID_W (ID_W), .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE)
    ) u_base_die (
        .clk (clk), .rst (rst),
        .req_valid (req_valid), .req_ready (req_ready), .req_opcode (req_opcode),
        .req_addr (req_addr), .req_id (req_id), .req_wdata (req_wdata),
        .req_be (req_be), .req_qos (req_qos),
        .rsp_valid (rsp_valid), .rsp_ready (rsp_ready), .rsp_id (rsp_id),
        .rsp_data (rsp_data), .rsp_hit (rsp_hit), .rsp_ce (rsp_ce),
        .rsp_ue (rsp_ue), .rsp_poison (rsp_poison),
        .mem_req_valid (mem_req_valid), .mem_req_ready (mem_req_ready),
        .mem_req_addr (mem_req_addr), .mem_req_id (mem_req_id),
        .mem_req_we (mem_req_we), .mem_req_wdata (mem_req_wdata),
        .mem_rsp_valid (mem_rsp_valid), .mem_rsp_addr (mem_rsp_addr),
        .mem_rsp_id (mem_rsp_id), .mem_rsp_data (mem_rsp_data),
        .pad_out (base_pad_out), .pad_oe (base_pad_oe), .pad_in (base_pad_in),
        .rpr_row_valid (rpr_row_valid), .rpr_row_addr (rpr_row_addr),
        .rpr_col_valid (rpr_col_valid), .rpr_col_id (rpr_col_id),
        .bank_sleep (bank_sleep), .bank_deep_sleep (bank_deep_sleep),
        .bank_retention (bank_retention), .wa_code (wa_code), .ra_code (ra_code),
        .temp_sample (temp_sample), .temp_raw (temp_raw),
        .psel (psel), .penable (penable), .pwrite (pwrite), .paddr (paddr),
        .pwdata (pwdata), .prdata (prdata), .pready (pready), .pslverr (pslverr),
        .irq_nonfatal (irq_nonfatal), .irq_fatal (irq_fatal),
        .cache_ready (cache_ready), .temp_max_global (temp_max_global),
        .slice_ready_vec (slice_ready_vec)
    );

    // -------------------------------------------------------------------------
    // Three hybrid-bonded cache dielets
    // -------------------------------------------------------------------------
    genvar s;
    generate
        for (s = 0; s < SLICES; s = s + 1) begin : g_die
            vc3d_stack_die_top #(
                .CH_NUM (CH_NUM), .PHYS_LANE (PHYS_LANE)
            ) u_cache_die (
                .clk (clk), .rst (rst),
                .pad_out (die_pad_out[s*PADW +: PADW]),
                .pad_oe  (die_pad_oe[s*PADW +: PADW]),
                .pad_in  (die_pad_in[s*PADW +: PADW]),
                .rpr_row_valid (rpr_row_valid[s*RROW +: RROW]),
                .rpr_row_addr  (rpr_row_addr[s*RRA +: RRA]),
                .rpr_col_valid (rpr_col_valid[s*RCOL +: RCOL]),
                .rpr_col_id    (rpr_col_id[s*RCI +: RCI]),
                .bank_sleep      (bank_sleep[s*BANKS +: BANKS]),
                .bank_deep_sleep (bank_deep_sleep[s*BANKS +: BANKS]),
                .bank_retention  (bank_retention[s*BANKS +: BANKS]),
                .wa_code (wa_code[s*4 +: 4]), .ra_code (ra_code[s*4 +: 4]),
                .temp_raw (temp_raw[s*TEMPW +: TEMPW]),
                .temp_sample (temp_sample[s]),
                .bank_busy (), .die_link_up ()
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // The bond field itself: base pad_out drives die pad_in and vice versa.
    // -------------------------------------------------------------------------
`ifdef VC3D_BOND_DELAY_MODEL
    // One-cycle transport in each direction models the flight time plus the
    // receiver sampling flop; the deskew logic must tolerate it.
    reg [SLICES*PADW-1:0] b2d_q, d2b_q;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            b2d_q <= {(SLICES*PADW){1'b0}};
            d2b_q <= {(SLICES*PADW){1'b0}};
        end
        else begin
            b2d_q <= base_pad_out & base_pad_oe;
            d2b_q <= die_pad_out  & die_pad_oe;
        end
    end
    assign die_pad_in  = b2d_q;
    assign base_pad_in = d2b_q;
`else
    assign die_pad_in  = base_pad_out & base_pad_oe;
    assign base_pad_in = die_pad_out  & die_pad_oe;
`endif

endmodule
