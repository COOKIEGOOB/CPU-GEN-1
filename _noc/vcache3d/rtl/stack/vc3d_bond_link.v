/*
* CPU-GEN-1 : VCACHE-3D -- full hybrid-bond link (8 channels) + repair glue.
*
* Bandwidth budget
* ----------------
*   8 channels x 144 payload bits/beat = 1152 bits = 144 B per clock, each way.
*   At the 3.0 GHz stacked-array clock that is 432 GB/s per slice and
*   1.30 TB/s across the three slices -- comfortably above the 32 B/cycle
*   per-core L3 read interface that Zen-class cores present, so the bond link
*   is never the bottleneck; the array banks are.
*
*   A 64 B line + its ECC is 576 bits = 4 beats on ONE channel, or one beat
*   spread across 4 channels.  The controller uses the latter (line-striped)
*   mapping so that a line transfer completes in a single beat and the +4
*   cycle stacked-access penalty is dominated by array access, not transport.
*
* Failure containment: each channel reports link_up / link_fatal.  A dead
* channel removes 4 of the 16 ways from service (way_mask_o) instead of
* killing the cache.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_bond_link #(
    parameter CH_NUM      = 8,
    parameter PAYLOAD_W   = 144,
    parameter CMD_W       = 4,
    parameter SIGNAL_LANE = 164,
    parameter PHYS_LANE   = 172
) (
    input  wire                          clk,
    input  wire                          rst,

    // ---- transaction layer ---------------------------------------------------
    input  wire [CH_NUM-1:0]             tx_valid,
    output wire [CH_NUM-1:0]             tx_ready,
    input  wire [CH_NUM*CMD_W-1:0]       tx_cmd,
    input  wire [CH_NUM*PAYLOAD_W-1:0]   tx_payload,

    output wire [CH_NUM-1:0]             rx_valid,
    output wire [CH_NUM*CMD_W-1:0]       rx_cmd,
    output wire [CH_NUM*PAYLOAD_W-1:0]   rx_payload,
    output wire [CH_NUM-1:0]             rx_crc_err,

    // ---- pads ------------------------------------------------------------------
    output wire [CH_NUM*PHYS_LANE-1:0]   pad_out,
    output wire [CH_NUM*PHYS_LANE-1:0]   pad_oe,
    input  wire [CH_NUM*PHYS_LANE-1:0]   pad_in,

    // ---- control ---------------------------------------------------------------
    input  wire                          link_enable,
    input  wire                          train_req,
    output wire [CH_NUM*3-1:0]           link_state,
    output wire [CH_NUM-1:0]             link_up,
    output wire                          link_up_all,
    output wire [CH_NUM-1:0]             link_fatal,
    output wire [15:0]                   way_mask,      // ways still usable
    output wire [CH_NUM*16-1:0]          crc_err_count,
    output wire [CH_NUM*16-1:0]          retrain_count,

    // ---- lane repair ------------------------------------------------------------
    input  wire                          lane_solve_start,
    output wire                          lane_solve_done,
    output wire                          lane_unrepairable,
    output wire [15:0]                   dead_lane_total
);

    wire [CH_NUM*SIGNAL_LANE*8-1:0] lane_map;
    wire [CH_NUM*PHYS_LANE-1:0]     lane_error_vec;
    wire [CH_NUM*PHYS_LANE-1:0]     lane_healthy_vec;
    wire [CH_NUM-1:0]               solve_done;
    wire [CH_NUM-1:0]               solve_unrep;
    wire [CH_NUM*8-1:0]             dead_cnt;

    // Lanes are retired when the per-lane BER monitor keeps flagging them.
    reg  [CH_NUM*PHYS_LANE-1:0] lane_dead_q;
    always @(posedge clk or posedge rst) begin
        if (rst) lane_dead_q <= {(CH_NUM*PHYS_LANE){1'b0}};
        else     lane_dead_q <= lane_dead_q | lane_error_vec;
    end

    genvar c;
    generate
        for (c = 0; c < CH_NUM; c = c + 1) begin : g_ch
            vc3d_lane_repair #(
                .SIGNAL_LANE (SIGNAL_LANE),
                .PHYS_LANE   (PHYS_LANE),
                .SPARE       (PHYS_LANE - SIGNAL_LANE)
            ) u_lane_repair (
                .clk          (clk),
                .rst          (rst),
                .start        (lane_solve_start),
                .lane_dead    (lane_dead_q[c*PHYS_LANE +: PHYS_LANE]),
                .lane_map     (lane_map[c*SIGNAL_LANE*8 +: SIGNAL_LANE*8]),
                .busy         (),
                .done         (solve_done[c]),
                .unrepairable (solve_unrep[c]),
                .dead_count   (dead_cnt[c*8 +: 8])
            );

            vc3d_bond_channel #(
                .CH_ID       (c),
                .PAYLOAD_W   (PAYLOAD_W),
                .CMD_W       (CMD_W),
                .SIGNAL_LANE (SIGNAL_LANE),
                .SPARE_LANE  (PHYS_LANE - SIGNAL_LANE),
                .PHYS_LANE   (PHYS_LANE)
            ) u_channel (
                .clk              (clk),
                .rst              (rst),
                .tx_valid         (tx_valid[c]),
                .tx_ready         (tx_ready[c]),
                .tx_cmd           (tx_cmd[c*CMD_W +: CMD_W]),
                .tx_payload       (tx_payload[c*PAYLOAD_W +: PAYLOAD_W]),
                .rx_valid         (rx_valid[c]),
                .rx_cmd           (rx_cmd[c*CMD_W +: CMD_W]),
                .rx_payload       (rx_payload[c*PAYLOAD_W +: PAYLOAD_W]),
                .rx_crc_err       (rx_crc_err[c]),
                .pad_out          (pad_out[c*PHYS_LANE +: PHYS_LANE]),
                .pad_oe           (pad_oe[c*PHYS_LANE +: PHYS_LANE]),
                .pad_in           (pad_in[c*PHYS_LANE +: PHYS_LANE]),
                .lane_enable      ({PHYS_LANE{1'b1}}),
                .lane_dead        (lane_dead_q[c*PHYS_LANE +: PHYS_LANE]),
                .lane_deskew      ({(PHYS_LANE*2){1'b0}}),
                .lane_map         (lane_map[c*SIGNAL_LANE*8 +: SIGNAL_LANE*8]),
                .lane_invert      ({PHYS_LANE{1'b0}}),
                .link_train_req   (train_req),
                .link_enable      (link_enable),
                .link_state       (link_state[c*3 +: 3]),
                .link_up          (link_up[c]),
                .crc_err_count    (crc_err_count[c*16 +: 16]),
                .retrain_count    (retrain_count[c*16 +: 16]),
                .lane_error_vec   (lane_error_vec[c*PHYS_LANE +: PHYS_LANE]),
                .lane_healthy_vec (lane_healthy_vec[c*PHYS_LANE +: PHYS_LANE]),
                .link_fatal       (link_fatal[c])
            );
        end
    endgenerate

    assign link_up_all       = &link_up;
    assign lane_solve_done   = &solve_done;
    assign lane_unrepairable = |solve_unrep;

    // Channel c serves stacked ways [4 + c*1.5]; with 8 channels and 12
    // stacked ways every channel backs 1.5 ways, so a dead channel removes a
    // pair of ways (conservative rounding).
    reg [15:0] way_mask_r;
    integer k;
    always @* begin
        way_mask_r = 16'hffff;
        for (k = 0; k < CH_NUM; k = k + 1) begin
            if (link_fatal[k]) begin
                way_mask_r[`VC3D_STACK_WAY_BASE + ((k*3)/2)]     = 1'b0;
                way_mask_r[`VC3D_STACK_WAY_BASE + ((k*3)/2) + 1] = 1'b0;
            end
        end
    end
    assign way_mask = way_mask_r;

    reg [15:0] dead_total_r;
    integer m;
    always @* begin
        dead_total_r = 16'd0;
        for (m = 0; m < CH_NUM; m = m + 1)
            dead_total_r = dead_total_r + {8'd0, dead_cnt[m*8 +: 8]};
    end
    assign dead_lane_total = dead_total_r;

endmodule
