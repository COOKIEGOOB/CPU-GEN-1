/*
* CPU-GEN-1 : VCACHE-3D -- one hybrid-bond channel (link layer + PHY glue).
*
* A channel is the unit of training, repair and flow control across the
* base-die / cache-die bond interface.  Eight channels form the full link.
*
* Per beat a channel carries:
*      144 bits  payload (one ECC-protected 128+16 subline)
*        4 bits  command  (`VC3D_BOND_CMD_*)
*       16 bits  CRC-16/CCITT over {cmd, payload}
*    = 164 signal lanes, plus 8 spare lanes for repair  = 172 physical pads.
*
* Lane repair
* -----------
* Every signal lane owns an 8-bit physical-lane map register.  The repair
* controller programs the map so that a dead pad is bypassed onto a spare.
* The map is loaded from eFuse at reset (hard repair) and can be overridden at
* runtime (soft repair) after a BER excursion is observed by the lane monitors.
*
* Flow control and retry
* ----------------------
* The channel is credit-based (CREDITS beats in flight).  A CRC failure on a
* received beat raises `rx_crc_err`, which the transaction layer turns into a
* replay of the outstanding beat; three consecutive failures escalate to a
* retrain, and a failed retrain escalates to VC3D_BOND_ST_FAIL, which the
* subsystem reports as a fatal link error (ways served by the channel are
* disabled and the cache continues at reduced capacity).
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_bond_channel #(
    parameter CH_ID       = 0,
    parameter PAYLOAD_W   = 144,
    parameter CMD_W       = 4,
    parameter CRC_W       = 16,
    parameter SIGNAL_LANE = 164,
    parameter SPARE_LANE  = 8,
    parameter PHYS_LANE   = 172,
    parameter CREDITS     = 8
) (
    input  wire                        clk,
    input  wire                        rst,

    // ---- transaction layer (base-die side) ---------------------------------
    input  wire                        tx_valid,
    output wire                        tx_ready,
    input  wire [CMD_W-1:0]            tx_cmd,
    input  wire [PAYLOAD_W-1:0]        tx_payload,

    output reg                         rx_valid,
    output reg  [CMD_W-1:0]            rx_cmd,
    output reg  [PAYLOAD_W-1:0]        rx_payload,
    output reg                         rx_crc_err,

    // ---- physical pads ------------------------------------------------------
    output wire [PHYS_LANE-1:0]        pad_out,
    output wire [PHYS_LANE-1:0]        pad_oe,
    input  wire [PHYS_LANE-1:0]        pad_in,

    // ---- lane configuration (from the repair controller) --------------------
    input  wire [PHYS_LANE-1:0]        lane_enable,
    input  wire [PHYS_LANE-1:0]        lane_dead,
    input  wire [PHYS_LANE*2-1:0]      lane_deskew,
    input  wire [SIGNAL_LANE*8-1:0]    lane_map,       // logical -> physical
    input  wire [PHYS_LANE-1:0]        lane_invert,

    // ---- link control -------------------------------------------------------
    input  wire                        link_train_req,
    input  wire                        link_enable,
    output reg  [2:0]                  link_state,
    output reg                         link_up,
    output reg  [15:0]                 crc_err_count,
    output reg  [15:0]                 retrain_count,
    output wire [PHYS_LANE-1:0]        lane_error_vec,
    output wire [PHYS_LANE-1:0]        lane_healthy_vec,
    output reg                         link_fatal
);

    localparam BEAT_W = CMD_W + PAYLOAD_W;      // 148

    // =========================================================================
    // Training pattern generator (shared by both directions)
    // =========================================================================
    reg [31:0] lfsr;
    always @(posedge clk or posedge rst) begin
        if (rst) lfsr <= `VC3D_BOND_TRAIN_PATTERN;
        else     lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    end

    reg [15:0] train_cnt;
    wire       training = (link_state == `VC3D_BOND_ST_LANE_TRAIN) ||
                          (link_state == `VC3D_BOND_ST_DESKEW)     ||
                          (link_state == `VC3D_BOND_ST_RETRAIN);

    // =========================================================================
    // Transmit path
    // =========================================================================
    reg  [7:0] credit;
    assign tx_ready = link_up & (credit != 8'd0);

    wire [BEAT_W-1:0] tx_beat = {tx_cmd, tx_payload};
    wire [CRC_W-1:0]  tx_crc;

    vc3d_bond_crc16 #(.DW(BEAT_W)) u_tx_crc (
        .data_i (tx_beat),
        .crc_o  (tx_crc)
    );

    wire [SIGNAL_LANE-1:0] tx_logical_c = {tx_crc, tx_beat};
    wire                   tx_fire      = tx_valid & tx_ready;

    // The CRC-16 tree, the lane steering and the 900 um run to the bond field
    // do not fit in one 333 ps cycle together (they miss by ~50 ps).  The
    // encoded beat is therefore registered between the CRC and the pad
    // drivers; the receiver's credit loop already tolerates the extra beat.
    reg [SIGNAL_LANE-1:0] tx_logical_q;
    reg                   tx_fire_q;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_logical_q <= {SIGNAL_LANE{1'b0}};
            tx_fire_q    <= 1'b0;
        end
        else begin
            tx_logical_q <= tx_logical_c;
            tx_fire_q    <= tx_fire;
        end
    end

    wire [SIGNAL_LANE-1:0] tx_logical = tx_logical_q;

    // logical -> physical steering
    wire [PHYS_LANE-1:0] phys_tx_bit;
    wire [PHYS_LANE-1:0] phys_tx_en;

    reg [PHYS_LANE-1:0] phys_bit_r;
    reg [PHYS_LANE-1:0] phys_en_r;
    integer li;
    always @* begin
        phys_bit_r = {PHYS_LANE{1'b0}};
        phys_en_r  = {PHYS_LANE{1'b0}};
        for (li = 0; li < SIGNAL_LANE; li = li + 1) begin
            phys_bit_r[lane_map[li*8 +: 8]] = training ? lfsr[li % 32]
                                                       : tx_logical[li];
            phys_en_r [lane_map[li*8 +: 8]] = tx_fire_q | training;
        end
    end
    assign phys_tx_bit = phys_bit_r;
    assign phys_tx_en  = phys_en_r;

    // =========================================================================
    // PHY lanes
    // =========================================================================
    wire [PHYS_LANE-1:0] phys_rx_bit;
    genvar gl;
    generate
        for (gl = 0; gl < PHYS_LANE; gl = gl + 1) begin : g_lane
            vc3d_bond_lane #(
                .LANE_ID (gl)
            ) u_lane (
                .clk           (clk),
                .rst           (rst),
                .tx_bit        (phys_tx_bit[gl]),
                .tx_en         (phys_tx_en[gl]),
                .pad_out       (pad_out[gl]),
                .pad_oe        (pad_oe[gl]),
                .pad_in        (pad_in[gl]),
                .rx_bit        (phys_rx_bit[gl]),
                .lane_enable   (lane_enable[gl]),
                .lane_dead     (lane_dead[gl]),
                .deskew_beats  (lane_deskew[gl*2 +: 2]),
                .deskew_phase  (lane_deskew[gl*2 +: 2]),
                .invert        (lane_invert[gl]),
                .train_en      (training),
                .train_expect  (lfsr[gl % 32]),
                .train_error   (lane_error_vec[gl]),
                .error_count   (),
                .sample_count  (),
                .lane_healthy  (lane_healthy_vec[gl])
            );
        end
    endgenerate

    // =========================================================================
    // Receive path
    // =========================================================================
    reg [SIGNAL_LANE-1:0] rx_logical;
    integer lj;
    always @* begin
        rx_logical = {SIGNAL_LANE{1'b0}};
        for (lj = 0; lj < SIGNAL_LANE; lj = lj + 1) begin
            rx_logical[lj] = phys_rx_bit[lane_map[lj*8 +: 8]];
        end
    end

    wire [BEAT_W-1:0] rx_beat_w = rx_logical[BEAT_W-1:0];
    wire [CRC_W-1:0]  rx_crc_w  = rx_logical[SIGNAL_LANE-1:BEAT_W];
    wire [CRC_W-1:0]  rx_crc_calc;

    vc3d_bond_crc16 #(.DW(BEAT_W)) u_rx_crc (
        .data_i (rx_beat_w),
        .crc_o  (rx_crc_calc)
    );

    wire rx_beat_valid = link_up & (rx_beat_w[BEAT_W-1:PAYLOAD_W] != `VC3D_BOND_CMD_IDLE);
    wire rx_crc_bad    = rx_beat_valid & (rx_crc_calc != rx_crc_w);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_valid   <= 1'b0;
            rx_cmd     <= {CMD_W{1'b0}};
            rx_payload <= {PAYLOAD_W{1'b0}};
            rx_crc_err <= 1'b0;
        end
        else begin
            rx_valid   <= rx_beat_valid & ~rx_crc_bad;
            rx_cmd     <= rx_beat_w[BEAT_W-1:PAYLOAD_W];
            rx_payload <= rx_beat_w[PAYLOAD_W-1:0];
            rx_crc_err <= rx_crc_bad;
        end
    end

    // =========================================================================
    // Credits
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst)                   credit <= CREDITS[7:0];
        else if (!link_up)         credit <= CREDITS[7:0];
        else if (tx_fire & ~rx_valid) credit <= credit - 8'd1;
        else if (~tx_fire & rx_valid && credit != CREDITS[7:0]) credit <= credit + 8'd1;
    end

    // =========================================================================
    // Link training FSM
    // =========================================================================
    reg [2:0]  consec_crc_err;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            link_state     <= `VC3D_BOND_ST_RESET;
            link_up        <= 1'b0;
            train_cnt      <= 16'd0;
            crc_err_count  <= 16'd0;
            retrain_count  <= 16'd0;
            consec_crc_err <= 3'd0;
            link_fatal     <= 1'b0;
        end
        else begin
            if (rx_crc_bad && crc_err_count != 16'hffff)
                crc_err_count <= crc_err_count + 16'd1;

            case (link_state)
                `VC3D_BOND_ST_RESET: begin
                    link_up   <= 1'b0;
                    train_cnt <= 16'd0;
                    if (link_enable) link_state <= `VC3D_BOND_ST_LANE_TRAIN;
                end

                `VC3D_BOND_ST_LANE_TRAIN: begin
                    train_cnt <= train_cnt + 16'd1;
                    if (train_cnt == `VC3D_BOND_TRAIN_CYCLES[15:0]) begin
                        train_cnt  <= 16'd0;
                        link_state <= `VC3D_BOND_ST_DESKEW;
                    end
                end

                `VC3D_BOND_ST_DESKEW: begin
                    train_cnt <= train_cnt + 16'd1;
                    if (train_cnt == 16'd255) begin
                        train_cnt  <= 16'd0;
                        link_state <= (|lane_error_vec) ? `VC3D_BOND_ST_REPAIR
                                                        : `VC3D_BOND_ST_CRC_CHECK;
                    end
                end

                `VC3D_BOND_ST_REPAIR: begin
                    // The repair controller reprograms lane_map while we sit
                    // here; it drops link_train_req when the map is stable.
                    if (!link_train_req) link_state <= `VC3D_BOND_ST_LANE_TRAIN;
                end

                `VC3D_BOND_ST_CRC_CHECK: begin
                    train_cnt <= train_cnt + 16'd1;
                    if (train_cnt == 16'd63) begin
                        train_cnt  <= 16'd0;
                        link_state <= `VC3D_BOND_ST_ACTIVE;
                    end
                end

                `VC3D_BOND_ST_ACTIVE: begin
                    link_up <= 1'b1;
                    if (rx_crc_bad) consec_crc_err <= consec_crc_err + 3'd1;
                    else if (rx_valid) consec_crc_err <= 3'd0;
                    if (consec_crc_err == 3'd3 || link_train_req) begin
                        link_up    <= 1'b0;
                        link_state <= `VC3D_BOND_ST_RETRAIN;
                    end
                    if (!link_enable) link_state <= `VC3D_BOND_ST_RESET;
                end

                `VC3D_BOND_ST_RETRAIN: begin
                    consec_crc_err <= 3'd0;
                    train_cnt      <= train_cnt + 16'd1;
                    if (train_cnt == 16'd0) retrain_count <= retrain_count + 16'd1;
                    if (train_cnt == `VC3D_BOND_TRAIN_CYCLES[15:0]) begin
                        train_cnt <= 16'd0;
                        if (retrain_count > 16'd7) link_state <= `VC3D_BOND_ST_FAIL;
                        else                       link_state <= `VC3D_BOND_ST_DESKEW;
                    end
                end

                `VC3D_BOND_ST_FAIL: begin
                    link_up    <= 1'b0;
                    link_fatal <= 1'b1;
                    if (!link_enable) begin
                        link_fatal <= 1'b0;
                        link_state <= `VC3D_BOND_ST_RESET;
                    end
                end

                default: link_state <= `VC3D_BOND_ST_RESET;
            endcase
        end
    end

endmodule
