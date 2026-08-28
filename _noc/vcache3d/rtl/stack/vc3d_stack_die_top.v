/*
* CPU-GEN-1 : VCACHE-3D -- CACHE DIE top level.
*
* Everything in this module is physically on the stacked SRAM dielet that is
* hybrid bonded face-to-face onto the base die.  It contains NO cache control
* logic -- no tags, no coherence, no MSHRs -- exactly like a real V-Cache
* dielet: the stacked die is dense SRAM plus the minimum periphery, so that
* essentially all of its area is bitcell and it can be built on a density-
* optimised process variant.
*
* Contents:
*   * the bond-link cache-die endpoint (8 channels, mirrored from the base die)
*   * command decode (READ / WRITE / MBIST / REFRESH / PWR / TRAIN)
*   * the banked SRAM array (generated: rtl/stack/vc3d_stack_bank_array.v)
*   * the repair register fabric outputs (programmed from the base die)
*   * bank power gating, retention control, and 16 thermal sensors
*
* Clocking: the cache die is source-synchronous to the base die -- it receives
* the forwarded clock over dedicated bond pads.  There is one clock domain.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_stack_die_top #(
    parameter CH_NUM     = 8,
    parameter PAYLOAD_W  = 144,
    parameter CMD_W      = 4,
    parameter PHYS_LANE  = 172,
    parameter BANKS      = 32,
    parameter BANK_W     = 5,
    parameter SUBS       = 16,
    parameter SUB_W      = 4,
    parameter ROW_W      = 10,
    parameter SPARE_ROWS = 4,
    parameter SPARE_COLS = 4,
    parameter COL_ID_W   = 6,
    parameter DDR        = `VC3D_BOND_DDR_ENABLE
) (
    // forwarded clock / reset over the bond interface
    input  wire                                      clk,
    input  wire                                      rst,

    // hybrid-bond pads (mate 1:1 with the base die)
    output wire [CH_NUM*PHYS_LANE-1:0]               pad_out,
    output wire [CH_NUM*PHYS_LANE-1:0]               pad_oe,
    input  wire [CH_NUM*PHYS_LANE-1:0]               pad_in,

    // repair fabric, driven from the base-die repair map over the bond link
    input  wire [BANKS*SUBS*SPARE_ROWS-1:0]          rpr_row_valid,
    input  wire [BANKS*SUBS*SPARE_ROWS*ROW_W-1:0]    rpr_row_addr,
    input  wire [BANKS*SUBS*SPARE_COLS-1:0]          rpr_col_valid,
    input  wire [BANKS*SUBS*SPARE_COLS*COL_ID_W-1:0] rpr_col_id,

    // power / assist
    input  wire [BANKS-1:0]                          bank_sleep,
    input  wire [BANKS-1:0]                          bank_deep_sleep,
    input  wire [BANKS-1:0]                          bank_retention,
    input  wire [3:0]                                wa_code,
    input  wire [3:0]                                ra_code,

    // thermal sensors physically located over the array
    output wire [`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw,
    input  wire                                      temp_sample,

    // status back to the base die
    output wire [BANKS-1:0]                          bank_busy,
    output wire                                      die_link_up
);

    // -------------------------------------------------------------------------
    // Bond endpoint (cache-die side)
    // -------------------------------------------------------------------------
    wire [CH_NUM-1:0]           rx_valid;
    wire [CH_NUM*CMD_W-1:0]     rx_cmd;
    wire [CH_NUM*PAYLOAD_W-1:0] rx_payload;
    wire [CH_NUM-1:0]           rx_crc_err;

    reg  [CH_NUM-1:0]           tx_valid;
    wire [CH_NUM-1:0]           tx_ready;
    reg  [CH_NUM*CMD_W-1:0]     tx_cmd;
    reg  [CH_NUM*PAYLOAD_W-1:0] tx_payload;

    vc3d_bond_link #(
        .CH_NUM    (CH_NUM),
        .PAYLOAD_W (PAYLOAD_W),
        .CMD_W     (CMD_W),
        .PHYS_LANE (PHYS_LANE),
        .DDR       (DDR)
    ) u_bond (
        .clk               (clk),
        .rst               (rst),
        .tx_valid          (tx_valid),
        .tx_ready          (tx_ready),
        .tx_cmd            (tx_cmd),
        .tx_payload        (tx_payload),
        .rx_valid          (rx_valid),
        .rx_cmd            (rx_cmd),
        .rx_payload        (rx_payload),
        .rx_crc_err        (rx_crc_err),
        .pad_out           (pad_out),
        .pad_oe            (pad_oe),
        .pad_in            (pad_in),
        .link_enable       (1'b1),
        .train_req         (1'b0),
        .link_state        (),
        .link_up           (),
        .link_up_all       (die_link_up),
        .link_fatal        (),
        .way_mask          (),
        .crc_err_count     (),
        .retrain_count     (),
        .lane_solve_start  (1'b0),
        .lane_solve_done   (),
        .lane_unrepairable (),
        .dead_lane_total   ()
    );

    // -------------------------------------------------------------------------
    // Command decode.  Channel 0 carries the address for a line access;
    // channels 0..3 carry the data.  Channel 4 is the maintenance channel.
    // -------------------------------------------------------------------------
    wire        cmd_read  = rx_valid[0] && (rx_cmd[0*CMD_W +: CMD_W] == `VC3D_BOND_CMD_READ);
    wire        cmd_write = rx_valid[0] && (rx_cmd[0*CMD_W +: CMD_W] == `VC3D_BOND_CMD_WRITE);
    wire        cmd_mnt   = rx_valid[4] && (rx_cmd[4*CMD_W +: CMD_W] == `VC3D_BOND_CMD_MBIST);

    wire [PAYLOAD_W-1:0] addr_beat = rx_payload[0*PAYLOAD_W +: PAYLOAD_W];
    wire [PAYLOAD_W-1:0] mnt_beat  = rx_payload[4*PAYLOAD_W +: PAYLOAD_W];

    wire [BANK_W-1:0] fn_bank = addr_beat[PAYLOAD_W-1 -: BANK_W];
    wire [SUB_W-1:0]  fn_sub  = addr_beat[PAYLOAD_W-BANK_W-1 -: SUB_W];
    wire [ROW_W-1:0]  fn_row  = addr_beat[PAYLOAD_W-BANK_W-SUB_W-1 -: ROW_W];
    wire [7:0]        fn_tag  = addr_beat[PAYLOAD_W-BANK_W-SUB_W-ROW_W-1 -: 8];

    wire [BANK_W-1:0] mn_bank = mnt_beat[PAYLOAD_W-1 -: BANK_W];
    wire [SUB_W-1:0]  mn_sub  = mnt_beat[PAYLOAD_W-BANK_W-1 -: SUB_W];
    wire [ROW_W-1:0]  mn_row  = mnt_beat[PAYLOAD_W-BANK_W-SUB_W-1 -: ROW_W];

    wire              dec_ce   = cmd_read | cmd_write | cmd_mnt;
    wire              dec_we   = cmd_write;
    wire [BANK_W-1:0] dec_bank = cmd_mnt ? mn_bank : fn_bank;
    wire [SUB_W-1:0]  dec_sub  = cmd_mnt ? mn_sub  : fn_sub;
    wire [ROW_W-1:0]  dec_row  = cmd_mnt ? mn_row  : fn_row;
    // Four quadrant arrays hold the four 16 B quarters of a 64 B line; channels
    // 0..3 each carry one quarter, so quadrant q is written from channel q.
    // A maintenance (MBIST) beat drives all four quadrants with the same data
    // so that a march test covers every quadrant in one pass.
    wire [4*PAYLOAD_W-1:0] dec_wdata =
        cmd_mnt ? {4{mnt_beat}} : rx_payload[0 +: 4*PAYLOAD_W];

    // Command decode register.  Pad receive -> bank decode -> subarray decode
    // -> repair CAM -> column remap -> 1.1 mm wire -> macro setup is ~700 ps,
    // past the 667 ps array cycle.  Registering the decode splits it and costs
    // one array cycle, which is accounted for in the +6 cycle stacked adder.
    reg                  arr_ce_q, arr_we_q;
    reg [BANK_W-1:0]     arr_bank_q;
    reg [SUB_W-1:0]      arr_sub_q;
    reg [ROW_W-1:0]      arr_row_q;
    reg [4*PAYLOAD_W-1:0] arr_wdata_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            arr_ce_q    <= 1'b0;
            arr_we_q    <= 1'b0;
            arr_bank_q  <= {BANK_W{1'b0}};
            arr_sub_q   <= {SUB_W{1'b0}};
            arr_row_q   <= {ROW_W{1'b0}};
            arr_wdata_q <= {(4*PAYLOAD_W){1'b0}};
        end
        else begin
            arr_ce_q    <= dec_ce;
            arr_we_q    <= dec_we;
            arr_bank_q  <= dec_bank;
            arr_sub_q   <= dec_sub;
            arr_row_q   <= dec_row;
            arr_wdata_q <= dec_wdata;
        end
    end

    wire                 arr_ce    = arr_ce_q;
    wire                 arr_we    = arr_we_q;
    wire [BANK_W-1:0]    arr_bank  = arr_bank_q;
    wire [SUB_W-1:0]     arr_sub   = arr_sub_q;
    wire [ROW_W-1:0]     arr_row   = arr_row_q;
    wire [4*PAYLOAD_W-1:0] arr_wdata = arr_wdata_q;

    wire [4*PAYLOAD_W-1:0] arr_rdata;
    wire [3:0]             arr_rvalid_q4;
    wire                   arr_rvalid = arr_rvalid_q4[0];
    wire [4*BANKS-1:0]     q_bank_busy;

    // -------------------------------------------------------------------------
    // The generated bank arrays: four quadrants of 32 banks x 16 subarrays.
    //
    // One quadrant stores 32 x 16 x 1024 x 128 b = 8 MiB of payload, i.e. one
    // 16 B quarter of every (set, way) pair.  Four quadrants therefore hold a
    // full 32 MiB of lines -- the whole slice.  In the default hybrid mode the
    // controller maps only ways 4..15 (24 MiB) here and keeps ways 0..3 on the
    // base die; in stack-only mode (VC3D_CFG_STACK_ONLY) the base data array is
    // power gated and all 16 ways live on the dielet.  The dielet is built to
    // the larger of the two, exactly like a real V-Cache dielet that carries a
    // whole cache image.
    //
    // The four quadrant macros of a given (bank, sub, row) are ganged in one
    // row block and share wordline drivers, so a defective row is repaired in
    // all four quadrants together; that is why one repair fabric feeds all of
    // them.  Column repair uses the same shifted-column offset per quadrant,
    // which costs at most three extra spare columns per defect and removes an
    // entire copy of the repair register file.
    // -------------------------------------------------------------------------
    genvar q;
    generate
        for (q = 0; q < 4; q = q + 1) begin : g_quad
            // Dielet Frequency Upgrade: direct SRAM macro slicing.  The four
            // quadrants now use divided-local-bitline 256 x 148 macro slices
            // (vc3d_stack_macro_die_array) instead of the legacy 1024 x 148
            // macros, cutting macro access from ~465 ps to ~260 ps and lifting
            // the dielet clock from 1.5 GHz to `VC3D_STACK_DIELET_CLOCK_MHZ.
            vc3d_stack_macro_die_array u_array (
                .clk             (clk),
                .rst             (rst),
                .ce              (arr_ce),
                .we              (arr_we),
                .bank_sel        (arr_bank),
                .sub_sel         (arr_sub),
                .row_addr        (arr_row),
                .wdata           (arr_wdata[q*PAYLOAD_W +: PAYLOAD_W]),
                .wbit_mask       ({PAYLOAD_W{1'b1}}),
                .rdata           (arr_rdata[q*PAYLOAD_W +: PAYLOAD_W]),
                .rvalid          (arr_rvalid_q4[q]),
                .rpr_row_valid   (rpr_row_valid),
                .rpr_row_addr    (rpr_row_addr),
                .rpr_col_valid   (rpr_col_valid),
                .rpr_col_id      (rpr_col_id),
                .bank_sleep      (bank_sleep),
                .bank_deep_sleep (bank_deep_sleep),
                .bank_retention  (bank_retention),
                .wa_code         (wa_code),
                .ra_code         (ra_code),
                .bank_busy       (q_bank_busy[q*BANKS +: BANKS]),
                .spare_row_hit   (),
                .access_blocked  ()
            );
        end
    endgenerate

    assign bank_busy = q_bank_busy[0*BANKS +: BANKS] |
                       q_bank_busy[1*BANKS +: BANKS] |
                       q_bank_busy[2*BANKS +: BANKS] |
                       q_bank_busy[3*BANKS +: BANKS];

    // -------------------------------------------------------------------------
    // Response: send the array data back on the same channel group.
    // -------------------------------------------------------------------------
    reg mnt_pending;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_valid    <= {CH_NUM{1'b0}};
            tx_cmd      <= {(CH_NUM*CMD_W){1'b0}};
            tx_payload  <= {(CH_NUM*PAYLOAD_W){1'b0}};
            mnt_pending <= 1'b0;
        end
        else begin
            tx_valid <= {CH_NUM{1'b0}};
            if (cmd_mnt) mnt_pending <= 1'b1;

            if (arr_rvalid) begin
                if (mnt_pending) begin
                    mnt_pending                          <= 1'b0;
                    tx_valid[4]                          <= 1'b1;
                    tx_cmd[4*CMD_W +: CMD_W]             <= `VC3D_BOND_CMD_MBIST;
                    tx_payload[4*PAYLOAD_W +: PAYLOAD_W] <= arr_rdata[0 +: PAYLOAD_W];
                end
                else begin
                    tx_valid[3:0]                        <= 4'b1111;
                    tx_cmd[0*CMD_W +: CMD_W]             <= `VC3D_BOND_CMD_READ;
                    tx_cmd[1*CMD_W +: CMD_W]             <= `VC3D_BOND_CMD_READ;
                    tx_cmd[2*CMD_W +: CMD_W]             <= `VC3D_BOND_CMD_READ;
                    tx_cmd[3*CMD_W +: CMD_W]             <= `VC3D_BOND_CMD_READ;
                    tx_payload[0*PAYLOAD_W +: PAYLOAD_W] <= arr_rdata[0*PAYLOAD_W +: PAYLOAD_W];
                    tx_payload[1*PAYLOAD_W +: PAYLOAD_W] <= arr_rdata[1*PAYLOAD_W +: PAYLOAD_W];
                    tx_payload[2*PAYLOAD_W +: PAYLOAD_W] <= arr_rdata[2*PAYLOAD_W +: PAYLOAD_W];
                    tx_payload[3*PAYLOAD_W +: PAYLOAD_W] <= arr_rdata[3*PAYLOAD_W +: PAYLOAD_W];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Thermal sensors: 16 ring-oscillator sensors distributed over the array.
    // The behavioural model reports a base temperature plus a term
    // proportional to local bank activity, which is what the throttle loop and
    // the pd/thermal model are validated against.
    // -------------------------------------------------------------------------
    genvar t;
    generate
        for (t = 0; t < `VC3D_TEMP_SENSOR_NUM; t = t + 1) begin : g_temp
            vc3d_thermal_sensor #(
                .SENSOR_ID (t)
            ) u_sensor (
                .clk        (clk),
                .rst        (rst),
                .sample     (temp_sample),
                .activity   (bank_busy[(t*2) +: 2]),
                .temp_o     (temp_raw[t*`VC3D_TEMP_WIDTH +: `VC3D_TEMP_WIDTH])
            );
        end
    endgenerate

endmodule
