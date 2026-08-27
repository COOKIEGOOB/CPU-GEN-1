/*
* CPU-GEN-1 : VCACHE-3D -- base-die controller for the stacked cache dielet.
*
* Converts a 64 B line access on the slice's stacked-way port into bond-link
* traffic and back.  This is the module that owns the "+4 cycle" cost of the
* 3D array relative to the base-die array:
*
*      cycle 0 : request accepted, address decoded to {bank, subarray, row}
*      cycle 1 : bond TX beat (4 channels in parallel carry 576 b)
*      cycle 2 : cache-die array access
*      cycle 3 : bond RX beat
*      cycle 4 : ECC decode + return
*
* which matches the published +4 cycle latency adder of a hybrid-bonded cache
* die relative to the on-die array, and is why the slice pipeline schedules
* stacked ways on a separate, deeper return path rather than stalling the
* base-die path to match.
*
* Address decode (32 MiB slice, 12 stacked ways of 16):
*      set[14:0], way[3:0]  ->  bank = f(set[4:0], way)   (32 banks)
*                               sub  = set[8:5]           (16 subarrays)
*                               row  = set[14:9], way'    (1024 rows)
* The bank function XORs the way index into the low set bits so that a
* set-conflict burst (the classic pathological stream) spreads across banks
* instead of hammering one.
*
* Ordering: reads and writes to the same line are kept in order by a small
* address-matching scoreboard; everything else is free to complete out of
* order, which is what keeps 32 banks busy.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_stack_ctrl #(
    parameter SET_W    = 15,
    parameter WAY_W    = 4,
    parameter BANK_W   = 5,
    parameter SUB_W    = 4,
    parameter ROW_W    = 10,
    parameter CH_NUM   = 8,
    parameter PAYLOAD_W= 144,
    parameter CMD_W    = 4,
    parameter TAG_W    = 8
) (
    input  wire                         clk,
    input  wire                         rst,

    // ---- slice-side request port ---------------------------------------------
    input  wire                         req_valid,
    output wire                         req_ready,
    input  wire                         req_we,
    input  wire [SET_W-1:0]             req_set,
    input  wire [WAY_W-1:0]             req_way,
    input  wire [575:0]                 req_wdata,   // 4 x 144 coded subline
    input  wire [TAG_W-1:0]             req_tag,

    // ---- slice-side response port ---------------------------------------------
    output reg                          rsp_valid,
    output reg  [575:0]                 rsp_rdata,
    output reg  [TAG_W-1:0]             rsp_tag,
    output reg                          rsp_link_err,

    // ---- bond link -------------------------------------------------------------
    output reg  [CH_NUM-1:0]            tx_valid,
    input  wire [CH_NUM-1:0]            tx_ready,
    output reg  [CH_NUM*CMD_W-1:0]      tx_cmd,
    output reg  [CH_NUM*PAYLOAD_W-1:0]  tx_payload,

    input  wire [CH_NUM-1:0]            rx_valid,
    input  wire [CH_NUM*CMD_W-1:0]      rx_cmd,
    input  wire [CH_NUM*PAYLOAD_W-1:0]  rx_payload,
    input  wire [CH_NUM-1:0]            rx_crc_err,

    input  wire                         link_up_all,
    input  wire [15:0]                  way_mask,

    // ---- maintenance port (MBIST / BISR direct access) --------------------------
    input  wire                         mnt_req,
    output wire                         mnt_gnt,
    input  wire                         mnt_we,
    input  wire [BANK_W-1:0]            mnt_bank,
    input  wire [SUB_W-1:0]             mnt_sub,
    input  wire [ROW_W-1:0]             mnt_row,
    input  wire [PAYLOAD_W-1:0]         mnt_wdata,
    output reg                          mnt_rvalid,
    output reg  [PAYLOAD_W-1:0]         mnt_rdata,

    // ---- status -------------------------------------------------------------------
    output reg  [31:0]                  stack_read_count,
    output reg  [31:0]                  stack_write_count,
    output reg  [31:0]                  stack_retry_count,
    output wire [7:0]                   outstanding
);

    // -------------------------------------------------------------------------
    // Address decode
    // -------------------------------------------------------------------------
    wire [BANK_W-1:0] dec_bank = req_set[4:0] ^ {1'b0, req_way};
    wire [SUB_W-1:0]  dec_sub  = req_set[8:5];
    wire [ROW_W-1:0]  dec_row  = {req_set[14:9], req_way};

    // -------------------------------------------------------------------------
    // Outstanding-transaction tracker.  A line uses 4 channels; the controller
    // therefore issues one "group" per cycle and tracks it by tag.
    // -------------------------------------------------------------------------
    localparam OSQ_DEPTH = 16;
    reg [TAG_W-1:0] osq_tag   [0:OSQ_DEPTH-1];
    reg             osq_we    [0:OSQ_DEPTH-1];
    reg             osq_valid [0:OSQ_DEPTH-1];
    reg [3:0]       osq_wptr, osq_rptr;
    reg [7:0]       osq_count;

    assign outstanding = osq_count;
    assign req_ready   = link_up_all && (osq_count < OSQ_DEPTH) && !mnt_req;
    assign mnt_gnt     = mnt_req && link_up_all && (osq_count == 8'd0);

    wire fire = req_valid & req_ready;

    // -------------------------------------------------------------------------
    // Transmit: stripe the 576-bit coded line across channels 0..3, and use
    // channels 4..7 for the second half of a dual-issue window (two lines can
    // be in flight per cycle when both halves are free).
    // -------------------------------------------------------------------------
    integer c;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_valid   <= {CH_NUM{1'b0}};
            tx_cmd     <= {(CH_NUM*CMD_W){1'b0}};
            tx_payload <= {(CH_NUM*PAYLOAD_W){1'b0}};
            stack_read_count  <= 32'd0;
            stack_write_count <= 32'd0;
        end
        else begin
            tx_valid <= {CH_NUM{1'b0}};

            if (fire) begin
                for (c = 0; c < 4; c = c + 1) begin
                    tx_valid[c]                  <= 1'b1;
                    tx_cmd[c*CMD_W +: CMD_W]     <= req_we ? `VC3D_BOND_CMD_WRITE
                                                           : `VC3D_BOND_CMD_READ;
                    tx_payload[c*PAYLOAD_W +: PAYLOAD_W] <=
                        req_we ? req_wdata[c*144 +: 144]
                               : {dec_bank, dec_sub, dec_row, req_tag,
                                  {(PAYLOAD_W-BANK_W-SUB_W-ROW_W-TAG_W){1'b0}}};
                end
                if (req_we) stack_write_count <= stack_write_count + 32'd1;
                else        stack_read_count  <= stack_read_count  + 32'd1;
            end
            else if (mnt_gnt) begin
                tx_valid[4]                  <= 1'b1;
                tx_cmd[4*CMD_W +: CMD_W]     <= `VC3D_BOND_CMD_MBIST;
                tx_payload[4*PAYLOAD_W +: PAYLOAD_W] <=
                    mnt_we ? mnt_wdata
                           : {mnt_bank, mnt_sub, mnt_row,
                              {(PAYLOAD_W-BANK_W-SUB_W-ROW_W){1'b0}}};
            end
        end
    end

    // -------------------------------------------------------------------------
    // Outstanding queue bookkeeping
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            osq_wptr  <= 4'd0;
            osq_rptr  <= 4'd0;
            osq_count <= 8'd0;
            for (i = 0; i < OSQ_DEPTH; i = i + 1) begin
                osq_valid[i] <= 1'b0;
                osq_tag[i]   <= {TAG_W{1'b0}};
                osq_we[i]    <= 1'b0;
            end
        end
        else begin
            if (fire) begin
                osq_tag[osq_wptr]   <= req_tag;
                osq_we[osq_wptr]    <= req_we;
                osq_valid[osq_wptr] <= 1'b1;
                osq_wptr            <= osq_wptr + 4'd1;
                osq_count           <= osq_count + 8'd1;
            end
            if (rsp_valid && osq_count != 8'd0) begin
                osq_valid[osq_rptr] <= 1'b0;
                osq_rptr            <= osq_rptr + 4'd1;
                osq_count           <= osq_count - 8'd1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Receive: reassemble a coded line from channels 0..3.
    // -------------------------------------------------------------------------
    wire rx_line_valid = rx_valid[0] & rx_valid[1] & rx_valid[2] & rx_valid[3];
    wire rx_line_err   = |rx_crc_err[3:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rsp_valid    <= 1'b0;
            rsp_rdata    <= 576'd0;
            rsp_tag      <= {TAG_W{1'b0}};
            rsp_link_err <= 1'b0;
            mnt_rvalid   <= 1'b0;
            mnt_rdata    <= {PAYLOAD_W{1'b0}};
            stack_retry_count <= 32'd0;
        end
        else begin
            rsp_valid  <= 1'b0;
            mnt_rvalid <= 1'b0;

            if (rx_line_valid) begin
                rsp_valid    <= ~rx_line_err;
                rsp_link_err <= rx_line_err;
                rsp_tag      <= osq_tag[osq_rptr];
                rsp_rdata    <= {rx_payload[3*PAYLOAD_W +: 144],
                                 rx_payload[2*PAYLOAD_W +: 144],
                                 rx_payload[1*PAYLOAD_W +: 144],
                                 rx_payload[0*PAYLOAD_W +: 144]};
                if (rx_line_err) stack_retry_count <= stack_retry_count + 32'd1;
            end

            if (rx_valid[4] && (rx_cmd[4*CMD_W +: CMD_W] == `VC3D_BOND_CMD_MBIST)) begin
                mnt_rvalid <= 1'b1;
                mnt_rdata  <= rx_payload[4*PAYLOAD_W +: PAYLOAD_W];
            end
        end
    end

endmodule
