/*
* CPU-GEN-1 : VCACHE-3D -- single hybrid-bond lane (base die <-> cache die).
*
* A hybrid-bond lane is a direct Cu-Cu pad pair at ~9 um pitch.  Electrically
* it is a very short, very low-capacitance wire (a few fF), so it needs no
* SerDes and no termination -- but it DOES need:
*
*   * per-lane deskew   : bond-pad RC and base/stack clock-tree mismatch give
*                         picosecond-class skew that accumulates across a
*                         160-lane channel; each lane owns a 0..3 beat delay
*                         line plus a fine 4-step phase code;
*   * per-lane repair   : a single defective pad must not kill the dielet, so
*                         lanes can be marked dead and their traffic shifted
*                         onto the channel's spare-lane pool;
*   * per-lane margining: an eye/BER monitor samples the lane against a known
*                         training pattern so field firmware can retire a
*                         degrading pad before it produces uncorrectable data.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps

module vc3d_bond_lane #(
    parameter LANE_ID       = 0,
    parameter MAX_DESKEW    = 4      // beats of programmable delay
) (
    input  wire        clk,
    input  wire        rst,

    // transmit side ----------------------------------------------------------
    input  wire        tx_bit,
    input  wire        tx_en,
    output wire        pad_out,
    output wire        pad_oe,

    // receive side -----------------------------------------------------------
    input  wire        pad_in,
    output wire        rx_bit,

    // configuration ----------------------------------------------------------
    input  wire        lane_enable,
    input  wire        lane_dead,          // marked bad by repair/BIST
    input  wire [1:0]  deskew_beats,
    input  wire [1:0]  deskew_phase,       // fine code, consumed by the PHY cell
    input  wire        invert,             // pad polarity swap (routing aid)

    // training / margining ---------------------------------------------------
    input  wire        train_en,
    input  wire        train_expect,
    output reg         train_error,
    output reg [15:0]  error_count,
    output reg [15:0]  sample_count,
    output wire        lane_healthy
);

    // -------------------------------------------------------------------------
    // Transmit: optional polarity inversion, gated by enable/dead.
    // -------------------------------------------------------------------------
    wire tx_active = lane_enable & ~lane_dead;
    assign pad_out = tx_active ? (tx_bit ^ invert) : 1'b0;
    assign pad_oe  = tx_active & tx_en;

    // -------------------------------------------------------------------------
    // Receive: programmable 0..MAX_DESKEW-1 beat delay line.
    // -------------------------------------------------------------------------
    reg [MAX_DESKEW-1:0] rx_pipe;
    always @(posedge clk or posedge rst) begin
        if (rst) rx_pipe <= {MAX_DESKEW{1'b0}};
        else     rx_pipe <= {rx_pipe[MAX_DESKEW-2:0], (pad_in ^ invert)};
    end

    wire rx_raw = (deskew_beats == 2'd0) ? (pad_in ^ invert) : rx_pipe[deskew_beats-1];
    assign rx_bit = tx_active ? rx_raw : 1'b0;

    // -------------------------------------------------------------------------
    // Training comparison and BER accumulation.
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            train_error  <= 1'b0;
            error_count  <= 16'd0;
            sample_count <= 16'd0;
        end
        else if (train_en) begin
            train_error <= (rx_raw != train_expect);
            if (rx_raw != train_expect && error_count != 16'hffff)
                error_count <= error_count + 16'd1;
            if (sample_count != 16'hffff)
                sample_count <= sample_count + 16'd1;
        end
        else begin
            train_error <= 1'b0;
        end
    end

    // A lane is healthy when it is enabled, not retired, and has seen enough
    // clean training samples (the repair controller uses this vector directly).
    assign lane_healthy = lane_enable & ~lane_dead &
                          (error_count == 16'd0) & (sample_count != 16'd0);

endmodule
