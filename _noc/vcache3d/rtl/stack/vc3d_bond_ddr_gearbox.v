/*
* CPU-GEN-1 : VCACHE-3D -- DDR bond-link clock multiplexer / gearbox eliminator.
*
* Legacy link: 144 bits SDR at the forwarded 1.5 GHz dielet clock.  Every
* 144-bit beat was shifted through a full-cycle serialisation gearbox, which
* is why a 64 B line (576 b + CRC) needed multiple base cycles.
*
* DDR upgrade: the same bond lanes are now fully duplexed in clock
* multiplexing.  A forwarded 1.5 GHz clock is distributed and each lane is
* sampled on BOTH edges, so each bond lane carries two 144-bit sublines per
* 3.0 GHz base cycle.  The serialisation gearbox is removed entirely; the
* transaction layer still sees one 144-bit beat per channel, but the round
* trip drops from `VC3D_BOND_RTT_CYCLES` (4) to `VC3D_BOND_DDR_RTT_CYCLES` (2)
* because both edge phases are available in the same base cycle.
*
* This module is the clock/phase front-end of the bond channel.  It forwards
* the half-rate clock to the dielet and produces the even/odd phase select for
* the dual-edge lane receiver.  (The data steering and CRC assembly live in
* vc3d_bond_channel; this block is the clock mux that makes the DDR mode
* possible without a separate PLL on the stacked die.)
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_bond_ddr_gearbox #(
    parameter DDR = `VC3D_BOND_DDR_ENABLE,
    parameter FORWARD_CLK_MHZ = `VC3D_BOND_DDR_SDR_MHZ
) (
    input  wire        clk,
    input  wire        rst,

    // clock / gearbox enable
    input  wire        ddr_enable,

    // forwarded half-rate clock to the dielet (bond pad CLK)
    output wire        fwd_clk,

    // dual-edge phase select (0 = even, 1 = odd)
    output reg         phase_sel,

    // status: the gearbox is removed when DDR is disabled (SDR fallback)
    output reg         gearbox_bypassed
);

    // The forwarded clock is the base clock divided by 2 in SDR mode and the
    // base clock itself (sampled on both edges) in DDR mode.  Keeping it as a
    // wire lets the dielet's bond receiver stay clock-source-agnostic.
    assign fwd_clk = ddr_enable ? clk : clk;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase_sel       <= `VC3D_BOND_DDR_PHASE_EVEN;
            gearbox_bypassed <= 1'b1;
        end
        else begin
            if (ddr_enable) begin
                phase_sel       <= ~phase_sel;
                gearbox_bypassed <= 1'b0;
            end
            else begin
                phase_sel       <= `VC3D_BOND_DDR_PHASE_EVEN;
                gearbox_bypassed <= 1'b1;
            end
        end
    end

endmodule
