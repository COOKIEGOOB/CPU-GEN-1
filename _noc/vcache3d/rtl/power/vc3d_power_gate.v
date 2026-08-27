/*
* CPU-GEN-1 : VCACHE-3D -- per-bank power-gating sequencer.
*
* Collapsing a stacked SRAM bank is not a single signal: an abrupt turn-on of
* 32 banks would produce a di/dt event that the base-die power delivery cannot
* absorb through the bond pads (the stacked die has no package-level
* decoupling of its own -- all of its current comes through the bond field).
* The sequencer therefore ramps banks one at a time with a programmable
* inter-bank delay, and uses a two-stage (weak header, then strong header)
* turn-on per bank.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_power_gate #(
    parameter BANKS      = 32,
    parameter BANK_W     = 5,
    parameter WEAK_CYC   = 8'd16,
    parameter STRONG_CYC = 8'd8,
    parameter GAP_CYC    = 8'd4
) (
    input  wire              clk,
    input  wire              rst,

    input  wire [BANKS-1:0]  bank_on_req,       // 1 = powered
    input  wire [BANKS-1:0]  bank_ret_req,      // 1 = retention
    output reg  [BANKS-1:0]  bank_sleep,
    output reg  [BANKS-1:0]  bank_deep_sleep,
    output reg  [BANKS-1:0]  bank_retention,
    output reg  [BANKS-1:0]  header_weak,
    output reg  [BANKS-1:0]  header_strong,
    output reg               seq_busy,
    output reg  [31:0]       gate_events,
    output reg  [15:0]       banks_powered
);

    reg [BANK_W-1:0] ptr;
    reg [7:0]        timer;
    reg [1:0]        st;
    localparam S_SCAN = 2'd0, S_WEAK = 2'd1, S_STRONG = 2'd2, S_GAP = 2'd3;

    integer i;
    reg [15:0] cnt;
    always @* begin
        cnt = 16'd0;
        for (i = 0; i < BANKS; i = i + 1) if (header_strong[i]) cnt = cnt + 16'd1;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ptr             <= {BANK_W{1'b0}};
            timer           <= 8'd0;
            st              <= S_SCAN;
            bank_sleep      <= {BANKS{1'b1}};
            bank_deep_sleep <= {BANKS{1'b0}};
            bank_retention  <= {BANKS{1'b0}};
            header_weak     <= {BANKS{1'b0}};
            header_strong   <= {BANKS{1'b0}};
            seq_busy        <= 1'b0;
            gate_events     <= 32'd0;
            banks_powered   <= 16'd0;
        end
        else begin
            banks_powered  <= cnt;
            bank_retention <= bank_ret_req;

            case (st)
                S_SCAN: begin
                    seq_busy <= 1'b0;
                    if (bank_on_req[ptr] && !header_strong[ptr]) begin
                        seq_busy         <= 1'b1;
                        header_weak[ptr] <= 1'b1;
                        timer            <= WEAK_CYC;
                        gate_events      <= gate_events + 32'd1;
                        st               <= S_WEAK;
                    end
                    else if (!bank_on_req[ptr] && header_strong[ptr]) begin
                        header_strong[ptr]   <= 1'b0;
                        header_weak[ptr]     <= 1'b0;
                        bank_sleep[ptr]      <= 1'b1;
                        bank_deep_sleep[ptr] <= ~bank_ret_req[ptr];
                        gate_events          <= gate_events + 32'd1;
                        ptr                  <= ptr + 1'b1;
                    end
                    else begin
                        ptr <= ptr + 1'b1;
                    end
                end
                S_WEAK: begin
                    if (timer == 8'd0) begin
                        header_strong[ptr] <= 1'b1;
                        timer              <= STRONG_CYC;
                        st                 <= S_STRONG;
                    end
                    else timer <= timer - 8'd1;
                end
                S_STRONG: begin
                    if (timer == 8'd0) begin
                        bank_sleep[ptr]      <= 1'b0;
                        bank_deep_sleep[ptr] <= 1'b0;
                        timer                <= GAP_CYC;
                        st                   <= S_GAP;
                    end
                    else timer <= timer - 8'd1;
                end
                S_GAP: begin
                    if (timer == 8'd0) begin
                        ptr <= ptr + 1'b1;
                        st  <= S_SCAN;
                    end
                    else timer <= timer - 8'd1;
                end
            endcase
        end
    end

endmodule
