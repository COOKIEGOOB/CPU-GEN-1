/*
* CPU-GEN-1 : VCACHE-3D -- three-stage pipelined interleave hash.
*
* The exact modulo-3 interleave needs a 42-bit population count, a small adder
* and three fold-and-subtract steps.  Combinationally that is ~545 ps in the
* SS corner -- far past a 333 ps cycle -- so the hash is pipelined:
*
*   S0 : split the line address into even/odd bit sets and popcount each
*        (two 21-input counts, carry-save)                     ~185 ps
*   S1 : total = even + 2*odd, then two mod-3 folds            ~190 ps
*   S2 : final fold + conditional subtract, slice decode       ~150 ps
*
* Latency is 3 cycles, fully pipelined (one request per cycle).  Those three
* cycles are hidden: they overlap the tag lookup of the previous request, and
* on a miss they are irrelevant next to the DRAM round trip.  A cache that
* used a power-of-two slice count would not need this, but it would also waste
* a third of the capacity -- 96 MiB is 3 x 32 MiB, and this is the price.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_addr_hash_pipe #(
    parameter PADDR_W = 48,
    parameter SET_W   = 15,
    parameter SLICES  = 3
) (
    input  wire                clk,
    input  wire                rst,

    input  wire                in_valid,
    input  wire [PADDR_W-1:0]  in_addr,
    input  wire [1:0]          mode,
    input  wire [1:0]          force_slice,

    output reg                 out_valid,
    output reg  [PADDR_W-1:0]  out_addr,
    output reg  [1:0]          out_slice,
    output reg  [SET_W-1:0]    out_set,
    output reg  [26:0]         out_tag
);

    localparam LA_W = PADDR_W - 6;

    // ------------------------------------------------------------------ S0
    wire [LA_W-1:0] la = in_addr[PADDR_W-1:6];

    reg [5:0] even_sum, odd_sum;
    integer i;
    always @* begin
        even_sum = 6'd0;
        odd_sum  = 6'd0;
        for (i = 0; i < LA_W; i = i + 1) begin
            if (i % 2 == 0) even_sum = even_sum + {5'd0, la[i]};
            else            odd_sum  = odd_sum  + {5'd0, la[i]};
        end
    end

    reg              s0_valid;
    reg [PADDR_W-1:0] s0_addr;
    reg [5:0]        s0_even, s0_odd;
    reg [1:0]        s0_mode, s0_force;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s0_valid <= 1'b0; s0_addr <= {PADDR_W{1'b0}};
            s0_even  <= 6'd0; s0_odd  <= 6'd0;
            s0_mode  <= 2'd0; s0_force <= 2'd0;
        end
        else begin
            s0_valid <= in_valid;
            s0_addr  <= in_addr;
            s0_even  <= even_sum;
            s0_odd   <= odd_sum;
            s0_mode  <= mode;
            s0_force <= force_slice;
        end
    end

    // ------------------------------------------------------------------ S1
    wire [6:0] total = {1'b0, s0_even} + {s0_odd, 1'b0};
    wire [6:0] fold1 = {2'd0, total[6:2]} + {5'd0, total[1:0]};
    wire [6:0] fold2 = {2'd0, fold1[6:2]} + {5'd0, fold1[1:0]};

    reg              s1_valid;
    reg [PADDR_W-1:0] s1_addr;
    reg [6:0]        s1_fold;
    reg [1:0]        s1_mode, s1_force;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s1_valid <= 1'b0; s1_addr <= {PADDR_W{1'b0}}; s1_fold <= 7'd0;
            s1_mode  <= 2'd0; s1_force <= 2'd0;
        end
        else begin
            s1_valid <= s0_valid;
            s1_addr  <= s0_addr;
            s1_fold  <= fold2;
            s1_mode  <= s0_mode;
            s1_force <= s0_force;
        end
    end

    // ------------------------------------------------------------------ S2
    wire [6:0] fold3 = {2'd0, s1_fold[6:2]} + {5'd0, s1_fold[1:0]};
    wire [6:0] red   = (fold3 >= 7'd3) ? (fold3 - 7'd3) : fold3;
    wire [1:0] mod3  = red[1:0];

    wire [LA_W-1:0]  la2   = s1_addr[PADDR_W-1:6];
    wire [SET_W-1:0] setix = la2[SET_W-1:0] ^ la2[2*SET_W-1:SET_W] ^
                             {{(2*SET_W-LA_W){1'b0}}, la2[LA_W-1:2*SET_W]};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out_valid <= 1'b0;
            out_addr  <= {PADDR_W{1'b0}};
            out_slice <= 2'd0;
            out_set   <= {SET_W{1'b0}};
            out_tag   <= 27'd0;
        end
        else begin
            out_valid <= s1_valid;
            out_addr  <= s1_addr;
            out_set   <= setix;
            out_tag   <= s1_addr[47:21];
            case (s1_mode)
                2'd0:    out_slice <= mod3;
                2'd1:    out_slice <= la2[1:0];
                2'd2:    out_slice <= mod3;
                default: out_slice <= s1_force;
            endcase
        end
    end

endmodule
