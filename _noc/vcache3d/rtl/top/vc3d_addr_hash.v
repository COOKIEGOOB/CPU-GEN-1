/*
* CPU-GEN-1 : VCACHE-3D -- address interleave hash for a 3-slice cache.
*
* The hard part of a 96 MiB = 3 x 32 MiB cache is that THREE IS NOT A POWER OF
* TWO.  A naive `slice = addr[7:6] % 3` wastes a quarter of the address space
* and creates a systematic hot slice.  This module implements an exact,
* uniform, single-cycle modulo-3 interleave:
*
*   mod-3 of a binary number
*     2^k mod 3 = 1 for even k, 2 for odd k
*   therefore
*     addr mod 3 = ( sum(bits at even positions)
*                  + 2 * sum(bits at odd positions) ) mod 3
*   which is a two-population-count + small adder tree + one 6-bit mod-3
*   reduction -- about 4 levels of logic on a 42-bit line address, comfortably
*   inside one cycle at the target frequency, and EXACTLY uniform: every third
*   consecutive line lands on a different slice, so a linear stream is
*   perfectly balanced across all three slices.
*
* Set index: the low set bits alone would correlate with the slice choice
* (both derive from the same address bits), so the set index is a XOR fold of
* three address fields.  This is the standard "hashed set index" that stops a
* 2 MiB-strided access pattern from mapping onto one set.
*
* Alternative modes (CSR-selectable) for bring-up and for 1/2/4-slice builds:
*   MODE_MOD3   : the mod-3 interleave described above (default)
*   MODE_LOW2   : slice = addr[7:6] (power-of-two, 4th slice unused)
*   MODE_HASH   : slice = XOR-folded hash mod 3 (decorrelates pathological
*                 strides that happen to be multiples of 3 lines)
*   MODE_DIRECT : slice = a CSR-provided constant (single-slice debug)
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_addr_hash #(
    parameter PADDR_W = 48,
    parameter SET_W   = 15,
    parameter SLICES  = 3
) (
    input  wire [PADDR_W-1:0] addr,
    input  wire [1:0]         mode,
    input  wire [1:0]         force_slice,
    output wire [1:0]         slice_sel,
    output wire [SET_W-1:0]   set_index,
    output wire [26:0]        tag_out
);

    localparam MODE_MOD3   = 2'd0;
    localparam MODE_LOW2   = 2'd1;
    localparam MODE_HASH   = 2'd2;
    localparam MODE_DIRECT = 2'd3;

    // line address (drop the 6-bit line offset)
    wire [PADDR_W-7:0] la = addr[PADDR_W-1:6];
    localparam LA_W = PADDR_W - 6;      // 42

    // ---------------------------------------------------------------------
    // population counts of even- and odd-indexed bits
    // ---------------------------------------------------------------------
    reg [5:0] even_sum, odd_sum;
    integer i;
    always @* begin
        even_sum = 6'd0;
        odd_sum  = 6'd0;
        for (i = 0; i < LA_W; i = i + 1) begin
            if (i[0] == 1'b0) even_sum = even_sum + {5'd0, la[i]};
            else              odd_sum  = odd_sum  + {5'd0, la[i]};
        end
    end

    // total = even + 2*odd  (max 21 + 42 = 63, fits in 6 bits)
    wire [6:0] total = {1'b0, even_sum} + {odd_sum, 1'b0};

    // 7-bit modulo 3 by folding: x mod 3 = (x>>2 + (x&3)) mod 3, iterated
    function [1:0] mod3;
        input [6:0] x;
        reg [6:0] t;
        begin
            t = x;
            t = {2'd0, t[6:2]} + {5'd0, t[1:0]};   // <= 33
            t = {2'd0, t[6:2]} + {5'd0, t[1:0]};   // <= 10
            t = {2'd0, t[6:2]} + {5'd0, t[1:0]};   // <= 4
            if (t >= 7'd3) t = t - 7'd3;
            mod3 = t[1:0];
        end
    endfunction

    wire [1:0] mod3_sel = mod3(total);

    // ---------------------------------------------------------------------
    // XOR-folded hash variant
    // ---------------------------------------------------------------------
    wire [11:0] fold = la[11:0] ^ la[23:12] ^ la[35:24] ^ {6'd0, la[41:36]};
    wire [1:0]  hash_sel = mod3({1'b0, fold[5:0]} + {1'b0, fold[11:6]});

    assign slice_sel = (mode == MODE_MOD3)   ? mod3_sel :
                       (mode == MODE_LOW2)   ? la[1:0]  :
                       (mode == MODE_HASH)   ? hash_sel :
                                               force_slice;

    // ---------------------------------------------------------------------
    // Hashed set index and tag
    // ---------------------------------------------------------------------
    wire [SET_W-1:0] raw_set = la[SET_W-1:0];
    wire [SET_W-1:0] fold_a  = la[2*SET_W-1:SET_W];
    wire [SET_W-1:0] fold_b  = {{(2*SET_W-LA_W){1'b0}}, la[LA_W-1:2*SET_W]};

    assign set_index = raw_set ^ fold_a ^ fold_b;
    assign tag_out   = addr[47:21];

endmodule
