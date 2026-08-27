/*
* CPU-GEN-1 : VCACHE-3D -- two-stage pipelined 64 B line ECC decoder.
*
* Same code as vc3d_ecc_line_dec, split across a register boundary:
*
*   stage 1 : four SECDED(128,9) syndrome XOR trees        (~124 ps)
*   ------ pipeline register: syndrome, data, poison ------
*   stage 2 : 1-of-128 correction compare + correction XOR (~90 ps)
*
* The combinational decoder is 379 ps in the SS corner, which does not fit a
* 333 ps cycle; split, both halves fit with >50 ps of margin
* (pd/scripts/timing_model.py, paths "ECC stage 1/2").  The cost is one extra
* cycle of L3 hit latency, which is the correct trade: an unpipelined ECC would
* have forced the whole cache down to ~2.6 GHz.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_ecc_line_dec_pipe (
    input  wire         clk,
    input  wire         rst,

    input  wire         in_valid,
    input  wire [575:0] coded_i,

    output reg          out_valid,
    output reg  [511:0] line_o,
    output reg          ce_o,
    output reg          ue_o,
    output reg  [3:0]   ce_vec_o,
    output reg  [3:0]   ue_vec_o,
    output reg  [35:0]  syndrome_o,
    output reg          poison_o,
    output reg  [3:0]   written_o
);

    // ---------------- stage 1 : syndromes ----------------
    wire [8:0]  s1_syn [0:3];
    wire [3:0]  s1_poison;

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_syn
            wire [143:0] word = coded_i[g*144 +: 144];
            vc3d_secded_syn_128 u_syn (
                .data_i     (word[127:0]),
                .check_i    (word[136:128]),
                .syndrome_o (s1_syn[g])
            );
            assign s1_poison[g] = word[137];
        end
    endgenerate

    reg          q_valid;
    reg [575:0]  q_coded;
    reg [35:0]   q_syn;
    reg [3:0]    q_poison;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            q_valid  <= 1'b0;
            q_coded  <= 576'd0;
            q_syn    <= 36'd0;
            q_poison <= 4'd0;
        end
        else begin
            q_valid  <= in_valid;
            q_coded  <= coded_i;
            q_syn    <= {s1_syn[3], s1_syn[2], s1_syn[1], s1_syn[0]};
            q_poison <= s1_poison;
        end
    end

    // ---------------- stage 2 : correction ----------------
    wire [127:0] s2_data   [0:3];
    wire [3:0]   s2_ce, s2_ue, s2_poison;

    generate
        for (g = 0; g < 4; g = g + 1) begin : g_cor
            vc3d_secded_cor_128 u_cor (
                .data_i     (q_coded[g*144 +: 128]),
                .syndrome_i (q_syn[g*9 +: 9]),
                .poison_i   (q_poison[g]),
                .data_o     (s2_data[g]),
                .ce_o       (s2_ce[g]),
                .ue_o       (s2_ue[g]),
                .poison_o   (s2_poison[g])
            );
        end
    endgenerate

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out_valid  <= 1'b0;
            line_o     <= 512'd0;
            ce_o       <= 1'b0;
            ue_o       <= 1'b0;
            ce_vec_o   <= 4'd0;
            ue_vec_o   <= 4'd0;
            syndrome_o <= 36'd0;
            poison_o   <= 1'b0;
            written_o  <= 4'd0;
        end
        else begin
            out_valid  <= q_valid;
            line_o     <= {s2_data[3], s2_data[2], s2_data[1], s2_data[0]};
            ce_o       <= |s2_ce;
            ue_o       <= |s2_ue;
            ce_vec_o   <= s2_ce;
            ue_vec_o   <= s2_ue;
            syndrome_o <= q_syn;
            poison_o   <= |s2_poison;
            written_o  <= {q_coded[3*144+141], q_coded[2*144+141],
                           q_coded[1*144+141], q_coded[0*144+141]};
        end
    end

endmodule
