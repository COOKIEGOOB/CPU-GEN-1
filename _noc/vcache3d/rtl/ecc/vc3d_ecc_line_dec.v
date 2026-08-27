/*
* CPU-GEN-1 : VCACHE-3D -- 64 B cache-line ECC decoder / corrector.
*
* Consumes the four 144-bit words produced by vc3d_ecc_line_enc, corrects one
* bit per subline, and reports a per-line summary:
*
*   ce_o          any subline corrected a single-bit error
*   ue_o          any subline saw an uncorrectable (even, non-zero) syndrome,
*                 or a poisoned subline was consumed
*   ce_vec_o/ue_vec_o   per-subline detail, used by the CE tracker to decide
*                 whether a subline is degrading and should be repaired
*   syndrome_o    concatenated syndromes, logged verbatim so that firmware can
*                 map a repeated syndrome back to a physical bit line
*
* The corrector is combinational; the consumer registers `line_o`.  The whole
* decode is one XOR tree plus a 9-bit compare per bit, which fits the same
* cycle as the data-array output mux at the target frequency (see
* pd/reports/timing_summary.rpt).
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_ecc_line_dec (
    input  wire [575:0] coded_i,
    output wire [511:0] line_o,
    output wire         ce_o,
    output wire         ue_o,
    output wire [3:0]   ce_vec_o,
    output wire [3:0]   ue_vec_o,
    output wire [3:0]   ce_in_check_vec_o,
    output wire [35:0]  syndrome_o,
    output wire         poison_o,
    output wire [3:0]   written_o,
    output wire [1:0]   seq_o
);

    wire [3:0] poison_vec;
    wire [3:0] written_bits;
    wire [1:0] seq_w [0:3];

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_subline
            wire [143:0] word    = coded_i[g*144 +: 144];
            wire [127:0] data    = word[127:0];
            wire [8:0]   check   = word[136:128];
            wire         poison  = word[137];
            wire [3:0]   written = word[141:138];
            wire [1:0]   seq     = word[143:142];

            wire [127:0] corrected;
            wire [8:0]   syndrome;

            vc3d_secded_dec_128 u_dec (
                .data_i       (data),
                .check_i      (check),
                .poison_i     (poison),
                .data_o       (corrected),
                .syndrome_o   (syndrome),
                .ce_o         (ce_vec_o[g]),
                .ue_o         (ue_vec_o[g]),
                .ce_in_check_o(ce_in_check_vec_o[g]),
                .poison_o     (poison_vec[g])
            );

            assign line_o[g*128 +: 128]  = corrected;
            assign syndrome_o[g*9 +: 9]  = syndrome;
            assign written_bits[g]       = written[g];
            assign seq_w[g]              = seq;
        end
    endgenerate

    assign ce_o      = |ce_vec_o;
    assign ue_o      = |ue_vec_o;
    assign poison_o  = |poison_vec;
    assign written_o = written_bits;
    assign seq_o     = seq_w[0];

endmodule
