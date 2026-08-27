/*
* CPU-GEN-1 : VCACHE-3D -- 64 B cache-line ECC encoder.
*
* A 512-bit line is protected as FOUR independent SECDED(128,9) sublines.
* Each subline is padded to 144 bits (128 data + 9 check + 1 poison + 6 spare)
* so that a subline exactly fills one hybrid-bond beat and one stacked-array
* word.  The spare bits carry a 4-bit "written" nibble and a 2-bit sequence
* number that let the scrubber distinguish never-written array content from a
* genuine uncorrectable error after power-on (all-zero data with all-zero
* check bits is a legal codeword, so a raw array read of an unwritten line
* would otherwise look clean).
*
* Why 4 x 128 rather than 1 x 512:
*   * 4 x SECDED(128,9) costs 36 check bits per line (7.0%) versus 11 bits
*     (2.1%) for a single SECDED(512,11), but it corrects up to four errors
*     per line as long as they land in different sublines.  A hard failure in
*     one bond lane, one bit line, or one subarray column therefore stays
*     correctable across the whole line -- that is the property that makes a
*     3D-stacked cache serviceable in the field.
*   * the XOR depth of a 128-bit code is one level shallower, which matters
*     because the decoder sits in the L3 hit path.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_ecc_line_enc (
    input  wire [511:0] line_i,
    input  wire         poison_i,
    input  wire [3:0]   written_i,     // per-subline "has been written" nibble
    input  wire [1:0]   seq_i,
    output wire [575:0] coded_o        // 4 x 144
);

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_subline
            wire [127:0] data  = line_i[g*128 +: 128];
            wire [8:0]   check;

            vc3d_secded_enc_128 u_enc (
                .data_i  (data),
                .check_o (check)
            );

            // 144-bit stacked-array / bond-beat word layout:
            //   [127:0]   data
            //   [136:128] SECDED check
            //   [137]     poison
            //   [141:138] written nibble
            //   [143:142] sequence
            assign coded_o[g*144 +: 144] = { seq_i, written_i, poison_i, check, data };
        end
    endgenerate

endmodule
