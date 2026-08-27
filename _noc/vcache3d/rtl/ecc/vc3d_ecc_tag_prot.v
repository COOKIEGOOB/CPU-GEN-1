/*
* CPU-GEN-1 : VCACHE-3D -- tag + coherence-state ECC wrapper.
*
* The tag array is far smaller than the data array but a tag error is far more
* dangerous: a flipped tag bit turns a miss into a false hit (silent data
* corruption) and a flipped state bit breaks coherence.  Production caches
* therefore protect tags at least as strongly as data.
*
* Layout protected here (32 bits, SECDED(32,7)):
*     [26:0]  tag
*     [29:27] coherence state (I / SC / UC / UD / SD / reserved)
*     [30]    valid
*     [31]    dirty
*
* On an uncorrectable tag error the wrapper asserts `force_miss_o`, which the
* pipeline treats as "this way is invalid": the line is dropped (if clean) or
* poisoned and reported (if dirty).  This converts silent corruption into a
* contained, reported event -- the behaviour a server part must have.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_ecc_tag_prot (
    input  wire        clk,
    input  wire        rst,

    // encode side
    input  wire [26:0] tag_i,
    input  wire [2:0]  state_i,
    input  wire        valid_i,
    input  wire        dirty_i,
    output wire [38:0] tag_word_o,     // 32 payload + 7 check

    // decode side
    input  wire [38:0] tag_word_i,
    output wire [26:0] tag_o,
    output wire [2:0]  state_o,
    output wire        valid_o,
    output wire        dirty_o,
    output wire        ce_o,
    output wire        ue_o,
    output wire        force_miss_o,
    output wire [6:0]  syndrome_o
);

    // ---------------- encode ----------------
    wire [31:0] enc_payload = {dirty_i, valid_i, state_i, tag_i};
    wire [6:0]  enc_check;

    vc3d_secded_enc_32 u_enc (
        .data_i  (enc_payload),
        .check_o (enc_check)
    );

    assign tag_word_o = {enc_check, enc_payload};

    // ---------------- decode ----------------
    wire [31:0] dec_payload = tag_word_i[31:0];
    wire [6:0]  dec_check   = tag_word_i[38:32];
    wire [31:0] corrected;

    vc3d_secded_dec_32 u_dec (
        .data_i        (dec_payload),
        .check_i       (dec_check),
        .poison_i      (1'b0),
        .data_o        (corrected),
        .syndrome_o    (syndrome_o),
        .ce_o          (ce_o),
        .ue_o          (ue_o),
        .ce_in_check_o (),
        .poison_o      ()
    );

    assign tag_o        = corrected[26:0];
    assign state_o      = corrected[29:27];
    assign valid_o      = corrected[30] & ~ue_o;
    assign dirty_o      = corrected[31];
    assign force_miss_o = ue_o;

endmodule
