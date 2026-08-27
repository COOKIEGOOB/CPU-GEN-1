/*
* CPU-GEN-1 : VCACHE-3D -- parallel CRC-16/CCITT over one bond beat.
*
* Polynomial x^16 + x^12 + x^5 + 1 (0x1021), init 0xFFFF, no reflection.
* The CRC covers the 144-bit payload plus the 4-bit command field of a beat.
* It is a transport check only: data integrity end-to-end is guaranteed by the
* SECDED code that travels with the payload.  The CRC exists so that a
* transient bond fault is retried instead of being reported as a UE.
*
* Implemented as an unrolled bit-serial LFSR (combinational), which the
* generated bond fabric replicates per channel.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps

module vc3d_bond_crc16 #(
    parameter DW = 148
) (
    input  wire [DW-1:0] data_i,
    output wire [15:0]   crc_o
);

    function [15:0] crc_step;
        input [15:0] crc;
        input        bit_in;
        reg          fb;
        begin
            fb = crc[15] ^ bit_in;
            crc_step = {crc[14:0], 1'b0};
            crc_step[0]  = fb;
            crc_step[5]  = crc[4]  ^ fb;
            crc_step[12] = crc[11] ^ fb;
        end
    endfunction

    reg [15:0] acc;
    integer i;
    always @* begin
        acc = 16'hffff;
        for (i = DW-1; i >= 0; i = i - 1) begin
            acc = crc_step(acc, data_i[i]);
        end
    end

    assign crc_o = acc;

endmodule
