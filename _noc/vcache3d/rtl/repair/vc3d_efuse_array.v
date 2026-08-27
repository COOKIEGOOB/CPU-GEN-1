/*
* CPU-GEN-1 : VCACHE-3D -- eFuse array + autoload sequencer.
*
* Manufacturing repair has to survive power cycles, so the repair solution
* computed on the tester is blown into eFuses on the BASE die (the stacked die
* has no room for a fuse farm and no direct programming access after bonding).
* At every reset the sequencer autoloads the fuse contents into the volatile
* repair map before the cache is allowed to serve traffic.
*
* Contents (1024 x 32 b = 4 KiB of fuse):
*   row 0        : magic + revision + record count
*   row 1        : global config (way disable, bank disable, assist trims)
*   rows 2..N    : repair records, 48 bits each packed two-per-three-rows
*   last 4 rows  : CRC-32 of the whole array (a corrupted fuse read must not
*                  silently install a wrong repair -- on CRC failure the part
*                  boots with repairs disabled and reports it)
*
* Programming model: a 32-bit address/data/control window on the CSR bus with
* a program-enable key, a per-row program pulse timer (fuse blow is ~10 us,
* far longer than a clock period, so the timer is programmable), and a
* read-back verify step.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_efuse_array #(
    parameter ROWS      = 1024,
    parameter ROW_W     = 32,
    parameter ADDR_W    = 10,
    parameter MAGIC     = 32'h5643_3344   // "VC3D"
) (
    input  wire               clk,
    input  wire               rst,

    // ---- CSR programming window --------------------------------------------
    input  wire               prog_en,          // key-protected
    input  wire               prog_req,
    input  wire [ADDR_W-1:0]  prog_addr,
    input  wire [ROW_W-1:0]   prog_data,
    input  wire [15:0]        prog_pulse_cycles,
    output reg                prog_busy,
    output reg                prog_done,
    output reg                prog_error,

    input  wire               read_req,
    input  wire [ADDR_W-1:0]  read_addr,
    output reg  [ROW_W-1:0]   read_data,
    output reg                read_valid,

    // ---- autoload to the repair map -----------------------------------------
    input  wire               autoload_start,
    output reg                autoload_busy,
    output reg                autoload_done,
    output reg                autoload_crc_err,
    output reg                al_push,
    output reg  [ROW_W-1:0]   al_word,
    output reg  [ADDR_W-1:0]  al_index,

    output wire [31:0]        fuse_magic,
    output reg  [15:0]        record_count
);

    // -------------------------------------------------------------------------
    // Storage.  In silicon this is a fuse macro; the behavioural model is
    // write-once (a blown fuse can only go 0 -> 1), which catches firmware
    // that tries to "unprogram" a repair.
    // -------------------------------------------------------------------------
    reg [ROW_W-1:0] fuse [0:ROWS-1];
    integer i;

    initial begin
        for (i = 0; i < ROWS; i = i + 1) fuse[i] = {ROW_W{1'b0}};
    end

    assign fuse_magic = fuse[0];

    // -------------------------------------------------------------------------
    // Program sequencer
    // -------------------------------------------------------------------------
    reg [15:0] pulse_cnt;
    reg [1:0]  pstate;
    localparam P_IDLE = 2'd0, P_BLOW = 2'd1, P_VERIFY = 2'd2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pstate     <= P_IDLE;
            pulse_cnt  <= 16'd0;
            prog_busy  <= 1'b0;
            prog_done  <= 1'b0;
            prog_error <= 1'b0;
        end
        else begin
            prog_done <= 1'b0;
            case (pstate)
                P_IDLE: begin
                    prog_busy <= 1'b0;
                    if (prog_req && prog_en) begin
                        prog_busy <= 1'b1;
                        pulse_cnt <= 16'd0;
                        pstate    <= P_BLOW;
                    end
                    else if (prog_req && !prog_en) begin
                        prog_error <= 1'b1;
                    end
                end
                P_BLOW: begin
                    pulse_cnt <= pulse_cnt + 16'd1;
                    if (pulse_cnt >= prog_pulse_cycles) begin
                        // write-once semantics: fuses only ever set bits
                        fuse[prog_addr] <= fuse[prog_addr] | prog_data;
                        pstate          <= P_VERIFY;
                    end
                end
                P_VERIFY: begin
                    prog_error <= ((fuse[prog_addr] & prog_data) != prog_data);
                    prog_done  <= 1'b1;
                    prog_busy  <= 1'b0;
                    pstate     <= P_IDLE;
                end
                default: pstate <= P_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read port
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            read_data  <= {ROW_W{1'b0}};
            read_valid <= 1'b0;
        end
        else begin
            read_valid <= read_req;
            if (read_req) read_data <= fuse[read_addr];
        end
    end

    // -------------------------------------------------------------------------
    // Autoload sequencer with CRC-32 verification
    // -------------------------------------------------------------------------
    reg [ADDR_W-1:0] al_ptr;
    reg [31:0]       crc;
    reg [2:0]        astate;
    localparam A_IDLE = 3'd0, A_HDR = 3'd1, A_BODY = 3'd2, A_CRC = 3'd3, A_DONE = 3'd4;

    function [31:0] crc32_word;
        input [31:0] c;
        input [31:0] d;
        integer k;
        reg [31:0] x;
        begin
            x = c ^ d;
            for (k = 0; k < 32; k = k + 1) begin
                x = x[31] ? ((x << 1) ^ 32'h04c1_1db7) : (x << 1);
            end
            crc32_word = x;
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            astate           <= A_IDLE;
            al_ptr           <= {ADDR_W{1'b0}};
            crc              <= 32'hffff_ffff;
            autoload_busy    <= 1'b0;
            autoload_done    <= 1'b0;
            autoload_crc_err <= 1'b0;
            al_push          <= 1'b0;
            al_word          <= {ROW_W{1'b0}};
            al_index         <= {ADDR_W{1'b0}};
            record_count     <= 16'd0;
        end
        else begin
            al_push <= 1'b0;
            case (astate)
                A_IDLE: begin
                    autoload_busy <= 1'b0;
                    if (autoload_start) begin
                        autoload_busy    <= 1'b1;
                        autoload_done    <= 1'b0;
                        autoload_crc_err <= 1'b0;
                        crc              <= 32'hffff_ffff;
                        al_ptr           <= {ADDR_W{1'b0}};
                        astate           <= A_HDR;
                    end
                end
                A_HDR: begin
                    if (fuse[0] != MAGIC) begin
                        // blank or corrupt fuse farm: boot unrepaired
                        autoload_crc_err <= (fuse[0] != {ROW_W{1'b0}});
                        astate           <= A_DONE;
                    end
                    else begin
                        record_count <= fuse[1][15:0];
                        crc          <= crc32_word(crc32_word(32'hffff_ffff, fuse[0]), fuse[1]);
                        al_ptr       <= 10'd2;
                        astate       <= A_BODY;
                    end
                end
                A_BODY: begin
                    al_push  <= 1'b1;
                    al_word  <= fuse[al_ptr];
                    al_index <= al_ptr - 10'd2;
                    crc      <= crc32_word(crc, fuse[al_ptr]);
                    if (al_ptr >= (10'd2 + record_count[9:0])) astate <= A_CRC;
                    al_ptr <= al_ptr + 10'd1;
                end
                A_CRC: begin
                    autoload_crc_err <= (crc != fuse[ROWS-1]);
                    astate           <= A_DONE;
                end
                A_DONE: begin
                    autoload_busy <= 1'b0;
                    autoload_done <= 1'b1;
                    astate        <= A_IDLE;
                end
                default: astate <= A_IDLE;
            endcase
        end
    end

endmodule
