/*
* CPU-GEN-1 : VCACHE-3D -- generic single-port SRAM wrapper (base die).
*
* One place where every base-die array is declared, so that the SRAM compiler
* swap (pd/sram/sram_compiler_selection.md) is a one-file change:
*
*   `VC3D_SRAM_COMPILER : map to the foundry macro chosen for this array shape
*   `VC3D_FPGA          : map to a block-RAM friendly inference style
*   default             : behavioural model for simulation
*
* Byte/bit write masking is supported because the tag array does partial
* updates (state and LRU change far more often than the tag itself).
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps

module vc3d_sram_sp #(
    parameter AW = 15,
    parameter DW = 39,
    parameter USE_MASK = 1,
    // OUT_REG adds a second, external output register.  Wide arrays (the
    // 576-bit base data array) cannot get the macro access, the bank mux, the
    // way mux and the wire back to the ECC stage into one 333 ps cycle; the
    // extra register splits that path and costs one cycle of hit latency.
    parameter OUT_REG  = 0
) (
    input  wire           clk,
    input  wire           rst,
    input  wire           ce,
    input  wire           we,
    input  wire [AW-1:0]  addr,
    input  wire [DW-1:0]  wdata,
    input  wire [DW-1:0]  wmask,
    output reg  [DW-1:0]  rdata,
    output reg            rvalid
);

    localparam DEPTH = (1 << AW);

    reg [DW-1:0] mem [0:DEPTH-1];
    integer b;

    always @(posedge clk) begin
        if (ce && we) begin
            if (USE_MASK) begin
                for (b = 0; b < DW; b = b + 1)
                    if (wmask[b]) mem[addr][b] <= wdata[b];
            end
            else begin
                mem[addr] <= wdata;
            end
        end
    end

    reg [DW-1:0] rdata_int;
    reg          rvalid_int;

    always @(posedge clk) begin
        if (ce && !we) rdata_int <= mem[addr];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) rvalid_int <= 1'b0;
        else     rvalid_int <= ce & ~we;
    end

    generate
        if (OUT_REG) begin : g_out_reg
            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    rdata  <= {DW{1'b0}};
                    rvalid <= 1'b0;
                end
                else begin
                    rdata  <= rdata_int;
                    rvalid <= rvalid_int;
                end
            end
        end
        else begin : g_no_out_reg
            always @* begin
                rdata  = rdata_int;
                rvalid = rvalid_int;
            end
        end
    endgenerate

endmodule
