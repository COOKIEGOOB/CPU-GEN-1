/*
* CPU-GEN-1 : VCACHE-3D -- direct SRAM macro slice (leaf cell).
*
* This is the physical unit produced by the SRAM compiler after the Dielet
* Frequency Upgrade.  Instead of a single 1024 x 148 high-density macro with a
* 465 ps access time (which capped the dielet at 1.5 GHz), each 1024 x 148
* subarray is banked into VC3D_STACK_MACRO_SLICES (default 4) narrower macros
* of VC3D_STACK_MACRO_DEPTH x 148 bits with divided local bitlines:
*
*     1024 x 148  macro  (465 ps access, 1.5 GHz cap)
*        |
*        +-- 512 x 148 macro x 2   (local bitlines, ~330 ps, 1.9 GHz)
*        |
*        +-- 256 x 148 macro x 4   (divided local bitlines, ~260 ps, 2.4 GHz)
*
* Divided local bitlines cut the bitline capacitance roughly with the number of
* slices, which is what moves the array access from 465 ps to ~260 ps.  The
* dielet can then run at `VC3D_STACK_DIELET_CLOCK_MHZ` (2.0 - 2.4 GHz) while the
* base die keeps the 3.0 GHz core clock, dropping the stacked-access latency
* from +9 cycles to +4/+5 cycles.
*
* Each cell instance is kept explicitly expanded in the generated
* rtl/stack/vc3d_stack_macro_slice_array.v so the physical floorplan can place
* and power-gate every local-bitline array independently, exactly as the
* production 3D-SRAM floorplan expects.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_stack_macro_cell #(
    parameter SLICE_ID  = 0,
    parameter DEPTH     = `VC3D_STACK_MACRO_DEPTH,   // 256
    parameter AW        = `VC3D_STACK_MACRO_AW,      // 8
    parameter DW        = 144,
    parameter ACCESS_PS = `VC3D_STACK_MACRO_ACCESS_PS,
    parameter OUT_REG   = 1
) (
    input  wire                clk,
    input  wire                rst,

    // functional single-port cell
    input  wire                ce,
    input  wire                we,
    input  wire [AW-1:0]       addr,
    input  wire [DW-1:0]       wdata,
    input  wire [DW-1:0]       wbit_mask,
    output wire [DW-1:0]       rdata,
    output wire                rvalid,

    // divided local-bitline control (physical floorplan hint)
    input  wire                local_bl_sel,
    output reg                 cell_active,

    // micro-architectural status
    output reg  [15:0]         access_count,
    output reg  [15:0]         stall_count
);

    // -------------------------------------------------------------------------
    // The divided local bitline is the key timing feature: the global bitline
    // is split at the slice boundary, so each cell only drives
    // DEPTH/DW-equivalent local wires.  The parameter ACCESS_PS is consumed by
    // pd/scripts/timing_model.py; keeping it here lets the RTL and the STA
    // model stay in lock-step.
    // -------------------------------------------------------------------------
    localparam [15:0] ACCESS_TARGET = ACCESS_PS[15:0];

    // -------------------------------------------------------------------------
    // Behavioural storage (matched to the compiler cell, OUT_REG flavour).
    // -------------------------------------------------------------------------
    reg [DW-1:0] mem [0:DEPTH-1];
    reg [DW-1:0] rd_q;
    reg          rd_val_q;

    wire do_access = ce & local_bl_sel;
    wire do_write  = do_access & we;
    wire do_read   = do_access & ~we;

    integer i;
    always @(posedge clk) begin
        if (do_write) begin
            for (i = 0; i < DW; i = i + 1)
                if (wbit_mask[i]) mem[addr][i] <= wdata[i];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_q    <= {DW{1'b0}};
            rd_val_q <= 1'b0;
        end
        else begin
            rd_q     <= mem[addr];
            rd_val_q <= do_read;
        end
    end

    assign rdata  = rd_q;
    assign rvalid = rd_val_q;

    // -------------------------------------------------------------------------
    // Activity / stall accounting (per-slice local bitline marginality).
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cell_active  <= 1'b0;
            access_count <= 16'd0;
            stall_count  <= 16'd0;
        end
        else begin
            cell_active <= do_access;
            if (do_access && access_count != 16'hffff) access_count <= access_count + 16'd1;
            if (do_access && !rd_val_q && stall_count != 16'hffff) stall_count <= stall_count + 16'd1;
        end
    end

endmodule
