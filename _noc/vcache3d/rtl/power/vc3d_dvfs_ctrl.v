/*
* CPU-GEN-1 : VCACHE-3D -- DVFS / clock-gating / power-state controller.
*
* Owns the subsystem's operating point.  Inputs: the thermal throttle level,
* the measured access rate, and the firmware-requested level.  Output: the
* clock divider and voltage request for (a) the base-die logic and (b) the
* stacked array, which are separate rails.
*
* The two rails matter.  A stacked HD-SRAM array runs at a LOWER voltage and a
* LOWER maximum frequency than the base-die logic, so the design must be able
* to run the array at, e.g., 0.75 V / 2.0 GHz while the base-die controller
* runs at 0.95 V / 3.0 GHz.  Level tables below encode that split; the ratio
* is what the pd/openroad flow constrains through two clock groups.
*
* Idle policy: after IDLE_CYCLES with no access the array drops to retention
* (data preserved, ~10x lower leakage); the first access pays a programmable
* wake latency, tracked so that firmware can trade latency for power.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_dvfs_ctrl #(
    parameter IDLE_CYCLES = 32'd65536,
    parameter WAKE_CYCLES = 8'd64
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        access_active,
    input  wire [2:0]  throttle_level,
    input  wire        thermal_critical,
    input  wire [2:0]  fw_level_req,
    input  wire        fw_level_valid,
    input  wire        retention_enable,

    output reg  [2:0]  logic_level,
    output reg  [2:0]  array_level,
    output reg  [3:0]  logic_clk_div,
    output reg  [3:0]  array_clk_div,
    output reg  [7:0]  logic_vid,          // 5 mV per LSB above 0.50 V
    output reg  [7:0]  array_vid,
    output reg         array_retention,
    output reg         array_wake_busy,
    output reg  [2:0]  power_state,
    output reg  [31:0] retention_entries,
    output reg  [31:0] wake_stall_cycles
);

    // level -> {clk_div, vid} tables
    // logic rail : 3.60 / 3.40 / 3.20 / 3.00 / 2.60 / 2.20 / 1.80 / 1.20 GHz
    // array rail : 2.40 / 2.30 / 2.20 / 2.00 / 1.80 / 1.50 / 1.20 / 0.80 GHz
    function [11:0] logic_tbl;
        input [2:0] lvl;
        begin
            case (lvl)
                3'd0: logic_tbl = {4'd1, 8'd90};   // 0.95 V
                3'd1: logic_tbl = {4'd1, 8'd86};
                3'd2: logic_tbl = {4'd1, 8'd82};
                3'd3: logic_tbl = {4'd2, 8'd78};
                3'd4: logic_tbl = {4'd2, 8'd72};
                3'd5: logic_tbl = {4'd3, 8'd66};
                3'd6: logic_tbl = {4'd4, 8'd60};
                default: logic_tbl = {4'd6, 8'd54};
            endcase
        end
    endfunction

    function [11:0] array_tbl;
        input [2:0] lvl;
        begin
            case (lvl)
                3'd0: array_tbl = {4'd1, 8'd70};   // 0.75 V
                3'd1: array_tbl = {4'd1, 8'd68};
                3'd2: array_tbl = {4'd2, 8'd66};
                3'd3: array_tbl = {4'd2, 8'd64};
                3'd4: array_tbl = {4'd3, 8'd62};
                3'd5: array_tbl = {4'd4, 8'd60};
                3'd6: array_tbl = {4'd5, 8'd58};
                default: array_tbl = {4'd8, 8'd56};
            endcase
        end
    endfunction

    reg [31:0] idle_cnt;
    reg [7:0]  wake_cnt;
    reg [2:0]  req_level;

    always @* begin
        // the most restrictive of firmware request and thermal throttle wins
        req_level = fw_level_valid ? fw_level_req : 3'd0;
        if (throttle_level > req_level) req_level = throttle_level;
        if (thermal_critical)           req_level = 3'd7;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            logic_level       <= 3'd0;
            array_level       <= 3'd0;
            logic_clk_div     <= 4'd1;
            array_clk_div     <= 4'd1;
            logic_vid         <= 8'd90;
            array_vid         <= 8'd70;
            array_retention   <= 1'b0;
            array_wake_busy   <= 1'b0;
            power_state       <= `VC3D_PWR_ACTIVE;
            idle_cnt          <= 32'd0;
            wake_cnt          <= 8'd0;
            retention_entries <= 32'd0;
            wake_stall_cycles <= 32'd0;
        end
        else begin
            logic_level   <= req_level;
            array_level   <= req_level;
            {logic_clk_div, logic_vid} <= logic_tbl(req_level);
            {array_clk_div, array_vid} <= array_tbl(req_level);

            // ---- idle / retention -------------------------------------------
            if (access_active) begin
                idle_cnt <= 32'd0;
                if (array_retention) begin
                    array_retention <= 1'b0;
                    array_wake_busy <= 1'b1;
                    wake_cnt        <= WAKE_CYCLES;
                end
            end
            else if (!array_retention && retention_enable) begin
                if (idle_cnt >= IDLE_CYCLES) begin
                    array_retention   <= 1'b1;
                    retention_entries <= retention_entries + 32'd1;
                end
                else idle_cnt <= idle_cnt + 32'd1;
            end

            if (array_wake_busy) begin
                wake_stall_cycles <= wake_stall_cycles + 32'd1;
                if (wake_cnt == 8'd0) array_wake_busy <= 1'b0;
                else                  wake_cnt <= wake_cnt - 8'd1;
            end

            // ---- power state -------------------------------------------------
            if (thermal_critical)        power_state <= `VC3D_PWR_THROTTLE;
            else if (array_retention)    power_state <= `VC3D_PWR_RET;
            else if (access_active)      power_state <= `VC3D_PWR_ACTIVE;
            else                         power_state <= `VC3D_PWR_IDLE;
        end
    end

endmodule
