/*
* CPU-GEN-1 : VCACHE-3D -- thermal aggregation and throttle controller.
*
* Consumes the 16 stacked-die sensors plus the base-die sensor and produces:
*   * the hottest reading and its sensor id (for firmware and for the fan/PL
*     control loop that lives outside this subsystem)
*   * a graded throttle level 0..7 that the DVFS controller and the request
*     admission logic consume
*   * bank-level sleep hints so that the coldest banks stay active while the
*     hot spot cools -- a stacked cache throttles by REDUCING ACCESS RATE to a
*     hot bank, not by clocking the whole die down, because the hot spot is
*     usually one bank field under a hot compute core
*   * a critical shutdown that puts the stacked array in retention
*
* Hysteresis is mandatory: throttling drops power, which drops temperature,
* which would immediately un-throttle and oscillate.  Entry and exit
* thresholds are therefore separated by HYST (default 5.0 C) and the loop runs
* on a slow sample tick, not every clock.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_thermal_throttle #(
    parameter SENSORS   = 16,
    parameter TEMP_W    = 12,
    parameter BANKS     = 32,
    parameter HYST      = 12'd50,        // 5.0 C
    parameter SAMPLE_DIV= 1024
) (
    input  wire                      clk,
    input  wire                      rst,

    input  wire [SENSORS*TEMP_W-1:0] temp_raw,
    input  wire [TEMP_W-1:0]         base_die_temp,
    output reg                       temp_sample,

    input  wire [TEMP_W-1:0]         throttle_threshold,
    input  wire [TEMP_W-1:0]         critical_threshold,
    input  wire                      throttle_enable,

    output reg  [TEMP_W-1:0]         temp_max,
    output reg  [3:0]                temp_max_id,
    output reg  [TEMP_W-1:0]         temp_avg,
    output reg  [2:0]                throttle_level,
    output reg                       throttle_active,
    output reg                       critical,
    output reg  [BANKS-1:0]          bank_sleep_hint,
    output reg  [31:0]               throttle_events,
    output reg  [31:0]               throttle_cycles
);

    reg [15:0] div_cnt;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt     <= 16'd0;
            temp_sample <= 1'b0;
        end
        else begin
            temp_sample <= 1'b0;
            if (div_cnt >= SAMPLE_DIV-1) begin
                div_cnt     <= 16'd0;
                temp_sample <= 1'b1;
            end
            else div_cnt <= div_cnt + 16'd1;
        end
    end

    // max / average across sensors
    reg [TEMP_W-1:0] mx;
    reg [3:0]        mx_id;
    reg [TEMP_W+4:0] sum;
    integer s;
    always @* begin
        mx    = base_die_temp;
        mx_id = 4'hf;
        sum   = {(TEMP_W+5){1'b0}};
        for (s = 0; s < SENSORS; s = s + 1) begin
            if (temp_raw[s*TEMP_W +: TEMP_W] > mx) begin
                mx    = temp_raw[s*TEMP_W +: TEMP_W];
                mx_id = s[3:0];
            end
            sum = sum + {5'd0, temp_raw[s*TEMP_W +: TEMP_W]};
        end
    end

    wire [TEMP_W-1:0] avg = sum[TEMP_W+3:4];   // /16

    // graded thresholds: level n engages at threshold + n * 2.0 C
    reg [2:0] level_next;
    integer l;
    always @* begin
        level_next = 3'd0;
        for (l = 0; l < 8; l = l + 1) begin
            if (mx >= (throttle_threshold + (l * 12'd20))) level_next = l[2:0];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            temp_max        <= {TEMP_W{1'b0}};
            temp_max_id     <= 4'd0;
            temp_avg        <= {TEMP_W{1'b0}};
            throttle_level  <= 3'd0;
            throttle_active <= 1'b0;
            critical        <= 1'b0;
            bank_sleep_hint <= {BANKS{1'b0}};
            throttle_events <= 32'd0;
            throttle_cycles <= 32'd0;
        end
        else begin
            if (throttle_active) throttle_cycles <= throttle_cycles + 32'd1;

            if (temp_sample) begin
                temp_max    <= mx;
                temp_max_id <= mx_id;
                temp_avg    <= avg;
                critical    <= (mx >= critical_threshold);

                if (!throttle_enable) begin
                    throttle_active <= 1'b0;
                    throttle_level  <= 3'd0;
                    bank_sleep_hint <= {BANKS{1'b0}};
                end
                else if (!throttle_active && (mx >= throttle_threshold)) begin
                    throttle_active <= 1'b1;
                    throttle_level  <= level_next;
                    throttle_events <= throttle_events + 32'd1;
                    // put the two banks under the hottest sensor to sleep
                    bank_sleep_hint <= (32'd3 << (mx_id * 2));
                end
                else if (throttle_active && (mx + HYST < throttle_threshold)) begin
                    throttle_active <= 1'b0;
                    throttle_level  <= 3'd0;
                    bank_sleep_hint <= {BANKS{1'b0}};
                end
                else if (throttle_active) begin
                    throttle_level  <= level_next;
                    bank_sleep_hint <= (32'd3 << (mx_id * 2));
                end
            end
        end
    end

endmodule
