/*
* CPU-GEN-1 : VCACHE-3D -- distributed thermal sensor (behavioural model).
*
* Thermal behaviour is THE limiting factor of a 3D-stacked cache: the cache
* dielet sits between the compute die and the heat spreader, so the compute
* die's heat flows through it, and the dielet's own leakage rises with that
* temperature.  A stacked-cache design that does not model, measure and
* throttle on temperature is not a real product -- which is why 16 sensors are
* distributed over the array and read every sample window.
*
* Model (matched to pd/thermal/thermal_model.py):
*      T_sensor = T_ambient
*               + R_th_die     * P_local(activity)
*               + R_th_stack   * P_neighbour
*               + leakage_feedback(T)
* with a first-order thermal RC giving the sensor a realistic time constant
* (the array cannot heat 20 C in one clock, and a throttle loop tuned against
* an instantaneous model would oscillate).
*
* Output is 12-bit, 0.1 C per LSB, 0.0 .. 409.5 C.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"

module vc3d_thermal_sensor #(
    parameter SENSOR_ID   = 0,
    parameter T_AMBIENT   = 12'd450,    // 45.0 C package ambient
    parameter R_TH_LOCAL  = 12'd6,      // 0.6 C per activity unit
    parameter TAU_SHIFT   = 6           // RC time constant = 2^6 samples
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        sample,
    input  wire [1:0]  activity,
    output reg  [11:0] temp_o
);

    reg [19:0] filt;      // 8 fractional bits of headroom

    wire [11:0] target = T_AMBIENT
                       + (R_TH_LOCAL * {10'd0, activity})
                       + ((SENSOR_ID % 4) * 12'd5);   // static gradient

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            filt   <= {T_AMBIENT, 8'd0};
            temp_o <= T_AMBIENT;
        end
        else if (sample) begin
            // first-order IIR: filt += (target - filt) >> TAU_SHIFT
            filt   <= filt + (({target, 8'd0} - filt) >>> TAU_SHIFT);
            temp_o <= filt[19:8];
        end
    end

endmodule
