/*
* CPU-GEN-1 : VCACHE-3D -- bond-lane repair solver.
*
* Builds the logical->physical lane map for one bond channel from the observed
* dead-lane vector, using shift redundancy: logical lane i is assigned to the
* (i+1)-th physical lane that is not dead.  With SPARE spare pads a channel
* survives SPARE dead pads anywhere in the field, which is what makes a
* hybrid-bonded interface manufacturable at 9 um pitch across ~1400 pads.
*
* The solve is sequential (one lane per cycle, 172 cycles) because it runs only
* during link training -- spending area on a combinational solver would be
* pointless.  While solving, `busy` holds the channel in the REPAIR state.
*
* If more than SPARE lanes are dead the channel is declared unrepairable; the
* subsystem then disables the ways served by that channel and continues at
* reduced capacity rather than failing the part.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_lane_repair #(
    parameter SIGNAL_LANE = 164,
    parameter PHYS_LANE   = 172,
    parameter SPARE       = 8
) (
    input  wire                       clk,
    input  wire                       rst,

    input  wire                       start,
    input  wire [PHYS_LANE-1:0]       lane_dead,

    output reg  [SIGNAL_LANE*8-1:0]   lane_map,
    output reg                        busy,
    output reg                        done,
    output reg                        unrepairable,
    output reg  [7:0]                 dead_count
);

    reg [7:0] phys_ptr;
    reg [7:0] log_ptr;

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy         <= 1'b0;
            done         <= 1'b0;
            unrepairable <= 1'b0;
            phys_ptr     <= 8'd0;
            log_ptr      <= 8'd0;
            dead_count   <= 8'd0;
            // identity map at reset
            for (i = 0; i < SIGNAL_LANE; i = i + 1)
                lane_map[i*8 +: 8] <= i[7:0];
        end
        else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy         <= 1'b1;
                phys_ptr     <= 8'd0;
                log_ptr      <= 8'd0;
                dead_count   <= 8'd0;
                unrepairable <= 1'b0;
            end
            else if (busy) begin
                if (log_ptr >= SIGNAL_LANE) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
                else if (phys_ptr >= PHYS_LANE) begin
                    // ran out of physical lanes before mapping every signal
                    busy         <= 1'b0;
                    done         <= 1'b1;
                    unrepairable <= 1'b1;
                end
                else if (lane_dead[phys_ptr]) begin
                    phys_ptr   <= phys_ptr + 8'd1;
                    dead_count <= dead_count + 8'd1;
                end
                else begin
                    lane_map[log_ptr*8 +: 8] <= phys_ptr;
                    log_ptr  <= log_ptr + 8'd1;
                    phys_ptr <= phys_ptr + 8'd1;
                end
            end
        end
    end

endmodule
