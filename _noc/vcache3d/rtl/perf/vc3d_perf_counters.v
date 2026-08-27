/*
* CPU-GEN-1 : VCACHE-3D -- 64 x 48-bit performance counters.
*
* A cache this large is useless without measurement: the whole value
* proposition ("more capacity converts DRAM traffic into cache hits") is an
* empirical claim, and the counters here are how it is proved on silicon.
*
* Counter map (index = CSR PERF_SEL):
*    0 cycles                 8  ce_events            16 base_hit_rate_num
*    1 requests               9  ue_events            17 stack_hit_rate_num
*    2 hits                   10 scrub_reads          18 dead_cycles
*    3 misses                 11 scrub_writes         19 repair_events
*    4 base_hits              12 link_crc_errors      20 wake_stalls
*    5 stack_hits             13 link_retrains        21..63 reserved/derived
*    6 fills                  14 throttle_cycles
*    7 writebacks             15 retention_cycles
*
* All counters are 48-bit saturating and freeze together on an overflow of the
* cycle counter so that ratios stay coherent.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"

module vc3d_perf_counters #(
    parameter NUM   = 64,
    parameter WIDTH = 48
) (
    input  wire             clk,
    input  wire             rst,
    input  wire [5:0]       sel,
    output wire [WIDTH-1:0] value,

    input  wire ev_req,
    input  wire ev_hit,
    input  wire ev_miss,
    input  wire ev_base_hit,
    input  wire ev_stack_hit,
    input  wire ev_fill,
    input  wire ev_writeback,
    input  wire ev_ce,
    input  wire ev_ue,
    input  wire ev_scrub_read,
    input  wire ev_scrub_write,
    input  wire ev_link_crc,
    input  wire ev_link_retrain,
    input  wire ev_throttle,
    input  wire ev_retention,
    input  wire ev_repair
);

    reg [WIDTH-1:0] ctr [0:NUM-1];
    wire [NUM-1:0]  inc;

    assign inc[0]  = 1'b1;
    assign inc[1]  = ev_req;
    assign inc[2]  = ev_hit;
    assign inc[3]  = ev_miss;
    assign inc[4]  = ev_base_hit;
    assign inc[5]  = ev_stack_hit;
    assign inc[6]  = ev_fill;
    assign inc[7]  = ev_writeback;
    assign inc[8]  = ev_ce;
    assign inc[9]  = ev_ue;
    assign inc[10] = ev_scrub_read;
    assign inc[11] = ev_scrub_write;
    assign inc[12] = ev_link_crc;
    assign inc[13] = ev_link_retrain;
    assign inc[14] = ev_throttle;
    assign inc[15] = ev_retention;
    assign inc[16] = ev_base_hit & ev_hit;
    assign inc[17] = ev_stack_hit & ev_hit;
    assign inc[18] = ~ev_req;
    assign inc[19] = ev_repair;
    assign inc[20] = 1'b0;

    genvar g;
    generate
        for (g = 21; g < NUM; g = g + 1) begin : g_rsvd
            assign inc[g] = 1'b0;
        end
    endgenerate

    wire frozen = &ctr[0];

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < NUM; i = i + 1) ctr[i] <= {WIDTH{1'b0}};
        end
        else if (!frozen) begin
            for (i = 0; i < NUM; i = i + 1)
                if (inc[i]) ctr[i] <= ctr[i] + {{(WIDTH-1){1'b0}}, 1'b1};
        end
    end

    assign value = ctr[sel];

endmodule
