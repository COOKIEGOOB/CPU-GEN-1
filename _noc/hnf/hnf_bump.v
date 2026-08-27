/*
* Copyright (c) 2024 Beijing Institute of Open Source Chip
* OpenNoC is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*
* hnf_bump: optional pipeline stage used to break long combinational paths
*           between the HNF L3 data SRAM, the data buffer (DBF) and the
*           SRAM control muxes in hnf_mem_ctl.
*
*   ADD_PIPE_STAGE = 0 (default)
*       Pure wire (data_out = data_in). Zero added latency, zero added logic.
*       This is the performance-oriented default: the module exists so that a
*       timing stage can be enabled without rewiring hnf.v.
*
*   ADD_PIPE_STAGE = 1
*       Inserts one register. Adds exactly 1 cycle of latency on the path
*       this instance sits on. clk_bypass gates the stage: while
*       clk_bypass = 1'b1 the stage streams, while clk_bypass = 1'b0 it holds
*       its previous value (stop-mode / clock-gating support).
*
*   DATA_WIDTH
*       Bit width of the path. 512 = full L3/DBF data words
*       (`CACHE_LINE_WIDTH with CHIE_DATA_WIDTH_PARAM = 256); pass the native
*       width (`LOC_INDEX_WIDTH / `LOC_WAY_NUM) for the control-path
*       instances so an enabled stage does not register 512 bits for a
*       12/16-bit control value.
*
* Note: this module intentionally does NOT take the `HNF_PARAM list (the
* macro already expands to a `#(...)` parameter port list, which cannot be
* extended on the module declaration). It needs none of the CHI parameters -
* only the two above.
*/

module hnf_bump #(
    parameter DATA_WIDTH     = 512,
    parameter ADD_PIPE_STAGE = 0
) (
    //global inputs
    clk,
    rst,
    clk_bypass, //gate: 1 = stage streams, 0 = stage holds

    //input
    data_in,

    //output
    data_out
);

    //global inputs
    input  wire                                clk;
    input  wire                                rst;
    input  wire                                clk_bypass;
    input  wire [DATA_WIDTH-1:0]               data_in;
    output wire [DATA_WIDTH-1:0]               data_out;

    reg [DATA_WIDTH-1:0] stage_q;

    generate
        if (ADD_PIPE_STAGE == 0) begin : gen_comb
            // clean wire: no registers, no useless inverter pairs on the
            // L3 <-> DBF <-> mem_ctl critical paths
            assign data_out = data_in;
        end
        else begin : gen_pipe
            always @(posedge clk or posedge rst) begin
                if (rst)
                    stage_q <= {DATA_WIDTH{1'b0}};
                else if (clk_bypass)
                    stage_q <= data_in;
                // else: hold (stop-mode / clock-gated)
            end
            assign data_out = stage_q;
        end
    endgenerate

endmodule
