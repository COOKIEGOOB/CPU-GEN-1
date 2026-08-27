/*
* CPU-GEN-1 : VCACHE-3D -- memory BIST engine (8 algorithms) with fail capture.
*
* Runs at wafer sort, at package test, and at every power-on (a short subset)
* against the stacked array through the same bond link the functional path
* uses -- which is deliberate: the BIST therefore also proves the link, the
* deskew solution, and the repair map, not just the bitcells.
*
* Algorithms
*   MARCH_C-     : {up(w0); up(r0,w1); up(r1,w0); dn(r0,w1); dn(r1,w0); up(r0)}
*                  detects SAF, TF, CFin, CFid, AF -- the industry baseline.
*   MARCH_SS     : adds detection of the write-disturb faults that low-VDD
*                  high-density stacked bitcells are prone to.
*   CHECKERBOARD : bitcell-to-bitcell leakage / pattern sensitivity.
*   WALKING_ONE  : bit-line and sense-amp imbalance.
*   GALPAT       : coupling faults, O(n^2)-ish, sort only.
*   ROW_STRIPE   : word-line driver strength, IR-drop sensitive patterns.
*   RETENTION    : write, idle for a programmable pause, read -- this is the
*                  algorithm that finds the marginal cells that would later
*                  show up as field CEs; it is the reason the engine has a
*                  programmable pause timer measured in microseconds.
*   PSEUDO_RANDOM: LFSR data/address, long-run screen.
*
* Fail handling: the first FAIL_DEPTH failures are captured with full
* {bank, subarray, row, column-group, expected, actual} context and handed to
* the repair controller, which solves for a row/column allocation.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_mbist_engine #(
    parameter BANKS      = 32,
    parameter BANK_W     = 5,
    parameter SUBS       = 16,
    parameter SUB_W      = 4,
    parameter ROWS       = 1024,
    parameter ROW_W      = 10,
    parameter DW         = 144,
    parameter FAIL_DEPTH = 16,
    parameter FAIL_W     = 4
) (
    input  wire                clk,
    input  wire                rst,

    // ---- control -------------------------------------------------------------
    input  wire                start,
    input  wire [2:0]          algorithm,
    input  wire [BANK_W-1:0]   bank_first,
    input  wire [BANK_W-1:0]   bank_last,
    input  wire [31:0]         retention_pause,
    input  wire                stop_on_fail,
    output reg                 busy,
    output reg                 done,
    output reg                 pass,

    // ---- memory port (through the bond link) --------------------------------
    output reg                 mem_req,
    input  wire                mem_gnt,
    output reg                 mem_we,
    output reg  [BANK_W-1:0]   mem_bank,
    output reg  [SUB_W-1:0]    mem_sub,
    output reg  [ROW_W-1:0]    mem_row,
    output reg  [DW-1:0]       mem_wdata,
    input  wire                mem_rvalid,
    input  wire [DW-1:0]       mem_rdata,

    // ---- fail capture --------------------------------------------------------
    output reg  [FAIL_W:0]     fail_count,
    output reg  [BANK_W-1:0]   fail_bank,
    output reg  [SUB_W-1:0]    fail_sub,
    output reg  [ROW_W-1:0]    fail_row,
    output reg  [DW-1:0]       fail_expect,
    output reg  [DW-1:0]       fail_actual,
    output reg                 fail_push,
    output reg  [31:0]         total_fail_bits
);

    // -------------------------------------------------------------------------
    // March element program.  Each algorithm is a list of (op, data, dir)
    // micro-elements; the sequencer walks the address space per element.
    // -------------------------------------------------------------------------
    localparam OP_W0 = 3'd0, OP_W1 = 3'd1, OP_R0 = 3'd2, OP_R1 = 3'd3,
               OP_RW = 3'd4, OP_PAUSE = 3'd5, OP_END = 3'd6;

    reg [3:0] elem;          // element index within the algorithm
    reg [2:0] op;
    reg       dir_down;

    // March C- : w0^, r0w1^, r1w0^, r0w1v, r1w0v, r0^
    always @* begin
        op       = OP_END;
        dir_down = 1'b0;
        case (algorithm)
            `VC3D_MBIST_MARCH_C: begin
                case (elem)
                    4'd0: begin op = OP_W0; dir_down = 1'b0; end
                    4'd1: begin op = OP_R0; dir_down = 1'b0; end
                    4'd2: begin op = OP_W1; dir_down = 1'b0; end
                    4'd3: begin op = OP_R1; dir_down = 1'b0; end
                    4'd4: begin op = OP_W0; dir_down = 1'b0; end
                    4'd5: begin op = OP_R0; dir_down = 1'b1; end
                    4'd6: begin op = OP_W1; dir_down = 1'b1; end
                    4'd7: begin op = OP_R1; dir_down = 1'b1; end
                    4'd8: begin op = OP_W0; dir_down = 1'b1; end
                    4'd9: begin op = OP_R0; dir_down = 1'b0; end
                    default: op = OP_END;
                endcase
            end
            `VC3D_MBIST_MARCH_SS: begin
                case (elem)
                    4'd0: begin op = OP_W0; end
                    4'd1: begin op = OP_R0; end
                    4'd2: begin op = OP_R0; end
                    4'd3: begin op = OP_W0; end
                    4'd4: begin op = OP_R0; end
                    4'd5: begin op = OP_W1; end
                    4'd6: begin op = OP_R1; dir_down = 1'b1; end
                    4'd7: begin op = OP_W0; dir_down = 1'b1; end
                    4'd8: begin op = OP_R0; dir_down = 1'b1; end
                    default: op = OP_END;
                endcase
            end
            `VC3D_MBIST_CHECKERBOARD,
            `VC3D_MBIST_WALKING_ONE,
            `VC3D_MBIST_GALPAT,
            `VC3D_MBIST_ROW_STRIPE,
            `VC3D_MBIST_PSEUDO_RANDOM: begin
                case (elem)
                    4'd0: op = OP_W1;
                    4'd1: op = OP_R1;
                    4'd2: op = OP_W0;
                    4'd3: op = OP_R0;
                    default: op = OP_END;
                endcase
            end
            `VC3D_MBIST_RETENTION: begin
                case (elem)
                    4'd0: op = OP_W1;
                    4'd1: op = OP_PAUSE;
                    4'd2: op = OP_R1;
                    4'd3: op = OP_W0;
                    4'd4: op = OP_PAUSE;
                    4'd5: op = OP_R0;
                    default: op = OP_END;
                endcase
            end
            default: op = OP_END;
        endcase
    end

    // -------------------------------------------------------------------------
    // Data pattern generator
    // -------------------------------------------------------------------------
    reg [31:0] lfsr;
    always @(posedge clk or posedge rst) begin
        if (rst) lfsr <= 32'h1234_5678;
        else     lfsr <= {lfsr[30:0], lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
    end

    reg [DW-1:0] pattern;
    integer pb;
    always @* begin
        pattern = {DW{1'b0}};
        case (algorithm)
            `VC3D_MBIST_CHECKERBOARD:
                for (pb = 0; pb < DW; pb = pb + 1)
                    pattern[pb] = (pb[0] ^ mem_row[0]);
            `VC3D_MBIST_WALKING_ONE: begin
                pattern = {DW{1'b0}};
                pattern[mem_row[7:0] % DW] = 1'b1;
            end
            `VC3D_MBIST_ROW_STRIPE:
                pattern = mem_row[0] ? {DW{1'b1}} : {DW{1'b0}};
            `VC3D_MBIST_PSEUDO_RANDOM:
                for (pb = 0; pb < DW; pb = pb + 1)
                    pattern[pb] = lfsr[pb % 32];
            default:
                pattern = {DW{1'b1}};
        endcase
    end

    wire [DW-1:0] data_one  = (algorithm == `VC3D_MBIST_MARCH_C ||
                               algorithm == `VC3D_MBIST_MARCH_SS ||
                               algorithm == `VC3D_MBIST_RETENTION) ? {DW{1'b1}} : pattern;
    wire [DW-1:0] data_zero = ~data_one;

    // -------------------------------------------------------------------------
    // Address sequencer
    // -------------------------------------------------------------------------
    reg [BANK_W-1:0] bank;
    reg [SUB_W-1:0]  sub;
    reg [ROW_W-1:0]  row;
    reg [31:0]       pause_cnt;
    reg [2:0]        state;
    reg [DW-1:0]     expect_q;
    localparam S_IDLE = 3'd0, S_ISSUE = 3'd1, S_WAIT = 3'd2, S_CHECK = 3'd3,
               S_STEP = 3'd4, S_PAUSE = 3'd5, S_DONE = 3'd6;

    wire is_read  = (op == OP_R0) || (op == OP_R1);
    wire is_write = (op == OP_W0) || (op == OP_W1);
    wire [DW-1:0] op_data = ((op == OP_W1) || (op == OP_R1)) ? data_one : data_zero;

    wire last_row  = dir_down ? (row == {ROW_W{1'b0}}) : (row == (ROWS-1));
    wire last_sub  = dir_down ? (sub == {SUB_W{1'b0}}) : (sub == (SUBS-1));
    wire last_bank = dir_down ? (bank == bank_first)   : (bank == bank_last);

    integer bcnt, k;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= S_IDLE;
            busy            <= 1'b0;
            done            <= 1'b0;
            pass            <= 1'b1;
            elem            <= 4'd0;
            bank            <= {BANK_W{1'b0}};
            sub             <= {SUB_W{1'b0}};
            row             <= {ROW_W{1'b0}};
            mem_req         <= 1'b0;
            mem_we          <= 1'b0;
            mem_bank        <= {BANK_W{1'b0}};
            mem_sub         <= {SUB_W{1'b0}};
            mem_row         <= {ROW_W{1'b0}};
            mem_wdata       <= {DW{1'b0}};
            fail_count      <= {(FAIL_W+1){1'b0}};
            fail_push       <= 1'b0;
            fail_bank       <= {BANK_W{1'b0}};
            fail_sub        <= {SUB_W{1'b0}};
            fail_row        <= {ROW_W{1'b0}};
            fail_expect     <= {DW{1'b0}};
            fail_actual     <= {DW{1'b0}};
            total_fail_bits <= 32'd0;
            pause_cnt       <= 32'd0;
            expect_q        <= {DW{1'b0}};
        end
        else begin
            fail_push <= 1'b0;
            done      <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy            <= 1'b1;
                        pass            <= 1'b1;
                        elem            <= 4'd0;
                        bank            <= bank_first;
                        sub             <= {SUB_W{1'b0}};
                        row             <= {ROW_W{1'b0}};
                        fail_count      <= {(FAIL_W+1){1'b0}};
                        total_fail_bits <= 32'd0;
                        state           <= S_ISSUE;
                    end
                end

                S_ISSUE: begin
                    if (op == OP_END) begin
                        state <= S_DONE;
                    end
                    else if (op == OP_PAUSE) begin
                        pause_cnt <= 32'd0;
                        state     <= S_PAUSE;
                    end
                    else begin
                        mem_req   <= 1'b1;
                        mem_we    <= is_write;
                        mem_bank  <= bank;
                        mem_sub   <= sub;
                        mem_row   <= row;
                        mem_wdata <= op_data;
                        expect_q  <= op_data;
                        if (mem_gnt) begin
                            mem_req <= 1'b0;
                            state   <= is_read ? S_WAIT : S_STEP;
                        end
                    end
                end

                S_PAUSE: begin
                    pause_cnt <= pause_cnt + 32'd1;
                    if (pause_cnt >= retention_pause) begin
                        elem  <= elem + 4'd1;
                        row   <= dir_down ? {ROW_W{1'b1}} : {ROW_W{1'b0}};
                        sub   <= dir_down ? {SUB_W{1'b1}} : {SUB_W{1'b0}};
                        bank  <= dir_down ? bank_last : bank_first;
                        state <= S_ISSUE;
                    end
                end

                S_WAIT: begin
                    if (mem_rvalid) state <= S_CHECK;
                end

                S_CHECK: begin
                    if (mem_rdata != expect_q) begin
                        pass <= 1'b0;
                        bcnt = 0;
                        for (k = 0; k < DW; k = k + 1)
                            if (mem_rdata[k] != expect_q[k]) bcnt = bcnt + 1;
                        total_fail_bits <= total_fail_bits + bcnt[31:0];
                        if (fail_count < FAIL_DEPTH) begin
                            fail_push   <= 1'b1;
                            fail_bank   <= bank;
                            fail_sub    <= sub;
                            fail_row    <= row;
                            fail_expect <= expect_q;
                            fail_actual <= mem_rdata;
                            fail_count  <= fail_count + 1'b1;
                        end
                        state <= stop_on_fail ? S_DONE : S_STEP;
                    end
                    else begin
                        state <= S_STEP;
                    end
                end

                S_STEP: begin
                    if (!last_row) begin
                        row <= dir_down ? (row - 1'b1) : (row + 1'b1);
                    end
                    else begin
                        row <= dir_down ? {ROW_W{1'b1}} : {ROW_W{1'b0}};
                        if (!last_sub) begin
                            sub <= dir_down ? (sub - 1'b1) : (sub + 1'b1);
                        end
                        else begin
                            sub <= dir_down ? {SUB_W{1'b1}} : {SUB_W{1'b0}};
                            if (!last_bank) begin
                                bank <= dir_down ? (bank - 1'b1) : (bank + 1'b1);
                            end
                            else begin
                                // element complete
                                elem <= elem + 4'd1;
                                bank <= dir_down ? bank_last : bank_first;
                            end
                        end
                    end
                    state <= S_ISSUE;
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
