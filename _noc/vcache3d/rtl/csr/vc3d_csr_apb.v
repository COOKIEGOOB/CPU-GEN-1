/*
* CPU-GEN-1 : VCACHE-3D -- APB CSR block (one 4 KiB window per slice).
*
* Everything the firmware, the RAS driver, the tester and the power manager
* need is here.  Notable design rules:
*
*   * Destructive actions (MBIST start, full repair, fuse programming) are
*     protected by a 16-bit key written in the same cycle; a stray write can
*     therefore not blow fuses on a production part.
*   * Status registers are read-only and never stall the bus (pready is always
*     high after one wait state) because the RAS driver polls them from an
*     interrupt handler.
*   * The error log pops on read of ELOG_POP so a driver can drain it with a
*     tight read loop.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_csr_apb #(
    parameter SLICE_ID = 0,
    parameter UNLOCK_KEY = 16'hC3D0
) (
    input  wire        clk,
    input  wire        rst,

    // APB
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [13:0] paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    output reg         pslverr,

    // control outputs
    output reg         o_ecc_enable,
    output reg         o_scrub_enable,
    output reg         o_scrub_oneshot,
    output reg  [31:0] o_scrub_period,
    output reg  [3:0]  o_scrub_burst,
    output reg  [15:0] o_ce_threshold,
    output reg  [15:0] o_way_disable,
    output reg         o_stack_enable,
    output reg         o_throttle_enable,
    output reg         o_retention_enable,
    output reg  [11:0] o_throttle_threshold,
    output reg  [11:0] o_critical_threshold,
    output reg         o_mbist_start,
    output reg  [2:0]  o_mbist_algo,
    output reg         o_repair_start,
    output reg         o_poweron_start,
    output reg         o_fuse_prog_enable,
    output reg         o_elog_pop,
    output reg         o_clear_first,
    output reg         o_clear_counts,
    output reg  [2:0]  o_dvfs_level,
    output reg         o_dvfs_valid,
    output reg  [5:0]  o_perf_sel,

    // status inputs
    input  wire [47:0] i_perf_value,
    input  wire [31:0] i_hit_count,
    input  wire [31:0] i_miss_count,
    input  wire [31:0] i_ce_count,
    input  wire [31:0] i_ue_count,
    input  wire [15:0] i_scrub_progress,
    input  wire        i_scrub_active,
    input  wire [31:0] i_scrub_sweeps,
    input  wire [63:0] i_elog_lo,
    input  wire [63:0] i_elog_hi,
    input  wire [7:0]  i_elog_level,
    input  wire        i_elog_overflow,
    input  wire [63:0] i_first_ce_lo,
    input  wire [63:0] i_first_ue_lo,
    input  wire [3:0]  i_repair_phase,
    input  wire        i_repair_busy,
    input  wire        i_repair_failed,
    input  wire [15:0] i_rows_repaired,
    input  wire [15:0] i_cols_repaired,
    input  wire [15:0] i_lanes_repaired,
    input  wire [15:0] i_unrepaired,
    input  wire        i_mbist_busy,
    input  wire        i_mbist_done,
    input  wire        i_mbist_pass,
    input  wire        i_link_up,
    input  wire        i_link_fatal,
    input  wire [15:0] i_dead_lanes,
    input  wire [11:0] i_temp_max,
    input  wire [11:0] i_temp_avg,
    input  wire [3:0]  i_temp_max_id,
    input  wire [2:0]  i_throttle_level,
    input  wire        i_throttle_active,
    input  wire [2:0]  i_power_state,
    input  wire [15:0] i_banks_powered,
    input  wire [7:0]  i_logic_vid,
    input  wire [7:0]  i_array_vid
);

    assign pready = 1'b1;

    wire wr = psel & penable & pwrite;
    wire rd = psel & penable & ~pwrite;
    wire key_ok = (pwdata[31:16] == UNLOCK_KEY);

    // -------------------------------------------------------------------------
    // Writes
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            o_ecc_enable         <= 1'b1;
            o_scrub_enable       <= 1'b1;
            o_scrub_oneshot      <= 1'b0;
            o_scrub_period       <= `VC3D_SCRUB_DEFAULT_PERIOD;
            o_scrub_burst        <= `VC3D_SCRUB_BURST;
            o_ce_threshold       <= `VC3D_SCRUB_CE_THRESHOLD;
            o_way_disable        <= 16'd0;
            o_stack_enable       <= 1'b1;
            o_throttle_enable    <= 1'b1;
            o_retention_enable   <= 1'b1;
            o_throttle_threshold <= `VC3D_TEMP_THROTTLE_C;
            o_critical_threshold <= `VC3D_TEMP_CRITICAL_C;
            o_mbist_start        <= 1'b0;
            o_mbist_algo         <= `VC3D_MBIST_MARCH_C;
            o_repair_start       <= 1'b0;
            o_poweron_start      <= 1'b1;      // self-start after reset
            o_fuse_prog_enable   <= 1'b0;
            o_elog_pop           <= 1'b0;
            o_clear_first        <= 1'b0;
            o_clear_counts       <= 1'b0;
            o_dvfs_level         <= 3'd0;
            o_dvfs_valid         <= 1'b0;
            o_perf_sel           <= 6'd0;
            pslverr              <= 1'b0;
        end
        else begin
            // one-shot strobes
            o_mbist_start   <= 1'b0;
            o_repair_start  <= 1'b0;
            o_poweron_start <= 1'b0;
            o_scrub_oneshot <= 1'b0;
            o_elog_pop      <= 1'b0;
            o_clear_first   <= 1'b0;
            o_clear_counts  <= 1'b0;
            pslverr         <= 1'b0;

            if (wr) begin
                case (paddr)
                    `VC3D_CSR_CONTROL: begin
                        o_stack_enable     <= pwdata[0];
                        o_ecc_enable       <= pwdata[1];
                        o_retention_enable <= pwdata[2];
                        o_poweron_start    <= pwdata[8];
                    end
                    `VC3D_CSR_ECC_CTRL: begin
                        o_ecc_enable   <= pwdata[0];
                        o_ce_threshold <= pwdata[31:16];
                        o_clear_first  <= pwdata[1];
                        o_clear_counts <= pwdata[2];
                    end
                    `VC3D_CSR_SCRUB_CTRL: begin
                        o_scrub_enable  <= pwdata[0];
                        o_scrub_oneshot <= pwdata[1];
                        o_scrub_burst   <= pwdata[7:4];
                    end
                    `VC3D_CSR_SCRUB_PERIOD: o_scrub_period <= pwdata;
                    `VC3D_CSR_ELOG_POP:     o_elog_pop     <= 1'b1;
                    `VC3D_CSR_REPAIR_CTRL: begin
                        if (key_ok) o_repair_start <= pwdata[0];
                        else        pslverr        <= 1'b1;
                    end
                    `VC3D_CSR_EFUSE_CTRL: begin
                        if (key_ok) o_fuse_prog_enable <= pwdata[0];
                        else        pslverr            <= 1'b1;
                    end
                    `VC3D_CSR_MBIST_CTRL: begin
                        if (key_ok) begin
                            o_mbist_start <= pwdata[0];
                            o_mbist_algo  <= pwdata[6:4];
                        end
                        else pslverr <= 1'b1;
                    end
                    `VC3D_CSR_THERMAL_CTRL: begin
                        o_throttle_enable    <= pwdata[0];
                        o_throttle_threshold <= pwdata[15:4];
                        o_critical_threshold <= pwdata[27:16];
                    end
                    `VC3D_CSR_DVFS_CTRL: begin
                        o_dvfs_valid <= pwdata[0];
                        o_dvfs_level <= pwdata[6:4];
                    end
                    `VC3D_CSR_PERF_SEL:      o_perf_sel     <= pwdata[5:0];
                    `VC3D_CSR_WAY_DISABLE:   o_way_disable  <= pwdata[15:0];
                    default: pslverr <= 1'b1;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // Reads
    // -------------------------------------------------------------------------
    always @* begin
        prdata = 32'd0;
        case (paddr)
            // "V3D" signature, hardware revision 1, slice id
            `VC3D_CSR_ID:            prdata = {16'h5633, 8'd1, SLICE_ID[7:0]};
            // {ways, line bytes, MiB per slice / 1024 * 1024, slice count}
            `VC3D_CSR_CAPABILITY:    prdata = {8'd16, 8'd64, 8'd32, 8'd3};
            `VC3D_CSR_CONTROL:       prdata = {23'd0, o_retention_enable,
                                               o_ecc_enable, o_stack_enable, 6'd0};
            `VC3D_CSR_STATUS:        prdata = {8'd0, i_banks_powered[7:0],
                                               3'd0, i_power_state,
                                               i_link_fatal, i_link_up,
                                               i_repair_failed, i_repair_busy,
                                               i_repair_phase};
            `VC3D_CSR_ECC_STATUS:    prdata = {i_unrepaired, 14'd0,
                                               i_elog_overflow, i_scrub_active};
            `VC3D_CSR_CE_COUNT:      prdata = i_ce_count;
            `VC3D_CSR_UE_COUNT:      prdata = i_ue_count;
            `VC3D_CSR_SCRUB_CTRL:    prdata = {24'd0, o_scrub_burst, 2'd0,
                                               o_scrub_oneshot, o_scrub_enable};
            `VC3D_CSR_SCRUB_PERIOD:  prdata = o_scrub_period;
            `VC3D_CSR_SCRUB_ADDR:    prdata = {16'd0, i_scrub_progress};
            `VC3D_CSR_SCRUB_PROGRESS:prdata = i_scrub_sweeps;
            `VC3D_CSR_ELOG_HEAD:     prdata = {24'd0, i_elog_level};
            `VC3D_CSR_ELOG_DATA0:    prdata = i_elog_lo[31:0];
            `VC3D_CSR_ELOG_DATA1:    prdata = i_elog_hi[31:0];
            `VC3D_CSR_REPAIR_CTRL:   prdata = {28'd0, i_repair_phase};
            `VC3D_CSR_REPAIR_DATA_LO:prdata = {i_cols_repaired, i_rows_repaired};
            `VC3D_CSR_REPAIR_DATA_HI:prdata = {i_unrepaired, i_lanes_repaired};
            `VC3D_CSR_MBIST_STATUS:  prdata = {29'd0, i_mbist_pass,
                                               i_mbist_done, i_mbist_busy};
            `VC3D_CSR_BOND_STATUS:   prdata = {14'd0, i_link_fatal, i_link_up,
                                               i_dead_lanes};
            `VC3D_CSR_THERMAL_STATUS:prdata = {i_temp_avg, i_temp_max_id,
                                               i_throttle_active, i_throttle_level,
                                               12'd0} | {20'd0, i_temp_max};
            `VC3D_CSR_TEMP_MAX:      prdata = {4'd0, i_temp_max, 4'd0, i_temp_avg};
            `VC3D_CSR_DVFS_CTRL:     prdata = {i_array_vid, i_logic_vid,
                                               9'd0, o_dvfs_level, 4'd0};
            `VC3D_CSR_PERF_SEL:      prdata = {26'd0, o_perf_sel};
            `VC3D_CSR_PERF_LO:       prdata = i_perf_value[31:0];
            `VC3D_CSR_PERF_HI:       prdata = {16'd0, i_perf_value[47:32]};
            `VC3D_CSR_WAY_DISABLE:   prdata = {16'd0, o_way_disable};
            default:                 prdata = 32'hdead_c0de;
        endcase
    end

endmodule
