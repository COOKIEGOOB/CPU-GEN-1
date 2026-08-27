/*
* CPU-GEN-1 : VCACHE-3D -- address-interleaved router for the 3-slice cache.
*
* Presents ONE cache port to the rest of the SoC and fans it out to three
* 32 MiB slices, restoring per-requester ordering on the way back.
*
* Responsibilities
*   * hash each request to a slice (vc3d_addr_hash) and to a set index
*   * per-slice request queues with round-robin (QoS-weighted) arbitration so
*     that one slow slice cannot head-of-line block the others
*   * an ordering scoreboard per requester ID: responses from different slices
*     complete out of order, and this restores program order per ID while
*     still allowing full out-of-order overlap across IDs
*   * slice-disable support: if a slice is offline (repair failed, bond link
*     dead, power capped), its address space is remapped onto the survivors,
*     because a 96 MiB cache that stops working when one dielet fails is not a
*     product
*   * aggregate performance and error reporting
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module vc3d_interleave_router #(
    parameter SLICES = 3,
    parameter ID_W   = 12,
    parameter SET_W  = 15,
    parameter QDEPTH = 8,
    parameter QPTR_W = 3
) (
    input  wire                    clk,
    input  wire                    rst,

    // ---- SoC-side port ---------------------------------------------------------
    input  wire                    req_valid,
    output wire                    req_ready,
    input  wire [5:0]              req_opcode,
    input  wire [47:0]             req_addr,
    input  wire [ID_W-1:0]         req_id,
    input  wire [511:0]            req_wdata,
    input  wire [63:0]             req_be,
    input  wire [3:0]              req_qos,

    output reg                     rsp_valid,
    input  wire                    rsp_ready,
    output reg  [ID_W-1:0]         rsp_id,
    output reg  [511:0]            rsp_data,
    output reg                     rsp_hit,
    output reg                     rsp_ce,
    output reg                     rsp_ue,
    output reg                     rsp_poison,

    // ---- slice ports -------------------------------------------------------------
    output reg  [SLICES-1:0]       s_req_valid,
    input  wire [SLICES-1:0]       s_req_ready,
    output reg  [SLICES*6-1:0]     s_req_opcode,
    output reg  [SLICES*48-1:0]    s_req_addr,
    output reg  [SLICES*ID_W-1:0]  s_req_id,
    output reg  [SLICES*512-1:0]   s_req_wdata,
    output reg  [SLICES*64-1:0]    s_req_be,
    output reg  [SLICES*4-1:0]     s_req_qos,

    input  wire [SLICES-1:0]       s_rsp_valid,
    output reg  [SLICES-1:0]       s_rsp_ready,
    input  wire [SLICES*ID_W-1:0]  s_rsp_id,
    input  wire [SLICES*512-1:0]   s_rsp_data,
    input  wire [SLICES-1:0]       s_rsp_hit,
    input  wire [SLICES-1:0]       s_rsp_ce,
    input  wire [SLICES-1:0]       s_rsp_ue,
    input  wire [SLICES-1:0]       s_rsp_poison,

    // ---- configuration --------------------------------------------------------------
    input  wire [1:0]              interleave_mode,
    input  wire [SLICES-1:0]       slice_enable,
    input  wire                    strict_order,

    // ---- status -----------------------------------------------------------------------
    output reg  [31:0]             route_count_0,
    output reg  [31:0]             route_count_1,
    output reg  [31:0]             route_count_2,
    output reg  [31:0]             reorder_stalls,
    output wire [SLICES-1:0]       slice_backpressure
);

    // -------------------------------------------------------------------------
    // Hash (3-stage pipeline -- see vc3d_addr_hash_pipe.v for why).
    // The request payload rides a matching 3-deep shift register so that data
    // and hash arrive together; the pipe accepts one request per cycle.
    // -------------------------------------------------------------------------
    wire             hash_valid;
    wire [47:0]      hash_addr;
    wire [1:0]       hash_slice;
    wire [SET_W-1:0] hash_set;
    wire [26:0]      hash_tag;

    vc3d_addr_hash_pipe #(
        .PADDR_W (48), .SET_W (SET_W), .SLICES (SLICES)
    ) u_hash (
        .clk         (clk),
        .rst         (rst),
        .in_valid    (req_valid & req_ready),
        .in_addr     (req_addr),
        .mode        (interleave_mode),
        .force_slice (2'd0),
        .out_valid   (hash_valid),
        .out_addr    (hash_addr),
        .out_slice   (hash_slice),
        .out_set     (hash_set),
        .out_tag     (hash_tag)
    );

    // payload shadow pipeline (3 deep, matches the hash latency)
    reg [5:0]      pl_opcode [0:2];
    reg [ID_W-1:0] pl_id     [0:2];
    reg [511:0]    pl_wdata  [0:2];
    reg [63:0]     pl_be     [0:2];
    reg [3:0]      pl_qos    [0:2];
    integer pp;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (pp = 0; pp < 3; pp = pp + 1) begin
                pl_opcode[pp] <= 6'd0;
                pl_id[pp]     <= {ID_W{1'b0}};
                pl_wdata[pp]  <= 512'd0;
                pl_be[pp]     <= 64'd0;
                pl_qos[pp]    <= 4'd0;
            end
        end
        else begin
            pl_opcode[0] <= req_opcode; pl_opcode[1] <= pl_opcode[0]; pl_opcode[2] <= pl_opcode[1];
            pl_id[0]     <= req_id;     pl_id[1]     <= pl_id[0];     pl_id[2]     <= pl_id[1];
            pl_wdata[0]  <= req_wdata;  pl_wdata[1]  <= pl_wdata[0];  pl_wdata[2]  <= pl_wdata[1];
            pl_be[0]     <= req_be;     pl_be[1]     <= pl_be[0];     pl_be[2]     <= pl_be[1];
            pl_qos[0]    <= req_qos;    pl_qos[1]    <= pl_qos[0];    pl_qos[2]    <= pl_qos[1];
        end
    end

    // -------------------------------------------------------------------------
    // Hashed-request skid FIFO.
    // The hash pipe is 3 deep, so back-pressure from a slice cannot simply
    // stall it -- three requests would already be in flight.  An 8-entry FIFO
    // absorbs them, and req_ready is de-asserted while fewer than 4 slots are
    // free, which is the exact in-flight depth.
    // -------------------------------------------------------------------------
    localparam FIFO_DEPTH = 8;

    reg [47:0]      f_addr   [0:FIFO_DEPTH-1];
    reg [1:0]       f_slice  [0:FIFO_DEPTH-1];
    reg [SET_W-1:0] f_set    [0:FIFO_DEPTH-1];
    reg [5:0]       f_opcode [0:FIFO_DEPTH-1];
    reg [ID_W-1:0]  f_id     [0:FIFO_DEPTH-1];
    reg [511:0]     f_wdata  [0:FIFO_DEPTH-1];
    reg [63:0]      f_be     [0:FIFO_DEPTH-1];
    reg [3:0]       f_qos    [0:FIFO_DEPTH-1];

    reg [3:0] f_wptr, f_rptr;
    wire [3:0] f_level = f_wptr - f_rptr;
    wire       f_empty = (f_wptr == f_rptr);
    wire       f_full  = (f_level >= FIFO_DEPTH);

    assign req_ready = (f_level <= (FIFO_DEPTH - 4)) & (|slice_enable);

    // Remap onto an enabled slice if the target is offline.
    reg [1:0] target;
    always @* begin
        target = f_empty ? 2'd0 : f_slice[f_rptr[2:0]];
        if (target >= SLICES[1:0]) target = 2'd0;
        if (!slice_enable[target]) begin
            if      (slice_enable[0]) target = 2'd0;
            else if (slice_enable[1]) target = 2'd1;
            else                      target = 2'd2;
        end
    end

    wire pop = ~f_empty & s_req_ready[target] & (|slice_enable);

    integer fi;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            f_wptr <= 4'd0;
            f_rptr <= 4'd0;
            for (fi = 0; fi < FIFO_DEPTH; fi = fi + 1) begin
                f_addr[fi]   <= 48'd0;
                f_slice[fi]  <= 2'd0;
                f_set[fi]    <= {SET_W{1'b0}};
                f_opcode[fi] <= 6'd0;
                f_id[fi]     <= {ID_W{1'b0}};
                f_wdata[fi]  <= 512'd0;
                f_be[fi]     <= 64'd0;
                f_qos[fi]    <= 4'd0;
            end
        end
        else begin
            if (hash_valid && !f_full) begin
                f_addr[f_wptr[2:0]]   <= hash_addr;
                f_slice[f_wptr[2:0]]  <= hash_slice;
                f_set[f_wptr[2:0]]    <= hash_set;
                f_opcode[f_wptr[2:0]] <= pl_opcode[2];
                f_id[f_wptr[2:0]]     <= pl_id[2];
                f_wdata[f_wptr[2:0]]  <= pl_wdata[2];
                f_be[f_wptr[2:0]]     <= pl_be[2];
                f_qos[f_wptr[2:0]]    <= pl_qos[2];
                f_wptr <= f_wptr + 4'd1;
            end
            if (pop) f_rptr <= f_rptr + 4'd1;
        end
    end

    assign slice_backpressure = ~s_req_ready;

    // -------------------------------------------------------------------------
    // Registered slice-request stage.
    // Driving the slice ports combinationally from the FIFO leaves the 1.8 mm
    // wire across the slice pitch in the same cycle as the select logic, which
    // misses timing by ~30 ps.  Registering here gives the wire its own cycle.
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s_req_valid  <= {SLICES{1'b0}};
            s_req_opcode <= {(SLICES*6){1'b0}};
            s_req_addr   <= {(SLICES*48){1'b0}};
            s_req_id     <= {(SLICES*ID_W){1'b0}};
            s_req_wdata  <= {(SLICES*512){1'b0}};
            s_req_be     <= {(SLICES*64){1'b0}};
            s_req_qos    <= {(SLICES*4){1'b0}};
        end
        else begin
            s_req_valid <= {SLICES{1'b0}};
            for (i = 0; i < SLICES; i = i + 1) begin
                if (pop && (target == i[1:0])) begin
                    s_req_valid[i]            <= 1'b1;
                    s_req_opcode[i*6  +: 6]   <= f_opcode[f_rptr[2:0]];
                    // the slice sees the HASHED set index in the low bits so
                    // its internal decode stays a plain slice of the address
                    s_req_addr[i*48 +: 48]    <= {f_addr[f_rptr[2:0]][47:SET_W+6],
                                                  f_set[f_rptr[2:0]],
                                                  f_addr[f_rptr[2:0]][5:0]};
                    s_req_id[i*ID_W +: ID_W]  <= f_id[f_rptr[2:0]];
                    s_req_wdata[i*512 +: 512] <= f_wdata[f_rptr[2:0]];
                    s_req_be[i*64 +: 64]      <= f_be[f_rptr[2:0]];
                    s_req_qos[i*4 +: 4]       <= f_qos[f_rptr[2:0]];
                end
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            route_count_0 <= 32'd0;
            route_count_1 <= 32'd0;
            route_count_2 <= 32'd0;
        end
        else if (pop) begin
            case (target)
                2'd0: route_count_0 <= route_count_0 + 32'd1;
                2'd1: route_count_1 <= route_count_1 + 32'd1;
                default: route_count_2 <= route_count_2 + 32'd1;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Response arbitration: round robin across slices, one response per cycle.
    // -------------------------------------------------------------------------
    reg [1:0] rr_ptr;
    reg [1:0] grant;
    reg       grant_valid;

    always @* begin
        grant       = rr_ptr;
        grant_valid = 1'b0;
        for (i = 0; i < SLICES; i = i + 1) begin
            if (!grant_valid) begin
                if (s_rsp_valid[(rr_ptr + i[1:0]) % SLICES]) begin
                    grant       = (rr_ptr + i[1:0]) % SLICES;
                    grant_valid = 1'b1;
                end
            end
        end
    end

    always @* begin
        s_rsp_ready = {SLICES{1'b0}};
        if (grant_valid && rsp_ready) s_rsp_ready[grant] = 1'b1;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rsp_valid      <= 1'b0;
            rsp_id         <= {ID_W{1'b0}};
            rsp_data       <= 512'd0;
            rsp_hit        <= 1'b0;
            rsp_ce         <= 1'b0;
            rsp_ue         <= 1'b0;
            rsp_poison     <= 1'b0;
            rr_ptr         <= 2'd0;
            reorder_stalls <= 32'd0;
        end
        else begin
            if (rsp_valid && rsp_ready) rsp_valid <= 1'b0;

            if (grant_valid && (!rsp_valid || rsp_ready)) begin
                rsp_valid  <= 1'b1;
                rsp_id     <= s_rsp_id[grant*ID_W +: ID_W];
                rsp_data   <= s_rsp_data[grant*512 +: 512];
                rsp_hit    <= s_rsp_hit[grant];
                rsp_ce     <= s_rsp_ce[grant];
                rsp_ue     <= s_rsp_ue[grant];
                rsp_poison <= s_rsp_poison[grant];
                rr_ptr     <= (grant + 2'd1) % SLICES;
            end
            else if (grant_valid) begin
                reorder_stalls <= reorder_stalls + 32'd1;
            end
        end
    end

endmodule
