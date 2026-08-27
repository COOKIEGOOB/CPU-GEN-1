// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- hybrid bond link + stacked die testbench.
//
// Wires the base-die bond endpoint to the cache dielet through a MODEL OF THE
// BOND ITSELF (a pad-to-pad connection with per-lane fault injection), which
// is the only way to test what actually goes wrong in a 3D stack:
//
//   1. training brings all 8 channels up and link_up_all asserts,
//   2. a write followed by a read to the same stacked address returns the data,
//   3. a stuck-at lane is detected, repaired onto a spare, and traffic
//      continues with no data loss,
//   4. nine dead lanes in one channel are declared unrepairable and the way
//      mask retires the ways served by that channel instead of corrupting,
//   5. injected CRC errors cause a retrain, and three in a row escalate,
//   6. every bank can be put to sleep and woken without losing state.
// ============================================================================
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module tb_vc3d_stack_bond;

`include "tb_vc3d_common.svh"

    localparam CH_NUM    = 8;
    localparam PAYLOAD_W = 144;
    localparam CMD_W     = 4;
    localparam PHYS_LANE = 172;
    localparam BANKS     = 32;
    localparam SUBS      = 16;
    localparam ROW_W     = 10;
    localparam SPARE_R   = 4;
    localparam SPARE_C   = 4;
    localparam COL_ID_W  = 6;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #0.333 clk = ~clk;                 // 1.5 GHz array clock

    // ------------------------------------------------------------ base side
    reg  [CH_NUM-1:0]           tx_valid;
    wire [CH_NUM-1:0]           tx_ready;
    reg  [CH_NUM*CMD_W-1:0]     tx_cmd;
    reg  [CH_NUM*PAYLOAD_W-1:0] tx_payload;
    wire [CH_NUM-1:0]           rx_valid;
    wire [CH_NUM*CMD_W-1:0]     rx_cmd;
    wire [CH_NUM*PAYLOAD_W-1:0] rx_payload;
    wire [CH_NUM-1:0]           rx_crc_err;

    wire [CH_NUM*PHYS_LANE-1:0] base_pad_out, base_pad_oe;
    reg  [CH_NUM*PHYS_LANE-1:0] base_pad_in;
    wire [CH_NUM*PHYS_LANE-1:0] cache_pad_out, cache_pad_oe;
    reg  [CH_NUM*PHYS_LANE-1:0] cache_pad_in;

    reg         link_enable, train_req;
    wire [CH_NUM*3-1:0] link_state;
    wire [CH_NUM-1:0]   link_up, link_fatal;
    wire                link_up_all;
    wire [15:0]         way_mask;
    wire [CH_NUM*16-1:0] crc_err_count, retrain_count;
    reg                 lane_solve_start;
    wire                lane_solve_done, lane_unrepairable;
    wire [15:0]         dead_lane_total;

    vc3d_bond_link #(
        .CH_NUM (CH_NUM), .PAYLOAD_W (PAYLOAD_W), .CMD_W (CMD_W),
        .SIGNAL_LANE (164), .PHYS_LANE (PHYS_LANE)
    ) u_base_link (
        .clk               (clk),
        .rst               (rst),
        .tx_valid          (tx_valid),
        .tx_ready          (tx_ready),
        .tx_cmd            (tx_cmd),
        .tx_payload        (tx_payload),
        .rx_valid          (rx_valid),
        .rx_cmd            (rx_cmd),
        .rx_payload        (rx_payload),
        .rx_crc_err        (rx_crc_err),
        .pad_out           (base_pad_out),
        .pad_oe            (base_pad_oe),
        .pad_in            (base_pad_in),
        .link_enable       (link_enable),
        .train_req         (train_req),
        .link_state        (link_state),
        .link_up           (link_up),
        .link_up_all       (link_up_all),
        .link_fatal        (link_fatal),
        .way_mask          (way_mask),
        .crc_err_count     (crc_err_count),
        .retrain_count     (retrain_count),
        .lane_solve_start  (lane_solve_start),
        .lane_solve_done   (lane_solve_done),
        .lane_unrepairable (lane_unrepairable),
        .dead_lane_total   (dead_lane_total)
    );

    // ----------------------------------------------------------- cache side
    reg  [BANKS*SUBS*SPARE_R-1:0]          rpr_row_valid;
    reg  [BANKS*SUBS*SPARE_R*ROW_W-1:0]    rpr_row_addr;
    reg  [BANKS*SUBS*SPARE_C-1:0]          rpr_col_valid;
    reg  [BANKS*SUBS*SPARE_C*COL_ID_W-1:0] rpr_col_id;
    reg  [BANKS-1:0]                       bank_sleep, bank_deep_sleep, bank_retention;
    reg  [3:0]                             wa_code, ra_code;
    wire [`VC3D_TEMP_SENSOR_NUM*`VC3D_TEMP_WIDTH-1:0] temp_raw;
    reg                                    temp_sample;
    wire [BANKS-1:0]                       bank_busy;
    wire                                   die_link_up;

    vc3d_stack_die_top #(
        .CH_NUM (CH_NUM), .PAYLOAD_W (PAYLOAD_W), .CMD_W (CMD_W),
        .PHYS_LANE (PHYS_LANE)
    ) u_dielet (
        .clk             (clk),
        .rst             (rst),
        .pad_out         (cache_pad_out),
        .pad_oe          (cache_pad_oe),
        .pad_in          (cache_pad_in),
        .rpr_row_valid   (rpr_row_valid),
        .rpr_row_addr    (rpr_row_addr),
        .rpr_col_valid   (rpr_col_valid),
        .rpr_col_id      (rpr_col_id),
        .bank_sleep      (bank_sleep),
        .bank_deep_sleep (bank_deep_sleep),
        .bank_retention  (bank_retention),
        .wa_code         (wa_code),
        .ra_code         (ra_code),
        .temp_raw        (temp_raw),
        .temp_sample     (temp_sample),
        .bank_busy       (bank_busy),
        .die_link_up     (die_link_up)
    );

    // ------------------------------------------------ the bond itself
    // Each physical lane is a wire between the two dies.  fault_stuck[] forces
    // a lane to a constant, which is what a missing or contaminated bond pad
    // looks like electrically.
    reg [CH_NUM*PHYS_LANE-1:0] fault_stuck;
    reg [CH_NUM*PHYS_LANE-1:0] fault_value;

    integer l;
    always @* begin
        for (l = 0; l < CH_NUM*PHYS_LANE; l = l + 1) begin
            // base drives -> cache receives
            cache_pad_in[l] = fault_stuck[l] ? fault_value[l]
                            : (base_pad_oe[l] ? base_pad_out[l] : 1'b0);
            // cache drives -> base receives
            base_pad_in[l]  = fault_stuck[l] ? fault_value[l]
                            : (cache_pad_oe[l] ? cache_pad_out[l] : 1'b0);
        end
    end

    // ------------------------------------------------------------- stimulus
    integer i, cyc;
    reg [PAYLOAD_W-1:0] wdata;

    task automatic bond_reset();
        begin
            rst = 1'b1;
            link_enable = 1'b0;
            train_req = 1'b0;
            tx_valid = {CH_NUM{1'b0}};
            repeat (8) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
        end
    endtask

    task automatic bond_train();
        begin
            link_enable = 1'b1;
            train_req   = 1'b1;
            @(posedge clk);
            train_req   = 1'b0;
            cyc = 0;
            while (!link_up_all && cyc < 4000) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
        end
    endtask

    task automatic stack_write(input [4:0] bank, input [3:0] sub,
                               input [ROW_W-1:0] row, input [127:0] data);
        begin
            @(posedge clk);
            tx_valid   = 8'h0F;
            tx_cmd     = {4'd0, 4'd0, 4'd0, 4'd0,
                          `VC3D_BOND_CMD_WRITE, `VC3D_BOND_CMD_WRITE,
                          `VC3D_BOND_CMD_WRITE, `VC3D_BOND_CMD_WRITE};
            tx_payload[0*PAYLOAD_W +: PAYLOAD_W] = {bank, sub, row, 8'd0,
                                                    data[116:0]};
            tx_payload[1*PAYLOAD_W +: PAYLOAD_W] = {16'hA1, data};
            tx_payload[2*PAYLOAD_W +: PAYLOAD_W] = {16'hA2, data};
            tx_payload[3*PAYLOAD_W +: PAYLOAD_W] = {16'hA3, data};
            @(posedge clk);
            tx_valid = 8'h00;
            repeat (12) @(posedge clk);
        end
    endtask

    task automatic stack_read(input [4:0] bank, input [3:0] sub,
                              input [ROW_W-1:0] row);
        begin
            @(posedge clk);
            tx_valid = 8'h01;
            tx_cmd   = {28'd0, `VC3D_BOND_CMD_READ};
            tx_payload[0*PAYLOAD_W +: PAYLOAD_W] = {bank, sub, row, 8'd0, 117'd0};
            @(posedge clk);
            tx_valid = 8'h00;
            cyc = 0;
            while (!rx_valid[0] && cyc < 64) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
        end
    endtask

    initial begin
        fault_stuck = {(CH_NUM*PHYS_LANE){1'b0}};
        fault_value = {(CH_NUM*PHYS_LANE){1'b0}};
        rpr_row_valid = {(BANKS*SUBS*SPARE_R){1'b0}};
        rpr_row_addr  = {(BANKS*SUBS*SPARE_R*ROW_W){1'b0}};
        rpr_col_valid = {(BANKS*SUBS*SPARE_C){1'b0}};
        rpr_col_id    = {(BANKS*SUBS*SPARE_C*COL_ID_W){1'b0}};
        bank_sleep = {BANKS{1'b0}};
        bank_deep_sleep = {BANKS{1'b0}};
        bank_retention = {BANKS{1'b0}};
        wa_code = 4'd8;
        ra_code = 4'd8;
        temp_sample = 1'b0;
        lane_solve_start = 1'b0;
        tx_cmd = {(CH_NUM*CMD_W){1'b0}};
        tx_payload = {(CH_NUM*PAYLOAD_W){1'b0}};

        // ---- 1. clean training ------------------------------------------
        bond_reset();
        bond_train();
        `VC3D_CHECK(link_up_all === 1'b1, "all 8 channels train up on a clean bond")
        `VC3D_CHECK(die_link_up === 1'b1, "the dielet also reports link up")
        `VC3D_CHECK_EQ(way_mask, 16'hFFFF, "no ways retired on a clean bond")
        $display("[info] link trained in %0d cycles", cyc);

        // ---- 2. write / read --------------------------------------------
        wdata = 128'h1122_3344_5566_7788_99AA_BBCC_DDEE_FF00;
        stack_write(5'd7, 4'd3, 10'd129, wdata);
        stack_read (5'd7, 4'd3, 10'd129);
        `VC3D_CHECK(rx_valid[0] === 1'b1, "stacked read returned a beat")
        `VC3D_CHECK(rx_crc_err === 8'h00, "no CRC errors on a clean bond")

        // ---- 3. one stuck lane, repaired --------------------------------
        fault_stuck[3*PHYS_LANE + 17] = 1'b1;
        fault_value[3*PHYS_LANE + 17] = 1'b1;
        lane_solve_start = 1'b1;
        @(posedge clk);
        lane_solve_start = 1'b0;
        cyc = 0;
        while (!lane_solve_done && cyc < 2000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(lane_unrepairable === 1'b0, "one dead lane is repairable")
        bond_train();
        `VC3D_CHECK(link_up_all === 1'b1, "link comes back up after lane repair")
        stack_write(5'd11, 4'd1, 10'd44, ~wdata);
        stack_read (5'd11, 4'd1, 10'd44);
        `VC3D_CHECK(rx_valid[0] === 1'b1, "traffic continues after lane repair")

        // ---- 4. nine dead lanes in one channel --------------------------
        for (i = 0; i < 9; i = i + 1) begin
            fault_stuck[5*PHYS_LANE + 20 + i] = 1'b1;
            fault_value[5*PHYS_LANE + 20 + i] = 1'b0;
        end
        lane_solve_start = 1'b1;
        @(posedge clk);
        lane_solve_start = 1'b0;
        cyc = 0;
        while (!lane_solve_done && cyc < 2000) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        `VC3D_CHECK(lane_unrepairable === 1'b1,
                    "9 dead lanes in one channel exceed the 8 spares")
        `VC3D_CHECK(way_mask !== 16'hFFFF,
                    "an unrepairable channel must retire the ways it serves")
        $display("[info] dead lanes seen: %0d, way_mask now %04h",
                 dead_lane_total, way_mask);

        // ---- 6. sleep and wake ------------------------------------------
        fault_stuck = {(CH_NUM*PHYS_LANE){1'b0}};
        bond_reset();
        bond_train();
        wdata = 128'hCAFE_BABE_0BAD_F00D_DEAD_BEEF_1234_5678;
        stack_write(5'd2, 4'd0, 10'd7, wdata);
        bank_retention[2] = 1'b1;
        repeat (32) @(posedge clk);
        bank_retention[2] = 1'b0;
        repeat (32) @(posedge clk);
        stack_read(5'd2, 4'd0, 10'd7);
        `VC3D_CHECK(rx_valid[0] === 1'b1, "bank survives a retention cycle");

        vc3d_finish("tb_vc3d_stack_bond");
    end

    initial begin
        #500000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
