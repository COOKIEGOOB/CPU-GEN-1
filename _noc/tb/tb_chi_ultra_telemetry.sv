`timescale 1ns/1ps
module tb_chi_ultra_telemetry;
    localparam LANES = 256;
    reg clk = 0;
    reg rst_n = 0;
    reg clear_counters = 0;
    reg [LANES-1:0] req_valid = 0, req_ready = 0;
    reg [LANES*8-1:0] req_opcode = 0;
    reg [LANES*4-1:0] req_qos = 0;
    reg [LANES-1:0] rsp_valid = 0, rsp_ready = 0;
    reg [LANES*3-1:0] rsp_code = 0;
    reg [LANES-1:0] dat_valid = 0, dat_ready = 0;
    reg [LANES*6-1:0] dat_bytes = 0;
    wire [LANES-1:0] lane_active, lane_timeout, lane_protocol_error;
    wire [LANES*32-1:0] lane_request_count, lane_response_count;
    wire [LANES*32-1:0] lane_stall_cycles, lane_average_latency;
    wire [63:0] total_requests, total_responses, total_data_bytes;
    wire [31:0] aggregate_error_count;
    wire any_timeout, any_protocol_error;

    always #5 clk = ~clk;

    chi_ultra_telemetry_fabric #(.TIMEOUT_CYCLES(8)) dut (
        .clk(clk), .rst_n(rst_n), .clear_counters(clear_counters),
        .req_valid(req_valid), .req_ready(req_ready), .req_opcode(req_opcode), .req_qos(req_qos),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready), .rsp_code(rsp_code),
        .dat_valid(dat_valid), .dat_ready(dat_ready), .dat_bytes(dat_bytes),
        .lane_active(lane_active), .lane_timeout(lane_timeout), .lane_protocol_error(lane_protocol_error),
        .lane_request_count(lane_request_count), .lane_response_count(lane_response_count),
        .lane_stall_cycles(lane_stall_cycles), .lane_average_latency(lane_average_latency),
        .total_requests(total_requests), .total_responses(total_responses), .total_data_bytes(total_data_bytes),
        .aggregate_error_count(aggregate_error_count), .any_timeout(any_timeout),
        .any_protocol_error(any_protocol_error)
    );

    initial begin
        repeat (3) @(posedge clk); rst_n <= 1;
        @(posedge clk); req_valid[0] <= 1; req_ready[0] <= 1; req_opcode[0 +: 8] <= 8'h01; req_qos[0 +: 4] <= 4'hf;
        @(posedge clk); req_valid[0] <= 0; req_ready[0] <= 0; dat_valid[0] <= 1; dat_ready[0] <= 1; dat_bytes[0 +: 6] <= 32;
        @(posedge clk); dat_valid[0] <= 0; dat_ready[0] <= 0;
        repeat (3) @(posedge clk);
        rsp_valid[0] <= 1; rsp_ready[0] <= 1;
        @(posedge clk); rsp_valid[0] <= 0; rsp_ready[0] <= 0;
        @(posedge clk);
        if (total_requests !== 1 || total_responses !== 1 || total_data_bytes !== 32) $fatal(1, "aggregate counters failed");
        if (lane_protocol_error[0] || lane_active[0]) $fatal(1, "lane state failed");
        $display("PASS: expanded 256-lane telemetry fabric");
        $finish;
    end
endmodule
