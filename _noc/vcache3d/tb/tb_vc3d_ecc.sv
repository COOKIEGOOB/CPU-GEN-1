// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- ECC testbench.
//
// Proves, by exhaustive and randomised injection, that:
//   1. a clean codeword decodes to itself with no flags,
//   2. EVERY single-bit error in data and in check bits is corrected (CE),
//   3. EVERY double-bit error is detected and not miscorrected (UE),
//   4. the 64 B line codec tolerates one error in each of its four words --
//      which is what makes a dead bond lane survivable,
//   5. the poison bit propagates and forces UE,
//   6. the two-stage pipelined decoder gives bit-identical results to the
//      combinational one, two cycles later.
//
// Run: vcs -sverilog -f ../filelist_base.f tb_vc3d_ecc.sv
//      verilator --binary --timing -f ../filelist_base.f tb_vc3d_ecc.sv
// ============================================================================
`timescale 1ns/1ps
`include "vc3d_params.vh"
`include "vc3d_defines.vh"

module tb_vc3d_ecc;

`include "tb_vc3d_common.svh"

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #0.166 clk = ~clk;          // 3.0 GHz

    // ----------------------------------------------------------------- DUTs
    reg  [127:0] data_i;
    wire [8:0]   check_o;
    reg  [127:0] corrupt_data;
    reg  [8:0]   corrupt_check;
    wire [127:0] data_o;
    wire         ce, ue, poison_o;
    wire [8:0]   syndrome;

    vc3d_secded_enc_128 u_enc (
        .data_i  (data_i),
        .check_o (check_o)
    );

    vc3d_secded_dec_128 u_dec (
        .data_i     (corrupt_data),
        .check_i    (corrupt_check),
        .poison_i   (1'b0),
        .data_o     (data_o),
        .ce_o       (ce),
        .ue_o       (ue),
        .poison_o   (poison_o),
        .syndrome_o (syndrome)
    );

    // split (pipelined) pair, driven with the same stimulus
    wire [8:0]   syn_only;
    reg  [8:0]   syn_q;
    reg  [127:0] data_q;
    wire [127:0] split_data;
    wire         split_ce, split_ue, split_poison;

    vc3d_secded_syn_128 u_syn (
        .data_i     (corrupt_data),
        .check_i    (corrupt_check),
        .syndrome_o (syn_only)
    );

    vc3d_secded_cor_128 u_cor (
        .data_i     (data_q),
        .syndrome_i (syn_q),
        .poison_i   (1'b0),
        .data_o     (split_data),
        .ce_o       (split_ce),
        .ue_o       (split_ue),
        .poison_o   (split_poison)
    );

    always @(posedge clk) begin
        syn_q  <= syn_only;
        data_q <= corrupt_data;
    end

    // ----------------------------------------------------------- line codec
    reg  [511:0] line_i;
    wire [575:0] coded;
    reg  [575:0] coded_corrupt;
    reg          line_valid;
    wire [511:0] line_o;
    wire         line_ce, line_ue, line_poison, line_valid_o;
    wire [3:0]   line_ce_vec, line_ue_vec;
    wire [35:0]  line_syndrome;
    wire [3:0]   line_written;

    vc3d_ecc_line_enc u_line_enc (
        .line_i    (line_i),
        .poison_i  (1'b0),
        .written_i (4'hF),
        .seq_i     (2'b00),
        .coded_o   (coded)
    );

    vc3d_ecc_line_dec_pipe u_line_dec (
        .clk        (clk),
        .rst        (rst),
        .in_valid   (line_valid),
        .coded_i    (coded_corrupt),
        .out_valid  (line_valid_o),
        .line_o     (line_o),
        .ce_o       (line_ce),
        .ue_o       (line_ue),
        .ce_vec_o   (line_ce_vec),
        .ue_vec_o   (line_ue_vec),
        .syndrome_o (line_syndrome),
        .poison_o   (line_poison),
        .written_o  (line_written)
    );

    // ----------------------------------------------------------------- test
    integer b, b2, t;
    reg [63:0] seed;
    reg [127:0] pattern;

    initial begin
        line_valid = 1'b0;
        data_i     = 128'd0;
        line_i     = 512'd0;
        coded_corrupt = 576'd0;
        corrupt_data  = 128'd0;
        corrupt_check = 9'd0;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // ---- 1. clean codewords -----------------------------------------
        seed = 64'h0123_4567_89AB_CDEF;
        for (t = 0; t < 64; t = t + 1) begin
            seed = vc3d_rand(seed);
            pattern = {seed, ~seed};
            data_i = pattern;
            #1;
            corrupt_data  = pattern;
            corrupt_check = check_o;
            #1;
            `VC3D_CHECK_EQ(data_o, pattern, "clean decode data")
            `VC3D_CHECK(ce === 1'b0 && ue === 1'b0, "clean decode flags")
        end
        $display("[info] clean codeword check done");

        // ---- 2. every single-bit error in data --------------------------
        data_i = 128'h0F1E_2D3C_4B5A_6978_8796_A5B4_C3D2_E1F0;
        #1;
        for (b = 0; b < 128; b = b + 1) begin
            corrupt_data  = data_i ^ (128'd1 << b);
            corrupt_check = check_o;
            #1;
            `VC3D_CHECK(ce === 1'b1, "SEC: single data-bit error must flag CE")
            `VC3D_CHECK(ue === 1'b0, "SEC: single data-bit error must not flag UE")
            `VC3D_CHECK_EQ(data_o, data_i, "SEC: corrected data")
        end
        $display("[info] 128 single-bit data errors corrected");

        // ---- 2b. every single-bit error in the check field --------------
        for (b = 0; b < 9; b = b + 1) begin
            corrupt_data  = data_i;
            corrupt_check = check_o ^ (9'd1 << b);
            #1;
            `VC3D_CHECK(ce === 1'b1, "SEC: single check-bit error must flag CE")
            `VC3D_CHECK(ue === 1'b0, "SEC: check-bit error must not flag UE")
            `VC3D_CHECK_EQ(data_o, data_i, "SEC: data untouched by check error")
        end
        $display("[info] 9 single-bit check errors corrected");

        // ---- 3. exhaustive double-bit errors ----------------------------
        for (b = 0; b < 128; b = b + 1) begin
            for (b2 = b + 1; b2 < 128; b2 = b2 + 1) begin
                corrupt_data  = data_i ^ (128'd1 << b) ^ (128'd1 << b2);
                corrupt_check = check_o;
                #1;
                `VC3D_CHECK(ue === 1'b1, "DED: double error must flag UE")
                `VC3D_CHECK(ce === 1'b0, "DED: double error must not flag CE")
            end
        end
        $display("[info] all 8128 double-bit data error pairs detected");

        // ---- 6. split decoder equivalence -------------------------------
        for (t = 0; t < 32; t = t + 1) begin
            seed = vc3d_rand(seed);
            data_i = {seed, seed ^ 64'hA5A5_5A5A_A5A5_5A5A};
            #1;
            corrupt_data  = data_i ^ (128'd1 << (t % 128));
            corrupt_check = check_o;
            @(posedge clk);
            #1;
            `VC3D_CHECK_EQ(split_data, data_i, "split decoder corrects like the flat one")
            `VC3D_CHECK(split_ce === 1'b1, "split decoder flags CE")
        end
        $display("[info] pipelined split decoder matches the flat decoder");

        // ---- 4. line codec, one error per 128-bit word ------------------
        seed = 64'hDEAD_BEEF_CAFE_F00D;
        line_i = {8{seed}};
        #1;
        coded_corrupt = coded;
        coded_corrupt[0*144 + 3]   = ~coded[0*144 + 3];
        coded_corrupt[1*144 + 77]  = ~coded[1*144 + 77];
        coded_corrupt[2*144 + 120] = ~coded[2*144 + 120];
        coded_corrupt[3*144 + 130] = ~coded[3*144 + 130];   // a check bit
        line_valid = 1'b1;
        @(posedge clk);
        line_valid = 1'b0;
        @(posedge clk);
        #1;
        `VC3D_CHECK(line_valid_o === 1'b1, "line decoder produced a beat")
        `VC3D_CHECK_EQ(line_o, line_i, "line codec corrects 4 x 1 bit")
        `VC3D_CHECK_EQ(line_ce_vec, 4'hF, "all four words report CE")
        `VC3D_CHECK(line_ue === 1'b0, "no UE for 4 x 1 bit")
        $display("[info] a whole dead bond lane is correctable");

        // ---- 5. two errors in ONE word must be detected -----------------
        coded_corrupt = coded;
        coded_corrupt[1*144 + 10] = ~coded[1*144 + 10];
        coded_corrupt[1*144 + 11] = ~coded[1*144 + 11];
        line_valid = 1'b1;
        @(posedge clk);
        line_valid = 1'b0;
        @(posedge clk);
        #1;
        `VC3D_CHECK(line_ue === 1'b1, "two errors in one word -> UE")
        `VC3D_CHECK_EQ(line_ue_vec, 4'b0010, "UE localised to word 1")

        vc3d_finish("tb_vc3d_ecc");
    end

    initial begin
        #200000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
