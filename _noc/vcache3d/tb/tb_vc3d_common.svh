// ============================================================================
// CPU-GEN-1 : VCACHE-3D -- shared testbench helpers.
//
// Included by every tb_vc3d_*.sv.  Kept in plain SystemVerilog-2005-with-a-few
// -2012-conveniences so that it elaborates under Verilator, Icarus (with -g2012)
// and VCS alike.
// ============================================================================
`ifndef VC3D_TB_COMMON_SVH
`define VC3D_TB_COMMON_SVH

integer vc3d_errors = 0;
integer vc3d_checks = 0;

`define VC3D_CHECK(cond, msg) \
    begin \
        vc3d_checks = vc3d_checks + 1; \
        if (!(cond)) begin \
            vc3d_errors = vc3d_errors + 1; \
            $display("[FAIL] %0t %s:%0d : %s", $time, `__FILE__, `__LINE__, msg); \
        end \
    end

`define VC3D_CHECK_EQ(got, exp, msg) \
    begin \
        vc3d_checks = vc3d_checks + 1; \
        if ((got) !== (exp)) begin \
            vc3d_errors = vc3d_errors + 1; \
            $display("[FAIL] %0t %s : got %0h expected %0h", $time, msg, got, exp); \
        end \
    end

task automatic vc3d_finish(input string name);
    begin
        $display("----------------------------------------------------------");
        $display("%s : %0d checks, %0d failures", name, vc3d_checks, vc3d_errors);
        if (vc3d_errors == 0) $display("%s : PASS", name);
        else                  $display("%s : FAIL", name);
        $display("----------------------------------------------------------");
        if (vc3d_errors != 0) $fatal(1, "test failed");
        $finish;
    end
endtask

// Simple xorshift so that every simulator produces the same stimulus.
function automatic [63:0] vc3d_rand(input [63:0] s);
    reg [63:0] x;
    begin
        x = s;
        x = x ^ (x << 13);
        x = x ^ (x >> 7);
        x = x ^ (x << 17);
        vc3d_rand = x;
    end
endfunction

`endif
