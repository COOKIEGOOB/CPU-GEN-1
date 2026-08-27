# Verification

No EDA tool is available in this environment -- there is no simulator, no
synthesiser and no PDK. Everything below therefore runs on Python alone and is
executed for real, and every claim in the docs is produced by one of these
programs rather than asserted.

| what | how | status |
|---|---|---|
| RTL structure, ports, macros | `gen/vc3d_lint.py` | 47 files, 89 modules, 0 errors, 0 warnings |
| algorithms | `model/test_vc3d_model.py` | 25/25 pass |
| ECC matrices | self-test inside `gen/vc3d_gen_ecc.py` | SEC/DED proven exhaustively for all 5 widths |
| timing | `pd/scripts/timing_model.py` | all paths meet at 3.0 / 1.5 GHz |
| area | `pd/scripts/area_model.py` | 23.4 mm2 base + 3 x 10.57 mm2 dielet |
| power | `pd/scripts/power_model.py` | 2.4 W idle, 3.9 W peak |
| thermal | `pd/thermal/thermal_model.py` | +3.3 C stacking penalty, throttle at 16 W |
| package | `pd/package/check_bond_alignment.py` | 1376 lane pairs mate, mirror correct |
| testbenches | `tb/*.sv` | written for VCS/Verilator, elaborated structurally by the linter |

## The linter

`gen/vc3d_lint.py` is not a style checker. It strips comments, then verifies:

* `module`/`endmodule`, `begin`/`end`, `case`/`endcase`, `function`/`task`/
  `generate` balance, and parenthesis balance;
* every `` `MACRO `` used is defined in `include/` or `tb/`;
* no duplicate module names;
* **full instantiation checking**: the instantiated module exists in-tree,
  every `.port` is a real port of it, no port is connected twice, and it warns
  on unconnected inputs.

That last check is the one that catches real integration bugs. It was verified
to be live by deliberately mistyping a port and watching it fail.

## The golden model

`model/vc3d_model.py` reimplements the algorithms **independently** of the RTL,
in Python, from the specification rather than from the Verilog:

* Hsiao SECDED construction and decode
* CRC-16/CCITT (checked against the standard 0x29B1 vector)
* exact modulo-3 interleave and its balance properties
* XOR-fold set indexing
* column shift-redundancy remapping
* the bond lane repair solver
* scrub stride coverage
* the thermal IIR filter
* a 3-slice functional cache model (capacity, associativity, eviction)

25 property tests, including: 96 MiB capacity, a 9-dead-lane channel is
unrepairable, stride 4097 covers all 32768 sets while 4096 does not, and the
weighted-popcount hash equals `(addr >> 6) % 3` for a million addresses.

## The testbenches

Five self-checking SystemVerilog testbenches, written to run under VCS or
Verilator (`make sim-all`):

| testbench | proves |
|---|---|
| `tb_vc3d_ecc.sv` | all 128 single-bit and all 8128 double-bit patterns; 4x1-bit line correction; split decoder equals the flat one |
| `tb_vc3d_stack_bond.sv` | training, stacked read/write through a modelled bond, lane repair, 9-lane unrepairable, retention |
| `tb_vc3d_repair.sv` | MBIST finds injected defects, spares are allocated, fuses burned, autoloaded after a power cycle, MBIST then passes |
| `tb_vc3d_slice.sv` | miss/fill/hit, a 16-way set spanning both dies, dirty write-back, CE tolerance, streaming, CSR |
| `tb_vc3d_96mib.sv` | interleave balance across three slices, residency, no lost requests, global CSR |

They are not executable here, which is stated plainly rather than papered over.
What *is* executable has been run, and the timing model in particular changed
the design: its first run found six violated paths, and the RTL was fixed --
banked arrays, a split ECC decoder, registered bank outputs, a pipelined hash,
a registered router output stage -- rather than the targets being lowered.
