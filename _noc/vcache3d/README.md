# VCACHE-3D — 96 MiB 3D-stacked last level cache

A complete 3D-stacked cache subsystem for CPU-GEN-1: three 32 MiB slices, each
made of a base-die logic tier and a hybrid-bonded stacked SRAM dielet, with
production ECC, background scrubbing, spare rows/columns/lanes, a manufacturing
repair controller, an address-interleaved router, and physical-design models
for frequency, area, power, thermals, SRAM selection and package connectivity.

```
make gen      # regenerate generated RTL + filelists (deterministic)
make lint     # structural + full port checking (47 files, 0 errors)
make model    # 25 golden-model property tests
make reports  # PPA / thermal / package models -> pd/reports/
make sim-all  # testbenches (requires VCS or Verilator)
```

## Headlines

| | |
|---|---|
| capacity | 96 MiB, 16 way, 64 B lines |
| frequency | 3.0 GHz base tier, 1.5 GHz cache tier, all paths meet |
| latency | 12 cycles base-die hit, 21 cycles stacked hit |
| bandwidth | 216 GB/s per slice, 648 GB/s per package |
| area | 23.4 mm² base die + 3 × 10.57 mm² dielet = 31.7 mm² |
| power | 2.41 W idle, 3.88 W peak |
| thermals | +3.3 °C stacking penalty, throttle at 16 W local core power |
| bonds | 1376 signal + 384 PG + 24 control per slice at 9 µm pitch |
| yield | 99.94 % post-repair at D0 = 0.07 /cm² |

Full numbers, and the programs that produce them: [`docs/PPA_REPORT.md`](docs/PPA_REPORT.md).

## Layout

| path | contents |
|---|---|
| `include/` | parameters (`vc3d_params.vh`) and encodings (`vc3d_defines.vh`) |
| `rtl/ecc/` | SECDED codecs (generated) + line codec, scrubber, CE tracker, error log |
| `rtl/stack/` | subarray, bank array (generated), bond lane/CRC/channel/link, dielet top |
| `rtl/repair/` | eFuse array, MBIST engine, repair map, lane repair, repair controller |
| `rtl/slice/` | tag array, base data array, slice pipeline, slice top |
| `rtl/power/` | DVFS, thermal sensor, throttle, bank power control |
| `rtl/perf/`, `rtl/csr/` | 64 performance counters, APB CSR block |
| `rtl/top/` | address hash (+ pipelined), router, 96 MiB top, package top, HN-F adapter |
| `gen/` | deterministic RTL generators and the structural linter |
| `model/` | independent Python golden models + property tests |
| `tb/` | five self-checking SystemVerilog testbenches |
| `pd/` | SRAM selection, OpenROAD config, SDC, floorplan/PDN, timing/area/power/thermal/package models |
| `docs/` | architecture, bond interface, ECC and repair, PPA, verification, integration, CSR map |

## Reading order

1. [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — what the thing is
2. [`docs/BOND_INTERFACE.md`](docs/BOND_INTERFACE.md) — how the two dies talk
3. [`docs/ECC_AND_REPAIR.md`](docs/ECC_AND_REPAIR.md) — how it survives defects
4. [`docs/PPA_REPORT.md`](docs/PPA_REPORT.md) — what it costs
5. [`docs/VERIFICATION.md`](docs/VERIFICATION.md) — what was actually run, and what was not
6. [`docs/INTEGRATION.md`](docs/INTEGRATION.md) — how to attach it to the HN-F

## Honest status

There is no simulator, synthesiser or PDK in this environment. The RTL is
checked structurally (including full instantiation port checking), the
algorithms are checked against independent Python models, and the physical
numbers come from analytical models that are committed here and can be rerun.
The five testbenches are written for VCS/Verilator but have not been executed.
The timing model is not decoration: its first run found six violated paths, and
the RTL was restructured — banked arrays, split ECC decoder, registered bank
outputs, pipelined hash, registered router stage — until every path met.
