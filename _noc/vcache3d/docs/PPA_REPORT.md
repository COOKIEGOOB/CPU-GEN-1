# VCACHE-3D PPA report

Every number here is produced by a program in `pd/`; rerun them with
`make reports`, which writes `pd/reports/*.rpt`. Nothing in this file is a
guess typed in by hand.

## 1. Frequency

Target: 3.0 GHz base tier, 2.2 GHz cache tier (macro-sliced dielet), SS corner
(0.855 V logic, 0.68 V array, 125 C).

| tier | worst path | slack | Fmax |
|---|---|---|---|
| base | router FIFO -> request register -> 1.8 mm wire -> slice | +4.3 ps | 3.04 GHz |
| cache | macro slice -> 4:1 slice mux -> 16:1 sub mux -> bank flop | +5.3 ps | 2.26 GHz |

**All paths meet.** They did not at first. The initial run reported -372 ps on
the base tier and -845 ps on the cache tier, and the RTL was changed until the
model passed:

| fix | why |
|---|---|
| tag array banked 4x (8192x39) | a 32768-deep macro is 240 ps |
| base data array banked 4x (8192x576) | a 32768-deep 576 b macro is 285 ps |
| registered macro outputs (`OUT_REG`) on the base data array | macro + bank mux + way mux + 600 um did not fit |
| tag compare on raw bits, SECDED in parallel | serialising decode->compare costs ~140 ps |
| line ECC split + speculative S6 return + replay only on error | the flat decoder is 379 ps and only runs off the critical path |
| **direct SRAM macro slicing (4x 256x148, divided local bitlines)** | the 465 ps 1024x148 macro capped the dielet at 1.5 GHz |
| registered bank output on the dielet | macro access + 4:1 slice + 16:1 + 32:1 + 1.1 mm did not fit |
| registered command decode on the dielet | pad -> decode -> repair CAM -> macro was ~700 ps |
| 3-stage pipelined mod-3 hash | the combinational hash is 545 ps |
| **DDR bond lanes (dual-edge, no gearbox)** | link round trip drops from 4 to 2 base cycles |
| registered router request stage + skid FIFO | the 1.8 mm slice-pitch wire needs its own cycle |

Delay primitives: FO4 14 ps, gate 18 ps, CK->Q 26 ps, setup 18 ps, skew 25 ps,
jitter 8 ps, M4 0.22 ps/um, M8 0.09 ps/um, hybrid bond pad 31 ps. Macro access
times: tag 8192x39 = 175 ps, base data 8192x576 = 190 ps, stacked 256x148
macro slice = **260 ps** (was 465 ps for 1024x148).

## 2. Latency

| access | cycles (3.0 GHz) | time |
|---|---|---|
| base-die hit (ways 0-3) | 12 | 4.0 ns |
| stacked hit (ways 4-15) | **16** | **5.33 ns** |
| miss (to memory) | 12 + DRAM | -- |

The stacked region now costs **+4 base cycles** (down from +9), matching the
published AMD 3D V-Cache latency adder. Three stacked changes get there in
series:
1. a speculative S6 raw-line return with a 1-cycle SECDED syndrome flag, made
   **combinational at the S5 data boundary** so the common zero-syndrome case
   needs no extra response-register cycle; the full correction runs in parallel
   on the base die and only stalls/replays a non-zero syndrome (well under
   0.0001% of accesses);
2. a DDR bond link (forwarded 1.5 GHz clock sampled on both edges) that
   removes the serialisation gearbox, saving 2 cycles round-trip;
3. direct SRAM macro slicing (4 × 256 × 148) that lifts the dielet from
   1.5 GHz to 2.2 GHz, shaving another 2–3 ns off the stacked access.

AMD quotes +4 core cycles for V-Cache; this design now lands at the same adder.
24 MiB of the 96 lives on the base die at the fast latency (a monolithic
V-Cache slice has no such region), and 5.33 ns is still an order of magnitude
better than the DRAM access it replaces.

## 3. Area

| block | area |
|---|---|
| stacked macro slice (256x148 x4 per subarray) | 0.0041 mm2 |
| macro slices per dielet | 8192 (4 quadrants x 32 banks x 16 subarrays x 4 slices) |
| dielet array | 8.46 mm2 |
| dielet total (82 % util) | **10.57 mm2** for 32 MiB physical |
| dielet density | 29.4 Mb/mm2 |
| tag array per slice | 0.87 mm2 |
| base data array per slice | 3.23 mm2 |
| slice total | 4.29 mm2 |
| base die (55 % util) | **23.41 mm2** |
| total silicon | **31.7 mm2** for 96 MiB |
| silicon per MiB | 0.33 mm2 |

A monolithic 96 MiB L3 in base-die HP SRAM would need 67.3 mm2 of the expensive
die. Stacking removes ~44 mm2 from it. That is the entire economic argument for
3D cache, and it is quantified here rather than asserted.

## 4. Power

| operating point | base | dielets | total |
|---|---|---|---|
| idle (90 % retention) | 1866 mW | 540 mW | **2.41 W** |
| light (0.05 req/cyc) | 1873 mW | 1130 mW | 3.00 W |
| nominal (0.30 req/cyc) | 1908 mW | 1721 mW | 3.63 W |
| peak (1.0 req/cyc) | 2007 mW | 1873 mW | **3.88 W** |

Leakage dominates, as it always does in a cache this size: SRAM leakage is
calibrated to ~2 mW/Mb (dielet HD) and ~6 mW/Mb (base HP) at 85 C. Per-bank
retention saves 0.49 W per dielet, which is most of the idle budget.

Energy per hit: **15.1 pJ** for a base-die way (0.24 pJ/B), **59.7 pJ** for a
stacked way (0.93 pJ/B). A DDR5 access is ~20 pJ/B, so even the expensive
stacked hit is ~20x cheaper than the DRAM access it avoids.

## 5. Thermal

Packaging uses the inverted Zen 5 stack: the dielet is face-up on the package
substrate and the base die backside is ground directly against the lid.  The
dielet is therefore OUTSIDE the hot base-die-to-lid thermal path, and only a
small fraction of the dielet's own 0.62 W has to cross the base die.  Steady
state for one 10.57 mm2 column, 45 C ambient, 12 W of core power underneath:

| node | temperature |
|---|---|
| base-die junction | 84.0 C |
| dielet junction | 84.0 C |
| lid | 82.0 C |
| same column without a dielet | 83.5 C |
| **stacking penalty** | **+0.5 C** |

Throttle asserts at ~16 W of local core power; the 105 C limit is reached at
~18 W, so the 4-step throttle in `rtl/power/vc3d_thermal_throttle.v` has to act
first. The dielet time constant to the lid is 0.6 ms, which is why the throttle
samples at 1 kHz and filters with a 6-bit-shift IIR.

Design choices that came out of this model: 36 um dielet thinning, 0.75 V /
2.2 GHz operation after macro slicing, per-bank sleep, an inverted (Zen 5)
packaging order, and a bond field spread along the die edge (the bonds are also
the best heat path between the dies).

## 6. Package

| | |
|---|---|
| signal pads per slice | 1376 |
| power/ground pads | 384 |
| control pads | 24 |
| pitch | 9 um |
| field | 3154 x 50 um, 65 % of the die edge |
| alignment check | 1376 lane pairs mate, mirror axis verified |

## 7. Against AMD 3D V-Cache

| | AMD (Zen 3/4 V-Cache) | VCACHE-3D |
|---|---|---|
| capacity | 32 MB base + 64 MB stacked = 96 MB | 24 MiB base + 72 MiB stacked = 96 MiB |
| associativity | 16 way | 16 way |
| line | 64 B | 64 B |
| stacked latency adder | +4 core cycles | **+4 base cycles** |
| bandwidth | ~700 GB/s per CCD | **1.30 TB/s per package (DDR bond + 512-bit DAT)** |
| bond | SoIC-X, ~9 um | hybrid bond, 9 um, 1376 signals per slice |
| dielet area | 36 mm2 (64 MB, N7) | 10.57 mm2 (32 MiB, N6 HD) |
| ECC | yes | SECDED on tags, data, and the link, with scrub |
| repair | yes | spare rows/cols/lanes, MBIST, eFuse, runtime escalation |

The stacked latency adder is now **+4 base cycles, matching the published AMD
3D V-Cache figure**. The one remaining *honest* gap is the absence of silicon
measurements (no manufactured part, no post-PnR STA). The two places it is
ahead are the base-die fast region and the fact that every number above is
reproducible from a program in this repository.
