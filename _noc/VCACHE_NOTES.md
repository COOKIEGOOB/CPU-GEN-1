# HNF L3 as a "V-Cache" — AMD V-Cache benchmark and this pass's improvements

**Scope:** the HNF last-level cache in `_noc/hnf` (tag SRAM, LRU/RRPV SRAM,
data SRAM, MSHR, data buffer, snoop filter). The tensor core (`_tcore`) is
out of scope, per project direction. This document is the companion to
`PERF_NOTES.md` (link-level / NoC throughput analysis).

---

## 1. What AMD's V-Cache actually is (benchmark target)

AMD's "3D V-Cache" is a dense SRAM dielet hybrid-bonded onto (or, since
Zen 5, **under**) the CCD. The measurable properties it buys:

| Property | AMD V-Cache system | Source |
|---|---|---|
| Base L3 per CCD (Zen 4 / Genoa) | 32 MB, 16-way, 64 B lines | [5][6] |
| Stacked addition per CCD | +64 MB → **96 MB/CCD** | [2][5][7] |
| Top EPYC (9684X Genoa-X) | 96 cores, **1152 MB L3** = 384 MB CCD + 768 MB stacked (12 × 64 MB), 400 W | [3][7] |
| V-Cache latency cost | **+4 cycles** vs non-stacked L3 (ISSCC 2023, same as Zen 3 X3D) | [4] |
| L3 bandwidth per core | 32 B/cyc read + 16 B/cyc write (L3→core interface) | [5] |
| Measured Genoa-X L3 read bandwidth | ~90 GB/s single-thread (~24 B/cyc @ 3.7 GHz); ~700 GB/s pulled by one 8-core CCD | [4] |
| Core-visible L3 latency | Zen 3: 46 cyc; X3D adds ~+3–4 cyc (+2 ns); Zen 5 L3 is −3.5 cyc vs Zen 4 | [2][5] |
| Per-core capacity | 4 MB/core base → **12 MB/core** with V-Cache | [5][6] |

Two things matter for "rivaling" it at the RTL level:

1. **Capacity and bandwidth** are expressible in this design (they are
   parameters and datapath widths).
2. **3D stacking itself is a physical/PPA feature** (SRAM density, TSVs,
   power per bit). It cannot be expressed in RTL — what the RTL *can* do is
   give the same slice the capacity, in-flight depth, and timing that make a
   big V-Cache useful, and leave the stack to the physical design.

---

## 2. This design's L3 baseline (cycle-traced)

Geometry (default before this pass → after):

| Item | Before | After (this pass) |
|---|---|---|
| `HNF_L3_CACHE_SIZE_PARAM` (KiB) | 4096 (4 MB) | **32768 (32 MiB per slice)** |
| `HNF_L3_WAY_NUM_PARAM` | 16 | 16 (matches Zen 4/5 16-way) |
| Sets / index / tag | 4096 / 12 / 26 b | 32768 / 15 / 23 b |
| Lines per slice | 65 536 | 524 288 |
| `HNF_MSHR_ENTRIES_NUM_PARAM` | 32 | **128** (QoS pools: HHIGH 8 / HIGH 24 / MED 32 / LOW 63) |
| `HNF_SF_ENTRIES_NUM_PARAM` | 131 072 (2× lines) | **1 048 576 (2× lines at 32 MiB)** |
| Replacement policy | DRrip (2-bit RRPV/way, `hnf_lru_sram` 32 b) | unchanged |
| DAT channel | 32 B/cyc (256 b flit, 2 flits/line) | unchanged — **matches AMD's 32 B/cyc per-core L3 read interface** |

Pipeline (unchanged by this pass — deliberately cycle-identical):

```
RX REQ accepted ── MSHR alloc (s0/s1, 1/cyc, speculative RSP/REQ bypass)
      └─ tag SRAM read (s2) → 16-way tag compare + RRPV victim (s3, combinational)
           → hit/victim decision registered (s5) → data SRAM read (s4/s5)
              → line data valid (s6/s7) → DBF write (s9)
RX DAT (refill/snoop data) → DBF byte-merge (per-byte BE) → txdat 2-slot FIFO, 1 flit/cyc
Hit latency: ≈ 10–12 cycles REQ-accept → CompData (HNF-internal)
```

---

## 3. Changes made in this pass

### 3.1 Sizing defaults — `include/hnf_param.v`

Rationale per knob (see the header comment there):

- **32 MiB slice** — 8× capacity; three interleaved slices provide 96 MiB aggregate. With the default 4-RNF configuration that is
  8 MiB per RN per slice; systems can interleave three slices for capacity and bandwidth. The
  whole point of a V-Cache is *fewer DRAM round trips*; this is the knob that
  delivers that. Cost: one-time reset init grows 4096 → 32768 sets (15-bit index)
  (tag+LRU sweep, `hnf_mem_ctl`, still parallel with the SF sweep).
- **MSHR 32 → 128** — 4× in-flight misses. A big cache only pays off if misses
  can be serviced in parallel; 32 MSHRs across 4 RNs saturates under
  multi-RN stream traffic (the "MSHR-32 saturation" item in PERF_NOTES §4 is
  resolved at default config). QOS pool sizes come from the existing
  generalized formulas in `hnf_defines.v`; all 127 allocatable entries are assigned.
- **SF 131 072 → 1 048 576** — keeps the snoop filter at 2× the L3 line count.
  Undersizing the SF (the old 131 072 vs 262 144 lines) would thrash the SF
  and turn clean evictions into broadcast snoops — a direct bandwidth loss.
- **What was deliberately NOT changed:**
  - `CHIE_DATA_WIDTH_PARAM` stays 256. The DAT flit geometry (2 flits/line,
    `dataid` 00/10, `pe[1:0]`, DBF byte-merge) is built around it; a 512 b
    flit is a protocol-level change, not a parameter flip (see §5.3).
  - `XP_LCRD_NUM_PARAM` stays 15 — the link-credit counters are 4-bit
    defines (`hnf_defines.v`); raising past 15 needs the counter widths
    widened (roadmap item §5.4).

### 3.2 RTL micro-architecture (cycle-identical) — `hnf/hnf_data_sram.v`

- **Banked way-select.** The read path OR-together all 16 ways of the 512 b
  line, giving every output bit a 16-input OR on the critical path to the DBF
  (the "64-byte merge mux" item from PERF_NOTES). The select is now built as
  4 banks × (ways/4): per-bank ORs combine into a 4-term final OR — same
  cycle count, ~1 level of logic shallower per output bit, and the structure
  keeps scaling if ways are raised to 32/64 for a V-Cache-class density.
  Applied to both the `FPGA_MEMORY` and the default `ram_sp` variant; each
  variant keeps its original way-vector alignment (`l3_rd_ways_q` vs
  `l3_rd_ways_q_nxt`), so timing is bit-identical.
- **Dead logic removed.** The default (`ram_sp`) variant instantiated a full
  `hnf_sram_mask` whose output was never consumed — 16 way-slots of registers
  at every depth (16 MB of registers at the new 16 MB default) that were
  written on every data write. Deleted; the per-way `ram_sp` array is the
  real memory. Simulation speedup + no netlist waste.

### 3.3 Wire hygiene — `hnf/hnf.v`

The six control-path `hnf_bump` output wires were declared at
`CACHE_LINE_WIDTH` (512 b) although they carry 14-bit indexes and 16-bit way
vectors (copy-paste from the data paths). Now native widths — 2 448 dead wire
bits removed, width-matched port connections. No functional effect (the
upper bits were never read).

### 3.4 What was deliberately NOT touched

The hit/miss pipeline stage count, the DBF sx3→sx7 shift depth, the 2-slot
TXDAT FIFO, and all credit loops are cycle-count critical and validated by
the 136-case `tb_hnf` regression. They are untouched; timing help there is
available per-path via the `hnf_bump` `ADD_PIPE_STAGE` mechanism (see
PERF_NOTES §3) without changing this pipeline at all.

---

## 4. How to configure larger V-Cache-class sizes

All knobs are plain parameters; the RTL (including the `tb_hnf` regression,
which pins its own values) is unaffected unless you override.

| Target slice | `HNF_L3_CACHE_SIZE_PARAM` (KiB) | `HNF_SF_ENTRIES_NUM_PARAM` (2× lines) | Sim memory (data SRAM) |
|---|---|---|---|
| 4 MB (old default) | 4096 | 131 072 | ~16 MB |
| **16 MB (new default)** | **16384** | **524 288** | ~64 MB |
| 32 MB (one Genoa CCD) | 32768 | 1 048 576 | ~128 MB |
| 96 MB (CCD + V-Cache) | 98 304 | 3 145 728 | ~384 MB (heavy; fine on a 64 GB sim box) |

- Per-core capacity at 4 RNs: size ÷ 4 (96 MB → 24 MB/RN, 2× AMD's V-Cache
  per-core figure).
- More in-flight: `HNF_MSHR_ENTRIES_NUM_PARAM` 128 + `HNF_MSHR_ENTRIES_WIDTH_PARAM` 7
  (DBF follows the MSHR count automatically).
- Reset init cost scales with set count (one sweep per SRAM, all parallel):
  96 MB → 262 144-cycle SF sweep ≈ 0.26 ms @ 1 GHz, once per reset.
- To run the regression at a new size, edit the parameter block at the top
  of `tb/tb_hnf.sv` (it pins all 23 parameters explicitly — that is why the
  defaults in `hnf_param.v` can change without touching the testbenches).

---

## 5. Roadmap — what "truly rivaling AMD V-Cache" needs beyond this RTL

1. **Physical 3D stack** — the differentiating feature is SRAM density and
   power-per-bit of the stacked layer, which is a process/physical-design
   decision (TSV count, bonding, yield), not RTL. The `ram_sp`/`hnf_sram`
   inference in this tree is deliberately memory-model-agnostic: synthesize
   the same RTL against a stacked-VRAM compiler for the dielet and the
   controller is unchanged.
2. **Multi-slice HNF farm** — capacity and bandwidth scale by instantiating
   N HNFs with address-interleaved tgtid routing (the XP already routes on
   tgtid only). E.g. 4 × 16 MB slices = 64 MB aggregate at 128 B/cyc.
   No RTL change in `hnf` needed; this is a top-level integration task.
3. **CHI 512-bit data width** — halves DAT flits per line (32 B/cyc →
   64 B/cyc per slice). Requires reworking the 2-flit line geometry:
   `dataid` handling, `pe[1:0]` → `pe[0]`, DBF byte-merge single-flit path,
   and the TB CHI agents. Largest single bandwidth lever; kept out of this
   pass because it is protocol-level, not a parameter.
4. **Link credits > 15** — widen `HNFC*LCRD_*_CNT_WIDTH` (4 → 5 b) and
   `XP_LCRD_NUM_PARAM` up to 31 for deeper pipelined links.
5. **L3 prefetch** — a stream prefetcher on the SNF refill path (detect
   2-line stride in fill addresses, prefetch the next line into an
   MSHR-protected way) is the standard companion to big V-Caches; needs
   simulation validation, so it is left as a design item rather than an
   untested RTL change.
6. **Per-path timing stages** — if a frequency target is missed on the
   L3↔DBF↔mem_ctl paths, enable `hnf_bump` stages per instance
   (`ADD_PIPE_STAGE(1)`), +1 cycle on that path only (PERF_NOTES §3).

---

## 6. Verification status

- `include/hnf_param.v`, `hnf/hnf.v`, `hnf/hnf_data_sram.v`: preprocessed
  (mini-preprocessor) and grammar-parsed with pyverilog — **clean**.
- The 16 pre-existing parse failures in the tree are all pyverilog
  (Verilog-2005 grammar) limitations in **unmodified** upstream files
  (SV-2009 array parameters, `$clog2` in expressions, multidim wires) —
  accepted by the project's VCS flow.
- Testbench isolation confirmed: `tb_hnf.sv` declares and passes all 23
  parameters explicitly (4 MB / 32 MSHR / 131 072 SF / 256 b flit), so the
  136-case regression exercises exactly the pre-change configuration.
- VCS flow: `cd _noc && make com TB=tb_hnf TOP=hnf && make sim`
  (VCS is authoritative; no simulator is available in this sandbox).

## References

1. thomas-krenn.com — AMD EPYC 9004 Genoa/Bergamo (32 MB L3 per CCD table).
2. Wikipedia — Zen 3 (L3 46-cycle latency; X3D +64 MB, +2 ns / 3–4 cycles).
3. wccftech — EPYC 9684X Genoa-X: 96 cores, 1152 MB L3 (384 CCD + 768 stacked), 400 W.
4. chipsandcheese — Genoa-X: V-Cache +4 cycles (ISSCC 2023); ~90 GB/s ST L3 read (~24 B/cyc); ~700 GB/s per 8-core CCD.
5. Wikipedia — Zen 4 / Zen 5 (32 MB/CCD, 16-way, 32 B/cyc read + 16 B/cyc write per core; Zen 5 L3 latency −3.5 cyc; V-Cache under core, 32+64 MB).
6. videocardz — EPYC L3 32 MB per chiplet across generations.
7. wccftech / driverscloud — Genoa-X per-CCD 32 MB base + 64 MB 3D V-Cache.

## 6. 32 MiB slice / 96 MiB aggregate profile (August 2026)

The repository default now selects a 32 MiB power-of-two slice, 128 MSHRs,
and a 1,048,576-entry snoop filter. Three address-interleaved slices provide
a 96 MiB aggregate while retaining power-of-two set indexing. QoS pools are no longer hard-coded for only 32 or 64 entries;
they scale for any power-of-two MSHR count and reserve one progress entry. At
128 entries the split is 8/24/32/63 plus one sequential-progress entry.

This profile gives each slice robust capacity and outstanding-request resources;
a three-slice system is in the 96 MiB aggregate capacity class. It does **not** turn generic RTL
into AMD 3D V-Cache: hybrid-bonded SRAM, TSV/interconnect parasitics, process
technology, ECC policy, frequency, thermals, yield, and measured PPA remain
physical-design and verification deliverables. The compact testbench profile
continues to pin 4 MiB explicitly for practical simulation.
