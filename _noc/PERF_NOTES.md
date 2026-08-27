# OpenNoC (`_noc`) — performance analysis & improvement notes

Scope: `hnf/ hni/ rni/ snf/` (CHI-E NoC nodes, upstream RV-BOSC/OpenNoC + local
modifications). Goal: latency / throughput. Dates from cycle traces in the RTL
(stage names `s0`, `sx1`…`sx9` are the designers' own pipeline markers).

---

## 1. What was changed in this pass

| Item | File(s) | Why | Risk |
|---|---|---|---|
| **Fixed `hnf_bump`** — the local "timing stage" scaffolding | `hnf/hnf_bump.v` | The old module was a **combinational double-NOT** (`out = ~(~in)`): synthesis collapses it to a wire, i.e. it provided **no timing help at all**, wasted ~2 inverter levels on the 512-bit L3↔DBF paths, and had unused `clk`/`rst` ports (warnings). New: `ADD_PIPE_STAGE=0` (default) is a clean zero-latency wire; `ADD_PIPE_STAGE=1` inserts a real, `clk_bypass`-gated register (+1 cycle, opt-in per instance). `DATA_WIDTH` parameter so control-path instances don't pad to 512 bits when a stage is enabled. | None with default params (default = pure wire, same netlist intent, minus the inverter pairs). Latency only changes if you explicitly enable a stage. |
| **Restored build/test infrastructure** | `include/`, `misc/`, `tb/`, `case/`, `file_list_tb.f`, `Makefile` | The tree **could not be compiled**: all 9 `include "…_defines.v"/"…_param.v"` files referenced by every node were missing from the repo. Restored verbatim from upstream `RV-BOSC/OpenNoC` (Mulan PSL v2 headers kept) + the 4 testbenches, 136 `tb_hnf` scenario files, and a Makefile (VCS flow + iverilog elaboration smoke targets). | None (additive). |
| `.gitignore` | repo root | Stops waveform/log dumps (`.vcd/.fsdb/.vpd/.svf/novas*/simv*…`) from accumulating in git — 13 such files were already committed under `_tcore/`; run `git rm --cached _tcore/*.vcd _tcore/*.fsdb* _tcore/*.log _tcore/*.svf _tcore/novas* _tcore/session.tcl _tcore/ucli.key` if you want them untracked. | None. |
| Docs comment for the bump wiring | `hnf/hnf.v` | Explains the new knobs and their latency cost. | None. |

**Deliberately not touched:** the upstream node RTL (MSHR state machines,
coherence pipeline, QoS sequencers, link credit loops). See §4 for why, and
§5 for the tuning knobs to work with.

---

## 2. Measured structural characteristics (cycle traces)

### HNF request path (the hot node)

| Step | Cycles | Where |
|---|---|---|
| RX REQ flit accept → decode | 0 (same cycle) | `hnf_link_rxreq_parse.v` (combinational `li_mshr_rxreq_*_s0`) |
| MSHR CAM + QoS slot alloc | same cycle (s0) | `hnf_mshr.v`, `hnf_mshr_qos.v` — `rxreq_retry_enable_s0 = li_req_dyn_alloc_fail_s0 & ~li_seq_alloc_s0` |
| Speculative TX for NoSnp ops | same cycle (bypass) | `hnf_mshr_bypass.v` — TXREQ/txrsp bypass competes with queued traffic at s1 |
| L3/SF/LRU tag pipeline | **6 stages** (SX1→SX7) | `hnf_cache_pipeline.v` — 1 lookup/cycle (per-stage valid, hazards tracked) |
| L3 data SRAM read (hit) | +1 | `hnf_data_sram.v` → `l3_rd_data_q` |
| DBF line write | sx9 | `hnf_cache_pipeline.v` → `hnf_data_buffer.v` |
| DBF read → 2× CompData | 2 flits @ 1 flit/cyc | `hnf_mshr_ctl.v` (`mshr_dbf_rd_*_sx1_q`) → `hnf_link_txdat_wrap.v` (2-slot pipeline; entry1/entry2 + per-half `pe` shift — I traced the flit sequence A0,A1,B0,B1: **no bubble when credits are available**) |

→ **L3 hit: ≈ 10–12 cycles** REQ-accept → first CompData on the wire.
→ **L3 miss: + SNF round trip** (its MSHR + memory latency) + replay of the DAT.

### Throughput ceilings (all verified in RTL)

* **TX DAT: 1 flit/cycle** = 32 B/cycle (256-bit flits); a 64 B line = 2 cycles.
* **L3 tag lookups: 1/cycle**; **MSHR allocs: 1/cycle**; **RX REQ accept: 1/cycle**.
* **REQ credit loop is 1:1, no steady-state stall** — traced the
  `rxreq_crdcntsm` table in `hnf_link_rxreq_parse.v`: accepted flit returns its
  L-credit the same cycle, retried flit returns it on the RetryAck cycle, plus a
  one-time pre-grant of `XP_LCRD_NUM_PARAM` (15) credits after reset. Same
  pattern in the SNF/HNI RX channels and the HNF TX channels' credit counters.

### SNF / HNI
Simple credit-based senders (`snf_txdat.v`, `hni_txdat.v`): 1 flit/cycle when a
credit is available; MSHR 32 entries each; data buffers indexed per MSHR entry.

### RNI (AXI → CHI)
Deeply pipelined d1→d6 reassembly (`rni_rd_buffer.v`: 4 banks, rp/rd FIFOs);
AR/AW entry tables (`rni_arctrl.v`/`rni_awctrl.v`, `RNI_AR_ENTRIES_NUM_PARAM`
slots) with QoS hi/lo retry classes. No structural bubble found in the read
path on paper — but it is the one node I would regression-test first under
mixed read/write bursts (see §5).

---

## 3. Where the performance actually lives now

The upstream datapath is clean on the happy path; the levers are:

1. **Frequency (your current fight).** The `hnf_bump` stages are now real
   options. If synthesis still can't close the L3 data path, enable per
   instance (each = +1 cycle on that path only):
   ```verilog
   hnf_bump #(.DATA_WIDTH(`CACHE_LINE_WIDTH), .ADD_PIPE_STAGE(1)) \
       tx_l3tobuffer( ... );
   ```
   Also in scope for the same treatment: the DBF per-byte merge logic in
   `hnf_data_buffer.v` (64 parallel ~4-way conditional byte muxes feeding one
   register row) and the `init/debug/cpl` mux bank in `hnf_mem_ctl.v`.
2. **Bandwidth:** `CHIE_DATA_WIDTH_PARAM` 256→512 doubles DAT bandwidth per
   flit (DBF entry = `CHIE_DATA_WIDTH_PARAM*2`); at the cost of link/XP width.
3. **Outstanding capacity:** `HNF_MSHR_ENTRIES_NUM_PARAM` (32) and the QoS pool
   split (`QOS_*_POOL_NUM` in `include/hnf_defines.v`) bound in-flight
   coherent traffic for 4 RNFs; shortfalls show up as RetryAck storms (check
   `retry_ack_fifo` activity in tb_hnf).
4. **DBF pressure:** 32 × 512-bit DBF; when full, L3 fills and MSHR retirements
   stall (`l3_fill_data_busy_sx_q`, `txdat_mshr_busy_sx`) — the ceiling for
   write-heavy mixes.
5. **L3 SRAM porting:** 1R1W per set-index; same-set hit+fill conflicts cost a
   cycle — look for it in the `l3_rd_busy`/`l3_fill_busy` toggling in sim.

---

## 4. Why I didn't rewrite the state machines

* No NoC testbench existed in this repo (restored now) and **no simulator is
  available in this sandbox** (no iverilog/verilator/VCS, no root).
* The MSHR/MSHR-QoS/coherence logic is the most timing- and correctness-critical
  code in the tree; blind "improvements" there are how coherence bugs are born.
  The happy-path structures (credit loops, 2-slot TX DAT pipeline, per-stage
  tag pipeline, speculative TX bypass) are sound as read in the RTL — the gains
  are in §3, not in re-architecting.
* Everything I changed is either additive (infra) or opt-in (bump stages).

## 5. Verify before/after any change

```sh
cd _noc
# syntax/elaboration smoke (needs iverilog):
make iverilog-hnf iverilog-snf iverilog-hni
# full regression (needs VCS, from upstream flow):
make com TB=tb_hnf TOP=hnf && make sim
make com TB=tb_rni TOP=rni && make sim
make com TB=tb_snf TOP=snf && make sim
```

`tb_hnf` replays the 136 `case/**.txt` CHI scenarios (see `case/README.md`).
For latency/throughput numbers, instrument the link flits (`txdatflitv` /
`rxreqflitv` edges) and record: REQ-accept → first-CompData delta per
transaction, and flits/cycle for each TX channel under a streaming pattern
(4 RNFs × ReadNoSnp, then × ReadShared/ReadUnique mixes).

## 6. Parking lot — `_tcore` (out of scope here, but seen while surveying)

* `fp16_to_fp9.v` is a 0-byte file although `to_fp9.v` instantiates the module
  and `tb_fp16_to_fp9.v` tests it → `_tcore` build is broken.
* `_tcore/filelist.f` references ~40 files that don't exist in the repo
  (`common_cell/*`, `tensor_core_exe.v`, …).
* `fp4_to_fp9` maps e2m1 exponent `10` (2.0/3.0) to +Inf (original code
  commented out with wrong bias math); subnormal 0.5 maps to 2⁻¹⁷. The
  testbench "expected" values were captured from the buggy RTL, so the tests
  pass while codifying the bug.
* `to_fp8_con_core.v` builds 8-bit words into 9-bit fp9 slots (sign lands in
  bit 7, mantissa LSB dropped).
* `define.v` duplicates `` `FP8 `` / `` `FP8E4M3 `` defines.
