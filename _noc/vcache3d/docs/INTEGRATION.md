# Integrating VCACHE-3D with the CHI HN-F

## The attach point

`_noc/hnf/hnf_data_sram.v` is the HN-F's L3 data array. It exposes a plain
SRAM-style port:

```
l3_index_q     set index
l3_rd_ways_q   one-hot read way select
l3_wr_ways_q   one-hot write way select
l3_wr_data_q   64 B line in
l3_rd_data_q   64 B line out, registered
```

`rtl/top/hnf_vcache3d_adapter.v` presents exactly that port and services it
from the 3D stack. The HN-F pipeline keeps its tags, its snoop filter, its
MSHRs and its coherence logic; only where the data lives changes.

## Two things the old port cannot express

**Variable latency.** A stacked way takes longer than a base-die way. The
adapter drives `l3_stall_req`, which the HN-F access stage already knows how to
honour for SRAM contention. Defining `VC3D_HNF_FIXED_LATENCY` instead pads
every access to the worst case, giving a zero-change integration at the cost of
making base-die hits as slow as stacked ones.

**Errors.** The original array cannot report anything. The adapter adds
`l3_ce`, `l3_ue` and `l3_poison`. Leaving them unconnected preserves the old
behaviour exactly; wiring them into the HN-F error path is a small change and
is what a production part would do.

## Directed accesses

The HN-F has already resolved the way before it touches the data array, so the
adapter issues `VC3D_OPC_DIRECT_READ` / `VC3D_OPC_DIRECT_WRITE`, which carry
the way in `addr[9:6]` and bypass the slice's own tag array. Re-running a tag
compare would add latency and could disagree with the HN-F -- two sources of
truth for the same question is a bug waiting to happen.

## Wiring three slices

For a full 96 MiB attach, instantiate `vc3d_package_top` (router + 3 slices +
3 dielets) and drive it from the HN-F's memory-side port instead of replacing
`hnf_data_sram`. That gives the interleave and the multi-slice capacity; the
adapter route is for a single-slice, minimum-change integration.

## Parameters that must agree

| VCACHE-3D | HN-F | note |
|---|---|---|
| `VC3D_SET_NUM` 32768 | `LOC_INDEX_WIDTH` 15 | one index bit per set |
| `VC3D_WAY_NUM` 16 | `LOC_WAY_NUM` 16 | associativity must match |
| `VC3D_LINE_BYTES` 64 | `CACHE_LINE_WIDTH` 512 | line size must match |
| `VC3D_PADDR_WIDTH` 48 | `ADDR_WIDTH` | address width |

`tb/tb_hnf.sv` pins all 23 HN-F parameters explicitly, so changing
`hnf_param.v` defaults for an experiment will not silently change that test.

## Build

```
make -C _noc/vcache3d gen      # regenerate generated RTL and filelists
make -C _noc/vcache3d lint     # structural + port checks
make -C _noc/vcache3d model    # golden-model property tests
make -C _noc/vcache3d reports  # PPA / thermal / package models
```

`filelist_base.f` builds the base die (42 files), `filelist_cache.f` the
dielet (8 files), `filelist_tb.f` the testbenches.
