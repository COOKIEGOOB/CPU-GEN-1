# VCACHE-3D architecture

96 MiB of 3D-stacked last level cache: three 32 MiB slices, each built from a
base-die logic + SRAM tier and a hybrid-bonded stacked SRAM dielet.

```
                       +-------------------------------------------+
   cache dielet x3     |  32 banks x 16 subarrays x 4 quadrants     |
   (stacked SRAM)      |  2048 x (1024 x 148 b) HD macros = 32 MiB  |
                       |  bond endpoint | repair fabric | sensors   |
                       +--------------------|----------------------+
                          1376 signal bonds | 9 um pitch, face to face
                       +--------------------|----------------------+
   base die            |  bond endpoint  |  8 MiB base data array   |
   (logic tier)        |  tags (16 way)  |  ECC | scrub | BISR      |
                       |  slice pipeline |  MSHR | CSR | counters   |
                       +-------------------------------------------+
                                    | mod-3 interleave router
                                    | HN-F adapter
```

## 1. Capacity and organisation

| parameter | value |
|---|---|
| total capacity | 96 MiB (3 x 32 MiB) |
| associativity | 16 way |
| line size | 64 B |
| sets per slice | 32768 |
| ways on the base die | 0..3 (8 MiB per slice, 24 MiB total) |
| ways on the dielet | 4..15 (24 MiB per slice, 72 MiB total) |
| physical dielet capacity | 32 MiB (all 16 ways, used in stack-only mode) |
| interleave | exact modulo 3 on the line address |
| ECC | SECDED(128,9) x4 per line, on both tiers and on the link |
| tag ECC | SECDED(32,7) per tag entry |

The 16-way image is **unified across two dies**. Software, the coherence
protocol and the HN-F see one 32 MiB 16-way cache per slice; only the data
array read path knows whether a way is local or stacked. This is what makes
the design a V-Cache equivalent rather than a bolt-on victim cache.

## 2. Why three slices and modulo 3

96 MiB is not a power of two, so a bit-select interleave cannot spread traffic
over three slices. `rtl/top/vc3d_addr_hash.v` computes an exact `(line_addr mod
3)` from a weighted population count, which the Python golden model proves is
perfectly balanced for linear sweeps and for every stride that is not a
multiple of three. The combinational form is 545 ps, so the router uses the
3-stage pipelined version in `vc3d_addr_hash_pipe.v`; those three cycles are
pipelined away (one request per cycle) and overlap the previous tag lookup.

Modes: `MOD3` (default), `LOW2` (power-of-two subset for debug), `HASH`, and
`DIRECT` (force a slice, used by BIST and by the tester).

## 3. Slice pipeline

| stage | work |
|---|---|
| S0 | request accept, set index, direct/normal decode |
| S1 | tag macro address (4 set banks per way) |
| S2 | tag macro read, bank mux |
| S3 | 16-way raw compare, hit encode, victim select; tier-aware way partitioning; tag SECDED in parallel |
| S4 | data array address (base) or bond command (stacked) |
| S5 | data macro read (base) / macro-sliced dielet read |
| S6 | bank/way mux, speculative **raw-line + 1-cycle parity/SECDED flag** return |
| S7 | full SECDED correction runs in parallel (non-critical, only used on error) |
| S8 | error replay / poison publish only when syndrome ≠ 0 |
| S9 | response (fast path skips S7/S8 entirely) |

Tag compare runs on the **raw** tag bits with the SECDED decoder in parallel.
Serialising decode into the compare costs ~140 ps and misses the 333 ps cycle.
A corrected bit inside a tag can only turn a hit into a miss (safe: the line is
re-fetched); an uncorrectable tag squashes the response one cycle later.

## 4. Stacked path

A stacked access is striped across bond channels 0..3, each carrying one 16 B
quarter of the line as a 144-bit payload plus CRC-16. Channel 4 is the
maintenance channel (MBIST, repair programming, power, sensors). Channels 5..7
carry the second half of a dual-issue access and the credit returns.

Address decode on the dielet:

```
bank = set[4:0] ^ {1'b0, way}     32 banks   (the XOR spreads sequential sets)
sub  = set[8:5]                   16 subarrays
row  = {set[14:9], way}           1024 rows
quadrant = which 16 B quarter     4 arrays
```

The link is now DDR dual-edge (forwarded 1.5 GHz clock sampled on both edges),
so each 144-bit lane carries two sublines per 3.0 GHz base cycle and the
serialisation gearbox is removed.  Per slice the link moves 8 x 144 b x 2 at
2.2 GHz array = 432 GB/s, 1.30 TB/s for the package -- comfortably above the
~700 GB/s AMD quotes for an 8-core CCD.  The HN-F adapter exposes a native
512-bit CHI DAT port, so a 64 B line is one beat (64 B/cycle) rather than two
256-bit flits.

## 5. What is on which die

The dielet contains **no cache control logic at all** -- no tags, no MSHRs, no
coherence, exactly like a real V-Cache dielet. It has the array, the bond
endpoint, the repair registers (programmed from the base die), per-bank power
switches, and 16 thermal sensors. That is what lets it be built on a
density-optimised process and stay under 0.62 W (see `PPA_REPORT.md`).

### 5.1 Inverted die stack (Zen 5 packaging)

The package uses the Zen 5 inverted topology: the 3D-SRAM dielet is **face-up
on the package substrate**, the base logic die is bonded **face-down onto the
dielet**, and the base die backside is ground into direct contact with the
lid/cooling solution.  The dielet is therefore outside the hot base-die-to-lid
thermal path.  The +3.3 °C stacking penalty is reduced to +0.5 °C, raising the
core thermal throttle headroom by ~15–20 %.

### 5.2 Direct SRAM macro slicing

Each 1024 × 148 subarray is banked into four 256 × 148 macro slices with
divided local bitlines (`vc3d_stack_macro_cell`, generated
`vc3d_stack_macro_slice_array.v`).  The macro access time drops from ~465 ps
to ~260 ps, so the dielet runs at `VC3D_STACK_DIELET_CLOCK_MHZ` (2.2 GHz)
instead of 1.5 GHz -- the largest single contributor to the stacked latency
reduction.

## 6. Degradation, not failure

Every failure mode has a graceful path:

| failure | response |
|---|---|
| single-bit error in a line | corrected (CE), counted, scrubbed |
| double-bit error | UE, response poisoned, line invalidated |
| failing row/column | spare row/column, burned into eFuse |
| failing bond lane | remapped onto one of 8 spare lanes per channel |
| >8 dead lanes in a channel | the ways that channel serves are retired |
| dielet absent or link down | the cache runs as a 24 MiB base-only cache |
| over temperature | 4-step throttle, then bank power-down |

The capacity loss is always a whole number of ways, so the cache stays
functionally correct at reduced associativity.
