# VCACHE-3D — SRAM compiler selection and macro budget

This document pins the memory macros the design is built against. Nothing in
the RTL assumes a specific vendor: `vc3d_stack_subarray.v` and `vc3d_sram_sp.v`
each have a ``` `VC3D_SRAM_COMPILER ``` branch whose pin list is the one
tabulated here, and a behavioural branch used for simulation.

## 1. Process assumptions

| Item | Base die | Cache dielet |
|---|---|---|
| Node | N5-class logic | N6-class high-density SRAM variant |
| Bitcell | HD 6T, 0.021 µm² | HD 6T, 0.0199 µm² (density-optimised variant) |
| Nominal VDD | 0.95 V (SVT logic) | 0.75 V array + 0.85 V periphery |
| Target f | 3.0 GHz | 2.0 GHz array access |
| Wafer bonding | — | face-to-face hybrid bond, 9 µm pad pitch |

The cache dielet deliberately uses an **older, cheaper, denser** node than the
base die. That is the same economic argument AMD makes for V-Cache: SRAM
scales poorly with logic nodes, so putting cache on a separate, density-tuned
die is cheaper per bit than growing the compute die.

## 2. Macro selection

### 2.1 Stacked array subarray — `VC3D_HD_SPSRAM_1024X148`

| Parameter | Value | Why |
|---|---|---|
| Words | 1024 | one word line group per subarray; keeps WL RC inside the 2.0 GHz budget |
| Bits/word | 148 | 144 payload + 4 spare bit lines (one per column group) |
| Ports | 1RW | a 2-port bitcell is ~1.4× the area; capacity wins |
| Column mux | 4:1 | balances bit-line length against sense-amp count |
| Redundancy | 4 spare rows, 4 spare columns | see §4 |
| Assist | WA + RA, 4-bit trim each | mandatory at 0.75 V |
| Power modes | SLP / DSLP / RET | retention keeps data at ~0.55 V |
| Area (est.) | 0.00413 mm² | 1024×148 = 151,552 b at 0.0199 µm²/bit ×1.37 periphery overhead = 4132 µm² |
| Access time | 380 ps typ / 465 ps SS 0.68 V 125 °C | fits the 500 ps array cycle |
| Read energy | 0.62 pJ/access | used by the power model |
| Leakage | 21 µA/macro @ 85 °C active, 2.1 µA retention | used by the power model |

Instances: 32 banks × 16 subarrays × 4 line-quarter quadrants = **2048 macros
per cache dielet** (512 per quadrant),
1536 across the three dielets.

### 2.2 Base-die data array — `VC3D_HP_SPSRAM_32768X576` (built as 8 sub-macros)

576-bit words are wider than any single compiler instance; the base data array
is therefore assembled from 8 × `HP_SPSRAM_32768X72` macros per way, 4 ways per
slice = 32 macros/slice.

| Parameter | Value |
|---|---|
| Words × bits | 32768 × 72 |
| Access time | 285 ps typ (HP bitcell, 0.95 V) |
| Area | 0.229 mm² per macro |
| Read energy | 3.1 pJ/access |

### 2.3 Tag array — `VC3D_HP_SPSRAM_32768X39`

16 ways × 1 macro per way per slice = 48 macros total.
Access time 240 ps typ; this is the frequency-limiting array on the base die,
which is why the tag decode + ECC + compare occupies its own pipeline stage.

## 3. Total macro count and area

| Block | Macros | Unit area (mm²) | Total (mm²) |
|---|---|---|---|
| Stacked subarrays (per dielet) | 2048 | 0.00413 | 8.46 |
| Base data (per slice) | 32 | 0.229 | 7.33 |
| Tag (per slice) | 16 | 0.124 | 1.98 |

Per-slice base-die memory area = 9.31 mm²; ×3 slices = 27.93 mm².
Per-dielet area = 8.46 mm² array / 0.82 utilisation + 0.11 mm² bond field +
0.10 mm² periphery logic = **10.57 mm²** (see `pd/scripts/area_model.py`, which
is the single source of truth for these numbers).  That is 32 MiB of payload
per dielet at 29.4 Mb/mm², of which 24 MiB is mapped in the default hybrid
configuration and all 32 MiB in stack-only mode.

## 4. Redundancy budget and yield

Per subarray: 4 spare rows + 4 spare columns.
Per dielet: 2048 spare rows and 2048 spare columns (the four quadrant macros
of a row block are ganged and share one repair entry), plus 8 spare bond lanes per
channel (64 per dielet).

With a defect density D0 = 0.07 defects/cm² and a 10.57 mm² dielet, the raw
Poisson yield of an unrepaired die would be

    Y_raw = exp(-0.07 * 0.1057) = 99.26 %

The repair scheme covers single-bit, single-row, single-column and single-lane
defects; the dominant residual is multi-row cluster defects. Modelling clusters
as 8 % of defects gives a post-repair yield of

    Y_repaired = 1 - 0.08 * (1 - 0.9926) = 99.94 %

which is what makes a 3-dielet product viable: the compound yield of three
dielets plus the base die is 0.9987³ × Y_base.

## 5. Pin mapping used by the RTL

```
VC3D_HD_SPSRAM_1024X148 (
    CLK, CEN, WEN, A[9:0], D[147:0], M[147:0], Q[147:0],
    WA[3:0], RA[3:0], SLP, DSLP, RET )
```
`vc3d_stack_subarray.v` drives exactly these pins in its
``` `VC3D_SRAM_COMPILER ``` branch; the redundancy steering is done in RTL
*outside* the macro, so a compiler without built-in redundancy can be used.
