#!/usr/bin/env python3
"""VCACHE-3D area model: macro area + synthesised logic area, per tier.

Logic area is estimated from an instance count derived from the RTL structure
(flop count x flop area + combinational gate equivalents), which is accurate to
about +/-15 % pre-synthesis and is enough to size the floorplan in
pd/scripts/floorplan_*.tcl.
"""

# ---- technology constants ---------------------------------------------------
FLOP_UM2      = 0.55      # N5-class DFF with reset
NAND2_UM2     = 0.16
SRAM_HD_BIT   = 0.0199    # um^2 per bit, N6 HD, includes periphery derate
SRAM_HP_BIT   = 0.0312    # um^2 per bit, N5 HP
PERIPH_DERATE = 1.37      # macro area / raw bitcell area
BOND_PAD_UM2  = 81.0      # 9 um pitch -> 81 um^2 per pad site
UTIL_BASE     = 0.55
UTIL_CACHE    = 0.82


def mm2(um2):
    return um2 / 1e6


# ---- cache dielet -----------------------------------------------------------
BANKS, SUBS, ROWS, BITS, QUADS = 32, 16, 1024, 148, 4
sub_bits   = ROWS * BITS
sub_area   = sub_bits * SRAM_HD_BIT * PERIPH_DERATE
macros     = SUBS * BANKS * QUADS
array_area = sub_area * macros

# cache-die logic: bond endpoint + decode + repair registers
cache_flops = (
    8 * 172 * 4          # per-lane rx/deskew flops
    + 8 * 200            # per-channel link layer
    + BANKS * BITS * QUADS   # bank output registers (all four quadrants)
    + SUBS * BANKS * (4 * 11 + 4 * 7)   # shared repair registers per subarray
    + 16 * 20            # thermal sensors
)
cache_gates = cache_flops * 6
cache_logic = cache_flops * FLOP_UM2 + cache_gates * NAND2_UM2
cache_pads  = 8 * 172 * BOND_PAD_UM2

cache_core  = (array_area + cache_logic + cache_pads) / UTIL_CACHE

# ---- base die, per slice ----------------------------------------------------
SETS, WAYS = 32768, 16
tag_bits   = SETS * WAYS * 39
tag_area   = tag_bits * SRAM_HP_BIT * PERIPH_DERATE
base_dbits = SETS * 4 * 576
base_darea = base_dbits * SRAM_HP_BIT * PERIPH_DERATE
rrpv_bits  = SETS * WAYS * 2
rrpv_area  = rrpv_bits * SRAM_HP_BIT * PERIPH_DERATE

slice_flops = (
    4000        # pipeline, MSHR-ish state, response staging
    + 2500      # scrub + CE tracker + error log
    + 3000      # BISR + MBIST + eFuse shadow
    + 8 * 172 * 3   # bond base-side lane flops
    + 64 * 48   # performance counters
    + 1500      # CSR
)
slice_gates = slice_flops * 7
slice_logic = slice_flops * FLOP_UM2 + slice_gates * NAND2_UM2
slice_pads  = 8 * 172 * BOND_PAD_UM2

slice_area  = (tag_area + base_darea + rrpv_area + slice_logic + slice_pads)

router_flops = 3 * 48 + 8 * (48 + 512 + 64 + 12 + 6 + 4) + 600
router_logic = router_flops * FLOP_UM2 + router_flops * 8 * NAND2_UM2

base_core = (3 * slice_area + router_logic) / UTIL_BASE


def main():
    print("=" * 78)
    print("VCACHE-3D  --  area model")
    print("=" * 78)
    print()
    print("CACHE DIELET (one per slice, three per package)")
    print(f"  subarray                    {ROWS} x {BITS} b = {sub_bits:,} b")
    print(f"  subarray macro area         {sub_area:10.0f} um^2  ({mm2(sub_area):.4f} mm^2)")
    print(f"  quadrants (line quarters)   {QUADS}")
    print(f"  macros per dielet           {macros}  ({SUBS*BANKS} per quadrant)")
    print(f"  array area                  {mm2(array_area):10.3f} mm^2")
    print(f"  cache-die logic             {mm2(cache_logic):10.3f} mm^2  "
          f"({cache_flops:,} flops)")
    print(f"  bond pad field (8 x 172)    {mm2(cache_pads):10.3f} mm^2  "
          f"({8*172} pads @ 9 um pitch)")
    print(f"  core area @ {UTIL_CACHE*100:.0f}% util      {mm2(cache_core):10.3f} mm^2")
    payload_mib = macros*ROWS*128/8/1024/1024
    print(f"  physical capacity           {payload_mib:.0f} MiB payload "
          f"(+ {macros*ROWS*20/8/1024/1024:.0f} MiB ECC/metadata)")
    print(f"  mapped in hybrid mode       24 MiB (ways 4..15); "
          f"{payload_mib:.0f} MiB in stack-only mode")
    print(f"  bit density                 "
          f"{macros*sub_bits/1e6/mm2(cache_core):.1f} Mb/mm^2")
    print()
    print("BASE DIE")
    print(f"  tag array per slice         {mm2(tag_area):10.3f} mm^2  "
          f"({tag_bits/8/1024/1024:.2f} MiB of tag)")
    print(f"  base data array per slice   {mm2(base_darea):10.3f} mm^2  "
          f"({base_dbits/8/1024/1024:.1f} MiB coded = 8 MiB data)")
    print(f"  RRPV array per slice        {mm2(rrpv_area):10.3f} mm^2")
    print(f"  slice logic                 {mm2(slice_logic):10.3f} mm^2  "
          f"({slice_flops:,} flops)")
    print(f"  bond pad field per slice    {mm2(slice_pads):10.3f} mm^2")
    print(f"  ---- per slice total        {mm2(slice_area):10.3f} mm^2")
    print(f"  router + global             {mm2(router_logic):10.3f} mm^2")
    print(f"  base core @ {UTIL_BASE*100:.0f}% util       {mm2(base_core):10.3f} mm^2")
    print()
    print("PACKAGE TOTAL")
    total_si = base_core + 3 * cache_core
    print(f"  base die                    {mm2(base_core):10.3f} mm^2")
    print(f"  3 x cache dielet            {3*mm2(cache_core):10.3f} mm^2")
    print(f"  total silicon               {mm2(total_si):10.3f} mm^2")
    print(f"  cache capacity              96 MiB")
    print(f"  silicon per MiB             {mm2(total_si)/96:10.4f} mm^2/MiB")
    print()
    print("  Comparison point: a monolithic 96 MiB L3 built entirely from HP")
    print("  base-die SRAM would need")
    mono = 96*1024*1024*8*1.075 * SRAM_HP_BIT * PERIPH_DERATE / UTIL_BASE
    print(f"      {mm2(mono):.1f} mm^2 of base-die area (vs "
          f"{mm2(base_core):.1f} mm^2 here),")
    print("  i.e. stacking removes ~{:.0f} mm^2 from the expensive die."
          .format(mm2(mono) - mm2(base_core)))
    print()
    print("  Floorplan targets written to pd/openroad/config.mk:")
    import math
    bs = math.sqrt(mm2(base_core)) * 1000
    cs = math.sqrt(mm2(cache_core)) * 1000
    print(f"      base tier  DIE_AREA  0 0 {bs:.0f} {bs:.0f} (um)")
    print(f"      cache tier DIE_AREA  0 0 {cs:.0f} {cs:.0f} (um)")


if __name__ == "__main__":
    main()
