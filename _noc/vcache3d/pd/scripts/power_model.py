#!/usr/bin/env python3
"""VCACHE-3D power model.

Activity-based dynamic power plus leakage, per tier, at three operating points.
Energies come from the macro datasheet numbers in pd/sram/sram_compiler_selection.md
and from CV^2 estimates for the bond link and the logic.
"""

# ---- per-access energies (pJ) ----------------------------------------------
E_HD_MACRO_RD   = 0.62      # 1024x148 HD macro, 0.75 V nominal
E_HD_MACRO_WR   = 0.74
E_HP_TAG_RD     = 0.41      # 8192x39 HP macro
E_HP_DATA_RD    = 2.90      # 8192x576 HP macro
E_HP_DATA_WR    = 3.35
E_BOND_BIT      = 0.019     # 2.4 fF pad + 0.9 mm wire at 0.75 V
E_ECC_ENC_LINE  = 1.10      # 4 x SECDED(128,9) encode
E_ECC_DEC_LINE  = 1.45
E_PIPE_LINE     = 4.20      # slice pipeline + router per request

# ---- leakage (mW) -----------------------------------------------------------
# Leakage is calibrated to ~2 mW/Mb for the HD dielet macros and ~6 mW/Mb for
# the HP base-die macros at 85 C, which is where N6/N5 foundry SRAM sits; SRAM
# leakage, not switching, dominates a cache this large.
LKG_HD_MACRO_MW   = 0.300    # 1024x148 = 148 kb HD macro at 85 C, 0.75 V
LKG_HD_RETENT_MW  = 0.060    # same macro in 0.55 V retention
LKG_HP_TAG_MW     = 2.00     # 8192x39  = 312 kb HP macro
LKG_HP_DATA_MW    = 28.0     # 8192x576 = 4.6 Mb HP macro
LKG_BASE_LOGIC_MW = 46.0     # per slice, synthesised logic
LKG_CACHE_LOGIC_MW = 8.0     # per dielet

MACROS_PER_DIELET = 2048
TAG_MACROS  = 16 * 4         # 16 ways x 4 set banks
DATA_MACROS = 4 * 4          # 4 base ways x 4 set banks

BASE_GHZ  = 3.0
ARRAY_GHZ = 1.5


class Point:
    def __init__(self, name, req_per_cyc, stack_frac, hit_rate, scrub_on=True,
                 sleep_frac=0.0):
        self.name = name
        self.req = req_per_cyc          # requests per base-die cycle, all slices
        self.stack_frac = stack_frac    # fraction of accesses served by the dielet
        self.hit = hit_rate
        self.scrub_on = scrub_on
        self.sleep_frac = sleep_frac    # fraction of dielet banks in retention

    def compute(self):
        f = BASE_GHZ * 1e9
        acc = self.req * f              # accesses per second

        # ---- base die ----
        e_tag  = 16 * E_HP_TAG_RD                    # all ways read in parallel
        e_data = (1 - self.stack_frac) * E_HP_DATA_RD
        e_ecc  = E_ECC_DEC_LINE + 0.3 * E_ECC_ENC_LINE
        e_pipe = E_PIPE_LINE
        e_bond = self.stack_frac * 2 * 8 * 148 * E_BOND_BIT  # cmd + data, 8 ch
        base_dyn_pj = acc * (e_tag + e_data + e_ecc + e_pipe + e_bond)
        base_dyn_mw = base_dyn_pj * 1e-9

        base_lkg = 3 * (TAG_MACROS * LKG_HP_TAG_MW +
                        DATA_MACROS * LKG_HP_DATA_MW +
                        LKG_BASE_LOGIC_MW)

        # ---- cache dielets ----
        stack_acc = acc * self.stack_frac
        # one line access touches 4 quadrant macros
        cache_dyn_mw = stack_acc * 4 * E_HD_MACRO_RD * 1e-9
        awake = MACROS_PER_DIELET * (1 - self.sleep_frac)
        asleep = MACROS_PER_DIELET * self.sleep_frac
        cache_lkg = 3 * (awake * LKG_HD_MACRO_MW +
                         asleep * LKG_HD_RETENT_MW +
                         LKG_CACHE_LOGIC_MW)

        # ---- scrub ----
        scrub_mw = 0.0
        if self.scrub_on:
            # one line every 4096 cycles per slice, burst of 4
            scrub_acc = 3 * f / 4096 * 4
            scrub_mw = scrub_acc * (E_HP_DATA_RD + E_ECC_DEC_LINE) * 1e-9

        return {
            "base_dyn": base_dyn_mw,
            "base_lkg": base_lkg,
            "cache_dyn": cache_dyn_mw,
            "cache_lkg": cache_lkg,
            "scrub": scrub_mw,
            "total": base_dyn_mw + base_lkg + cache_dyn_mw + cache_lkg + scrub_mw,
            "acc": acc,
        }


POINTS = [
    Point("idle (retention, no traffic)",      0.000, 0.75, 0.0, True,  0.90),
    Point("light (0.05 req/cyc)",              0.050, 0.75, 0.9, True,  0.50),
    Point("nominal (0.30 req/cyc)",            0.300, 0.75, 0.9, True,  0.10),
    Point("peak (1.00 req/cyc, 3 slices)",     1.000, 0.75, 0.9, False, 0.00),
]


def main():
    print("=" * 78)
    print("VCACHE-3D  --  power model")
    print("=" * 78)
    print(f"  base tier {BASE_GHZ} GHz / 0.95 V   cache tier {ARRAY_GHZ} GHz / 0.75 V")
    print()
    hdr = f"{'operating point':<32}{'base':>9}{'dielet':>9}{'scrub':>8}{'total':>9}"
    print(hdr)
    print(f"{'':<32}{'mW':>9}{'mW':>9}{'mW':>8}{'mW':>9}")
    print("-" * 78)
    for pt in POINTS:
        r = pt.compute()
        print(f"{pt.name:<32}"
              f"{r['base_dyn']+r['base_lkg']:9.0f}"
              f"{r['cache_dyn']+r['cache_lkg']:9.0f}"
              f"{r['scrub']:8.2f}"
              f"{r['total']:9.0f}")
    print()
    peak = POINTS[-1].compute()
    nom  = POINTS[2].compute()
    print(f"  peak total          {peak['total']/1000:.2f} W")
    print(f"  nominal total       {nom['total']/1000:.2f} W")
    print(f"  dielet peak (each)  {(peak['cache_dyn']+peak['cache_lkg'])/3/1000:.2f} W"
          f"   over 10.57 mm^2")
    dens = (peak['cache_dyn'] + peak['cache_lkg']) / 3 / 10.57 / 1000.0
    print(f"  dielet power density {dens:.3f} W/mm^2  "
          f"(thermal model uses this as the stacked-layer source term)")
    print()
    print("  Energy per L3 hit:")
    for frac, label in ((0.0, "base-die way (0..3) "), (1.0, "stacked way (4..15)")):
        e = (16 * E_HP_TAG_RD + (1 - frac) * E_HP_DATA_RD + E_ECC_DEC_LINE +
             E_PIPE_LINE + frac * (2 * 8 * 148 * E_BOND_BIT + 4 * E_HD_MACRO_RD))
        print(f"    {label}: {e:6.1f} pJ  ({e/64:.2f} pJ/byte)")
    print()
    print("  Note: the stacked hit costs more energy per access than the base-die")
    print("  hit because of the bond link, but far less than the DRAM access it")
    print("  replaces (~"
          "20 pJ/byte for DDR5), which is the entire point of the dielet.")
    print()
    print("  Power management implemented in RTL (rtl/power/):")
    print("    * per-bank sleep / deep-sleep / retention (bank_sleep[31:0])")
    print("    * 8 DVFS levels with a voltage/frequency handshake")
    print("    * thermal throttle driven by 16 on-dielet sensors")
    print("    * scrub suppression while throttled")


if __name__ == "__main__":
    main()
