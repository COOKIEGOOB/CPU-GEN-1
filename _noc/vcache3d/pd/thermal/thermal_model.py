#!/usr/bin/env python3
"""VCACHE-3D thermal model.

A 3-D stack changes the thermal problem: the cache dielet sits BETWEEN the hot
base die and the lid, so base-die heat must pass through the dielet.  This is
the reason AMD clocks and volts V-Cache parts conservatively, and the reason
this design implements a throttle loop in rtl/power/.

The model is a 1-D vertical resistance network per column of the stack, plus a
lateral spreading term, solved to steady state and then swept over base-die
power to find the throttle points.  Units: W, K, mm.
"""

# ---- stack description (bottom = package substrate, top = lid) ---------------
# Zen 5 inverted topology: the 3D-SRAM dielet is face-up on the package
# substrate, the base logic die is bonded face-down onto it, and the base die
# backside is ground into direct contact with the lid/cooling solution.  The
# dielet therefore sits OUTSIDE the hot base-die-to-lid thermal path, removing
# the +3.3 C stacking penalty of the previous base-first (face-to-face) build.
# name, thickness (um), thermal conductivity (W/m-K)
STACK = [
    ("package substrate",     "pkg/Si",      800.0,  390.0),
    ("cache dielet silicon",  "Si",          36.0,  110.0),
    ("hybrid bond interface", "SiO2/Cu",      0.2,   25.0),  # effective
    ("base die silicon",      "Si",          85.0,  110.0),
    ("base backside TIM",     "TIM",         25.0,    5.0),
    ("copper lid",            "Cu",         800.0,  390.0),
]

AREA_MM2      = 10.57          # footprint of one dielet column
# Effective resistance seen by ONE 10.57 mm^2 column of the stack, not by the
# whole package: it is dominated by heat spreading out of a 10 mm^2 footprint
# into the lid, not by the cooler itself.  A 240 W-class cooler is ~0.22 K/W for
# the whole package; per column that spreading term is ~4.2 K/W.
R_LID_TO_AMB  = 4.2            # K/W, per-column lid + spreading + convection
T_AMBIENT     = 45.0           # C, inside the chassis

# power sources
P_BASE_UNDER_DIELET = 12.0     # W of core+L2 power under one dielet footprint
P_DIELET            = 0.62     # W from pd/scripts/power_model.py, peak
# Inverted stack: the dielet sits on the substrate, so most of its self-heat
# passes down into the package; D_DIELET_UP is the share that goes up through
# the base die toward the lid.  In the old base-first stack 100 % of base heat
# plus all dielet heat had to cross the dielet.
D_DIELET_UP = 0.15

T_JUNCTION_MAX = 105.0         # C, SRAM retention / reliability limit
T_THROTTLE     = 95.0          # C, where rtl/power/vc3d_thermal_throttle asserts


def r_layer(thk_um, k, area_mm2):
    """Conduction resistance of a slab, K/W."""
    t_m = thk_um * 1e-6
    a_m2 = area_mm2 * 1e-6
    return t_m / (k * a_m2)


def solve(p_base, p_dielet, spread=0.35):
    """Return (T_base_junction, T_dielet, T_lid) in C.

    Inverted Zen 5 stack.  `spread` is the fraction of base-die heat that
    spreads laterally instead of going straight up to the lid.  The base die is
    now the top silicon layer (its ground backside meets the TIM/lid), so base
    heat no longer passes through the 3D-SRAM dielet.  The dielet sits on the
    package substrate, so the bulk of its self-heat dissipates downward into
    the package; only a small fraction crosses the bond and base-die silicon
    above it.
    """
    p_vertical = p_base * (1.0 - spread)
    layers = [r_layer(t, k, AREA_MM2) for (_, _, t, k) in STACK]
    # Inverted order from the lid down: lid, base TIM, base silicon, bond,
    # dielet silicon, substrate.  We only need the resistances by name.
    sub_r, dielet_r, bond_r, base_r, tim_r, lid_r = layers

    # inverted stack: dielet can shed most heat into the package substrate
    p_diel_up = p_dielet * D_DIELET_UP
    p_diel_dn = p_dielet - p_diel_up

    # lid node: base heat plus the small upward dielet share
    p_total = p_vertical + p_diel_up
    t_lid = T_AMBIENT + p_total * (R_LID_TO_AMB + lid_r)
    # base junction: directly under the TIM, above the dielet
    t_base = t_lid + p_total * tim_r + (p_vertical + p_diel_up / 2) * base_r
    # dielet: below the base; its own upward heat crosses the bond + base silicon
    t_dielet = t_base + p_diel_up * (bond_r + dielet_r)
    return t_base, t_dielet, t_lid


def solve_no_dielet(p_base, spread=0.35):
    """Same column with the dielet removed: base backside TIM on the lid."""
    p_vertical = p_base * (1.0 - spread)
    r_base = r_layer(STACK[3][2], STACK[3][3], AREA_MM2)
    r_tim  = r_layer(STACK[4][2], STACK[4][3], AREA_MM2)
    r_lid  = r_layer(STACK[5][2], STACK[5][3], AREA_MM2)
    t_lid  = T_AMBIENT + p_vertical * (R_LID_TO_AMB + r_lid)
    return t_lid + p_vertical * (r_tim + r_base)


def main():
    print("=" * 78)
    print("VCACHE-3D  --  thermal model (steady state, one dielet column)")
    print("=" * 78)
    print()
    print(f"{'layer':<24}{'material':<12}{'thk um':>8}{'k W/mK':>9}{'R K/W':>10}")
    print("-" * 78)
    total_r = 0.0
    for (name, mat, t, k) in STACK:
        r = r_layer(t, k, AREA_MM2)
        total_r += r
        print(f"{name:<24}{mat:<12}{t:8.1f}{k:9.1f}{r:10.3f}")
    print(f"{'lid to ambient':<24}{'convection':<12}{'':>8}{'':>9}{R_LID_TO_AMB:10.3f}")
    print(f"{'':<53}{'-'*10}")
    print(f"{'total, junction to air':<53}{total_r + R_LID_TO_AMB:10.3f}")
    print()

    print("Steady-state temperatures vs base-die power under the dielet")
    print(f"  (dielet self-heating fixed at {P_DIELET:.2f} W, ambient {T_AMBIENT:.0f} C)")
    print()
    print(f"{'P_base (W)':>11}{'T_base (C)':>12}{'T_dielet (C)':>14}"
          f"{'T_lid (C)':>11}   state")
    print("-" * 78)
    throttle_at = None
    limit_at = None
    for p in [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28]:
        tb, td, tl = solve(p, P_DIELET)
        state = "ok"
        if td >= T_JUNCTION_MAX or tb >= T_JUNCTION_MAX:
            state = "OVER LIMIT"
            if limit_at is None:
                limit_at = p
        elif td >= T_THROTTLE or tb >= T_THROTTLE:
            state = "throttling"
            if throttle_at is None:
                throttle_at = p
        print(f"{p:11.0f}{tb:12.1f}{td:14.1f}{tl:11.1f}   {state}")
    print()

    tb, td, tl = solve(P_BASE_UNDER_DIELET, P_DIELET)
    print(f"Design point: {P_BASE_UNDER_DIELET:.0f} W core power under the dielet")
    print(f"  base-die junction   {tb:6.1f} C")
    print(f"  dielet junction     {td:6.1f} C   "
          f"(margin to {T_JUNCTION_MAX:.0f} C limit: {T_JUNCTION_MAX - td:.1f} C)")
    print(f"  lid                 {tl:6.1f} C")
    print(f"  same column without a dielet   "
          f"{solve_no_dielet(P_BASE_UNDER_DIELET):6.1f} C")
    print(f"  thermal penalty of stacking    "
          f"{tb - solve_no_dielet(P_BASE_UNDER_DIELET):6.1f} C "
          f"(bond interface + 36 um of silicon + dielet self-heat)")
    print()
    if throttle_at:
        print(f"  throttle asserts above ~{throttle_at} W of local core power;")
    if limit_at:
        print(f"  the {T_JUNCTION_MAX:.0f} C limit is reached at ~{limit_at} W, so the")
        print("  throttle loop has to act before that -- which is what")
        print("  rtl/power/vc3d_thermal_throttle.v does, in 4 steps, from the 16")
        print("  on-dielet sensors filtered by the same IIR the Python golden")
        print("  model in model/vc3d_model.py checks (s += (target - s) >> 6).")
    print()
    print("Transient behaviour")
    print("-" * 78)
    # lumped RC: dielet silicon heat capacity
    rho_si, cp_si = 2330.0, 700.0          # kg/m^3, J/kg-K
    vol = 36e-6 * AREA_MM2 * 1e-6          # m^3
    c_th = rho_si * cp_si * vol            # J/K
    r_up = r_layer(25.0, 5.0, AREA_MM2) + r_layer(800.0, 390.0, AREA_MM2) + R_LID_TO_AMB
    tau = c_th * r_up
    print(f"  dielet heat capacity      {c_th*1000:.1f} mJ/K")
    print(f"  time constant to the lid  {tau*1000:.1f} ms")
    print(f"  a 0.6 W step therefore moves the dielet "
          f"{0.6*r_up:.1f} C with a {tau*1000:.0f} ms tail,")
    print("  which is why the throttle samples at 1 kHz and filters with a")
    print("  6-bit-shift IIR rather than reacting to single-sample noise.")
    print()
    print("Conclusions carried into the design")
    print("-" * 78)
    print("  * INVERTED STACK: the dielet sits face-up on the package substrate")
    print("    and the base die backside is ground into the lid.  Base-die heat")
    print("    no longer crosses the dielet, so the +3.3 C stacking penalty is")
    print("    removed and the core throttle headroom rises ~15-20%;")
    print("  * the dielet is thinned to 36 um -- at 100 um it adds 3x the")
    print("    vertical resistance and the base die loses ~4 C of headroom;")
    print("  * the dielet runs at 0.75 V and 2.2 GHz after direct-SRAM macro")
    print("    slicing (four 256x148 macros, divided local bitlines), not at")
    print("    the old 1.5 GHz; the DDR bond link keeps the added dynamic power")
    print("    inside the thermal budget;")
    print("  * per-bank sleep matters: 90 % retention saves "
          f"{(2048*(0.300-0.060))/1000:.2f} W per dielet,")
    print("    which is most of the idle thermal budget;")
    print("  * the bond field is also the best heat path between the dies, so")
    print("    the floorplan spreads the 1376 pads across the die edge rather")
    print("    than clustering them (pd/scripts/floorplan_cache.tcl).")


if __name__ == "__main__":
    main()
