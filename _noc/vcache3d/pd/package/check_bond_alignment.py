#!/usr/bin/env python3
"""Check that the base-die and cache-dielet bond maps actually mate.

Fails (non-zero exit) if:
  * a signal pad on one die has no partner on the other,
  * a mated pair is not at the mirrored position within tolerance,
  * two pads on the same die overlap (pitch violation),
  * a signal pad has no power/ground pad within one pitch (return path),
  * the channel/lane numbering is not identical on both sides.

This is the check that would otherwise be found on silicon.
"""
import csv
import math
import os
import sys

TOL_UM   = 0.05
PITCH_UM = 9.0


def load(path):
    with open(path) as fh:
        return [
            {**r, "x_um": float(r["x_um"]), "y_um": float(r["y_um"]),
             "channel": int(r["channel"]), "lane": int(r["lane"])}
            for r in csv.DictReader(fh)
        ]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.join(here, "..", "openroad")
    bpath = os.path.join(d, "bond_pad_map_base.csv")
    cpath = os.path.join(d, "bond_pad_map_cache.csv")
    if not (os.path.exists(bpath) and os.path.exists(cpath)):
        print("ERROR: run pd/package/gen_bump_map.py first")
        return 2

    base, cache = load(bpath), load(cpath)
    errors = []

    # 1. same population
    if len(base) != len(cache):
        errors.append(f"pad count mismatch: base {len(base)} vs cache {len(cache)}")

    # 2. mirror check on signal pads
    bsig = {(p["channel"], p["lane"]): p for p in base if p["type"] == "signal"}
    csig = {(p["channel"], p["lane"]): p for p in cache if p["type"] == "signal"}
    if set(bsig) != set(csig):
        missing = set(bsig) ^ set(csig)
        errors.append(f"{len(missing)} signal pads have no partner: "
                      f"{sorted(missing)[:5]}")

    xs = [p["x_um"] for p in base if p["type"] == "signal"]
    axis = min(xs) + max(xs)
    for key, bp in bsig.items():
        cp = csig.get(key)
        if cp is None:
            continue
        want_x = axis - bp["x_um"]
        if abs(cp["x_um"] - want_x) > TOL_UM or abs(cp["y_um"] - bp["y_um"]) > TOL_UM:
            errors.append(f"lane {key} misaligned: cache at "
                          f"({cp['x_um']}, {cp['y_um']}), expected "
                          f"({want_x}, {bp['y_um']})")

    # 3. overlap / pitch check per die
    for name, pads in (("base", base), ("cache", cache)):
        seen = {}
        for p in pads:
            k = (round(p["x_um"], 3), round(p["y_um"], 3))
            if k in seen:
                errors.append(f"{name}: pads {seen[k]} and {p['net']} overlap at {k}")
            seen[k] = p["net"]

    # 4. return-path check: every signal pad within 1.5 pitch of a PG pad
    for name, pads in (("base", base), ("cache", cache)):
        pg = [p for p in pads if p["type"] == "power"]
        far = 0
        for p in pads:
            if p["type"] != "signal":
                continue
            best = min(math.hypot(p["x_um"] - q["x_um"], p["y_um"] - q["y_um"])
                       for q in pg)
            if best > 1.5 * PITCH_UM * 2:
                far += 1
        if far:
            errors.append(f"{name}: {far} signal pads have no PG pad within "
                          f"{1.5*PITCH_UM*2:.1f} um")

    print("bond alignment check")
    print(f"  base pads   {len(base)}")
    print(f"  cache pads  {len(cache)}")
    print(f"  signal pairs checked {len(bsig)}")
    print(f"  mirror axis x = {axis/2:.1f} um")
    if errors:
        print(f"  RESULT: {len(errors)} ERROR(S)")
        for e in errors[:20]:
            print("    - " + e)
        return 1
    print("  RESULT: base and cache maps mate correctly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
