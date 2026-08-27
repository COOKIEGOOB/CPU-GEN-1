#!/usr/bin/env python3
"""Generate the hybrid-bond pad maps for the base die and the cache dielet.

The two dies are bonded FACE TO FACE, so the dielet map is the MIRROR of the
base map about the X axis of the bond field.  Getting that mirror wrong is the
classic 3D integration bug -- every lane connects to the wrong partner and the
link trains to garbage -- so both maps are emitted from this one script and
checked against each other by check_bond_alignment.py.

Field layout (per slice column):
  * 8 channels x 172 physical lanes = 1376 signal pads
  * plus power/ground pads on a 1:2 signal:PG ratio, interleaved so that every
    signal pad has a return path within one pitch
  * plus 24 clock/control pads (forwarded clock, reset, train, DFT)

Pitch is 9 um, which is the aggressive end of production hybrid bonding
(AMD's SoIC-X is in the same class).  The field is placed along the die edge
nearest the slice it serves.
"""
import csv
import os
import sys

PITCH_UM      = 9.0
CH_NUM        = 8
PHYS_LANE     = 172
PG_RATIO      = 2            # PG pads per 4 signal pads (1:2 => 2 per 4)
CTRL_PADS     = ["clk_fwd_p", "clk_fwd_n", "rst_n", "train_en", "train_ack",
                 "dft_tck", "dft_tms", "dft_tdi", "dft_tdo", "dft_trst_n",
                 "efuse_prog", "efuse_sense", "pwr_ok", "pwr_req",
                 "thermal_alert", "spare_ctl_0", "spare_ctl_1", "spare_ctl_2",
                 "spare_ctl_3", "spare_ctl_4", "spare_ctl_5", "spare_ctl_6",
                 "spare_ctl_7", "spare_ctl_8"]

ROWS_PER_CH   = 4            # 172 lanes -> 43 columns x 4 rows
COLS_PER_CH   = PHYS_LANE // ROWS_PER_CH   # 43

FIELD_X0      = 120.0        # um, offset of the field on the base die
FIELD_Y0      = 120.0
DIE_W_BASE    = 4838.0       # from pd/scripts/area_model.py
DIE_W_CACHE   = 3251.0


def build(die):
    """Return a list of pad dicts for `die` in ('base', 'cache')."""
    assert die in ("base", "cache")
    pads = []
    x_cursor = FIELD_X0

    for ch in range(CH_NUM):
        for col in range(COLS_PER_CH):
            for row in range(ROWS_PER_CH):
                lane = col * ROWS_PER_CH + row
                x = x_cursor + col * PITCH_UM
                y = FIELD_Y0 + row * PITCH_UM
                spare = lane >= (PHYS_LANE - 8)
                pads.append({
                    "net": f"bond_ch{ch}_lane{lane}" + ("_spare" if spare else ""),
                    "type": "signal",
                    "channel": ch,
                    "lane": lane,
                    "x_um": x,
                    "y_um": y,
                })
            # power/ground stitched into the same grid, one column every 4
            if (col % 4 == 3) or (col in (0, COLS_PER_CH - 1)):
                for row in range(ROWS_PER_CH):
                    pads.append({
                        "net": "VDD_BOND" if row % 2 == 0 else "VSS_BOND",
                        "type": "power",
                        "channel": ch,
                        "lane": -1,
                        "x_um": x_cursor + col * PITCH_UM + PITCH_UM / 2,
                        "y_um": FIELD_Y0 + row * PITCH_UM + PITCH_UM / 2,
                    })
        x_cursor += (COLS_PER_CH + 1) * PITCH_UM

    for i, name in enumerate(CTRL_PADS):
        pads.append({
            "net": name,
            "type": "control",
            "channel": -1,
            "lane": -1,
            "x_um": FIELD_X0 + i * PITCH_UM,
            "y_um": FIELD_Y0 - 2 * PITCH_UM,
        })

    if die == "cache":
        # face-to-face mirror about the vertical centre line of the bond field
        # mirror about the SIGNAL field centre line (the PG pads sit on a
        # half-pitch offset grid, so including them would shift the axis by
        # 4.5 um and every lane would land half a pitch off its partner)
        xs = [p["x_um"] for p in pads if p["type"] == "signal"]
        axis = (min(xs) + max(xs))
        for p in pads:
            p["x_um"] = round(axis - p["x_um"], 3)

    return pads


def write_csv(path, pads):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["net", "type", "channel", "lane",
                                           "x_um", "y_um"])
        w.writeheader()
        for p in pads:
            w.writerow(p)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "openroad")
    base = build("base")
    cache = build("cache")
    write_csv(os.path.join(out, "bond_pad_map_base.csv"), base)
    write_csv(os.path.join(out, "bond_pad_map_cache.csv"), cache)

    sig = sum(1 for p in base if p["type"] == "signal")
    pwr = sum(1 for p in base if p["type"] == "power")
    ctl = sum(1 for p in base if p["type"] == "control")
    xs = [p["x_um"] for p in base]
    ys = [p["y_um"] for p in base]
    print("bond pad map")
    print(f"  signal pads   {sig}   ({CH_NUM} ch x {PHYS_LANE} lanes)")
    print(f"  power pads    {pwr}")
    print(f"  control pads  {ctl}")
    print(f"  total         {len(base)}")
    print(f"  pitch         {PITCH_UM} um")
    print(f"  field extent  {max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} um")
    print(f"  field area    {(max(xs)-min(xs))*(max(ys)-min(ys))/1e6:.4f} mm^2")
    print(f"  base die      {DIE_W_BASE:.0f} um square -> field is "
          f"{100*(max(xs)-min(xs))/DIE_W_BASE:.1f} % of the edge")
    print("  wrote openroad/bond_pad_map_base.csv, bond_pad_map_cache.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
