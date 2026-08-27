#!/usr/bin/env python3
"""VCACHE-3D static timing model (pre-PnR signoff estimate).

Builds every timing path in the subsystem out of published-style component
delays (macro access time, gate delay, wire RC per mm, bond-pad RC) and reports
the slack at the target frequency.  This is what establishes the FREQUENCY
claim in docs/PPA_REPORT.md; it is replaced by OpenROAD STA once a PDK is
available, and the path list here is exactly the path list the SDC constrains.

Delay primitives (N5-class logic, N6-class HD SRAM, SS corner unless noted):
    inverter FO4              14 ps
    2-input gate FO4          18 ps
    flop CK->Q                26 ps
    flop setup                18 ps
    clock skew budget         25 ps
    wire, M4 intermediate     0.22 ps/µm (with optimal buffering)
    wire, M8 semi-global      0.09 ps/µm
    hybrid bond pad + ESD     31 ps round trip (9 µm pitch, ~2.4 fF)
"""

FO4   = 14.0
GATE  = 18.0
CKQ   = 26.0
SETUP = 18.0
SKEW  = 25.0
JITTER = 8.0
WIRE_M4 = 0.22      # ps per um
WIRE_M8 = 0.09
BOND    = 31.0

TARGET_BASE_GHZ  = 3.0
TARGET_ARRAY_GHZ = 1.5


def xor_tree(inputs):
    """delay of a balanced XOR tree with `inputs` terms"""
    levels = max(1, (inputs - 1).bit_length())
    return levels * GATE * 1.15          # XOR2 is ~1.15x a NAND2


def mux(inputs):
    levels = max(1, (inputs - 1).bit_length())
    return levels * GATE


class Path:
    def __init__(self, name, tier, stages):
        self.name = name
        self.tier = tier
        self.stages = stages           # list of (description, delay_ps)

    @property
    def total(self):
        return CKQ + sum(d for _, d in self.stages) + SETUP + SKEW + JITTER

    def report(self, period_ps):
        slack = period_ps - self.total
        yield f"  Path: {self.name}   [{self.tier} tier]"
        yield f"    clk->Q                                  {CKQ:7.1f} ps"
        for desc, d in self.stages:
            yield f"    {desc:<38}  {d:7.1f} ps"
        yield f"    setup + skew + jitter                   {SETUP+SKEW+JITTER:7.1f} ps"
        yield f"    {'-'*54}"
        yield f"    arrival                                 {self.total:7.1f} ps"
        yield f"    required (period)                       {period_ps:7.1f} ps"
        yield f"    SLACK                                   {slack:+7.1f} ps  " \
              f"{'MET' if slack >= 0 else 'VIOLATED'}"
        yield ""


PATHS = [
    # =========================================================================
    # BASE TIER -- 3.0 GHz (333 ps).  Every path below is a real register-to-
    # register path in the RTL; the pipeline boundaries quoted in the names are
    # the ones the RTL actually implements (that is why several blocks are
    # banked or split -- see the notes at the bottom of this report).
    # =========================================================================
    Path("router S0: req flop -> even/odd popcount -> S0 flop", "base", [
        ("42-bit split popcount (carry-save, 6 levels)", 6 * GATE),
        ("sum registers", GATE),
    ]),
    Path("router S1: S0 flop -> even+2*odd -> two mod-3 folds -> S1 flop", "base", [
        ("7-bit add", 3 * GATE),
        ("mod-3 fold x2", 6 * GATE),
    ]),
    Path("router S2: S1 flop -> final fold -> slice select -> FIFO write", "base", [
        ("final fold + conditional subtract", 4 * GATE),
        ("slice select decode", mux(3) + GATE),
        ("FIFO write port", GATE),
    ]),
    Path("router S3: FIFO -> request register -> 1.8 mm wire -> slice flop", "base", [
        ("FIFO read mux (8 entries)", mux(8)),
        ("slice enable remap", GATE * 2),
        ("wire across the slice pitch (1800 um, M8)", 1800 * WIRE_M8),
    ]),
    Path("tag address: req flop -> set hash -> bank decode -> tag macro A", "base", [
        ("set index bank split", GATE),
        ("4-bank decode", mux(4)),
        ("wire to tag macros (450 um, M8)", 450 * WIRE_M8),
        ("macro address setup", 45.0),
    ]),
    Path("tag read: tag macro (8192x39, banked) -> bank mux -> tag flop", "base", [
        ("macro HP_SPSRAM_8192X39 clk->Q (SS)", 175.0 - CKQ),
        ("4:1 bank mux", mux(4)),
        ("wire to compare stage (300 um, M4)", 300 * WIRE_M4),
    ]),
    Path("tag compare: tag flop -> 16-way raw compare -> hit encode -> hit flop", "base", [
        ("16 x 27-bit compare (XOR tree + reduce)", xor_tree(27) + GATE),
        ("hit one-hot -> way encode", mux(16)),
        ("hit/miss qualification", GATE),
    ]),
    Path("tag ECC (parallel): tag flop -> SECDED(32,7) syndrome -> flag flop", "base", [
        ("syndrome XOR tree (max row load 16)", xor_tree(16)),
        ("nz / parity reduce", GATE * 2),
    ]),
    Path("victim select: tag flop -> RRPV max reduce -> victim flop", "base", [
        ("16-way RRPV max reduce (2 b)", xor_tree(16) + GATE * 2),
        ("invalid-way priority encode", mux(16)),
    ]),
    Path("base data address: way/set flop -> bank decode -> data macro A", "base", [
        ("way decode + bank decode", mux(4) + mux(4)),
        ("wire to data macros (700 um, M8)", 700 * WIRE_M8),
        ("macro address setup", 45.0),
    ]),
    Path("base data read A: data macro (8192x576, banked) -> macro out flop", "base", [
        ("macro HP_SPSRAM_8192X576 clk->Q (SS)", 190.0 - CKQ),
        ("OUT_REG capture", GATE),
    ]),
    Path("base data read B: macro out flop -> bank mux -> way mux -> ECC flop", "base", [
        ("4:1 bank mux (576 b)", mux(4)),
        ("4:1 way mux (576 b)", mux(4)),
        ("wire to ECC stage (600 um, M8)", 600 * WIRE_M8),
    ]),
    Path("ECC stage 1: data flop -> 4x SECDED(128,9) syndrome -> syndrome flop", "base", [
        ("syndrome XOR tree, max row load 57", xor_tree(57)),
        ("syndrome pack", GATE),
    ]),
    Path("ECC stage 2: syndrome flop -> 1-of-128 compare -> correct -> rsp flop", "base", [
        ("syndrome == column compare (9 b, 128 wide)", GATE * 2),
        ("correction XOR", GATE),
        ("poison / CE / UE merge", GATE * 2),
        ("wire to response flop (400 um, M4)", 400 * WIRE_M4),
    ]),
    Path("bond TX A: payload flop -> CRC16 -> beat register", "base", [
        ("CRC-16 over 148 b (unrolled, depth 8)", 8 * GATE),
        ("beat register setup path", GATE),
    ]),
    Path("bond TX B: beat flop -> lane steering -> 900 um wire -> pad", "base", [
        ("logical->physical lane steering (2:1 per lane after solve)", GATE * 2),
        ("wire to the bond field (900 um, M8)", 900 * WIRE_M8),
        ("pad driver + ESD", 45.0),
    ]),
    Path("scrub/BISR maintenance: state flop -> address gen -> port arb -> flop", "base", [
        ("strided set add (15 b)", 4 * GATE),
        ("maintenance vs demand arbitration", GATE * 2),
        ("wire to the pipeline port (500 um, M4)", 500 * WIRE_M4),
    ]),

    # =========================================================================
    # CACHE TIER -- 1.5 GHz (667 ps).  The stacked array is on a density-tuned
    # process at 0.75 V; it is deliberately slower than the base tier, and the
    # base:array clock ratio is a clean 2:1.
    # =========================================================================
    Path("stacked cmd A: pad flop -> deskew -> command decode -> decode flop", "cache", [
        ("pad receiver deskew mux", mux(4)),
        ("command field extract", GATE * 2),
        ("bank decode 5->32", mux(32)),
        ("subarray decode 4->16", mux(16)),
    ]),
    Path("stacked cmd B: decode flop -> repair CAM -> remap -> macro A", "cache", [
        ("row-repair CAM compare (4 x 10 b)", xor_tree(10) + GATE),
        ("column shift-redundancy remap", GATE),
        ("wire to the bank (1100 um, M4)", 1100 * WIRE_M4),
        ("macro address setup", 60.0),
    ]),
    Path("stacked read A: subarray macro -> 16:1 sub mux -> bank output flop", "cache", [
        ("macro HD_SPSRAM_1024X148 clk->Q (SS 0.68 V)", 465.0 - CKQ),
        ("16:1 subarray mux (148 b)", mux(16)),
        ("wire within the bank (250 um, M4)", 250 * WIRE_M4),
    ]),
    Path("stacked read B: bank flop -> 32:1 bank mux -> CRC -> pad", "cache", [
        ("32:1 bank mux (148 b)", mux(32)),
        ("wire to the bond field (1100 um, M4)", 1100 * WIRE_M4),
        ("CRC-16 generate", 8 * GATE),
        ("pad driver + ESD", 45.0),
    ]),
    Path("stacked write: pad flop -> remap -> macro D", "cache", [
        ("pad receiver deskew mux", mux(4)),
        ("column shift-redundancy remap (2:1 per bit)", GATE),
        ("wire to the bank (1100 um, M4)", 1100 * WIRE_M4),
        ("macro D setup", 60.0),
    ]),
]


def main():
    base_period  = 1e3 / TARGET_BASE_GHZ     # ps
    array_period = 1e3 / TARGET_ARRAY_GHZ

    print("=" * 78)
    print("VCACHE-3D  --  static timing model  (pre-PnR analytical signoff)")
    print("=" * 78)
    print(f"Base tier target   : {TARGET_BASE_GHZ:.2f} GHz  "
          f"(period {base_period:.1f} ps)")
    print(f"Cache tier target  : {TARGET_ARRAY_GHZ:.2f} GHz  "
          f"(period {array_period:.1f} ps)")
    print("Corner             : SS, 0.855 V logic / 0.68 V array, 125 C")
    print("Wire model         : buffered M4 0.22 ps/um, M8 0.09 ps/um")
    print("Bond model         : 9 um hybrid bond pad, ~2.4 fF, 31 ps round trip")
    print()

    worst = {"base": None, "cache": None}
    for p in PATHS:
        period = base_period if p.tier == "base" else array_period
        for line in p.report(period):
            print(line)
        slack = period - p.total
        if worst[p.tier] is None or slack < worst[p.tier][1]:
            worst[p.tier] = (p.name, slack)

    print("=" * 78)
    print("SUMMARY")
    print("=" * 78)
    ok = True
    for tier in ("base", "cache"):
        name, slack = worst[tier]
        period = base_period if tier == "base" else array_period
        fmax = 1e3 / (period - slack)
        ok &= slack >= 0
        print(f"  {tier:6s} worst path : {name}")
        print(f"  {tier:6s} worst slack: {slack:+7.1f} ps   -> Fmax {fmax:.2f} GHz")
    print()
    print(f"  RESULT: {'ALL PATHS MEET' if ok else 'TIMING VIOLATIONS PRESENT'}")
    print()
    print("  Architectural decisions forced by this analysis (all implemented")
    print("  in the RTL, not assumed away):")
    print("    * tag array banked 4x (8192x39) -- a 32768-deep macro is 240 ps")
    print("      and does not fit a 333 ps cycle with wire+setup+skew;")
    print("    * tag hit compare runs on RAW tag bits with SECDED in parallel;")
    print("      serialising decode->compare costs ~140 ps and misses timing;")
    print("    * base data array banked 4x (8192x576);")
    print("    * line ECC decoder split into syndrome / correct stages")
    print("      (vc3d_ecc_line_dec_pipe.v);")
    print("    * stacked bank output registered before the 32:1 bank mux")
    print("      (emitted by gen/vc3d_gen_stack_array.py);")
    print("    * modulo-3 interleave hash pipelined over 3 stages")
    print("      (vc3d_addr_hash_pipe.v).")
    print()
    print("  Resulting latency (base-die clock cycles, 3.0 GHz):")
    print("    router hash pipeline                  3")
    print("    tag: address / macro read / compare   3")
    print("    base data: address / macro / mux      3")
    print("    ECC decode (2 stages) + response      3")
    print("    --------------------------------------")
    print("    base-die hit (ways 0-3)              12 cycles = 4.0 ns")
    print()
    print("    a stacked hit replaces the 3 base data-array cycles with")
    print("      bond TX (CRC stage + pad stage)      2 base cycles")
    print("      dielet command decode                2 array cycles")
    print("      subarray access + bank mux           2 array cycles")
    print("      bond RX + deskew                     2 base cycles")
    print("    = 2 + 8 + 2 = 12 base cycles at the 2:1 clock ratio")
    print("    --------------------------------------")
    print("    stacked hit (ways 4-15)              21 cycles = 7.0 ns")
    print()
    print("  So the stacked region costs +9 base cycles (+3.0 ns) over the")
    print("  base-die region.  AMD quotes +4 core cycles for V-Cache; the gap")
    print("  is the 1.5 GHz dielet clock, which is what buys the 29 Mb/mm^2")
    print("  density and the 0.62 W dielet power in the thermal model.  Two")
    print("  things make that trade work here:")
    print("    * ways 0-3 (8 MiB per slice, 24 MiB total) are on the base die")
    print("      at 12-cycle latency -- there is no equivalent fast region in")
    print("      a monolithic V-Cache slice, and the replacement policy keeps")
    print("      the hottest lines there;")
    print("    * 21 cycles is still ~15x faster than the DRAM access it")
    print("      replaces, which is the comparison that decides performance.")
    print()
    print("  NOTE: replace with OpenROAD STA (pd/openroad) once a PDK is")
    print("        installed; pd/sdc constrains exactly these paths.")


if __name__ == "__main__":
    main()
