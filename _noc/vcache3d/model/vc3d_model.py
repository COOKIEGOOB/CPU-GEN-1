#!/usr/bin/env python3
"""Executable golden models for the VCACHE-3D algorithms.

Every algorithm that the RTL implements in hardware is implemented here a
second time, independently, in Python.  The unit tests in
test_vc3d_model.py check the properties the RTL relies on (uniformity of the
interleave, correctability of the ECC, invertibility of the redundancy remap,
convergence of the thermal loop, coverage of the scrubber).  This is the
verification that is possible without a Verilog simulator, and it is the same
model that pd/thermal and docs/PPA_REPORT.md are computed from.

Models provided
---------------
    secded            Hsiao SECDED encode / decode / inject
    crc16_ccitt       bond-beat CRC
    mod3_interleave   exact modulo-3 slice hash (matches vc3d_addr_hash.v)
    hashed_set_index  XOR-folded set index
    column_redundancy shift-redundancy write/read remap
    lane_repair       bond lane map solver
    scrub_schedule    patrol scrub coverage / sweep time
    thermal_step      first-order thermal model used by the sensor RTL
    cache_model       a functional 3-slice, 16-way, base/stack cache
"""
from itertools import combinations


# ---------------------------------------------------------------------------
# SECDED (Hsiao) -- identical construction to gen/vc3d_gen_ecc.py
# ---------------------------------------------------------------------------
def odd_weight_columns(r):
    pool = []
    for w in range(3, r + 1, 2):
        pool.extend(combinations(range(r), w))
    return pool


def check_bit_count(data_bits):
    r = 1
    while (1 << r) < data_bits + r + 1:
        r += 1
    while len(odd_weight_columns(r)) < data_bits:
        r += 1
    return r


def build_h(data_bits):
    r = check_bit_count(data_bits)
    pool = odd_weight_columns(r)
    by_weight = {}
    for c in pool:
        by_weight.setdefault(len(c), []).append(c)
    load = [0] * r
    cols = []
    for w in sorted(by_weight):
        cands = by_weight[w]
        while cands and len(cols) < data_bits:
            best = min(cands, key=lambda c: (max(load[i] for i in c),
                                             sum(load[i] for i in c), c))
            cands.remove(best)
            cols.append(best)
            for i in best:
                load[i] += 1
        if len(cols) == data_bits:
            break
    return r, cols


class Secded:
    def __init__(self, data_bits):
        self.n = data_bits
        self.r, self.cols = build_h(data_bits)
        self.colval = []
        for c in self.cols:
            v = 0
            for i in c:
                v |= 1 << i
            self.colval.append(v)

    def encode(self, data):
        chk = 0
        for d in range(self.n):
            if (data >> d) & 1:
                chk ^= self.colval[d]
        return chk

    def syndrome(self, data, chk):
        return chk ^ self.encode(data)

    def decode(self, data, chk):
        """return (corrected_data, ce, ue)"""
        s = self.syndrome(data, chk)
        if s == 0:
            return data, False, False
        if bin(s).count("1") % 2 == 0:
            return data, False, True                 # even parity -> DED
        for d in range(self.n):
            if self.colval[d] == s:
                return data ^ (1 << d), True, False  # data bit flip
        return data, True, False                     # check-bit flip


# ---------------------------------------------------------------------------
# CRC-16/CCITT over one bond beat
# ---------------------------------------------------------------------------
def crc16_ccitt(value, width):
    crc = 0xFFFF
    for i in range(width - 1, -1, -1):
        bit = (value >> i) & 1
        fb = ((crc >> 15) & 1) ^ bit
        crc = ((crc << 1) & 0xFFFF)
        if fb:
            crc ^= 0x1021
        crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Interleave
# ---------------------------------------------------------------------------
def mod3_interleave(addr, line_bits=6, addr_bits=48):
    la = addr >> line_bits
    even = odd = 0
    for i in range(addr_bits - line_bits):
        if (la >> i) & 1:
            if i % 2 == 0:
                even += 1
            else:
                odd += 1
    return (even + 2 * odd) % 3


def hashed_set_index(addr, set_w=15, line_bits=6, addr_bits=48):
    la = addr >> line_bits
    mask = (1 << set_w) - 1
    return (la ^ (la >> set_w) ^ (la >> (2 * set_w))) & mask


# ---------------------------------------------------------------------------
# Column shift redundancy
# ---------------------------------------------------------------------------
def col_write_remap(word, group_width, repaired_col):
    """logical word (group_width bits) -> physical word (group_width+1 bits)"""
    if repaired_col is None:
        return word & ((1 << group_width) - 1)
    out = 0
    for g in range(group_width + 1):
        if g == 0:
            bit = word & 1
        elif g == group_width:
            bit = (word >> (group_width - 1)) & 1
        elif g > repaired_col:
            bit = (word >> (g - 1)) & 1
        else:
            bit = (word >> g) & 1
        out |= bit << g
    return out


def col_read_remap(phys, group_width, repaired_col):
    out = 0
    for g in range(group_width):
        if repaired_col is not None and g >= repaired_col:
            bit = (phys >> (g + 1)) & 1
        else:
            bit = (phys >> g) & 1
        out |= bit << g
    return out


# ---------------------------------------------------------------------------
# Bond lane repair
# ---------------------------------------------------------------------------
def lane_repair(dead, phys_lanes, signal_lanes):
    """dead: set of dead physical lane indices -> (map, unrepairable)"""
    lane_map = []
    phys = 0
    for _ in range(signal_lanes):
        while phys < phys_lanes and phys in dead:
            phys += 1
        if phys >= phys_lanes:
            return lane_map, True
        lane_map.append(phys)
        phys += 1
    return lane_map, False


# ---------------------------------------------------------------------------
# Scrub scheduling
# ---------------------------------------------------------------------------
def scrub_sweep_cycles(sets, ways, period, burst):
    lines = sets * ways
    return (lines / burst) * period


def scrub_coverage(sets, stride):
    """check the strided walk visits every set exactly once"""
    seen = set()
    s = 0
    for _ in range(sets):
        if s in seen:
            return False, len(seen)
        seen.add(s)
        s = (s + stride) % sets
    return len(seen) == sets, len(seen)


# ---------------------------------------------------------------------------
# Thermal
# ---------------------------------------------------------------------------
def thermal_step(state, target, tau_shift=6):
    return state + ((target - state) >> tau_shift)


def thermal_settle(target, start=450, tau_shift=6, steps=1024):
    s = start << 8
    t = target << 8
    for _ in range(steps):
        s = s + ((t - s) >> tau_shift)
    return s >> 8


# ---------------------------------------------------------------------------
# Functional cache model (used to reason about capacity and hit rates)
# ---------------------------------------------------------------------------
class Vc3dCache:
    """3 slices x 32 MiB, 16-way, 64 B lines, ways 0..3 base / 4..15 stacked."""

    def __init__(self, slices=3, sets=32768, ways=16, base_ways=4, line=64):
        self.slices, self.sets, self.ways = slices, sets, ways
        self.base_ways, self.line = base_ways, line
        self.tags = [[[None] * ways for _ in range(sets)] for _ in range(slices)]
        self.rrpv = [[[3] * ways for _ in range(sets)] for _ in range(slices)]
        self.stats = dict(hits=0, misses=0, base_hits=0, stack_hits=0, fills=0,
                          evictions=0)

    def capacity_bytes(self):
        return self.slices * self.sets * self.ways * self.line

    def access(self, addr):
        sl = mod3_interleave(addr)
        st = hashed_set_index(addr)
        tag = addr >> 21
        row = self.tags[sl][st]
        for w in range(self.ways):
            if row[w] == tag:
                self.stats["hits"] += 1
                if w < self.base_ways:
                    self.stats["base_hits"] += 1
                else:
                    self.stats["stack_hits"] += 1
                self.rrpv[sl][st][w] = 0
                return True
        self.stats["misses"] += 1
        self._install(sl, st, tag)
        return False

    def _install(self, sl, st, tag):
        row = self.tags[sl][st]
        rr = self.rrpv[sl][st]
        for w in range(self.ways):
            if row[w] is None:
                row[w] = tag
                rr[w] = 2
                self.stats["fills"] += 1
                return
        while True:
            for w in range(self.ways):
                if rr[w] == 3:
                    row[w] = tag
                    rr[w] = 2
                    self.stats["fills"] += 1
                    self.stats["evictions"] += 1
                    return
            for w in range(self.ways):
                rr[w] += 1
