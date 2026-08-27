#!/usr/bin/env python3
"""Property tests for the VCACHE-3D golden models (and therefore for the
algorithms the RTL implements).  Run: python3 model/test_vc3d_model.py"""
import random
import unittest

from vc3d_model import (Secded, crc16_ccitt, mod3_interleave, hashed_set_index,
                        col_write_remap, col_read_remap, lane_repair,
                        scrub_coverage, scrub_sweep_cycles, thermal_settle,
                        Vc3dCache, check_bit_count)


class TestSecded(unittest.TestCase):
    def test_check_bit_counts(self):
        self.assertEqual(check_bit_count(32), 7)
        self.assertEqual(check_bit_count(64), 8)
        self.assertEqual(check_bit_count(128), 9)
        self.assertEqual(check_bit_count(256), 10)
        self.assertEqual(check_bit_count(512), 11)

    def test_clean_roundtrip(self):
        for n in (32, 64, 128, 256, 512):
            c = Secded(n)
            rnd = random.Random(n)
            for _ in range(64):
                d = rnd.getrandbits(n)
                chk = c.encode(d)
                out, ce, ue = c.decode(d, chk)
                self.assertEqual(out, d)
                self.assertFalse(ce or ue)

    def test_single_bit_correction(self):
        for n in (128, 512):
            c = Secded(n)
            rnd = random.Random(n + 1)
            for _ in range(200):
                d = rnd.getrandbits(n)
                chk = c.encode(d)
                b = rnd.randrange(n)
                out, ce, ue = c.decode(d ^ (1 << b), chk)
                self.assertTrue(ce, "single-bit error must be correctable")
                self.assertFalse(ue)
                self.assertEqual(out, d, "correction must restore the data")

    def test_check_bit_error(self):
        c = Secded(128)
        d = 0xdead_beef_cafe_f00d_1234_5678_9abc_def0
        chk = c.encode(d)
        for b in range(c.r):
            out, ce, ue = c.decode(d, chk ^ (1 << b))
            self.assertTrue(ce)
            self.assertFalse(ue)
            self.assertEqual(out, d)

    def test_double_bit_detection(self):
        for n in (128, 512):
            c = Secded(n)
            rnd = random.Random(n + 2)
            for _ in range(300):
                d = rnd.getrandbits(n)
                chk = c.encode(d)
                a, b = rnd.sample(range(n), 2)
                out, ce, ue = c.decode(d ^ (1 << a) ^ (1 << b), chk)
                self.assertTrue(ue, "double-bit error must be detected")
                self.assertFalse(ce)

    def test_subline_isolation(self):
        """Four independent 128-bit codes must survive one error per subline --
        the property that makes a failing bond lane correctable."""
        c = Secded(128)
        rnd = random.Random(7)
        line = [rnd.getrandbits(128) for _ in range(4)]
        chks = [c.encode(x) for x in line]
        got = []
        for i in range(4):
            corrupted = line[i] ^ (1 << (i * 31 + 5))
            out, ce, ue = c.decode(corrupted, chks[i])
            self.assertTrue(ce)
            self.assertFalse(ue)
            got.append(out)
        self.assertEqual(got, line)


class TestCrc(unittest.TestCase):
    def test_known_vector(self):
        # CRC-16/CCITT-FALSE of "123456789" is 0x29B1
        data = int.from_bytes(b"123456789", "big")
        self.assertEqual(crc16_ccitt(data, 72), 0x29B1)

    def test_single_bit_detection(self):
        rnd = random.Random(3)
        for _ in range(200):
            v = rnd.getrandbits(148)
            c = crc16_ccitt(v, 148)
            b = rnd.randrange(148)
            self.assertNotEqual(crc16_ccitt(v ^ (1 << b), 148), c)


class TestInterleave(unittest.TestCase):
    def test_matches_modulo(self):
        rnd = random.Random(11)
        for _ in range(2000):
            a = rnd.getrandbits(48) & ~0x3F
            self.assertEqual(mod3_interleave(a), (a >> 6) % 3)

    def test_linear_stream_is_balanced(self):
        counts = [0, 0, 0]
        for i in range(30000):
            counts[mod3_interleave(i * 64)] += 1
        self.assertEqual(counts, [10000, 10000, 10000])

    def test_strided_stream_is_balanced(self):
        for stride_lines in (2, 4, 8, 16, 64, 1024):
            counts = [0, 0, 0]
            for i in range(3000):
                counts[mod3_interleave(i * stride_lines * 64)] += 1
            if stride_lines % 3 != 0:
                self.assertEqual(counts, [1000, 1000, 1000], stride_lines)

    def test_set_index_range(self):
        rnd = random.Random(13)
        for _ in range(1000):
            a = rnd.getrandbits(48)
            self.assertLess(hashed_set_index(a), 1 << 15)


class TestColumnRedundancy(unittest.TestCase):
    def test_remap_is_invertible(self):
        gw = 36
        rnd = random.Random(17)
        for col in [None] + list(range(gw)):
            for _ in range(50):
                w = rnd.getrandbits(gw)
                phys = col_write_remap(w, gw, col)
                back = col_read_remap(phys, gw, col)
                self.assertEqual(back, w, f"col={col}")

    def test_repaired_column_is_unused(self):
        """After repairing column c, no logical bit may be stored in the
        physical bit line c -- that is the whole point of the repair."""
        gw = 36
        c = 7
        for _ in range(50):
            w = random.getrandbits(gw)
            phys = col_write_remap(w, gw, c)
            broken = phys ^ (1 << c)          # stuck-at fault on that bit line
            self.assertEqual(col_read_remap(broken, gw, c),
                             col_read_remap(phys, gw, c))


class TestLaneRepair(unittest.TestCase):
    def test_no_dead_lane(self):
        m, bad = lane_repair(set(), 172, 164)
        self.assertFalse(bad)
        self.assertEqual(m, list(range(164)))

    def test_up_to_spare_count(self):
        for k in range(0, 9):
            dead = set(random.sample(range(172), k))
            m, bad = lane_repair(dead, 172, 164)
            self.assertFalse(bad, f"{k} dead lanes must be repairable")
            self.assertEqual(len(set(m)), 164)
            self.assertFalse(set(m) & dead)

    def test_beyond_spare_count_fails(self):
        dead = set(range(9))
        _, bad = lane_repair(dead, 172, 164)
        self.assertTrue(bad, "9 dead lanes with 8 spares must be unrepairable")


class TestScrub(unittest.TestCase):
    def test_stride_covers_every_set(self):
        ok, n = scrub_coverage(32768, 4097)
        self.assertTrue(ok)
        self.assertEqual(n, 32768)

    def test_bad_stride_does_not_cover(self):
        ok, _ = scrub_coverage(32768, 4096)
        self.assertFalse(ok)

    def test_sweep_time_budget(self):
        cyc = scrub_sweep_cycles(32768, 16, 4096, 4)
        secs = cyc / 3.0e9
        self.assertLess(secs, 1.0, "a full sweep must take well under a second")
        self.assertGreater(secs, 0.05)


class TestThermal(unittest.TestCase):
    def test_settles_to_target(self):
        for target in (450, 700, 950, 1050):
            self.assertAlmostEqual(thermal_settle(target), target, delta=2)

    def test_is_monotone(self):
        prev = 450
        s = 450 << 8
        for _ in range(50):
            s = s + (((900 << 8) - s) >> 6)
            self.assertGreaterEqual(s >> 8, prev)
            prev = s >> 8


class TestCacheModel(unittest.TestCase):
    def test_capacity(self):
        c = Vc3dCache()
        self.assertEqual(c.capacity_bytes(), 96 * 1024 * 1024)

    def test_working_set_fits(self):
        """A 64 MiB working set must fit in a 96 MiB cache: after one warm
        pass, the second pass has to be essentially all hits."""
        c = Vc3dCache()
        lines = (64 * 1024 * 1024) // 64
        for i in range(0, lines, 97):
            c.access(i * 64)
        before = dict(c.stats)
        for i in range(0, lines, 97):
            c.access(i * 64)
        hits = c.stats["hits"] - before["hits"]
        total = hits + (c.stats["misses"] - before["misses"])
        self.assertGreater(hits / total, 0.99)

    def test_base_ways_serve_hot_set(self):
        c = Vc3dCache()
        for _ in range(4):
            for i in range(1024):
                c.access(i * 64)
        self.assertGreater(c.stats["hits"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
