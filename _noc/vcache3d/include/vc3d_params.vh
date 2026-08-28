/*
* CPU-GEN-1 : VCACHE-3D  (3D hybrid-bonded stacked last-level cache)
* -----------------------------------------------------------------------------
* Global build-time parameter package for the stacked-cache subsystem.
*
* The subsystem models an AMD-3D-V-Cache-class memory system:
*
*   base die   : CHI HN-F slice logic, tag/state arrays, ECC, repair,
*                interleave router, telemetry, DVFS/thermal control
*   cache die  : dense SRAM dielet, hybrid bonded face-to-face onto the base
*                die, exposing banked subarrays through a bond-pad (TSV/BPV)
*                array rather than through top-level metal
*
* Capacity plan (default build):
*   3 slices x 32 MiB = 96 MiB aggregate cache
*     - 8 MiB of each slice lives on the base die   (base-die array)
*     - 24 MiB of each slice lives on the cache die (stacked array)
*   The controller presents ONE unified 32 MiB, 16-way, 64 B-line cache per
*   slice; the base/stack split is a physical partition of the way space.
*
* Every numeric knob below is overridable from the build; the parameters are
* consumed via `VC3D_PARAMS / `VC3D_PARAMS_INST macro pairs so that the whole
* hierarchy can be re-sized without touching module port lists.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`ifndef VC3D_PARAMS_VH
`define VC3D_PARAMS_VH

// -----------------------------------------------------------------------------
// 1. Cache geometry
// -----------------------------------------------------------------------------
`define VC3D_SLICE_NUM              3         // address-interleaved slices
`define VC3D_SLICE_ID_WIDTH         2
`define VC3D_SLICE_CAPACITY_KIB     32768     // 32 MiB per slice
`define VC3D_AGGREGATE_CAPACITY_KIB 98304     // 96 MiB total
`define VC3D_LINE_BYTES             64
`define VC3D_LINE_BITS              512
`define VC3D_WAY_NUM                16
`define VC3D_SET_NUM                32768     // 32 MiB / (64 B * 16 ways)
`define VC3D_SET_INDEX_WIDTH        15
`define VC3D_OFFSET_WIDTH           6
`define VC3D_PADDR_WIDTH            48
`define VC3D_TAG_WIDTH              27        // 48 - 15 - 6

// Way-space partition between the base die and the stacked cache die.
`define VC3D_BASE_WAY_NUM           4         //  8 MiB on the base die
`define VC3D_STACK_WAY_NUM          12        // 24 MiB on the cache die
`define VC3D_STACK_WAY_BASE         4         // stacked ways are [4 .. 15]

// -----------------------------------------------------------------------------
// 2. Stacked-die physical organisation
// -----------------------------------------------------------------------------
// The cache dielet is organised as BANKS x SUBARRAYS.  A subarray is the unit
// the SRAM compiler produces; a bank is the unit of independent access and of
// power gating; the bond-pad field over a bank is the unit of link training.
`define VC3D_STACK_BANK_NUM         32
`define VC3D_STACK_BANK_ID_WIDTH    5
`define VC3D_STACK_SUBARRAY_NUM     16
`define VC3D_STACK_SUBARRAY_WIDTH   4
`define VC3D_STACK_SUBARRAY_DEPTH   1024      // rows per subarray
`define VC3D_STACK_ROW_ADDR_WIDTH   10
`define VC3D_STACK_COL_MUX          4         // 4:1 column mux
`define VC3D_STACK_IO_WIDTH         144       // 128 data + 16 ECC per subarray IO
`define VC3D_STACK_DATA_IO_WIDTH    128
`define VC3D_STACK_ECC_IO_WIDTH     16

// Redundancy resources per subarray (manufacturing repair).
`define VC3D_SPARE_ROW_NUM          4
`define VC3D_SPARE_COL_NUM          4
`define VC3D_SPARE_ROW_ID_WIDTH     2
`define VC3D_SPARE_COL_ID_WIDTH     2

// -----------------------------------------------------------------------------
// 3. Hybrid-bond link (base die <-> cache die)
// -----------------------------------------------------------------------------
// Face-to-face hybrid bonding: sub-micron pitch bond pads, no microbumps.
// The link is source-synchronous, single-cycle-per-beat, with per-lane repair
// and a spare-lane pool, because a bond defect must not kill the dielet.
`define VC3D_BOND_CH_NUM            8         // independent bond channels
`define VC3D_BOND_CH_ID_WIDTH       3
`define VC3D_BOND_LANE_PER_CH       164       // 144 data + 4 cmd + 16 crc
`define VC3D_BOND_SPARE_LANE        8         // spare lanes per channel
`define VC3D_BOND_PHYS_LANE_PER_CH  172       // 164 signal + 8 spare
`define VC3D_BOND_CMD_WIDTH         4
`define VC3D_BOND_PAYLOAD_WIDTH     144
`define VC3D_BOND_TOTAL_LANE        1376      // 8 * (164 + 8 spare)
`define VC3D_BOND_TRAIN_PATTERN     32'hA5A5_5A5A
`define VC3D_BOND_TRAIN_CYCLES      1024
`define VC3D_BOND_CRC_WIDTH         16
`define VC3D_BOND_PITCH_NM          9000      // 9 um hybrid-bond pitch
`define VC3D_BOND_RTT_CYCLES        3         // matches published +4 cycle cost (pre-DDR)
`define VC3D_BOND_DDR_RTT_CYCLES    2         // DDR link round trip after gearbox removal
`define VC3D_BOND_DDR_ENABLE        1         // bond lanes run dual-edge at 3.0 GHz
`define VC3D_BOND_DDR_SDR_MHZ       1500      // forwarded SDR clock to the dielet
`define VC3D_BOND_DDR_TX_MHZ        3000      // DDR sampling on both edges
`define VC3D_BOND_DDR_BEATS         2         // sublines per channel per base cycle

// -----------------------------------------------------------------------------
// 4. ECC
// -----------------------------------------------------------------------------
// Data path : SECDED per 128-bit subline (4 sublines per 64 B line) giving
//             chip-kill-like tolerance of one failing bond lane per subline,
//             plus DECTED-class protection for the tag/state arrays.
`define VC3D_ECC_SUBLINE_BITS       128
`define VC3D_ECC_SUBLINE_NUM        4
`define VC3D_ECC_CHECK_BITS_128     9         // SECDED(128,9)
`define VC3D_ECC_CHECK_BITS_256     10
`define VC3D_ECC_CHECK_BITS_512     11
`define VC3D_ECC_CHECK_BITS_64      8
`define VC3D_ECC_CHECK_BITS_32      7
`define VC3D_ECC_TAG_DATA_BITS      32
`define VC3D_ECC_TAG_CHECK_BITS     7
`define VC3D_ECC_POISON_BITS        1

// -----------------------------------------------------------------------------
// 5. Scrubbing
// -----------------------------------------------------------------------------
`define VC3D_SCRUB_RATE_WIDTH       32
`define VC3D_SCRUB_DEFAULT_PERIOD   32'd4096  // cycles between scrub reads
`define VC3D_SCRUB_BURST            4         // lines per scrub grant
`define VC3D_SCRUB_LOG_DEPTH        32
`define VC3D_SCRUB_CE_THRESHOLD     16'd64    // CE count that triggers repair

// -----------------------------------------------------------------------------
// 6. Repair / BIST
// -----------------------------------------------------------------------------
`define VC3D_EFUSE_ROW_NUM          1024
`define VC3D_EFUSE_ROW_WIDTH        32
`define VC3D_REPAIR_ENTRY_NUM       256       // soft/hard repair map entries
`define VC3D_REPAIR_ENTRY_WIDTH     48
`define VC3D_MBIST_ALGO_NUM         8
`define VC3D_MBIST_PATTERN_WIDTH    128

// -----------------------------------------------------------------------------
// 7. Request / response transport inside the subsystem
// -----------------------------------------------------------------------------
`define VC3D_REQ_ID_WIDTH           12
`define VC3D_MSHR_NUM               128
`define VC3D_MSHR_ID_WIDTH          7
`define VC3D_OPC_WIDTH              6
`define VC3D_BE_WIDTH               64
`define VC3D_QOS_WIDTH              4
`define VC3D_SRCID_WIDTH            7

// Opcodes on the internal (post-router) slice port.
`define VC3D_OPC_READ_SHARED        6'h01
`define VC3D_OPC_READ_UNIQUE        6'h02
`define VC3D_OPC_READ_ONCE          6'h03
`define VC3D_OPC_WRITE_FULL         6'h04
`define VC3D_OPC_WRITE_PARTIAL      6'h05
`define VC3D_OPC_EVICT              6'h06
`define VC3D_OPC_CLEAN_INVALID      6'h07
`define VC3D_OPC_SCRUB_READ         6'h08
`define VC3D_OPC_SCRUB_WRITE        6'h09
`define VC3D_OPC_BIST_WRITE         6'h0a
`define VC3D_OPC_BIST_READ          6'h0b
`define VC3D_OPC_PREFETCH           6'h0c
// Directed (set, way) access used by the HN-F adapter: the HN-F has already
// done its own tag lookup, so these bypass the slice tag array entirely and
// address the arrays like a plain SRAM.  addr[9:6] carries the way.
`define VC3D_OPC_DIRECT_READ        6'h0d
`define VC3D_OPC_DIRECT_WRITE       6'h0e

// -----------------------------------------------------------------------------
// 7b. Native CHI DAT width (NoC / HN-F interface)
// -----------------------------------------------------------------------------
// CHI DAT flits are 256 bits in the stock OpenNoC integration.  The VCACHE-3D
// HN-F buffer is widened to a native 512-bit internal DAT so a 64 B line fits
// in one beat, doubling peak per-slice bandwidth to 64 B/cycle.
`define VC3D_HNF_DAT_WIDTH           512       // native internal DAT width
`define VC3D_HNF_DAT_FLIT_WIDTH      256       // legacy CHI DAT flit width
`define VC3D_HNF_DAT_FLITS_PER_LINE  2         // 2 legacy flits per 64 B line

// -----------------------------------------------------------------------------
// 7c. Speculative data return + parallel SECDED
// -----------------------------------------------------------------------------
// The demand data path returns raw line data speculatively at S6 together
// with a 1-cycle syndrome/parity flag.  The full SECDED correction runs in
// parallel on the base die; the array data is only stalled / replayed when the
// SECDED syndrome is non-zero (well under 0.0001% of accesses).
`define VC3D_SPEC_RETURN_ENABLE      1
`define VC3D_ECC_FAST_SYNDROME_PS    124
`define VC3D_ECC_FULL_DECODE_PS      379
`define VC3D_SPEC_REPLAY_DEPTH       8

// -----------------------------------------------------------------------------
// 8. Telemetry / DVFS / thermal
// -----------------------------------------------------------------------------
`define VC3D_TEMP_SENSOR_NUM        16
`define VC3D_TEMP_WIDTH             12        // 0.1 C resolution, 0..409.5 C
`define VC3D_TEMP_THROTTLE_C        12'd950   // 95.0 C
`define VC3D_TEMP_CRITICAL_C        12'd1050  // 105.0 C
`define VC3D_DVFS_LEVEL_NUM         8
`define VC3D_DVFS_LEVEL_WIDTH       3
`define VC3D_PERF_COUNTER_NUM       64
`define VC3D_PERF_COUNTER_WIDTH     48

// -----------------------------------------------------------------------------
// 9. CSR map (APB, 4 KiB window per slice + 4 KiB global)
// -----------------------------------------------------------------------------
`define VC3D_CSR_ADDR_WIDTH         14
`define VC3D_CSR_DATA_WIDTH         32

// -----------------------------------------------------------------------------
// 10. Direct SRAM macro slicing (2.0 - 2.4 GHz dielet)
// -----------------------------------------------------------------------------
// Each 1024 x 148 stacked macro is banked into VC3D_STACK_MACRO_SLICES smaller
// macros with divided local bitlines.  Four 256 x 148 slices cut the macro
// access time from ~465 ps to ~260 ps and let the dielet run at the target
// clock instead of 1.5 GHz.
`define VC3D_STACK_MACRO_SLICES      4         // 2 x 512 or 4 x 256 macro banks
`define VC3D_STACK_MACRO_DEPTH       256       // 1024 / 4
`define VC3D_STACK_MACRO_AW          8         // ceil(log2(256))
`define VC3D_STACK_MACRO_ACCESS_PS   260
`define VC3D_STACK_MACRO_LEGACY_PS   465
`define VC3D_STACK_DIELET_CLOCK_MHZ  2200      // 2.0 - 2.4 GHz target
`define VC3D_STACK_DIELET_BW_B_CYCLE 64

// -----------------------------------------------------------------------------
// 11. Tier-aware asymmetric replacement (latency-aware way partitioning)
// -----------------------------------------------------------------------------
// Ways 0..3 live on the base die (fast, ~12 cycles); ways 4..15 live on the
// stacked dielet (~4-5 cycle adder).  Hits are promoted into the fast ways,
// streaming/prefetch data is demoted to the stacked ways, and the fast ways
// are preferred for latency-critical requests.
`define VC3D_TIER_AWARE_REPL_ENABLE  1
`define VC3D_TIER_FAST_WAY_BASE      0
`define VC3D_TIER_FAST_WAY_NUM       4
`define VC3D_TIER_SLOW_WAY_BASE      4
`define VC3D_TIER_SLOW_WAY_NUM       12
`define VC3D_TIER_HOT_QOS_THRESHOLD  4'd8

// -----------------------------------------------------------------------------
// 12. Inverted physical die stack (Zen 5 packaging architecture)
// -----------------------------------------------------------------------------
// The 3D-SRAM dielet is placed face-up on the package substrate, the base
// logic die is bonded face-down onto it, and the base die is ground and placed
// directly against the lid/cooling solution.  This removes the previous
// +3.3 C thermal penalty on the hot base logic.
`define VC3D_DIE_STACK_INVERTED      1
`define VC3D_DIELET_UNDER_BASE       0         // 0 = dielet on substrate
`define VC3D_BASE_UNDER_LID          1         // 1 = base die under the lid

`endif /* VC3D_PARAMS_VH */
