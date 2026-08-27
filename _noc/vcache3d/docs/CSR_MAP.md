# CSR map

APB, 14-bit address. `paddr[13:12]` selects the window: 0..2 are the per-slice
4 KiB windows, 3 is the global window.

## Per-slice window

| offset | name | contents |
|---|---|---|
| 0x000 | ID | magic "VC3D", revision, slice id |
| 0x004 | CAPABILITY | capacity, ways, sets, stacked way count |
| 0x008 | CONTROL | enable, invalidate all, stack enable, force miss |
| 0x00c | STATUS | ready, link up, repair done, throttled |
| 0x010 | ECC_CTRL | ECC enable, poison on UE, CE threshold |
| 0x014 | ECC_STATUS | last class/source, valid |
| 0x018 | CE_COUNT | correctable error count |
| 0x01c | UE_COUNT | uncorrectable error count |
| 0x020 | SCRUB_CTRL | scrub enable, burst length, stride select |
| 0x024 | SCRUB_PERIOD | cycles between bursts (default 4096) |
| 0x028 | SCRUB_ADDR | set/way the scrubber is on |
| 0x02c | SCRUB_PROGRESS | sweeps completed |
| 0x030 | ELOG_HEAD | error log occupancy and head pointer |
| 0x034 | ELOG_DATA0 | address of the head entry |
| 0x038 | ELOG_DATA1 | way, syndrome, dirty, source |
| 0x03c | ELOG_POP | write to pop the head entry |
| 0x040 | REPAIR_CTRL | start poweron / full repair, phase (key protected) |
| 0x044 | REPAIR_INDEX | bank/sub/slot being programmed |
| 0x048 | REPAIR_DATA_LO | row address / column id |
| 0x04c | REPAIR_DATA_HI | valid bits and accounting |
| 0x050 | EFUSE_CTRL | program enable, autoload start, CRC status (key protected) |
| 0x054 | EFUSE_ADDR | fuse row address |
| 0x058 | EFUSE_DATA | fuse word, program or readback |
| 0x060 | MBIST_CTRL | start, algorithm, bank range, stop on fail |
| 0x064 | MBIST_STATUS | busy, done, pass, fail count |
| 0x068 | MBIST_FAIL_ADDR | bank/sub/row of the first fail |
| 0x06c | MBIST_FAIL_DATA | expected vs actual |
| 0x070 | BOND_CTRL | link enable, train request |
| 0x074 | BOND_STATUS | per-channel state, link up, fatal |
| 0x078 | BOND_LANE_MAP | logical to physical lane map readback |
| 0x07c | BOND_CRC_ERR | per-channel CRC error and retrain counts |
| 0x080 | THERMAL_CTRL | throttle thresholds, sample enable |
| 0x084 | THERMAL_STATUS | throttle level, per-sensor valid |
| 0x088 | TEMP_MAX | hottest sensor, 0.1 C units |
| 0x08c | DVFS_CTRL | level 0..7, voltage/frequency handshake |
| 0x090 | PERF_SEL | counter select (0..63) |
| 0x094 | PERF_LO | selected counter, low word |
| 0x098 | PERF_HI | selected counter, high word |
| 0x0a0 | INTERLEAVE_CTRL | MOD3 / LOW2 / HASH / DIRECT |
| 0x0a4 | SLICE_ENABLE | per-slice enable mask |
| 0x0a8 | WAY_DISABLE | retired ways (key protected) |
| 0x0ac | PREFETCH_CTRL | prefetch depth and enable |

Offsets are taken directly from `include/vc3d_defines.vh` (this table is
generated from it, so the two cannot drift apart).

## Global window (`paddr[13:12] == 3`)

| offset | name | contents |
|---|---|---|
| 0x000 | GLOBAL_ID | magic, slice count, total capacity |
| 0x004 | GLOBAL_STATUS | all slices ready, any error, any throttle |
| 0x008 | GLOBAL_CTRL | interleave mode, slice enable mask |
| 0x00C | GLOBAL_TEMP | hottest sensor across all three dielets |
| 0x010 | ROUTE_COUNT_0..2 | requests routed to each slice |

## Protection

Writes to the destructive registers (`REPAIR_CTRL`, `FUSE_CTRL`,
`WAY_DISABLE`, `BANK_DISABLE`) require the unlock key `0xC3D0` in
`pwdata[31:16]`. A write without the key returns `pslverr` and changes
nothing -- burning a fuse or retiring a way by accident is not recoverable.
