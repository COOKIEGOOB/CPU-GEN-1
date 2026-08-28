/*
* CPU-GEN-1 : VCACHE-3D  -- shared macro / encoding definitions.
*
* Contains everything that is an encoding rather than a size: error classes,
* repair-record layouts, bond-link framing, CSR offsets, MBIST algorithm ids
* and small helper macros.  Sizes live in vc3d_params.vh.
*
* SPDX-License-Identifier: MulanPSL-2.0
*/
`ifndef VC3D_DEFINES_VH
`define VC3D_DEFINES_VH
`include "vc3d_params.vh"

// -----------------------------------------------------------------------------
// Error classification (reported by the ECC decoders and logged by the ELOG)
// -----------------------------------------------------------------------------
`define VC3D_ERR_NONE               3'd0   // no error
`define VC3D_ERR_CE                 3'd1   // corrected single-bit error
`define VC3D_ERR_UE                 3'd2   // detected uncorrectable (double) error
`define VC3D_ERR_POISON             3'd3   // poisoned line consumed
`define VC3D_ERR_CHECKBIT_CE        3'd4   // CE located in the check-bit field
`define VC3D_ERR_LINK_CRC           3'd5   // bond-link CRC failure
`define VC3D_ERR_LINK_LANE          3'd6   // bond-link lane failure (repairable)
`define VC3D_ERR_TAG_UE             3'd7   // tag/state array uncorrectable

`define VC3D_ERRSRC_BASE_DATA       3'd0
`define VC3D_ERRSRC_STACK_DATA      3'd1
`define VC3D_ERRSRC_TAG             3'd2
`define VC3D_ERRSRC_STATE           3'd3
`define VC3D_ERRSRC_SF              3'd4
`define VC3D_ERRSRC_BOND            3'd5
`define VC3D_ERRSRC_SCRUB           3'd6
`define VC3D_ERRSRC_MBIST           3'd7

// -----------------------------------------------------------------------------
// Repair record layout  (VC3D_REPAIR_ENTRY_WIDTH = 48)
// -----------------------------------------------------------------------------
//   [47]    valid
//   [46]    hard (blown into eFuse) vs soft (runtime, volatile)
//   [45:44] type : 0 = row, 1 = column, 2 = lane, 3 = way-disable
//   [43:39] bank id
//   [38:35] subarray id
//   [34:25] row address (row repair) / column group (column repair)
//   [24:17] lane id (lane repair)
//   [16:15] spare resource id
//   [14:0]  reserved / CRC of the record
`define VC3D_RPR_VALID_BIT          47
`define VC3D_RPR_HARD_BIT           46
`define VC3D_RPR_TYPE_MSB           45
`define VC3D_RPR_TYPE_LSB           44
`define VC3D_RPR_BANK_MSB           43
`define VC3D_RPR_BANK_LSB           39
`define VC3D_RPR_SUB_MSB            38
`define VC3D_RPR_SUB_LSB            35
`define VC3D_RPR_ADDR_MSB           34
`define VC3D_RPR_ADDR_LSB           25
`define VC3D_RPR_LANE_MSB           24
`define VC3D_RPR_LANE_LSB           17
`define VC3D_RPR_SPARE_MSB          16
`define VC3D_RPR_SPARE_LSB          15

`define VC3D_RPR_TYPE_ROW           2'd0
`define VC3D_RPR_TYPE_COL           2'd1
`define VC3D_RPR_TYPE_LANE          2'd2
`define VC3D_RPR_TYPE_WAY           2'd3

// -----------------------------------------------------------------------------
// Bond-link framing
// -----------------------------------------------------------------------------
//  A beat carries one 144-bit ECC-protected subline plus a 16-bit control
//  field: {crc[15:0]} is carried on the control lanes together with cmd/valid.
`define VC3D_BOND_CMD_IDLE          4'd0
`define VC3D_BOND_CMD_READ          4'd1
`define VC3D_BOND_CMD_WRITE         4'd2
`define VC3D_BOND_CMD_RMW           4'd3
`define VC3D_BOND_CMD_REFRESH       4'd4   // retention/assist sweep
`define VC3D_BOND_CMD_TRAIN         4'd5
`define VC3D_BOND_CMD_REPAIR        4'd6
`define VC3D_BOND_CMD_PWR           4'd7
`define VC3D_BOND_CMD_MBIST         4'd8
`define VC3D_BOND_CMD_STATUS        4'd9

`define VC3D_BOND_ST_RESET          3'd0
`define VC3D_BOND_ST_LANE_TRAIN     3'd1
`define VC3D_BOND_ST_DESKEW         3'd2
`define VC3D_BOND_ST_REPAIR         3'd3
`define VC3D_BOND_ST_CRC_CHECK      3'd4
`define VC3D_BOND_ST_ACTIVE         3'd5
`define VC3D_BOND_ST_RETRAIN        3'd6
`define VC3D_BOND_ST_FAIL           3'd7

// DDR bond-link phase encodings
`define VC3D_BOND_DDR_PHASE_EVEN      1'b0
`define VC3D_BOND_DDR_PHASE_ODD       1'b1

// Tier-aware replacement policy hints
`define VC3D_TIER_CLASS_FAST         2'd0   // L1/L2 instr miss, pointer-chase load
`define VC3D_TIER_CLASS_NORMAL       2'd1   // general read/write demand
`define VC3D_TIER_CLASS_BULK         2'd2   // streaming / bulk copy
`define VC3D_TIER_CLASS_PREFETCH     2'd3   // prefetch (demote to stacked ways)
`define VC3D_TIER_INSERT_BASE        1'b0
`define VC3D_TIER_INSERT_STACK       1'b1

// Inverted die-stack orientation encodings
`define VC3D_STACK_FACE_UP_SUBSTRATE 1'b0
`define VC3D_STACK_FACE_DOWN_BASE    1'b1

// -----------------------------------------------------------------------------
// MBIST algorithms
// -----------------------------------------------------------------------------
`define VC3D_MBIST_MARCH_C          3'd0
`define VC3D_MBIST_MARCH_SS         3'd1
`define VC3D_MBIST_CHECKERBOARD     3'd2
`define VC3D_MBIST_WALKING_ONE      3'd3
`define VC3D_MBIST_GALPAT           3'd4
`define VC3D_MBIST_ROW_STRIPE       3'd5
`define VC3D_MBIST_RETENTION        3'd6
`define VC3D_MBIST_PSEUDO_RANDOM    3'd7

// -----------------------------------------------------------------------------
// Slice power state
// -----------------------------------------------------------------------------
`define VC3D_PWR_OFF                3'd0
`define VC3D_PWR_RET                3'd1   // retention (data kept, no access)
`define VC3D_PWR_IDLE               3'd2
`define VC3D_PWR_ACTIVE             3'd3
`define VC3D_PWR_THROTTLE           3'd4

// -----------------------------------------------------------------------------
// CSR offsets (byte offsets within a 4 KiB window)
// -----------------------------------------------------------------------------
`define VC3D_CSR_ID                 14'h000
`define VC3D_CSR_CAPABILITY         14'h004
`define VC3D_CSR_CONTROL            14'h008
`define VC3D_CSR_STATUS             14'h00c
`define VC3D_CSR_ECC_CTRL           14'h010
`define VC3D_CSR_ECC_STATUS         14'h014
`define VC3D_CSR_CE_COUNT           14'h018
`define VC3D_CSR_UE_COUNT           14'h01c
`define VC3D_CSR_SCRUB_CTRL         14'h020
`define VC3D_CSR_SCRUB_PERIOD       14'h024
`define VC3D_CSR_SCRUB_ADDR         14'h028
`define VC3D_CSR_SCRUB_PROGRESS     14'h02c
`define VC3D_CSR_ELOG_HEAD          14'h030
`define VC3D_CSR_ELOG_DATA0         14'h034
`define VC3D_CSR_ELOG_DATA1         14'h038
`define VC3D_CSR_ELOG_POP           14'h03c
`define VC3D_CSR_REPAIR_CTRL        14'h040
`define VC3D_CSR_REPAIR_INDEX       14'h044
`define VC3D_CSR_REPAIR_DATA_LO     14'h048
`define VC3D_CSR_REPAIR_DATA_HI     14'h04c
`define VC3D_CSR_EFUSE_CTRL         14'h050
`define VC3D_CSR_EFUSE_ADDR         14'h054
`define VC3D_CSR_EFUSE_DATA         14'h058
`define VC3D_CSR_MBIST_CTRL         14'h060
`define VC3D_CSR_MBIST_STATUS       14'h064
`define VC3D_CSR_MBIST_FAIL_ADDR    14'h068
`define VC3D_CSR_MBIST_FAIL_DATA    14'h06c
`define VC3D_CSR_BOND_CTRL          14'h070
`define VC3D_CSR_BOND_STATUS        14'h074
`define VC3D_CSR_BOND_LANE_MAP      14'h078
`define VC3D_CSR_BOND_CRC_ERR       14'h07c
`define VC3D_CSR_THERMAL_CTRL       14'h080
`define VC3D_CSR_THERMAL_STATUS     14'h084
`define VC3D_CSR_TEMP_MAX           14'h088
`define VC3D_CSR_DVFS_CTRL          14'h08c
`define VC3D_CSR_PERF_SEL           14'h090
`define VC3D_CSR_PERF_LO            14'h094
`define VC3D_CSR_PERF_HI            14'h098
`define VC3D_CSR_INTERLEAVE_CTRL    14'h0a0
`define VC3D_CSR_SLICE_ENABLE       14'h0a4
`define VC3D_CSR_WAY_DISABLE        14'h0a8
`define VC3D_CSR_PREFETCH_CTRL      14'h0ac

// -----------------------------------------------------------------------------
// Helper macros
// -----------------------------------------------------------------------------
`define VC3D_MAX(a,b)  (((a) > (b)) ? (a) : (b))
`define VC3D_MIN(a,b)  (((a) < (b)) ? (a) : (b))

// Synchronous reset flop helper used across the subsystem (active-high rst).
`define VC3D_FF(clk, rst, q, d, rstval) \
    always @(posedge clk or posedge rst) begin \
        if (rst) q <= rstval; else q <= d; \
    end

`endif /* VC3D_DEFINES_VH */
