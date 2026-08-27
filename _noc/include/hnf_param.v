/*
* Copyright (c) 2024 Beijing Institute of Open Source Chip
* OpenNoC is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*
* Author:
*    Ziqing Li <liziqing@bosc.ac.cn>
*    Chunyan Lin <linchunyan@bosc.ac.cn>
*/

`ifndef HNF_PARAM_H
`define HNF_PARAM_H
// ============================================================================
// V-Cache sizing defaults (see _noc/VCACHE_NOTES.md for the full analysis)
// ----------------------------------------------------------------------------
// These defaults are the repository's 32 MiB-per-slice high-capacity profile (3 slices = 96 MiB):
//   * 16-way associativity         (same geometry class as Zen 4/Zen 5 L3)
//   * 32 MiB per HNF slice               (base plus stacked-capacity class)
//   * 128 MSHRs                    (enough miss-level parallelism to feed the
//                                   larger array; QoS pools scale algebraically)
//   * 1,048,576 snoop-filter slots (2x the 524,288 cache-line population)
// HNF_L3_CACHE_SIZE_PARAM is in KiB. Testbenches pin the compact 4 MiB / 32
// MSHR / 131072-SF profile, avoiding a 32 MiB behavioral allocation in CI.
// This is an RTL capacity/throughput profile, not a claim of AMD equivalence:
// hybrid bonding, SRAM density, clock rate, power, and package thermals are
// physical implementation properties that must be established after PPA.
// ============================================================================
`define HNF_PARAM #( \
     parameter CHIE_REQ_ADDR_WIDTH_PARAM    = 44,    \
     parameter CHIE_SNP_ADDR_WIDTH_PARAM    = 41,    \
     parameter CHIE_NID_WIDTH_PARAM         = 7,     \
     parameter CHIE_DATA_WIDTH_PARAM        = 256,   \
     parameter CHIE_BE_WIDTH_PARAM          = 32,    \
     parameter CHIE_DATACHECK_WIDTH_PARAM   = 32,    \
     parameter CHIE_POISON_WIDTH_PARAM      = 4,     \
     parameter CHIE_REQ_RSVDC_WIDTH_PARAM   = 0,     \
     parameter CHIE_DAT_RSVDC_WIDTH_PARAM   = 0,     \
     parameter HNF_MSHR_RNF_NUM_PARAM       = 4,     \
     parameter HNF_MSHR_RNI_NUM_PARAM       = 0,     \
     parameter RNF_NID_LIST_PARAM           = {7'd48,7'd16,7'd40,7'd8}, \
     parameter RNI_NID_LIST_PARAM           = {7'd1}, \
     parameter HNF_NID_PARAM                = 0,     \
     parameter SNF_NID_PARAM                = 32,     \
     parameter XP_LCRD_NUM_PARAM            = 15,    \
     parameter HNF_SF_ENTRIES_NUM_PARAM     = 1048576,\
     parameter HNF_SF_WAY_NUM_PARAM         = 16,    \
     parameter HNF_MSHR_EXCL_RN_NUM_PARAM   = 32,    \
     parameter HNF_MSHR_EXCL_RN_WIDTH_PARAM = 5,     \
     parameter HNF_MSHR_ENTRIES_NUM_PARAM   = 128,   \
     parameter HNF_MSHR_ENTRIES_WIDTH_PARAM = 7,     \
     parameter HNF_L3_CACHE_SIZE_PARAM      = 32768, \
     parameter HNF_L3_WAY_NUM_PARAM         = 16     )

`define HNF_PARAM_INST #( \
    .CHIE_REQ_ADDR_WIDTH_PARAM          (CHIE_REQ_ADDR_WIDTH_PARAM         ), \
    .CHIE_SNP_ADDR_WIDTH_PARAM          (CHIE_SNP_ADDR_WIDTH_PARAM         ), \
    .CHIE_NID_WIDTH_PARAM               (CHIE_NID_WIDTH_PARAM              ), \
    .CHIE_DATA_WIDTH_PARAM              (CHIE_DATA_WIDTH_PARAM             ), \
    .CHIE_BE_WIDTH_PARAM                (CHIE_BE_WIDTH_PARAM               ), \
    .CHIE_DATACHECK_WIDTH_PARAM         (CHIE_DATACHECK_WIDTH_PARAM        ), \
    .CHIE_POISON_WIDTH_PARAM            (CHIE_POISON_WIDTH_PARAM           ), \
    .CHIE_REQ_RSVDC_WIDTH_PARAM         (CHIE_REQ_RSVDC_WIDTH_PARAM        ), \
    .CHIE_DAT_RSVDC_WIDTH_PARAM         (CHIE_DAT_RSVDC_WIDTH_PARAM        ), \
    .HNF_MSHR_RNF_NUM_PARAM             (HNF_MSHR_RNF_NUM_PARAM            ), \
    .HNF_MSHR_RNI_NUM_PARAM             (HNF_MSHR_RNI_NUM_PARAM            ), \
    .RNF_NID_LIST_PARAM                 (RNF_NID_LIST_PARAM                ), \
    .RNI_NID_LIST_PARAM                 (RNI_NID_LIST_PARAM                ), \
    .HNF_NID_PARAM                      (HNF_NID_PARAM                     ), \
    .SNF_NID_PARAM                      (SNF_NID_PARAM                     ), \
    .XP_LCRD_NUM_PARAM                  (XP_LCRD_NUM_PARAM                 ), \
    .HNF_SF_ENTRIES_NUM_PARAM           (HNF_SF_ENTRIES_NUM_PARAM          ), \
    .HNF_SF_WAY_NUM_PARAM               (HNF_SF_WAY_NUM_PARAM              ), \
    .HNF_MSHR_EXCL_RN_NUM_PARAM         (HNF_MSHR_EXCL_RN_NUM_PARAM        ), \
    .HNF_MSHR_EXCL_RN_WIDTH_PARAM       (HNF_MSHR_EXCL_RN_WIDTH_PARAM      ), \
    .HNF_MSHR_ENTRIES_NUM_PARAM         (HNF_MSHR_ENTRIES_NUM_PARAM        ), \
    .HNF_MSHR_ENTRIES_WIDTH_PARAM       (HNF_MSHR_ENTRIES_WIDTH_PARAM      ), \
    .HNF_L3_CACHE_SIZE_PARAM            (HNF_L3_CACHE_SIZE_PARAM           ), \
    .HNF_L3_WAY_NUM_PARAM               (HNF_L3_WAY_NUM_PARAM              ))

`endif /* HNF_PARAM_H */
