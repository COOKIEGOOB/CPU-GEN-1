# =============================================================================
# CPU-GEN-1 : VCACHE-3D -- base-die (logic tier) constraints
#
# Target: 3.0 GHz core clock, SS 0.855 V 125 C signoff corner.
# The bond interface is source-synchronous: the base die forwards its clock to
# the dielet, so the cache-tier constraints (sdc/vc3d_cache.sdc) are written
# against a virtual clock generated from the same source.
#
# SPDX-License-Identifier: MulanPSL-2.0
# =============================================================================

set CORE_PERIOD   0.3333
set BOND_PERIOD   0.6667
set APB_PERIOD    5.0

# -----------------------------------------------------------------------------
# clocks
# -----------------------------------------------------------------------------
create_clock -name core_clk -period $CORE_PERIOD [get_ports clk]
create_clock -name apb_clk  -period $APB_PERIOD  [get_ports csr_pclk]

# the forwarded bond clock is core_clk divided by two and launched from a pad
create_generated_clock -name bond_clk \
    -source [get_ports clk] -divide_by 2 [get_ports bond_clk_out]

set_clock_groups -asynchronous \
    -group {core_clk bond_clk} \
    -group {apb_clk}

# -----------------------------------------------------------------------------
# uncertainty and transition
#   jitter 8 ps + on-die skew budget 25 ps, as used by pd/scripts/timing_model.py
# -----------------------------------------------------------------------------
set_clock_uncertainty -setup 0.033 [get_clocks core_clk]
set_clock_uncertainty -hold  0.020 [get_clocks core_clk]
set_clock_uncertainty -setup 0.040 [get_clocks bond_clk]
set_clock_uncertainty -hold  0.025 [get_clocks bond_clk]
set_clock_transition  0.020 [all_clocks]
set_max_transition    0.120 [current_design]
set_max_capacitance   0.180 [current_design]
set_max_fanout        24    [current_design]

# -----------------------------------------------------------------------------
# I/O timing
#   requests arrive from the HN-F adapter, responses leave to it
# -----------------------------------------------------------------------------
set_input_delay  -clock core_clk 0.110 [remove_from_collection [all_inputs] \
                     [get_ports {clk rst csr_pclk}]]
set_output_delay -clock core_clk 0.110 [all_outputs]

# hybrid-bond pads: the flight time across a 9 um bond is ~31 ps round trip;
# the receive side is deskewed per lane by the training FSM, so the static
# constraint only has to cover the pad driver and the on-die run.
set_input_delay  -clock bond_clk -max 0.180 [get_ports {pad_in[*]}]
set_input_delay  -clock bond_clk -min 0.020 [get_ports {pad_in[*]}]
set_output_delay -clock bond_clk -max 0.180 [get_ports {pad_out[*]}]
set_output_delay -clock bond_clk -min 0.020 [get_ports {pad_out[*]}]
set_load 0.0024 [get_ports {pad_out[*]}]

# -----------------------------------------------------------------------------
# false / multicycle paths
# -----------------------------------------------------------------------------
# eFuse programming and MBIST control are quasi-static
set_false_path -from [get_ports {efuse_prog* efuse_sense*}]
set_multicycle_path -setup 4 -through [get_pins -hier *u_efuse*/*]
set_multicycle_path -hold  3 -through [get_pins -hier *u_efuse*/*]

# thermal sensor readout is sampled at 1 kHz
set_multicycle_path -setup 8 -through [get_pins -hier *u_sensor*/*]
set_multicycle_path -hold  7 -through [get_pins -hier *u_sensor*/*]

# CSR window crossing is handled by a 2-flop synchroniser
set_false_path -from [get_clocks apb_clk] -to [get_clocks core_clk]
set_false_path -from [get_clocks core_clk] -to [get_clocks apb_clk]

# reset is synchronised locally in every module
set_false_path -from [get_ports rst]

# -----------------------------------------------------------------------------
# path groups -- these mirror pd/scripts/timing_model.py one for one, so the
# analytical model and real STA report the same buckets
# -----------------------------------------------------------------------------
group_path -name ROUTER_HASH  -through [get_pins -hier *u_hash*/*]
group_path -name TAG_READ     -through [get_pins -hier *u_tag*/*]
group_path -name DATA_READ    -through [get_pins -hier *u_base_data*/*]
group_path -name ECC          -through [get_pins -hier *u_dec*/*]
group_path -name BOND         -through [get_pins -hier *u_bond*/*]

# -----------------------------------------------------------------------------
# design rule / macro timing
# -----------------------------------------------------------------------------
set_driving_cell -lib_cell BUFFD4BWP240H8P57 [all_inputs]
