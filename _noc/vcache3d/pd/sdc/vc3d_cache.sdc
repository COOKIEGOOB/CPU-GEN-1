# =============================================================================
# CPU-GEN-1 : VCACHE-3D -- cache-dielet (stacked SRAM tier) constraints
#
# Target: 1.5 GHz, SS 0.68 V array / 0.75 V periphery, 125 C.
# The dielet has NO local oscillator: its clock arrives over the bond field,
# so it is created here as a port clock with the base-die jitter folded in.
#
# SPDX-License-Identifier: MulanPSL-2.0
# =============================================================================

set ARRAY_PERIOD 0.6667

create_clock -name bond_clk -period $ARRAY_PERIOD [get_ports clk]

# forwarded clock: source jitter from the base die PLL plus bond flight
# variation; there is no local PLL to filter it, so the budget is larger than
# on the base tier.
set_clock_uncertainty -setup 0.045 [get_clocks bond_clk]
set_clock_uncertainty -hold  0.030 [get_clocks bond_clk]
set_clock_transition  0.025 [all_clocks]
set_max_transition    0.150 [current_design]
set_max_capacitance   0.200 [current_design]
set_max_fanout        20    [current_design]

set_input_delay  -clock bond_clk -max 0.180 [get_ports {pad_in[*]}]
set_input_delay  -clock bond_clk -min 0.020 [get_ports {pad_in[*]}]
set_output_delay -clock bond_clk -max 0.180 [get_ports {pad_out[*]}]
set_output_delay -clock bond_clk -min 0.020 [get_ports {pad_out[*]}]
set_load 0.0024 [get_ports {pad_out[*]}]

# repair fabric is programmed once at boot from the base-die eFuse image
set_false_path -from [get_ports {rpr_row_valid[*] rpr_row_addr[*] \
                                 rpr_col_valid[*] rpr_col_id[*]}]

# power control is quasi-static and crosses into gated banks
set_multicycle_path -setup 8 -through [get_ports {bank_sleep[*] \
                                                  bank_deep_sleep[*] \
                                                  bank_retention[*]}]
set_multicycle_path -hold  7 -through [get_ports {bank_sleep[*] \
                                                  bank_deep_sleep[*] \
                                                  bank_retention[*]}]

# write/read assist codes settle long before an access
set_false_path -from [get_ports {wa_code[*] ra_code[*]}]

set_false_path -from [get_ports rst]

group_path -name STACK_CMD   -through [get_pins -hier *g_quad*/u_array/*]
group_path -name STACK_READ  -through [get_pins -hier *u_bank_*/*]
group_path -name BOND_CACHE  -through [get_pins -hier *u_link*/*]

# The dielet is a memory tier: almost all of its area is macro, so the useful
# density knob is macro placement, not standard-cell utilisation.
set_max_area 0
