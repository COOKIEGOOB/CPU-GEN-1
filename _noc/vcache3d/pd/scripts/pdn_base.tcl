# =============================================================================
# VCACHE-3D base die power grid.
#
# Two supplies: VDD (logic, 0.95 V) and VDDM (macro periphery, 0.80 V), plus
# VDD_BOND for the bond field drivers.  The dielet is powered THROUGH this die,
# so the grid also has to carry the dielet's ~0.6 W up through the bond pads --
# that is what the dedicated VDD_BOND straps over the pad field are for.
# =============================================================================

add_global_connection -net VDD     -pin_pattern {^VDD$}     -power
add_global_connection -net VDDM    -pin_pattern {^VDDM$}    -power
add_global_connection -net VDD_BOND -pin_pattern {^VDD_BOND$} -power
add_global_connection -net VSS     -pin_pattern {^VSS$}     -ground

set_voltage_domain -name CORE -power VDD  -ground VSS
set_voltage_domain -name BOND -power VDD_BOND -ground VSS

define_pdn_grid -name core_grid -voltage_domains CORE
add_pdn_stripe -grid core_grid -layer M1  -width 0.06 -followpins
add_pdn_stripe -grid core_grid -layer M4  -width 0.60 -pitch 12.0 -offset 2.0
add_pdn_stripe -grid core_grid -layer M8  -width 2.40 -pitch 48.0 -offset 8.0
add_pdn_stripe -grid core_grid -layer M10 -width 6.00 -pitch 96.0 -offset 16.0
add_pdn_connect -grid core_grid -layers {M1 M4}
add_pdn_connect -grid core_grid -layers {M4 M8}
add_pdn_connect -grid core_grid -layers {M8 M10}

# SRAM macro rings: the 8192x576 data macros pull ~90 mA peak each
define_pdn_grid -name macro_grid -voltage_domains CORE -macro \
    -cells {VC3D_HP_SPSRAM_8192X576 VC3D_HP_SPSRAM_8192X39}
add_pdn_ring -grid macro_grid -layers {M5 M6} -widths {1.2 1.2} \
             -spacings {0.6 0.6} -core_offsets {1.2 1.2}
add_pdn_connect -grid macro_grid -layers {M4 M5}
add_pdn_connect -grid macro_grid -layers {M6 M8}

# bond field: dense M10 plate so the dielet supply does not droop
define_pdn_grid -name bond_grid -voltage_domains BOND
add_pdn_stripe -grid bond_grid -layer M10 -width 4.00 -pitch 18.0 -offset 4.0
add_pdn_connect -grid bond_grid -layers {M8 M10}

# IR budget: 3 % of 0.95 V = 28 mV static, checked after routing
set_pdn_ir_limit 0.028
