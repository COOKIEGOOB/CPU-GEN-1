# =============================================================================
# VCACHE-3D cache dielet power grid.
#
# The dielet has no package connection of its own: every ampere it draws comes
# up through the hybrid bond pads.  384 of the 1784 pads are power/ground for
# exactly this reason (1:2 PG:signal in the field, plus the ring), and the grid
# below is built as a plate rather than a sparse mesh because the current has
# to be collected from the whole array and delivered to the pad field on one
# edge.
#
# Two supplies: VDDA (array, 0.68 V nominal with write/read assist) and VDDP
# (periphery / bond endpoint, 0.75 V).  The split is what allows the array to
# sit at retention voltage while the link stays alive.
# =============================================================================

add_global_connection -net VDDA -pin_pattern {^VDDA$} -power
add_global_connection -net VDDP -pin_pattern {^VDDP$} -power
add_global_connection -net VSS  -pin_pattern {^VSS$}  -ground

set_voltage_domain -name ARRAY -power VDDA -ground VSS
set_voltage_domain -name PERI  -power VDDP -ground VSS

define_pdn_grid -name array_grid -voltage_domains ARRAY
add_pdn_stripe -grid array_grid -layer M3 -width 0.90 -pitch 8.0  -offset 1.0
add_pdn_stripe -grid array_grid -layer M6 -width 3.00 -pitch 32.0 -offset 6.0
add_pdn_stripe -grid array_grid -layer M8 -width 8.00 -pitch 40.0 -offset 8.0
add_pdn_connect -grid array_grid -layers {M3 M6}
add_pdn_connect -grid array_grid -layers {M6 M8}

# per-bank header switches for sleep / deep sleep / retention
define_power_switch -name bank_hdr -control bank_sleep \
    -power_switchable VDDA_SW -power VDDA -ground VSS
define_pdn_grid -name sw_grid -voltage_domains ARRAY -switch_cell bank_hdr

define_pdn_grid -name peri_grid -voltage_domains PERI
add_pdn_stripe -grid peri_grid -layer M8 -width 4.00 -pitch 24.0 -offset 4.0
add_pdn_connect -grid peri_grid -layers {M6 M8}

# The bond pads themselves are the top-level current collectors; M8 is the
# top routing layer on the dielet because its face is bonded, not bumped.
set_pdn_ir_limit 0.020
