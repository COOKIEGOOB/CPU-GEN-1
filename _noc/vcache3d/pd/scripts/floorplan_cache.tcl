# =============================================================================
# VCACHE-3D cache dielet floorplan (OpenROAD)
#
# The dielet is almost entirely macro: 2048 x (1024 x 148) HD macros in four
# quadrants of 32 banks x 16 subarrays.  Quadrant q holds the q-th 16 B quarter
# of every line, and the four quadrant macros of one (bank, sub, row) are
# ganged in a single row block so they share wordline drivers and one repair
# entry -- so they are placed adjacent, not scattered.
#
# The bond field runs along the bottom edge and MIRRORS the base die map; both
# come from the same CSV pair, checked by pd/package/check_bond_alignment.py.
# =============================================================================

set DIE_W  3251
set DIE_H  3251
set CORE_M 60

initialize_floorplan \
    -die_area  "0 0 $DIE_W $DIE_H" \
    -core_area "$CORE_M $CORE_M [expr $DIE_W-$CORE_M] [expr $DIE_H-$CORE_M]" \
    -site      unit

make_tracks

# macro geometry from the compiler datasheet (pd/sram/sram_compiler_selection.md)
set MACRO_W 62.4
set MACRO_H 66.2
set GANG_W  [expr 4*$MACRO_W + 6]     ;# four quadrants ganged

# 32 banks laid out 8 x 4; inside a bank, 16 subarrays laid out 4 x 4
for {set bank 0} {$bank < 32} {incr bank} {
    set bx [expr $CORE_M + ($bank % 8) * 390]
    set by [expr $CORE_M + ($bank / 8) * 760]
    for {set sub 0} {$sub < 16} {incr sub} {
        set sx [expr $bx + ($sub % 4) * ($GANG_W + 8)]
        set sy [expr $by + ($sub / 4) * ($MACRO_H + 12)]
        for {set q 0} {$q < 4} {incr q} {
            place_macro -macro_name \
              "g_quad\[$q\].u_array/g_bank\[$bank\].u_bank/g_sub\[$sub\].u_sub/u_macro" \
              -location "[expr $sx + $q*$MACRO_W] $sy" \
              -orientation [expr {$q % 2 ? "MY" : "R0"}]
        }
    }
}

# bond pads: mirrored map, same pitch, same order
set fh [open "../openroad/bond_pad_map_cache.csv" r]
gets $fh header
while {[gets $fh line] >= 0} {
    set f [split $line ","]
    set net  [lindex $f 0]
    set type [lindex $f 1]
    set x    [lindex $f 4]
    set y    [lindex $f 5]
    if {$type eq "signal" || $type eq "control"} {
        place_pin -pin_name $net -layer M8 -location "$x $y" \
                  -pin_size "4.5 4.5" -force_to_die_boundary
    }
}
close $fh

# The dielet has very little standard-cell logic; give it a thin ring of
# placement area around the macro field rather than scattering it between
# macros, so that macro-to-macro abutment stays clean.
create_blockage -region "$CORE_M $CORE_M [expr $DIE_W-$CORE_M] [expr $DIE_H-600]" \
                -soft
