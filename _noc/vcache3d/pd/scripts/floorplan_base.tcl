# =============================================================================
# VCACHE-3D base die floorplan (OpenROAD)
#
# Three slice columns side by side, each with its tag arrays at the top, base
# data arrays in the middle, and the bond pad field along the bottom edge so
# that the vertical bond wires are short and the dielet can sit directly above
# the data arrays (shortest bond runs go to the highest-traffic logic).
#
# Numbers come from pd/scripts/area_model.py -- rerun it after any RTL change
# that moves memory around.
# =============================================================================

set DIE_W   4838
set DIE_H   4838
set CORE_M  120

initialize_floorplan \
    -die_area  "0 0 $DIE_W $DIE_H" \
    -core_area "$CORE_M $CORE_M [expr $DIE_W-$CORE_M] [expr $DIE_H-$CORE_M]" \
    -site      unit

make_tracks

# -----------------------------------------------------------------------------
# slice columns
# -----------------------------------------------------------------------------
set SLICE_W [expr ($DIE_W - 2*$CORE_M) / 3]

for {set s 0} {$s < 3} {incr s} {
    set x0 [expr $CORE_M + $s*$SLICE_W]
    set x1 [expr $x0 + $SLICE_W - 40]

    # 16 ways x 4 set banks of tag, two rows of eight along the top
    for {set w 0} {$w < 16} {incr w} {
        for {set b 0} {$b < 4} {incr b} {
            set mx [expr $x0 + 60 + ($w % 8) * 185]
            set my [expr 3350 + ($w / 8) * 620 + $b * 150]
            place_macro -macro_name \
                "u_slice_${s}/u_tag/g_way[$w].g_bank[$b].u_macro" \
                -location "$mx $my" -orientation R0
        }
    }

    # 4 base ways x 4 set banks of data, the big blocks in the middle
    for {set w 0} {$w < 4} {incr w} {
        for {set b 0} {$b < 4} {incr b} {
            set mx [expr $x0 + 80 + $b * 340]
            set my [expr 1200 + $w * 500]
            place_macro -macro_name \
                "u_slice_${s}/u_base_data/g_way[$w].g_bank[$b].u_macro" \
                -location "$mx $my" -orientation R0
        }
    }
}

# -----------------------------------------------------------------------------
# bond pad field along the bottom edge, from the shared CSV
# -----------------------------------------------------------------------------
set fh [open "../openroad/bond_pad_map_base.csv" r]
gets $fh header
while {[gets $fh line] >= 0} {
    set f [split $line ","]
    set net  [lindex $f 0]
    set type [lindex $f 1]
    set x    [lindex $f 4]
    set y    [lindex $f 5]
    if {$type eq "signal" || $type eq "control"} {
        place_pin -pin_name $net -layer M10 -location "$x $y" \
                  -pin_size "4.5 4.5" -force_to_die_boundary
    }
}
close $fh

# keep routing off the bond field except for the pad stacks themselves
create_blockage -layer M9 -region "100 100 3300 200"

set_placement_padding -global -left 2 -right 2
