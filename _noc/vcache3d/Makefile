# =============================================================================
# CPU-GEN-1 : VCACHE-3D -- build, check and report targets.
#
#   make gen      regenerate every generated source (ECC codecs, stacked array,
#                 filelists) -- all generators are deterministic
#   make check    fail if a generated file is stale (CI gate)
#   make lint     structural + port linting of all RTL and testbenches
#   make model    run the Python golden-model property tests
#   make reports  run the analytical PPA/thermal/package models into pd/reports
#   make sim-*    run a testbench (needs a simulator; none is required for the
#                 checks above)
# =============================================================================
PYTHON ?= python3
VCS    ?= vcs
VERILATOR ?= verilator

.PHONY: all gen check lint model reports clean \
        sim-ecc sim-stack sim-repair sim-slice sim-96mib sim-all

all: gen lint model reports

gen:
	$(PYTHON) gen/vc3d_gen_ecc.py
	$(PYTHON) gen/vc3d_gen_stack_array.py
	$(PYTHON) gen/vc3d_gen_macro_slice_array.py
	$(PYTHON) gen/vc3d_gen_filelists.py

check:
	@cp -r rtl/ecc /tmp/vc3d_ecc_ref
	@cp rtl/stack/vc3d_stack_bank_array.v /tmp/vc3d_array_ref.v
	@cp rtl/stack/vc3d_stack_macro_slice_array.v /tmp/vc3d_macro_ref.v
	@cp filelist_base.f /tmp/vc3d_flist_ref.f
	@$(MAKE) --no-print-directory gen >/dev/null
	@diff -r /tmp/vc3d_ecc_ref rtl/ecc >/dev/null \
	    || (echo "ERROR: rtl/ecc is stale, run 'make gen'"; exit 1)
	@diff /tmp/vc3d_array_ref.v rtl/stack/vc3d_stack_bank_array.v >/dev/null \
	    || (echo "ERROR: vc3d_stack_bank_array.v is stale, run 'make gen'"; exit 1)
	@diff /tmp/vc3d_macro_ref.v rtl/stack/vc3d_stack_macro_slice_array.v >/dev/null \
	    || (echo "ERROR: vc3d_stack_macro_slice_array.v is stale, run 'make gen'"; exit 1)
	@diff /tmp/vc3d_flist_ref.f filelist_base.f >/dev/null \
	    || (echo "ERROR: filelist_base.f is stale, run 'make gen'"; exit 1)
	@rm -rf /tmp/vc3d_ecc_ref /tmp/vc3d_array_ref.v /tmp/vc3d_macro_ref.v /tmp/vc3d_flist_ref.f
	@echo "generated sources are up to date"

lint:
	$(PYTHON) gen/vc3d_lint.py

model:
	cd model && $(PYTHON) test_vc3d_model.py

reports:
	$(MAKE) -C pd/openroad reports
	@echo "reports written to pd/reports/"

# ---- simulation (optional: requires a simulator in PATH) --------------------
define RUN_VCS
	$(VCS) -full64 -sverilog -timescale=1ns/1ps \
	    -f filelist_base.f -f filelist_tb.f -top $(1) -o simv_$(1) && ./simv_$(1)
endef

sim-ecc:
	$(call RUN_VCS,tb_vc3d_ecc)

sim-stack:
	$(call RUN_VCS,tb_vc3d_stack_bond)

sim-repair:
	$(call RUN_VCS,tb_vc3d_repair)

sim-slice:
	$(call RUN_VCS,tb_vc3d_slice)

sim-96mib:
	$(call RUN_VCS,tb_vc3d_96mib)

sim-all: sim-ecc sim-stack sim-repair sim-slice sim-96mib

clean:
	rm -rf simv_* csrc *.vpd *.fsdb ucli.key obj_dir
	rm -f pd/reports/*.rpt
