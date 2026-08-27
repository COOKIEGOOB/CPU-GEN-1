# Thin driver around OpenROAD-flow-scripts for the two tiers of VCACHE-3D.
ORFS ?= $(HOME)/OpenROAD-flow-scripts/flow

.PHONY: all base cache bond reports clean check-config

all: base cache bond

base:
	$(MAKE) -C $(ORFS) DESIGN_CONFIG=$(CURDIR)/config.mk TIER=base

cache:
	$(MAKE) -C $(ORFS) DESIGN_CONFIG=$(CURDIR)/config.mk TIER=cache

bond:
	python3 ../package/gen_bump_map.py
	python3 ../package/check_bond_alignment.py

# Analytical signoff models: these run without any EDA tool and produce the
# reports in pd/reports/.  They are the numbers quoted in docs/PPA_REPORT.md.
reports:
	python3 ../scripts/timing_model.py  > ../reports/timing_summary.rpt
	python3 ../scripts/area_model.py    > ../reports/area_summary.rpt
	python3 ../scripts/power_model.py   > ../reports/power_summary.rpt
	python3 ../thermal/thermal_model.py > ../reports/thermal_summary.rpt
	python3 ../package/gen_bump_map.py         > ../reports/package_summary.rpt
	python3 ../package/check_bond_alignment.py >> ../reports/package_summary.rpt

check-config:
	@test -f config.mk && echo "config.mk present"
	@python3 -c "print('tiers: base, cache')"

clean:
	rm -f ../reports/*.rpt
