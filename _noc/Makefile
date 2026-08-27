# OpenNoC (BOSC) - build & verification flow
#
# Layout (this directory):
#   include/   CHI/HNF/HNI/RNI/SNF define & parameter files
#   misc/      cross-node helpers (BIQ, XP channel, FIFOs, ...)
#   hnf/ hni/ rni/ snf/   node RTL
#   tb/        testbenches (SystemVerilog - VCS)
#   case/      tb_hnf scenario files
#
# Intended tool: VCS (as used upstream). An iverilog elaboration smoke
# check is provided as best-effort (RTL is Verilog-2005 clean, the tbs
# are SystemVerilog and need VCS/Verilator).

VCS      ?= vcs
SIMV     ?= simv
TOP      ?= hnf        # hnf | rni | snf
TB       ?= tb_hnf     # tb_hnf | tb_rni | tb_snf

# ---- VCS (intended flow) -------------------------------------------------
VCS_OPTS   = -full64 +v2k -q -sverilog -fsdb +incdir+include+misc \
             -Mupdate -debug_acc+dmptf -debug_region+cell+encrypt

ifeq ($(TB),tb_hnf)
  VCS_OPTS += +define+tb_hnf
  TOP_SRC  = hnf
else ifeq ($(TB),tb_rni)
  VCS_OPTS += +define+ASSERT_CHECKER_ON
  TOP_SRC  = rni
else ifeq ($(TB),tb_snf)
  TOP_SRC  = snf
endif

com:
	$(VCS) $(VCS_OPTS) -o $(SIMV) -f file_list_tb.f -top $(TB)

sim:
	./$(SIMV) -l sim.log

run_dve:
	dve -full64 -vpd vcdplus.vpd &

# ---- iverilog best-effort elaboration check (RTL only, no tb) ------------
IV      ?= iverilog
IV_OPTS = -g2012 -Wall -s $(TOP) -o /dev/null \
          -Iinclude -Imisc \
          hnf/hnf.v hnf/hnf_bump.v hnf/hnf_link.v hnf/hnf_link_rxreq_parse.v \
          hnf/hnf_link_rxrsp_parse.v hnf/hnf_link_rxdat_parse.v \
          hnf/hnf_link_txreq_wrap.v hnf/hnf_link_txrsp_wrap.v \
          hnf/hnf_link_txsnp_wrap.v hnf/hnf_link_txdat_wrap.v \
          hnf/hnf_mshr.v hnf/hnf_mshr_ctl.v hnf/hnf_mshr_qos.v \
          hnf/hnf_mshr_bypass.v hnf/hnf_mshr_addr_buffer.v \
          hnf/hnf_mshr_global_monitor.v hnf/hnf_cache_pipeline.v \
          hnf/hnf_data_buffer.v hnf/hnf_mem_ctl.v \
          hnf/hnf_tag_sram.v hnf/hnf_sf_sram.v hnf/hnf_lru_sram.v \
          hnf/hnf_data_sram.v hnf/hnf_sram.v hnf/hnf_sram_mask.v \
          misc/hnf_biq.v misc/poll_function.v misc/poll_with_start_entry.v \
          misc/sync_fifo.v

iverilog-hnf:
	$(IV) $(IV_OPTS) -s hnf

iverilog-snf:
	$(IV) -g2012 -Wall -s snf -o /dev/null -Iinclude -Imisc \
	    snf/snf.v snf/snf_rxreq.v snf/snf_rxdat.v snf/snf_rxrsp.v \
	    snf/snf_txdat.v snf/snf_txrsp.v snf/snf_mshr.v snf/snf_qos.v \
	    snf/snf_data_buffer.v

iverilog-hni:
	$(IV) -g2012 -Wall -s hni -o /dev/null -Iinclude -Imisc \
	    hni/hni.v hni/hni_rxreq.v hni/hni_rxdat.v hni/hni_rxrsp.v \
	    hni/hni_txdat.v hni/hni_txrsp.v hni/hni_mshr.v hni/hni_qos.v \
	    hni/hni_data_buffer.v hni/hni_global_monitor.v

clean:
	rm -rf csrc *.vpd *.fsdb *.daidir *.log simv* *.key DVEfiles vivado_syn_output *.conf *.rc verdiLog

.PHONY: com sim run_dve iverilog-hnf iverilog-snf iverilog-hni clean
