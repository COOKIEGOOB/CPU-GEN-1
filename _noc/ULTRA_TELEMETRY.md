# Fully expanded CHI telemetry fabric

`misc/chi_ultra_telemetry_fabric.v` is a synthesis-ready, explicitly unrolled
256-lane observability block. It adds more than 44,000 lines of Verilog while
providing real hardware rather than comments or filler: per-lane request,
response, data, backpressure, timeout, protocol-error, opcode-class, QoS,
latency histogram, min/max/EWMA, high-water, retry, and byte counters, plus
balanced global reduction trees.

The implementation is intentionally expanded rather than written with a
`generate` loop. This makes every lane and every reduction-tree node directly
addressable by synthesis constraints, debug probes, coverage tools, and
physical-design scripts. The deterministic generator is checked in at
`tools/generate_chi_ultra_telemetry.py`.

## Build and verification

From `_noc`:

```sh
make generate-ultra       # reproducibly regenerate the checked-in RTL
make check-ultra-generated # fail if the artifact is stale
make iverilog-ultra       # compile and run the self-checking smoke test
```

The RTL is also included in `file_list_tb.f`, so the normal repository VCS flow
compiles it. It is a passive monitor: connect each packed lane to the valid,
ready, opcode, QoS, response, and DAT metadata at the desired CHI observation
points. `TIMEOUT_CYCLES` controls the oldest-outstanding watchdog threshold;
`clear_counters` clears statistics without disturbing an active transaction.
