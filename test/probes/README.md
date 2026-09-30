# Register-level probes (PCIe, kernel driver loaded)

Small C programs that watch the running board through `/dev/m2sdr0` while another process streams.

| Probe | What it shows |
|---|---|
| `tickprobe [n] [interval_us]` | sample counter rate and parity, time-base mode, timed TX gate counters and their rates |
| `readerprobe [seconds]` | how fast the TX DMA reader's table index advances (nominal = frame rate; much faster = it is racing through stale ring slots after an underrun) |
| `episode [seconds] [which]` | records reader index, gate counters and "armed − now" every ~170 µs and dumps the samples around the n-th late/stale event |

Build (after `make` in `litex_m2sdr/software/user`):

    S=../../litex_m2sdr/software
    for p in tickprobe readerprobe episode; do
      gcc -O2 -o $p $p.c -I$S/kernel -I$S/user/liblitepcie -I$S/user/libm2sdr $S/user/libm2sdr/libm2sdr.a -lm
    done

RX delivery lag is measured by the SoapySDR plugin itself: `M2SDR_RX_LAG_STATS=<seconds>` (see
`doc/rx_delivery_lag.md`).
