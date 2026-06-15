#!/usr/bin/env bash
set -u

VALIDATION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$VALIDATION_ROOT/scripts/env_ocudu_validation.sh"
source "$HOME/CLionProjects/env_m2sdr_current.sh" >/dev/null 2>&1

RUN_ROOT="${VALIDATION_RUN_ROOT:-$VALIDATION_ROOT/runs}"
OUT="$RUN_ROOT/$(date -u +%Y%m%dT%H%M%SZ)_soapy_rx_smoke_cpp_logged"
mkdir -p "$OUT"

BUILD_DIR="$VALIDATION_ROOT/scripts/.build"
SRC="$BUILD_DIR/soapy_rx_smoke.cpp"
BIN="$BUILD_DIR/soapy_rx_smoke"
LOG="$OUT/run.log"
SUMMARY="$OUT/SUMMARY.txt"

mkdir -p "$BUILD_DIR"

write_summary() {
  {
    echo "OUT=$OUT"
    grep -m1 '^compile_result=' "$LOG" || echo "compile_result=missing"
    grep -m1 '^soapy_find_exit=' "$LOG" || echo "soapy_find_exit=missing"
    grep -m1 '^soapy_probe_exit=' "$LOG" || echo "soapy_probe_exit=missing"
    grep -m1 '^record_exit=' "$LOG" || echo "record_exit=missing"
    grep -m1 '^bytes=' "$LOG" || echo "bytes=missing"
    grep -m1 '^samples=' "$LOG" || echo "samples=missing"
    echo -n "kernel_red_flags="
    grep -qiE 'kernel: .*bad page|kernel: .*BUG|kernel: .*oops|kernel: .*panic|kernel: .*litepcie|kernel: .*mmap:litepcie_mmap|kernel: .*segfault|kernel: .*blocked|kernel: .*hung' "$LOG" && echo "YES" || echo "NO"
    echo
    echo "===== last 160 log lines ====="
    tail -160 "$LOG" 2>/dev/null || true
  } > "$SUMMARY"
}
trap write_summary EXIT

cat > "$SRC" <<'CPP'
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Errors.hpp>

#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    const std::string out_path = (argc > 1) ? argv[1] : "rx_200ms.cf32";

    const double rate = 11520000.0;
    const double bw   = 10000000.0;
    const double freq = 1842500000.0;
    const double gain = 20.0;
    const size_t chan = 0;
    const double secs = 0.2;

    SoapySDR::Kwargs args;
    args["driver"] = "LiteXM2SDR";

    SoapySDR::Device *sdr = nullptr;
    SoapySDR::Stream *rx = nullptr;

    std::ofstream out(out_path, std::ios::binary);
    if (!out) {
        std::cerr << "failed_to_open_output=" << out_path << "\n";
        return 2;
    }

    try {
        sdr = SoapySDR::Device::make(args);
        if (!sdr) {
            std::cerr << "device_make_failed\n";
            return 3;
        }

        std::cerr << "setting_rate=" << rate << "\n";
        sdr->setSampleRate(SOAPY_SDR_RX, chan, rate);

        std::cerr << "setting_freq=" << freq << "\n";
        sdr->setFrequency(SOAPY_SDR_RX, chan, freq);

        std::cerr << "setting_gain=" << gain << "\n";
        sdr->setGain(SOAPY_SDR_RX, chan, gain);

        std::cerr << "setting_bw=" << bw << "\n";
        sdr->setBandwidth(SOAPY_SDR_RX, chan, bw);

        std::vector<size_t> channels = {chan};
        rx = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
        if (!rx) {
            std::cerr << "setupStream_failed\n";
            return 4;
        }

        const int act = sdr->activateStream(rx);
        std::cerr << "activateStream_ret=" << act << "\n";
        if (act != 0) {
            return 5;
        }

        std::vector<std::complex<float>> buf(2048);
        void *buffs[] = {buf.data()};

        uint64_t total_samples = 0;
        uint64_t chunks = 0;
        uint64_t timeouts = 0;
        uint64_t errors = 0;

        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds((int)(secs * 1000.0));

        while (std::chrono::steady_clock::now() < deadline) {
            int flags = 0;
            long long time_ns = 0;

            const int ret = sdr->readStream(rx, buffs, buf.size(), flags, time_ns, 200000);

            if (ret > 0) {
                out.write(reinterpret_cast<const char *>(buf.data()), ret * sizeof(std::complex<float>));
                total_samples += (uint64_t)ret;
                chunks++;
            } else if (ret == SOAPY_SDR_TIMEOUT) {
                timeouts++;
            } else {
                std::cerr << "readStream_error=" << ret << "\n";
                errors++;
                break;
            }
        }

        sdr->deactivateStream(rx);
        sdr->closeStream(rx);
        rx = nullptr;
        SoapySDR::Device::unmake(sdr);
        sdr = nullptr;

        out.close();

        std::cout << "samples=" << total_samples << "\n";
        std::cout << "chunks=" << chunks << "\n";
        std::cout << "timeouts=" << timeouts << "\n";
        std::cout << "errors=" << errors << "\n";

        return (total_samples > 0 && errors == 0) ? 0 : 10;
    } catch (const std::exception &e) {
        std::cerr << "exception=" << e.what() << "\n";
        if (rx && sdr) {
            try { sdr->deactivateStream(rx); } catch (...) {}
            try { sdr->closeStream(rx); } catch (...) {}
        }
        if (sdr) {
            try { SoapySDR::Device::unmake(sdr); } catch (...) {}
        }
        return 11;
    }
}
CPP

{
  echo "===== start ====="
  date -u
  uname -a
  echo "OUT=$OUT"

  echo
  echo "===== env ====="
  echo "VALIDATION_ROOT=$VALIDATION_ROOT"
  echo "M2SDR_ROOT=${M2SDR_ROOT:-unset}"
  echo "M2SDR_SW=${M2SDR_SW:-unset}"
  echo "SOAPY_SDR_ROOT=${SOAPY_SDR_ROOT:-unset}"
  echo "SOAPY_SDR_PLUGIN_PATH=${SOAPY_SDR_PLUGIN_PATH:-unset}"
  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-unset}"
  echo "M2SDR_DIRECT_DEINTERLEAVE=${M2SDR_DIRECT_DEINTERLEAVE:-unset}"

  echo
  echo "===== compile C++ Soapy RX smoke ====="
  echo "pkg_config_version=$(pkg-config --modversion SoapySDR 2>/dev/null || echo missing)"

  if g++ -O2 -std=c++17 "$SRC" -o "$BIN" $(pkg-config --cflags --libs SoapySDR); then
    echo "compile_result=PASS"
  else
    echo "compile_result=FAIL"
    exit 30
  fi

  echo
  echo "===== Soapy find/probe precheck ====="
  timeout -k 2s 20s SoapySDRUtil --find="driver=LiteXM2SDR"
  find_rc=$?
  echo "soapy_find_exit=$find_rc"

  timeout -k 2s 30s SoapySDRUtil --probe="driver=LiteXM2SDR"
  probe_rc=$?
  echo "soapy_probe_exit=$probe_rc"

  if [ "$find_rc" -ne 0 ] || [ "$probe_rc" -ne 0 ]; then
    echo "record_exit=not_run_probe_failed"
    echo "bytes=missing"
    exit 31
  fi

  echo
  echo "===== C++ Soapy RX-only 200 ms smoke ====="
  timeout -k 2s 8s "$BIN" "$OUT/rx_200ms.cf32"
  rc=$?
  echo "record_exit=$rc"

  echo
  echo "===== output file ====="
  ls -lh "$OUT/rx_200ms.cf32" 2>&1 || true
  stat -c 'bytes=%s' "$OUT/rx_200ms.cf32" 2>/dev/null || echo "bytes=missing"

  echo
  echo "===== kernel warnings/errors ====="
  journalctl -k -b --since '5 minutes ago' --no-pager | \
    grep -iE 'm2sdr|litepcie|bad page|BUG|oops|panic|dma|timeout|error|fail|segfault|blocked|hung' || true

  echo
  echo "===== end ====="
  date -u
} > "$LOG" 2>&1

echo "$OUT"
