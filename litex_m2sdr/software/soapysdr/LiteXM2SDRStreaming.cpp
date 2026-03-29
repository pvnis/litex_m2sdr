/*
 * SoapySDR driver for the LiteX M2SDR.
 *
 * Copyright (c) 2021-2026 Enjoy Digital.
 * Copyright (c) 2021 Julia Computing.
 * Copyright (c) 2015-2015 Fairwaves, Inc.
 * Copyright (c) 2015-2015 Rice University
 * SPDX-License-Identifier: Apache-2.0
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <chrono>
#include <cassert>
#include <thread>
#include <sys/mman.h>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <pthread.h>
#include <sched.h>

#include "ad9361/ad9361.h"
#include "ad9361/ad9361_api.h"

#include "LiteXM2SDRDevice.hpp"

/* Parse Soapy-style "channels" argument: "0", "1", "0,1", "[0,1]", "0 1". */
static std::vector<size_t> parse_channel_list(const std::string &channels_str)
{
    std::string s = channels_str;
    for (char &c : s) {
        if (c == '[' || c == ']')
            c = ' ';
        if (c == ',')
            c = ' ';
    }

    std::stringstream ss(s);
    std::vector<size_t> chans;
    std::string tok;
    while (ss >> tok) {
        char *end = nullptr;
        long v = std::strtol(tok.c_str(), &end, 10);
        if (!end || *end != '\0')
            throw std::runtime_error("Invalid channel token: " + tok);
        if (v < 0 || v > 1)
            throw std::runtime_error("Invalid channel index: must be 0 or 1");
        if (std::find(chans.begin(), chans.end(), static_cast<size_t>(v)) == chans.end())
            chans.push_back(static_cast<size_t>(v));
    }

    if (chans.empty())
        throw std::runtime_error("No channels parsed from: " + channels_str);
    if (chans.size() > 2)
        throw std::runtime_error("Invalid channel count: must be 1 or 2 channels");
    if (chans.size() == 2) {
        std::sort(chans.begin(), chans.end());
        if (!(chans[0] == 0 && chans[1] == 1))
            throw std::runtime_error("Dual channels must be 0 and 1");
    }
    return chans;
}

static inline long long steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static inline long long sample_count_to_ns(double sample_rate, long long samples)
{
    if (sample_rate <= 0.0) {
        return 0;
    }
    const double ns = (static_cast<double>(samples) * 1e9) / sample_rate;
    return static_cast<long long>(std::llround(ns));
}

/* RX DMA Header - always 16 bytes on LitePCIe (sync word + RX timestamp). */
#if USE_LITEPCIE
static constexpr size_t RX_DMA_HEADER_SIZE = 16;
#else
static constexpr size_t RX_DMA_HEADER_SIZE = 0;
#endif

static constexpr uint64_t DMA_HEADER_SYNC_WORD = 0x5aa55aa55aa55aa5ULL;

/* TX DMA Header - always 16 bytes on PCIe (sync word + timestamp for TimedTXArbiter). */
#if USE_LITEPCIE
static constexpr size_t TX_DMA_HEADER_SIZE = 16;
#else
static constexpr size_t TX_DMA_HEADER_SIZE = 0;
#endif

/* TX burst control flags embedded in the upper 16 bits of DMA header word 0. */
static constexpr uint64_t TX_FLAG_HAS_TIME  = (1ULL << 63);
static constexpr uint64_t TX_FLAG_END_BURST = (1ULL << 62);

#if USE_LITEPCIE
static int poll_timeout_ms_slice(const long remaining_us)
{
    if (remaining_us <= 0)
        return 0;
    const long slice_us = std::min<long>(remaining_us, 1000);
    return std::max<int>(1, static_cast<int>((slice_us + 999) / 1000));
}
#endif

#if USE_LITEPCIE || USE_VFIO
static bool debug_log_allowed(bool enabled, std::atomic<uint64_t> &seq, uint64_t limit)
{
    if (!enabled)
        return false;
    const uint64_t cur = ++seq;
    return (limit == 0 || cur <= limit);
}

static bool rx_ts_trace_enabled()
{
    static const bool enabled = []() {
        const char *env = std::getenv("M2SDR_SOAPY_RX_TS_TRACE");
        return env && env[0] != '\0' && env[0] != '0';
    }();
    return enabled;
}

static bool rx_ts_trace_allowed()
{
    static std::atomic<uint64_t> seq{0};
    static const uint64_t limit = []() {
        const char *env = std::getenv("M2SDR_SOAPY_RX_TS_TRACE_LIMIT");
        if (!env || env[0] == '\0')
            return uint64_t{200};
        return static_cast<uint64_t>(std::strtoull(env, nullptr, 0));
    }();
    return debug_log_allowed(rx_ts_trace_enabled(), seq, limit);
}

static void configure_worker_thread(std::thread &thread, const char *name, int rt_prio, int cpu)
{
    if (!thread.joinable())
        return;

    pthread_t tid = thread.native_handle();
#if defined(__linux__)
    if (name && *name)
        pthread_setname_np(tid, name);
#endif

    if (cpu >= 0) {
#if defined(__linux__)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu, &cpuset);
        const int ret = pthread_setaffinity_np(tid, sizeof(cpuset), &cpuset);
        if (ret != 0) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "%s worker affinity to CPU %d failed: %s",
                          name ? name : "worker", cpu, std::strerror(ret));
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "%s worker pinned to CPU %d",
                          name ? name : "worker", cpu);
        }
#endif
    }

    if (rt_prio > 0) {
        sched_param param{};
        param.sched_priority = rt_prio;
        const int ret = pthread_setschedparam(tid, SCHED_FIFO, &param);
        if (ret != 0) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "%s worker RT priority %d failed: %s",
                          name ? name : "worker", rt_prio, std::strerror(ret));
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "%s worker RT priority set to %d",
                          name ? name : "worker", rt_prio);
        }
    }
}
#endif

void SoapyLiteXM2SDR::checkRxReceiveError()
{
#if USE_LITEPCIE || USE_VFIO
    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
    if (!_rx_stream.recv_error.empty()) {
        const std::string err = _rx_stream.recv_error;
        _rx_stream.recv_error.clear();
        throw std::runtime_error(err);
    }
#endif
}

void SoapyLiteXM2SDR::rxReceiveLoop()
{
#if USE_LITEPCIE || USE_VFIO
    try {
        if (_rx_stream.timed_start_pending) {
            for (;;) {
                {
                    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
                    if (_rx_stream.recv_thread_stop) {
                        _rx_stream.recv_thread_running = false;
                        _rx_stream.recv_cv.notify_all();
                        return;
                    }
                }
                const long long now_ns = this->getHardwareTime("");
                if (now_ns >= _rx_stream.start_time_ns) {
                    break;
                }
                const long long remaining_ns = _rx_stream.start_time_ns - now_ns;
                const long sleep_us = static_cast<long>(std::max<long long>(50, std::min<long long>(remaining_ns / 1000, 1000)));
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            }
        }

#if USE_LITEPCIE
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        {
            struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
            mmap_dma_update.sw_count = _rx_stream.hw_count;
            checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
        }
#elif USE_VFIO
        _rx_stream.hw_count = m2sdr_vfio_rx_hw_count(_vfio);
        m2sdr_vfio_rx_sw_set(_vfio, _rx_stream.hw_count);
#endif
        _rx_stream.sw_count = _rx_stream.hw_count;
        _rx_stream.enqueue_count = _rx_stream.hw_count;
        _rx_stream.user_count = _rx_stream.hw_count;
        /* RC1: capture both time bases atomically under the lock so the consumer
         * never reads a partially-written pair from acquireReadBuffer pacing. */
        {
            const long long hw_time = _rx_stream.timed_start_pending
                ? _rx_stream.start_time_ns
                : this->getHardwareTime("");
            const auto steady_snap = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
            _rx_stream.time0_ns         = hw_time;
            _rx_stream.time0_steady     = steady_snap;
            _rx_stream.time0_count      = _rx_stream.hw_count;
            _rx_stream.time_base_locked = !_rx_stream.timed_start_pending;
            _rx_stream.last_time_ns     = hw_time;
            _rx_stream.timed_start_pending = false;
        }

        /* RC2: work items collected during the (short) metadata lock; deinterleave
         * runs outside the lock so the consumer is never blocked waiting for it. */
        struct RxWorkItem {
            const uint8_t *src_base;   /* pointer into DMA ring (valid until sw_count advances) */
            size_t          free_handle;
        };
        std::vector<RxWorkItem> rx_work_items;
        rx_work_items.reserve(16);

        for (;;) {
            {
                std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
                if (_rx_stream.recv_thread_stop) {
                    _rx_stream.recv_thread_running = false;
                    _rx_stream.recv_cv.notify_all();
                    return;
                }
            }

            const auto poll_start = std::chrono::steady_clock::now();
#if USE_LITEPCIE
            /* RC3: non-blocking poll so maximum IRQ latency is the idle-sleep
             * below (~50 µs), not the old 1 ms timeout.  At 30.72 MSPS one
             * DMA buffer arrives every ~67 µs, so a 1 ms timeout could let
             * ~15 buffers accumulate and be delivered as one burst. */
            const int poll_ret = poll(&_rx_stream.fds, 1, 0);
            const auto poll_end = std::chrono::steady_clock::now();
            if (poll_ret < 0) {
                if (errno == EINTR)
                    continue;
                throw std::runtime_error("RX receive worker poll failed, " + std::string(strerror(errno)) + ".");
            }
            const auto dma_start = std::chrono::steady_clock::now();
            litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
            const auto dma_end = std::chrono::steady_clock::now();
#elif USE_VFIO
            (void)m2sdr_vfio_rx_wait(_vfio, &_rx_stream.hw_count, _rx_stream.sw_count, 1000);
            const auto poll_end = std::chrono::steady_clock::now();
            const auto dma_start = poll_start;
            const auto dma_end = poll_end;
#endif

            rx_work_items.clear();
            int64_t newly_completed = 0;
            int64_t last_processed_handle = -1;
            bool has_pending = false;

            /* ── Metadata phase: short lock (no deinterleave here) ────────── */
            {
                std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
#if USE_LITEPCIE
                const uint32_t writer_loop_status =
                    litex_m2sdr_readl(_fd, CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_ADDR);
                const uint32_t writer_level =
                    litex_m2sdr_readl(_fd, CSR_PCIE_DMA0_WRITER_TABLE_LEVEL_ADDR);
                const int64_t writer_level_count = static_cast<int64_t>(writer_level & 0xffff);
                const int64_t hw_lag_count = _rx_stream.hw_count - _rx_stream.sw_count;
                const int64_t overflow_count =
                    (writer_level_count > 0) ? writer_level_count : hw_lag_count;
                if (overflow_count >
                    ((int64_t)_dma_mmap_info.dma_rx_buf_count / 2)) {
                    const int64_t lost_buffers = hw_lag_count;
#ifdef CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_LEVEL_ADDR
                    const uint32_t writer_fifo_level =
                        litex_m2sdr_readl(_fd, CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_LEVEL_ADDR);
#else
                    const uint32_t writer_fifo_level = 0xffffffffu;
#endif
                    SoapySDR_logf(SOAPY_SDR_WARNING,
                                  "RX overflow reset: hw=%lld sw=%lld user=%lld enq=%lld ready=%zu pending=%zu lost=%lld level_count=%lld loop_status=0x%08x level=0x%08x fifo_level=%u",
                                  (long long)_rx_stream.hw_count,
                                  (long long)_rx_stream.sw_count,
                                  (long long)_rx_stream.user_count,
                                  (long long)_rx_stream.enqueue_count,
                                  _rx_stream.ready_buffs.size(),
                                  _rx_stream.pending_dma_handles.size(),
                                  (long long)lost_buffers,
                                  (long long)overflow_count,
                                  writer_loop_status,
                                  writer_level,
                                  writer_fifo_level);
                    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
                    mmap_dma_update.sw_count = _rx_stream.hw_count;
                    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);

                    _rx_stream.ready_buffs.reset(_rx_stream.batch_buf_count);
                    _rx_stream.pending_dma_handles.clear();
                    _rx_stream.free_batch_handles.reset(_rx_stream.batch_buf_count);
                    for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
                        (void)_rx_stream.free_batch_handles.push(i);
                    _rx_stream.enqueue_count = _rx_stream.hw_count;
                    _rx_stream.user_count = _rx_stream.hw_count;
                    _rx_stream.sw_count = _rx_stream.hw_count;
                    _rx_stream.overflow_lost_buffers = lost_buffers;
                    _rx_stream.overflow = true;
                    _rx_stream.time0_ns = this->getHardwareTime("");
                    _rx_stream.time0_steady = std::chrono::steady_clock::now();
                    _rx_stream.time0_count = _rx_stream.user_count;
                    _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
                    _rx_stream.last_time_ns = _rx_stream.time0_ns;
                    _rx_stream.time_warned = false;
                    _rx_stream.recv_cv.notify_all();
                    continue;
                }
#else
                const int64_t overflow_count = _rx_stream.hw_count - _rx_stream.sw_count;
                if (overflow_count > ((int64_t)_dma_mmap_info.dma_rx_buf_count / 2)) {
                    const int64_t lost_buffers = overflow_count;
                    _rx_stream.ready_buffs.reset(_rx_stream.batch_buf_count);
                    _rx_stream.pending_dma_handles.clear();
                    _rx_stream.free_batch_handles.reset(_rx_stream.batch_buf_count);
                    for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
                        (void)_rx_stream.free_batch_handles.push(i);
                    _rx_stream.enqueue_count = _rx_stream.hw_count;
                    _rx_stream.user_count = _rx_stream.hw_count;
                    _rx_stream.sw_count = _rx_stream.hw_count;
                    _rx_stream.overflow_lost_buffers = lost_buffers;
                    _rx_stream.overflow = true;
                    _rx_stream.time0_ns = this->getHardwareTime("");
                    _rx_stream.time0_steady = std::chrono::steady_clock::now();
                    _rx_stream.time0_count = _rx_stream.user_count;
                    _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
                    _rx_stream.last_time_ns = _rx_stream.time0_ns;
                    _rx_stream.time_warned = false;
                    m2sdr_vfio_rx_sw_set(_vfio, _rx_stream.hw_count);
                    _rx_stream.recv_cv.notify_all();
                    continue;
                }
#endif

                newly_completed = _rx_stream.hw_count - _rx_stream.enqueue_count;
                while (_rx_stream.enqueue_count < _rx_stream.hw_count) {
                    _rx_stream.pending_dma_handles.push_back(_rx_stream.enqueue_count);
                    _rx_stream.enqueue_count++;
                }

                const size_t batch_buffers = 1;
                const size_t packet_elems = _rx_stream.packet_elems;
                const size_t packets_per_dma = _rx_stream.packets_per_dma;
                const size_t dma_payload_elems = _rx_stream.dma_payload_elems;
                const size_t packet_words = _rx_stream.packet_words;
                const size_t packet_bytes = packet_words * sizeof(uint64_t);
                while (_rx_stream.pending_dma_handles.size() >= batch_buffers &&
                       _rx_stream.free_batch_handles.size() >= packets_per_dma) {
                    const int64_t first_dma_handle = _rx_stream.pending_dma_handles.front();
                    _rx_stream.pending_dma_handles.pop_front();

                    const int first_offset = first_dma_handle % _dma_mmap_info.dma_rx_buf_count;
                    const uint8_t *dma_base = reinterpret_cast<const uint8_t *>(_rx_stream.buf) +
                                              first_offset * _dma_mmap_info.dma_rx_buf_size;

                    auto packet_expected_time = [&](size_t packet_index, long long &expected_time_ns) {
                        if (!_rx_stream.time_valid || !_rx_stream.time_base_locked)
                            return false;
                        const int64_t packet_sample_index =
                            static_cast<int64_t>(first_dma_handle - _rx_stream.time0_count) *
                                static_cast<int64_t>(dma_payload_elems) +
                            static_cast<int64_t>(packet_index * packet_elems);
                        const double ns = (static_cast<double>(packet_sample_index) * 1e9) / _rx_stream.samplerate;
                        expected_time_ns = _rx_stream.time0_ns + static_cast<long long>(std::llround(ns));
                        return true;
                    };

                    auto parse_packet_time = [&](size_t packet_index, int64_t trace_dma_handle,
                                                 int &ready_flags, long long &ready_time_ns,
                                                 long long &header_time_ns, bool &header_valid) {
                        ready_flags = 0;
                        ready_time_ns = 0;
                        header_time_ns = 0;
                        header_valid = false;
                        long long expected_time_ns = 0;
                        const bool have_expected_time = packet_expected_time(packet_index, expected_time_ns);
                        const uint8_t *header_ptr = dma_base + packet_index * packet_bytes;
                        const uint64_t header = *reinterpret_cast<const uint64_t *>(header_ptr);
                        if (header == DMA_HEADER_SYNC_WORD) {
                            header_time_ns =
                                static_cast<long long>(*reinterpret_cast<const uint64_t *>(header_ptr + 8));
                            bool header_ok = true;
                            if (have_expected_time) {
                                const long long delta_ns = std::llabs(header_time_ns - expected_time_ns);
                                const long long max_delta_ns = std::max<long long>(
                                    1000000LL,
                                    static_cast<long long>(std::llround(
                                        (static_cast<double>(packet_elems * 4) * 1e9) / _rx_stream.samplerate)));
                                if (delta_ns > max_delta_ns) {
                                    header_ok = false;
                                    if (rx_ts_trace_allowed()) {
                                        std::fprintf(stderr,
                                            "RXTS reject dma_first=%lld pkt=%zu header_time=%lld expected_time=%lld delta_ns=%lld max_delta_ns=%lld\n",
                                            (long long)trace_dma_handle,
                                            packet_index,
                                            header_time_ns,
                                            expected_time_ns,
                                            delta_ns,
                                            max_delta_ns);
                                    }
                                }
                            }
                            if (header_ok) {
                                ready_time_ns = header_time_ns;
                                ready_flags |= SOAPY_SDR_HAS_TIME;
                                header_valid = true;
                            }
                        }
                        if (!(ready_flags & SOAPY_SDR_HAS_TIME) && have_expected_time) {
                            ready_time_ns = expected_time_ns;
                            ready_flags |= SOAPY_SDR_HAS_TIME;
                        }
                    };

                    int first_ready_flags = 0;
                    long long first_ready_time_ns = 0;
                    long long first_header_time_ns = 0;
                    bool first_header_valid = false;
                    parse_packet_time(0,
                                      first_dma_handle,
                                      first_ready_flags,
                                      first_ready_time_ns,
                                      first_header_time_ns,
                                      first_header_valid);

                    if (!_rx_stream.time_base_locked) {
                        if (!first_header_valid) {
                            if (rx_ts_trace_allowed()) {
                                std::fprintf(stderr,
                                    "RXTS unlock-drop dma_first=%lld reason=no_valid_header\n",
                                    (long long)first_dma_handle);
                            }
                            /* Mark consumed so sw_count advance fires after the lock. */
                            last_processed_handle = std::max(last_processed_handle, first_dma_handle);
                            continue;
                        }
                        _rx_stream.time0_count = first_dma_handle;
                        _rx_stream.time0_ns = first_ready_time_ns;
                        _rx_stream.last_time_ns = first_ready_time_ns;
                        _rx_stream.time_base_locked = true;
                    }

                    for (size_t packet_index = 0; packet_index < packets_per_dma; ++packet_index) {
                        int ready_flags = 0;
                        long long ready_time_ns = 0;
                        long long header_time_ns = 0;
                        bool header_valid = false;
                        parse_packet_time(packet_index,
                                          first_dma_handle,
                                          ready_flags,
                                          ready_time_ns,
                                          header_time_ns,
                                          header_valid);
                        const long long packet_sample_index =
                            static_cast<int64_t>(first_dma_handle - _rx_stream.time0_count) *
                                static_cast<int64_t>(dma_payload_elems) +
                            static_cast<int64_t>(packet_index * packet_elems);
                        if (header_valid && _rx_stream.time_valid && _rx_stream.time_base_locked) {
                            const long long expected_time_ns =
                                _rx_stream.time0_ns + sample_count_to_ns(_rx_stream.samplerate, packet_sample_index);
                            const long long sample_period_ns =
                                std::max<long long>(1, sample_count_to_ns(_rx_stream.samplerate, 1));
                            const long long packet_period_ns =
                                std::max<long long>(sample_period_ns,
                                                    sample_count_to_ns(_rx_stream.samplerate,
                                                                       static_cast<long long>(packet_elems)));
                            const long long resync_threshold_ns =
                                std::max<long long>(sample_period_ns * 8, packet_period_ns / 2);
                            const long long delta_ns = header_time_ns - expected_time_ns;
                            if (std::llabs(delta_ns) > resync_threshold_ns) {
                                _rx_stream.time0_count = first_dma_handle;
                                _rx_stream.time0_ns =
                                    header_time_ns - sample_count_to_ns(
                                        _rx_stream.samplerate,
                                        static_cast<long long>(packet_index * packet_elems));
                                _rx_stream.last_time_ns = header_time_ns;
                                ready_time_ns = header_time_ns;
                                _rx_stream.continuity_resyncs++;
                                if (rx_ts_trace_allowed()) {
                                    std::fprintf(stderr,
                                        "RXTS resync dma_first=%lld pkt=%zu header_time=%lld expected_time=%lld delta_ns=%lld threshold_ns=%lld sample_index=%lld resyncs=%llu\n",
                                        (long long)first_dma_handle,
                                        packet_index,
                                        header_time_ns,
                                        expected_time_ns,
                                        delta_ns,
                                        resync_threshold_ns,
                                        packet_sample_index,
                                        (unsigned long long)_rx_stream.continuity_resyncs);
                                }
                            }
                        }
                        const uint8_t *src_base = dma_base + packet_index * packet_bytes + RX_DMA_HEADER_SIZE;
                        size_t free_handle = 0;
                        if (!_rx_stream.free_batch_handles.pop(free_handle))
                            break;
                        auto &packet = _rx_stream.packet_pool.at(free_handle);
                        packet.flags = ready_flags;
                        packet.numElems = packet_elems;
                        packet.timeNs = ready_time_ns;
                        packet.sampleIndex = packet_sample_index;
                        if (rx_ts_trace_allowed()) {
                            std::fprintf(stderr,
                                "RXTS stage dma_first=%lld pkt=%zu pkt_time=%lld handle=%lld num=%zu flags=0x%x\n",
                                (long long)first_dma_handle,
                                packet_index,
                                (long long)ready_time_ns,
                                (long long)free_handle,
                                packet.numElems,
                                packet.flags);
                        }
                        /* RC2: save (src_base, handle) for deinterleave outside the lock.
                         * The DMA buffer stays valid until sw_count is advanced below. */
                        rx_work_items.push_back({src_base, free_handle});
                    }
                    /* Track highest DMA handle processed so we can issue a single
                     * sw_count ioctl after all deinterleaves complete. */
                    last_processed_handle = std::max(last_processed_handle, first_dma_handle);
                }

                has_pending = !_rx_stream.pending_dma_handles.empty();

                if (_rx_stream.rx_debug) {
                    const auto poll_us = std::chrono::duration_cast<std::chrono::microseconds>(poll_end - poll_start).count();
                    const auto dma_us  = std::chrono::duration_cast<std::chrono::microseconds>(dma_end - dma_start).count();
                    const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(dma_end - poll_start).count();
                    if ((total_us >= _rx_stream.rx_debug_threshold_us || newly_completed > 1) &&
                        debug_log_allowed(_rx_stream.rx_debug, _rx_stream.rx_debug_seq, _rx_stream.rx_debug_limit)) {
                        std::fprintf(stderr,
                            "RXDBG worker poll_us=%lld dma_us=%lld total_us=%lld hw=%lld sw=%lld enq=%lld new=%lld q=%zu\n",
                            (long long)poll_us,
                            (long long)dma_us,
                            (long long)total_us,
                            (long long)_rx_stream.hw_count,
                            (long long)_rx_stream.sw_count,
                            (long long)_rx_stream.enqueue_count,
                            (long long)newly_completed,
                            _rx_stream.ready_buffs.size());
                    }
                }
            } /* end metadata lock */

            /* ── Deinterleave phase (no lock held — RC2) ─────────────────── */
            for (auto &item : rx_work_items) {
                auto &packet = _rx_stream.packet_pool.at(item.free_handle);
                uint8_t *dst = packet.data.data();
                for (size_t chan_index = 0; chan_index < _rx_stream.channels.size(); ++chan_index) {
                    const size_t chan = _rx_stream.channels[chan_index];
                    uint8_t *chan_dst = dst +
                        chan_index * _rx_stream.packet_elems * _rx_stream.soapy_bytes_per_complex;
                    this->deinterleave(
                        item.src_base + chan * _bytesPerComplex,
                        chan_dst,
                        static_cast<uint32_t>(packet.numElems),
                        _rx_stream.format,
                        0);
                }
                /* Push to ready queue now that packet data is fully written.
                 * ready_buffs is a lock-free SPSC — no mutex needed. */
                if (!_rx_stream.ready_buffs.push(item.free_handle)) {
                    /* Queue unexpectedly full; return handle to avoid leak. */
                    if (!_rx_stream.free_batch_handles.push(item.free_handle))
                        throw std::runtime_error("RX ready/free queue state inconsistent.");
                }
            }

            /* ── sw_count advance + consumer notification ─────────────────── */
            /* Advance sw_count only AFTER deinterleave so the hardware cannot
             * overwrite the DMA buffers while we are still reading them. */
            if (last_processed_handle >= 0) {
                const int64_t new_sw = last_processed_handle + 1;
#if USE_LITEPCIE
                if (new_sw > _rx_stream.sw_count) {
                    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
                    mmap_dma_update.sw_count = new_sw;
                    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
                }
#elif USE_VFIO
                if (new_sw > _rx_stream.sw_count)
                    m2sdr_vfio_rx_sw_set(_vfio, new_sw);
#endif
                /* Update sw_count and notify under a brief lock.  Holding the
                 * mutex here prevents a missed wakeup: if the consumer checks
                 * ready_buffs.empty() between our push and our notify_all, it
                 * will block in cv.wait() — but only after releasing the mutex,
                 * which cannot happen until we have released it here. */
                {
                    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
                    if (new_sw > _rx_stream.sw_count)
                        _rx_stream.sw_count = new_sw;
                }
                _rx_stream.recv_cv.notify_all();
            }

            /* ── RC3: avoid busy-spin when hardware has no new data ────────── */
            if (newly_completed == 0 && !has_pending) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        if (_rx_stream.recv_error.empty())
            _rx_stream.recv_error = std::string("RX receive worker failed: ") + e.what();
        _rx_stream.recv_thread_running = false;
        _rx_stream.recv_cv.notify_all();
    }
#endif
}

void SoapyLiteXM2SDR::startRxReceiveWorker()
{
#if USE_LITEPCIE || USE_VFIO
    stopRxReceiveWorker();
    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
    _rx_stream.ready_buffs.clear();
    _rx_stream.pending_dma_handles.clear();
    _rx_stream.recv_error.clear();
    _rx_stream.recv_thread_stop = false;
    _rx_stream.recv_thread_running = true;
    _rx_stream.enqueue_count = 0;
    _rx_stream.overflow_lost_buffers = 0;
    _rx_stream.recv_thread = std::thread(&SoapyLiteXM2SDR::rxReceiveLoop, this);
    configure_worker_thread(_rx_stream.recv_thread, "m2sdr-rx", _rx_stream.worker_rt_prio, _rx_stream.worker_cpu);
#endif
}

void SoapyLiteXM2SDR::stopRxReceiveWorker()
{
#if USE_LITEPCIE || USE_VFIO
    {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        _rx_stream.recv_thread_stop = true;
    }
    _rx_stream.recv_cv.notify_all();
    if (_rx_stream.recv_thread.joinable())
        _rx_stream.recv_thread.join();
    {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        _rx_stream.ready_buffs.reset(_rx_stream.batch_buf_count);
        _rx_stream.pending_dma_handles.clear();
        _rx_stream.free_batch_handles.reset(_rx_stream.batch_buf_count);
        for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
            (void)_rx_stream.free_batch_handles.push(i);
        _rx_stream.recv_thread_running = false;
    }
#endif
}

void SoapyLiteXM2SDR::checkTxWriteError()
{
#if USE_LITEPCIE || USE_VFIO
    std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
    if (!_tx_stream.write_error.empty()) {
        const std::string err = _tx_stream.write_error;
        _tx_stream.write_error.clear();
        throw std::runtime_error(err);
    }
#endif
}

void SoapyLiteXM2SDR::txWriteLoop()
{
#if USE_LITEPCIE || USE_VFIO
    if (_tx_stream.tx_queue_debug &&
        debug_log_allowed(_tx_stream.tx_queue_debug,
                          _tx_stream.tx_queue_debug_seq,
                          _tx_stream.tx_queue_debug_limit)) {
        std::fprintf(stderr, "TXQDBG worker-start q=%zu hw=%lld sw=%lld user=%lld\n",
            _tx_stream.pending_write_jobs.size(),
            (long long)_tx_stream.hw_count,
            (long long)_tx_stream.sw_count,
            (long long)_tx_stream.user_count);
    }
    for (;;) {
        TXStream::WriteJob job;
        {
            std::unique_lock<std::mutex> lock(_tx_stream.write_mutex);
            _tx_stream.write_cv.wait(lock, [this]() {
                return _tx_stream.write_thread_stop || !_tx_stream.pending_write_jobs.empty();
            });
            if (_tx_stream.write_thread_stop && _tx_stream.pending_write_jobs.empty()) {
                _tx_stream.write_thread_running = false;
                _tx_stream.write_cv.notify_all();
                return;
            }
            if (_tx_stream.tx_queue_debug &&
                debug_log_allowed(_tx_stream.tx_queue_debug,
                                  _tx_stream.tx_queue_debug_seq,
                                  _tx_stream.tx_queue_debug_limit)) {
                std::fprintf(stderr, "TXQDBG worker-wake q=%zu stop=%d hw=%lld sw=%lld user=%lld\n",
                    _tx_stream.pending_write_jobs.size(),
                    _tx_stream.write_thread_stop ? 1 : 0,
                    (long long)_tx_stream.hw_count,
                    (long long)_tx_stream.sw_count,
                    (long long)_tx_stream.user_count);
            }
            job = std::move(_tx_stream.pending_write_jobs.front());
            _tx_stream.pending_write_jobs.pop_front();
            _tx_stream.write_cv.notify_all();
        }

        try {
            const auto dequeue_tp = std::chrono::steady_clock::now();
            const long queued_us =
                std::chrono::duration_cast<std::chrono::microseconds>(dequeue_tp - job.enqueue_tp).count();
            if (_tx_stream.tx_queue_debug) {
                if (queued_us >= _tx_stream.tx_queue_debug_threshold_us &&
                    debug_log_allowed(_tx_stream.tx_queue_debug,
                                      _tx_stream.tx_queue_debug_seq,
                                      _tx_stream.tx_queue_debug_limit)) {
                    std::fprintf(stderr,
                        "TXQDBG dequeue queued_us=%ld numElems=%zu flags=0x%x q_after=%zu hw=%lld sw=%lld user=%lld\n",
                        queued_us,
                        job.numElems,
                        job.flags,
                        _tx_stream.pending_write_jobs.size(),
                        (long long)_tx_stream.hw_count,
                        (long long)_tx_stream.sw_count,
                        (long long)_tx_stream.user_count);
                }
            }
            const size_t mtu = this->getStreamMTU(TX_STREAM);
            if (job.numElems == 0) {
                size_t handle = 0;
                void *wr_buffs[4] = {};
                const int ret = this->acquireWriteBuffer(TX_STREAM, handle, wr_buffs, 1000);
                if (ret < 0) {
                    throw std::runtime_error("TX write worker zero-length acquireWriteBuffer failed: " + std::to_string(ret));
                }
                int eob_flags = job.flags;
                this->releaseWriteBuffer(TX_STREAM, handle, 0, eob_flags, job.timeNs);
                continue;
            }
            if (job.numElems > mtu) {
                throw std::runtime_error("TX staged packet exceeds MTU");
            }

            size_t handle = 0;
            void *wr_buffs[4] = {};
            const auto acq_t0 = std::chrono::steady_clock::now();
            const int ret = this->acquireWriteBuffer(TX_STREAM, handle, wr_buffs, 1000);
            const long acq_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - acq_t0).count();
            if (ret == SOAPY_SDR_TIMEOUT)
                continue;
            if (ret < 0) {
                throw std::runtime_error("TX write worker acquireWriteBuffer failed: " + std::to_string(ret));
            }

            std::memset(wr_buffs[0], 0, mtu * _nChannels * _bytesPerComplex);
            for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
                const uint8_t *src = job.data.data() + i * job.numElems * _tx_stream.soapy_bytes_per_complex;
                this->interleave(
                    src,
                    reinterpret_cast<uint8_t *>(wr_buffs[0]) + _tx_stream.channels[i] * _bytesPerComplex,
                    static_cast<uint32_t>(job.numElems),
                    _tx_stream.format,
                    0);
            }

            if (_tx_stream.tx_queue_debug &&
                debug_log_allowed(_tx_stream.tx_queue_debug,
                                  _tx_stream.tx_queue_debug_seq,
                                  _tx_stream.tx_queue_debug_limit)) {
                const long long now_ns = steady_now_ns();
                long long hw_now_ns = 0;
                long long lead_us = 0;
                bool hw_ok = false;
                try {
                    hw_now_ns = this->getHardwareTime("");
                    lead_us = (job.timeNs - hw_now_ns) / 1000;
                    hw_ok = true;
                } catch (...) {
                }
                std::fprintf(stderr,
                    "TXQDBG submit mono_ns=%lld queued_us=%ld acq_us=%ld handle=%zu numElems=%zu "
                    "flags=0x%x lead_us=%lld hw_now_ns=%lld ts_ns=%lld q=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                    now_ns,
                    queued_us,
                    acq_us,
                    handle,
                    job.numElems,
                    job.flags,
                    hw_ok ? lead_us : -1LL,
                    hw_ok ? hw_now_ns : -1LL,
                    job.timeNs,
                    _tx_stream.pending_write_jobs.size(),
                    (long long)_tx_stream.hw_count,
                    (long long)_tx_stream.sw_count,
                    (long long)_tx_stream.user_count,
                    (long long)(_tx_stream.user_count - _tx_stream.hw_count));
            }
            int packet_flags = job.flags;
            this->releaseWriteBuffer(TX_STREAM, handle, job.numElems, packet_flags, job.timeNs);
        } catch (const std::exception &e) {
            std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
            if (_tx_stream.write_error.empty())
                _tx_stream.write_error = std::string("TX write worker failed: ") + e.what();
            _tx_stream.write_thread_running = false;
            _tx_stream.write_cv.notify_all();
            return;
        }
    }
#endif
}

void SoapyLiteXM2SDR::startTxWriteWorker()
{
#if USE_LITEPCIE || USE_VFIO
    stopTxWriteWorker();
    std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
    _tx_stream.pending_write_jobs.clear();
    _tx_stream.write_error.clear();
    _tx_stream.write_thread_stop = false;
    _tx_stream.write_thread_running = true;
    if (_tx_stream.tx_queue_debug &&
        debug_log_allowed(_tx_stream.tx_queue_debug,
                          _tx_stream.tx_queue_debug_seq,
                          _tx_stream.tx_queue_debug_limit)) {
        std::fprintf(stderr, "TXQDBG write-worker-start q=%zu hw=%lld sw=%lld user=%lld\n",
            _tx_stream.pending_write_jobs.size(),
            (long long)_tx_stream.hw_count,
            (long long)_tx_stream.sw_count,
            (long long)_tx_stream.user_count);
    }
    _tx_stream.write_thread = std::thread(&SoapyLiteXM2SDR::txWriteLoop, this);
    configure_worker_thread(_tx_stream.write_thread, "m2sdr-txw", _tx_stream.worker_rt_prio, _tx_stream.worker_cpu);
#endif
}

void SoapyLiteXM2SDR::stopTxWriteWorker()
{
#if USE_LITEPCIE || USE_VFIO
    {
        std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
        _tx_stream.write_thread_stop = true;
    }
    _tx_stream.write_cv.notify_all();
    if (_tx_stream.write_thread.joinable())
        _tx_stream.write_thread.join();
    {
        std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
        _tx_stream.pending_write_jobs.clear();
        _tx_stream.write_thread_running = false;
    }
#endif
}

/* Setup and configure a stream for RX or TX. */
SoapySDR::Stream *SoapyLiteXM2SDR::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &args) {
    std::lock_guard<std::mutex> lock(_mutex);

    SoapySDR::Kwargs searchArgs = args;
    if (searchArgs.empty())
        searchArgs = _deviceArgs;

    /* Variable to hold the selected channels */
    std::vector<size_t> selected_channels;

    if (direction == SOAPY_SDR_RX) {
        if (_rx_stream.opened) {
            throw std::runtime_error("RX stream already opened.");
        }

        /* Determine channels: prioritize searchArgs over the provided vector */
        auto it = searchArgs.find("channels");
        if (it != searchArgs.end()) {
            selected_channels = parse_channel_list(it->second);
        } else if (!channels.empty()) {
            /* Use the provided channels vector if no device argument overrides it */
            selected_channels = channels;
        } else {
            /* Default to channel 0 if nothing provided */
            selected_channels = {0};
        }

        /* Validate selected channels */
        if (selected_channels.size() > 2) {
            throw std::runtime_error("Invalid RX channel count: must be 1 or 2 channels");
        }
        for (size_t chan : selected_channels) {
            if (chan > 1) {
                throw std::runtime_error("Invalid RX channel index: must be 0 (RX1) or 1 (RX2)");
            }
        }
        if (selected_channels.size() == 2 && (selected_channels[0] != 0 || selected_channels[1] != 1)) {
            throw std::runtime_error("Dual RX channels must be {0, 1} for RX1+RX2");
        }
        if (_tx_stream.opened && selected_channels.size() != _tx_stream.channels.size()) {
            throw std::runtime_error("RX/TX channel count mismatch; close TX or use matching channels");
        }

        /* Configure the file descriptor watcher. */
#if USE_LITEPCIE
        _rx_stream.fds.fd     = _fd;
        _rx_stream.dma.fds.fd = _fd;
#endif
        _rx_stream.fds.events = POLLIN;

#if USE_LITEPCIE
        /* Initialize RX DMA Writer */
        _rx_stream.dma.shared_fd  = 1;
        _rx_stream.dma.use_reader = 0;
        _rx_stream.dma.use_writer = 1;
        _rx_stream.dma.loopback   = 0;
        _rx_stream.dma.zero_copy  = 1;
        if (litepcie_dma_init(&_rx_stream.dma, "", _rx_stream.dma.zero_copy) < 0)
            throw std::runtime_error("DMA Writer/RX not available (litepcie_dma_init failed)." + std::string(strerror(errno)));

        /* Get Buffer and Parameters from RX DMA Writer */
        _rx_stream.buf = _rx_stream.dma.buf_rd;
        _rx_buf_size   = _rx_stream.dma.mmap_dma_info.dma_rx_buf_size - RX_DMA_HEADER_SIZE;
        _rx_buf_count  = _rx_stream.dma.mmap_dma_info.dma_rx_buf_count;

        /* Ensure the DMA is disabled initially to avoid counters being in a bad state. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
#elif USE_VFIO
        if (!_vfio)
            throw std::runtime_error("DMA Writer/RX: VFIO device not open.");
        if (m2sdr_vfio_rx_start(_vfio) < 0)
            throw std::runtime_error("DMA Writer/RX not available (m2sdr_vfio_rx_start failed).");
        _rx_stream.buf  = _vfio->rx_virt;
        _rx_buf_size    = M2SDR_DMA_BUFFER_SIZE - RX_DMA_HEADER_SIZE;
        _rx_buf_count   = M2SDR_DMA_BUFFER_COUNT;
        _rx_stream.hw_count = 0;
        _rx_stream.sw_count = 0;
#endif

        _rx_stream.opened = true;
        _rx_stream.format = format;
        _rx_stream.soapy_bytes_per_complex = getSoapyBytesPerComplex(format);
        _rx_stream.active_handle = -1;
        _rx_stream.active_num_elems = 0;
        _rx_stream.active_offset = 0;
        _rx_stream.active_buff = nullptr;
        _rx_stream.active_time_ns = 0;

        /* Log the selected RX channels for debugging */
        if (selected_channels.size() == 1) {
            SoapySDR_logf(SOAPY_SDR_INFO, "RX setupStream: Selected channel %zu", selected_channels[0]);
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "RX setupStream: Selected channels %zu, %zu",
                          selected_channels[0], selected_channels[1]);
        }

        _rx_stream.channels = selected_channels;
        _nChannels = _rx_stream.channels.size();
#if USE_LITEPCIE || USE_VFIO
        {
            const size_t dma_words = _dma_mmap_info.dma_rx_buf_size / sizeof(uint64_t);
            const size_t bytes_per_elem = _nChannels * _bytesPerComplex;
            size_t packet_words = std::max<size_t>(3, _rx_stream.packet_words);
            if (packet_words > dma_words)
                packet_words = dma_words;
            if ((dma_words % packet_words) != 0) {
                throw std::runtime_error(
                    "RX packet_words=" + std::to_string(packet_words) +
                    " must evenly divide the DMA size (" + std::to_string(dma_words) + " 64-bit words).");
            }
            const size_t frame_cycles = packet_words - 2;
            const size_t packet_payload_bytes = frame_cycles * sizeof(uint64_t);
            if ((packet_payload_bytes % bytes_per_elem) != 0) {
                throw std::runtime_error(
                    "RX packet payload size does not align with the current channel/sample layout.");
            }
            const size_t packet_elems = packet_payload_bytes / bytes_per_elem;
            const size_t packets_per_dma = dma_words / packet_words;
            const size_t dma_payload_elems = packet_elems * packets_per_dma;
            _rx_stream.packet_words = packet_words;
            _rx_stream.frame_cycles = frame_cycles;
            _rx_stream.packet_elems = packet_elems;
            _rx_stream.packets_per_dma = packets_per_dma;
            _rx_stream.dma_payload_elems = dma_payload_elems;
#if USE_LITEPCIE
            litex_m2sdr_writel(_fd, CSR_HEADER_RX_FRAME_CYCLES_ADDR, static_cast<uint32_t>(frame_cycles));
#endif

            size_t batch_buffers = 1;
            if (_rx_stream.batch_buffers_explicit && _rx_stream.batch_buffers > 1) {
                SoapySDR::logf(SOAPY_SDR_WARNING,
                    "RX packetized mode uses one DMA buffer per worker batch; ignoring rx_batch_buffers=%zu",
                    _rx_stream.batch_buffers);
            }
            _rx_stream.batch_buffers = batch_buffers;
            const size_t packet_stride = packet_elems * _nChannels * _rx_stream.soapy_bytes_per_complex;
            // Size the staged queue from the requested latency budget rather than mirroring the full DMA ring.
            // Mirroring the ring creates tens of milliseconds of elasticity, which lets the consumer fall behind
            // and then catch up in bursts. Keep enough packets for roughly one DMA buffer by default, but avoid a
            // larger fixed floor that would reintroduce bursty catch-up when the logical hardware packets are small.
            size_t batch_count = std::max<size_t>(8, packets_per_dma);
            if (_rx_stream.samplerate > 0.0) {
                const double batch_ms = (1000.0 * static_cast<double>(packet_elems)) /
                                        _rx_stream.samplerate;
                if (batch_ms > 0.0) {
                    const size_t queue_batches = static_cast<size_t>(
                        std::ceil(_rx_stream.batch_queue_ms / batch_ms));
                    batch_count = std::max(batch_count, std::max<size_t>(8, queue_batches));
                }
            }
            _rx_stream.slice_elems = packet_elems;
            _rx_stream.batch_buf_size = packet_stride;
            _rx_stream.batch_buf_count = batch_count;
            _rx_stream.packet_pool.clear();
            _rx_stream.packet_pool.resize(batch_count);
            for (size_t i = 0; i < batch_count; ++i) {
                _rx_stream.packet_pool[i].data.resize(packet_stride);
                _rx_stream.packet_pool[i].flags = 0;
                _rx_stream.packet_pool[i].timeNs = 0;
                _rx_stream.packet_pool[i].sampleIndex = 0;
                _rx_stream.packet_pool[i].numElems = 0;
            }
            _rx_stream.ready_buffs.reset(batch_count);
            _rx_stream.free_batch_handles.reset(batch_count);
            for (size_t i = 0; i < batch_count; ++i)
                (void)_rx_stream.free_batch_handles.push(i);
            SoapySDR_logf(SOAPY_SDR_INFO,
                          "RX staging packets=%zu packet_stride=%zu bytes packet_elems=%zu packet_words=%zu packets_per_dma=%zu (~%.1f ms total)",
                          batch_count,
                          packet_stride,
                          packet_elems,
                          packet_words,
                          packets_per_dma,
                          (_rx_stream.samplerate > 0.0)
                              ? (1000.0 * (static_cast<double>(batch_count * packet_elems)) /
                                 _rx_stream.samplerate)
                              : 0.0);
        }
#endif
    } else if (direction == SOAPY_SDR_TX) {
        if (_tx_stream.opened) {
            throw std::runtime_error("TX stream already opened.");
        }

        /* Determine channels: override provided vector if searchArgs contains "channels" */
        auto it = searchArgs.find("channels");
        if (it != searchArgs.end()) {
            selected_channels = parse_channel_list(it->second);
        } else if (!channels.empty()) {
            /* Use the provided channels vector if no override is present */
            selected_channels = channels;
        } else {
            /* Default to TX1 if nothing is specified */
            selected_channels = {0};
        }

        /* Validate selected channels */
        if (selected_channels.size() > 2) {
            throw std::runtime_error("Invalid TX channel count: must be 1 or 2 channels");
        }
        for (size_t chan : selected_channels) {
            if (chan > 1) {
                throw std::runtime_error("Invalid TX channel index: must be 0 (TX1) or 1 (TX2)");
            }
        }
        if (selected_channels.size() == 2 && (selected_channels[0] != 0 || selected_channels[1] != 1)) {
            throw std::runtime_error("Dual TX channels must be {0, 1} for TX1+TX2");
        }
        if (_rx_stream.opened && selected_channels.size() != _rx_stream.channels.size()) {
            throw std::runtime_error("RX/TX channel count mismatch; close RX or use matching channels");
        }

        /* Configure the file descriptor watcher. */

#if USE_LITEPCIE
        _tx_stream.fds.fd     = _fd;
        _tx_stream.dma.fds.fd = _fd;
#endif
        _tx_stream.fds.events = POLLOUT;

#if USE_LITEPCIE
        /* Initialize TX DMA Reader */
        _tx_stream.dma.shared_fd  = 1;
        _tx_stream.dma.use_reader = 1;
        _tx_stream.dma.use_writer = 0;
        _tx_stream.dma.loopback   = 0;
        _tx_stream.dma.zero_copy  = 1;
        if (litepcie_dma_init(&_tx_stream.dma, "", _tx_stream.dma.zero_copy) < 0)
            throw std::runtime_error("DMA Reader/TX not available (litepcie_dma_init failed).");

        /* Get Buffer and Parameters from TX DMA Reader */
        _tx_stream.buf = _tx_stream.dma.buf_wr;
        _tx_buf_size   = _tx_stream.dma.mmap_dma_info.dma_tx_buf_size - TX_DMA_HEADER_SIZE;
        _tx_buf_count  = _tx_stream.dma.mmap_dma_info.dma_tx_buf_count;

        /* Ensure the DMA is disabled initially to avoid counters being in a bad state. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
#elif USE_VFIO
        if (!_vfio)
            throw std::runtime_error("DMA Reader/TX: VFIO device not open.");
        if (m2sdr_vfio_tx_start(_vfio) < 0)
            throw std::runtime_error("DMA Reader/TX not available (m2sdr_vfio_tx_start failed).");
        _tx_stream.buf  = _vfio->tx_virt;
        _tx_buf_size    = M2SDR_DMA_BUFFER_SIZE - TX_DMA_HEADER_SIZE;
        _tx_buf_count   = M2SDR_DMA_BUFFER_COUNT;
        _tx_stream.hw_count = 0;
        _tx_stream.sw_count = 0;
#endif

        _tx_stream.opened = true;
        _tx_stream.format = format;
        _tx_stream.soapy_bytes_per_complex = getSoapyBytesPerComplex(format);
        _tx_stream.staging_num_elems = 0;
        _tx_stream.staging_flags = 0;
        _tx_stream.staging_time_ns = 0;
        _tx_stream.staging_data.clear();

        _tx_stream.channels = selected_channels;
        _nChannels = _tx_stream.channels.size();
    } else {
        throw std::runtime_error("Invalid direction.");
    }

    /* Configure 2T2R/1T1R mode (PHY), preserving other bits (e.g. loopback). */
    {
        uint32_t phy_ctrl = litex_m2sdr_readl(_fd, CSR_AD9361_PHY_CONTROL_ADDR);
        if (_nChannels == 1)
            phy_ctrl |=  1u;  /* set mode bit: 1T1R */
        else
            phy_ctrl &= ~1u;  /* clear mode bit: 2T2R */
        litex_m2sdr_writel(_fd, CSR_AD9361_PHY_CONTROL_ADDR, phy_ctrl);
    }

    /* AD9361 Channel en/dis */
    ad9361_phy->pdata->rx2tx2 = (_nChannels == 2);
    if (_nChannels == 1) {
        if (direction == SOAPY_SDR_RX)
            ad9361_phy->pdata->rx1tx1_mode_use_rx_num = _rx_stream.channels[0] == 0 ? RX_1 : RX_2;
        else if (direction == SOAPY_SDR_TX)
            ad9361_phy->pdata->rx1tx1_mode_use_tx_num = _tx_stream.channels[0] == 0 ? TX_1 : TX_2;
    } else {
        if (direction == SOAPY_SDR_RX)
            ad9361_phy->pdata->rx1tx1_mode_use_rx_num = RX_1 | RX_2;
        else if (direction == SOAPY_SDR_TX)
            ad9361_phy->pdata->rx1tx1_mode_use_tx_num = TX_1 | TX_2;
    }

    /* AD9361 Port Control: always keep 2T2R timing enabled so DATA_CLK = 2×Fs
     * regardless of channel count. Clearing this bit in 1T1R mode halves DATA_CLK
     * to Fs, which halves the PHY sample rate (15 MSPS instead of 30.72 MSPS). */
    struct ad9361_phy_platform_data *pd = ad9361_phy->pdata;
    pd->port_ctrl.pp_conf[0] |= (1 << 2);

    ad9361_set_no_ch_mode(ad9361_phy, _nChannels);

    return direction == SOAPY_SDR_RX ? RX_STREAM : TX_STREAM;
}

/* Close the specified stream and release associated resources. */
void SoapyLiteXM2SDR::closeStream(SoapySDR::Stream *stream) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (stream == RX_STREAM) {
#if USE_LITEPCIE || USE_VFIO
        stopRxReceiveWorker();
#endif
#if USE_LITEPCIE
        litepcie_dma_cleanup(&_rx_stream.dma);
        _rx_stream.buf = NULL;
        _rx_stream.batch_buf_size = 0;
        _rx_stream.batch_buf_count = 0;
        _rx_stream.packet_pool.clear();
#elif USE_VFIO
        if (_vfio) m2sdr_vfio_rx_stop(_vfio);
        _rx_stream.buf = nullptr;
        _rx_stream.batch_buf_size = 0;
        _rx_stream.batch_buf_count = 0;
        _rx_stream.packet_pool.clear();
#endif
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
        stopTxWriteWorker();
#if USE_LITEPCIE
        litepcie_dma_cleanup(&_tx_stream.dma);
        _tx_stream.buf = NULL;
#elif USE_VFIO
        if (_vfio) m2sdr_vfio_tx_stop(_vfio);
        _tx_stream.buf = nullptr;
#endif
        _tx_stream.opened = false;
    }
}

/* Activate the specified stream (configure the DMA engines). */
int SoapyLiteXM2SDR::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t /*numElems*/) {

    /* RX */
    if (stream == RX_STREAM) {
        for (size_t i = 0; i < _rx_stream.channels.size(); i++)
            channel_configure(SOAPY_SDR_RX, _rx_stream.channels[i]);
#if USE_LITEPCIE
        /* Crossbar Demux: Select PCIe streaming */
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_DEMUX_SEL_ADDR, 0);
        /* Configure the DMA engine for RX, but don't enable it yet. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
#elif USE_VFIO
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_DEMUX_SEL_ADDR, 0);
        _rx_stream.hw_count = 0;
        _rx_stream.sw_count = 0;
#endif
        _rx_stream.user_count = 0;
        _rx_stream.burst_end = false;
        _rx_stream.time0_ns = this->getHardwareTime("");
        _rx_stream.time0_steady = std::chrono::steady_clock::now();
        _rx_stream.time0_count = _rx_stream.user_count;
        _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
        _rx_stream.timed_start_pending = false;
        _rx_stream.start_time_ns = 0;
        _rx_stream.last_time_ns = _rx_stream.time0_ns;
        _rx_stream.time_warned = false;
        _rx_stream.continuity_resyncs = 0;
        _rx_stream.active_handle = -1;
        _rx_stream.active_num_elems = 0;
        _rx_stream.active_offset = 0;
        _rx_stream.active_buff = nullptr;
        _rx_stream.active_time_ns = 0;
#if USE_LITEPCIE || USE_VFIO
        {
            std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
            _rx_stream.ready_buffs.reset(_rx_stream.batch_buf_count);
            _rx_stream.pending_dma_handles.clear();
            _rx_stream.free_batch_handles.reset(_rx_stream.batch_buf_count);
            for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
                (void)_rx_stream.free_batch_handles.push(i);
        }
        _rx_stream.enqueue_count = 0;
        _rx_stream.overflow_lost_buffers = 0;
#endif
        if (flags & SOAPY_SDR_HAS_TIME) {
            _rx_stream.timed_start_pending = true;
            _rx_stream.start_time_ns = timeNs;
            _rx_stream.time0_ns = timeNs;
            SoapySDR::logf(SOAPY_SDR_INFO,
                "RX timed activation armed for %lld ns",
                (long long)timeNs);
        }
#if USE_LITEPCIE || USE_VFIO
        startRxReceiveWorker();
#endif

    /* TX */
    } else if (stream == TX_STREAM) {
        for (size_t i = 0; i < _tx_stream.channels.size(); i++)
            channel_configure(SOAPY_SDR_TX, _tx_stream.channels[i]);
#if USE_LITEPCIE
        /* Crossbar Mux: Select PCIe streaming */
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_MUX_SEL_ADDR, 0);
        /* Configure the DMA engine for TX, but don't enable it yet. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
        /* Start each stream from a clean timed-TX status baseline so stale
         * late/underrun counters from a prior run are not re-reported as
         * fresh async events to the new Soapy stream. */
#if defined(CSR_TIMED_TX_RESET_COUNTS_ADDR)
        litex_m2sdr_writel(_fd, CSR_TIMED_TX_RESET_COUNTS_ADDR, 1);
        litex_m2sdr_writel(_fd, CSR_TIMED_TX_RESET_COUNTS_ADDR, 0);
        _tx_stream.last_late_count     = 0;
        _tx_stream.last_underrun_count = 0;
#else
#if defined(CSR_TIMED_TX_LATE_COUNT_ADDR)
        _tx_stream.last_late_count = litex_m2sdr_readl(_fd, CSR_TIMED_TX_LATE_COUNT_ADDR);
#endif
#if defined(CSR_TIMED_TX_UNDERRUN_COUNT_ADDR)
        _tx_stream.last_underrun_count = litex_m2sdr_readl(_fd, CSR_TIMED_TX_UNDERRUN_COUNT_ADDR);
#endif
#endif
        _tx_stream.user_count = 0;
        /* Zero-fill the TX DMA ring: clears stale IQ data and old DMA headers
         * (sync words + timestamps) from any previous stream session.  Without
         * this, DMA re-reads old tone data at activateStream time, contaminating
         * the loopback RX capture floor and causing spurious arbiter LATE events. */
        if (_tx_stream.buf && _tx_buf_count > 0) {
            size_t total_tx_bytes = (size_t)_tx_buf_count *
                                    _tx_stream.dma.mmap_dma_info.dma_tx_buf_size;
            memset(_tx_stream.buf, 0, total_tx_bytes);
        }
#elif USE_VFIO
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_MUX_SEL_ADDR, 0);
        _tx_stream.hw_count = 0;
        _tx_stream.sw_count = 0;
        _tx_stream.user_count = 0;
#endif
        _tx_stream.burst_end = false;
        _tx_stream.dma_started = false;
        _tx_stream.staging_num_elems = 0;
        _tx_stream.staging_flags = 0;
        _tx_stream.staging_time_ns = 0;
        _tx_stream.tx_debug_seq = 0;
        _tx_stream.tx_submit_seq = 0;
        startTxWriteWorker();
        if (flags & SOAPY_SDR_HAS_TIME) {
            SoapySDR::logf(SOAPY_SDR_DEBUG,
                "TX timed activation requested for %lld ns; timing is enforced on buffer submission",
                (long long)timeNs);
        }
    }

    return 0;
}

/* Deactivate the specified stream (disable DMA engine). */
int SoapyLiteXM2SDR::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs) {
    if (stream == RX_STREAM) {
        /* Disable the DMA engine for RX. */
#if USE_LITEPCIE || USE_VFIO
        stopRxReceiveWorker();
#endif
#if USE_LITEPCIE
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
#elif USE_VFIO
        /* Ring keeps running; counters reset on next activateStream. */
#endif
        /* set burst_end: if readStream is called after this point SOAPY_SDR_END_BURST
         * will be set
         */
        _rx_stream.burst_end = true;
        _rx_stream.timed_start_pending = false;
        _rx_stream.start_time_ns = 0;
        if (flags & SOAPY_SDR_HAS_TIME) {
            SoapySDR::logf(SOAPY_SDR_WARNING,
                "RX timed deactivation requested for %lld ns; stopping immediately",
                (long long)timeNs);
        }
    } else if (stream == TX_STREAM) {
#if USE_LITEPCIE
        stopTxWriteWorker();
        /* Disable the DMA engine for TX. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
#elif USE_VFIO
        stopTxWriteWorker();
        /* Ring keeps running; counters reset on next activateStream. */
#endif
        if (flags & SOAPY_SDR_HAS_TIME) {
            SoapySDR::logf(SOAPY_SDR_WARNING,
                "TX timed deactivation requested for %lld ns; timing not supported, stopping immediately",
                (long long)timeNs);
        }
    }
    return 0;
}

/*******************************************************************
 * Direct buffer API
 ******************************************************************/

/* Retrieve the maximum transmission unit (MTU) for a stream. */
size_t SoapyLiteXM2SDR::getStreamMTU(SoapySDR::Stream *stream) const {
    if (stream == RX_STREAM) {
        if (_rx_stream.packet_elems != 0) {
            return _rx_stream.packet_elems;
        }
        if (_rx_stream.slice_elems != 0) {
            return _rx_stream.slice_elems;
        }
        return (_rx_stream.batch_buffers * _rx_buf_size) / (_nChannels * _bytesPerComplex);
    } else if (stream == TX_STREAM) {
        return _tx_buf_size / (_nChannels * _bytesPerComplex);
    } else {
        throw std::runtime_error("SoapySDR::getStreamMTU(): Invalid stream.");
    }
}

/* Retrieve the number of direct access buffers available for a stream. */
size_t SoapyLiteXM2SDR::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
    if (stream == RX_STREAM) {
        return _rx_stream.batch_buf_count ? _rx_stream.batch_buf_count : _rx_buf_count;
  } else if (stream == TX_STREAM) {
#if USE_VFIO
    return M2SDR_DMA_BUFFER_COUNT;
#else
    return _dma_mmap_info.dma_tx_buf_count;
#endif
    } else {
        throw std::runtime_error("SoapySDR::getNumDirectAccessBuffers(): Invalid stream.");
    }
}

/* Retrieve buffer addresses for a direct access buffer. */
int SoapyLiteXM2SDR::getDirectAccessBufferAddrs(
    SoapySDR::Stream *stream,
    const size_t handle,
    void **buffs) {
    if (stream == RX_STREAM) {
#if USE_LITEPCIE
        if (!_rx_stream.packet_pool.empty()) {
            buffs[0] = _rx_stream.packet_pool.at(handle).data.data();
        } else {
            buffs[0] = (char *)_rx_stream.buf + handle * _dma_mmap_info.dma_rx_buf_size + RX_DMA_HEADER_SIZE;
        }
#elif USE_VFIO
        buffs[0] = (char *)_rx_stream.buf + handle * _dma_mmap_info.dma_rx_buf_size + RX_DMA_HEADER_SIZE;
#endif
    } else if (stream == TX_STREAM) {
#if USE_LITEPCIE || USE_VFIO
        buffs[0] = (char *)_tx_stream.buf + handle * _dma_mmap_info.dma_tx_buf_size + TX_DMA_HEADER_SIZE;
#endif
    } else {
        throw std::runtime_error("SoapySDR::getDirectAccessBufferAddrs(): Invalid stream.");
    }
    return 0;
}

/***************************************************************************************************
 * DMA Buffer Management
 *
 * The DMA readers/writers utilize a zero-copy mechanism (i.e., a single buffer shared
 * with the kernel) and employ three counters to index that buffer:
 * - hw_count: Indicates the position where the hardware has read from or written to.
 * - sw_count: Indicates the position where userspace has read from or written to.
 * - user_count: Indicates the current position where userspace is reading from or
 *   writing to.
 *
 * The distinction between sw_count and user_count enables tracking of which buffers are
 * currently being processed. This feature is not directly supported by the LitePCIe DMA
 * library, so it is implemented separately.
 *
 * Separating user_count enables advancing read/write buffers without requiring a syscall
 * (interfacing with the kernel only when retiring buffers). However, this can result in
 * slower detection of overflows and underflows, so overflow/underflow detection is made
 * configurable.
 **************************************************************************************************/

#define DETECT_EVERY_OVERFLOW  false  /* Detect overflow on every buffer (true=expensive ioctl/buf). */
#define DETECT_EVERY_UNDERFLOW true  /* Detect underflow every time it occurs. */

static inline long long samples_to_ns(double sample_rate, long long samples)
{
    if (sample_rate <= 0.0) {
        return 0;
    }
    const double ns = (static_cast<double>(samples) * 1e9) / sample_rate;
    return static_cast<long long>(std::llround(ns));
}

/* Acquire a buffer for reading. */
int SoapyLiteXM2SDR::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs) {
    if (stream != RX_STREAM) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    if (_rx_stream.burst_end)
        flags |= SOAPY_SDR_END_BURST;

#if USE_LITEPCIE || USE_VFIO
    const auto acquire_start = std::chrono::steady_clock::now();
    checkRxReceiveError();
    size_t ready_handle = 0;
    size_t queue_before = 0;
    while (!_rx_stream.ready_buffs.pop(ready_handle)) {
        queue_before = _rx_stream.ready_buffs.size();
        if (_rx_stream.overflow) {
            _rx_stream.overflow = false;
            flags |= SOAPY_SDR_END_ABRUPT;
            const int64_t lost_buffers = _rx_stream.overflow_lost_buffers;
            if (lost_buffers > 0) {
                const int64_t max_count = (LITEX_OVERFLOW_COUNT_MASK >> LITEX_OVERFLOW_COUNT_SHIFT);
                const int64_t clamped = (lost_buffers > max_count) ? max_count : lost_buffers;
                flags |= LITEX_HAS_OVERFLOW_COUNT;
                flags |= ((int)clamped << LITEX_OVERFLOW_COUNT_SHIFT) & LITEX_OVERFLOW_COUNT_MASK;
            }
            return SOAPY_SDR_OVERFLOW;
        }
        if (timeoutUs == 0)
            return SOAPY_SDR_TIMEOUT;

        std::unique_lock<std::mutex> lock(_rx_stream.recv_mutex);
        const bool wait_forever = (timeoutUs < 0);
        const auto wait_dur = wait_forever ? std::chrono::milliseconds(100)
                                           : std::chrono::microseconds(timeoutUs);
        const auto ready_pred = [this]() {
            return !_rx_stream.ready_buffs.empty() || _rx_stream.overflow || !_rx_stream.recv_error.empty() ||
                   !_rx_stream.recv_thread_running;
        };
        bool woke = false;
        if (wait_forever) {
            _rx_stream.recv_cv.wait(lock, ready_pred);
            woke = true;
        } else {
            woke = _rx_stream.recv_cv.wait_for(lock, wait_dur, ready_pred);
        }
        if (!woke)
            return SOAPY_SDR_TIMEOUT;
        if (_rx_stream.ready_buffs.empty()) {
            if (_rx_stream.overflow) {
                _rx_stream.overflow = false;
                flags |= SOAPY_SDR_END_ABRUPT;
                const int64_t lost_buffers = _rx_stream.overflow_lost_buffers;
                if (lost_buffers > 0) {
                    const int64_t max_count = (LITEX_OVERFLOW_COUNT_MASK >> LITEX_OVERFLOW_COUNT_SHIFT);
                    const int64_t clamped = (lost_buffers > max_count) ? max_count : lost_buffers;
                    flags |= LITEX_HAS_OVERFLOW_COUNT;
                    flags |= ((int)clamped << LITEX_OVERFLOW_COUNT_SHIFT) & LITEX_OVERFLOW_COUNT_MASK;
                }
                return SOAPY_SDR_OVERFLOW;
            }
            if (!_rx_stream.recv_error.empty() || !_rx_stream.recv_thread_running) {
                lock.unlock();
                checkRxReceiveError();
                return SOAPY_SDR_STREAM_ERROR;
            }
            return SOAPY_SDR_TIMEOUT;
        }
    }
    queue_before = _rx_stream.ready_buffs.size();
    auto &ready = _rx_stream.packet_pool.at(ready_handle);

    handle = ready_handle;
    _rx_stream.user_count++;
    flags |= ready.flags;
    timeNs = ready.timeNs;
    if (_rx_stream.time_valid && _rx_stream.time_base_locked) {
        timeNs = _rx_stream.time0_ns + samples_to_ns(_rx_stream.samplerate, ready.sampleIndex);
        flags |= SOAPY_SDR_HAS_TIME;
    }
    if (rx_ts_trace_allowed()) {
        std::fprintf(stderr,
            "RXTS acquire handle=%lld num=%zu time=%lld flags=0x%x q_after=%zu user=%lld\n",
            (long long)ready_handle,
            ready.numElems,
            (long long)ready.timeNs,
            ready.flags,
            queue_before,
            (long long)_rx_stream.user_count);
    }

    /* Get the buffer. */
    {
        getDirectAccessBufferAddrs(stream, handle, (void **)buffs);
        const size_t samples_per_buffer = ready.numElems ? ready.numElems : getStreamMTU(stream);
        if (_rx_stream.time_valid && !(flags & SOAPY_SDR_HAS_TIME)) {
            timeNs = _rx_stream.time0_ns + samples_to_ns(_rx_stream.samplerate, ready.sampleIndex);
            const long long now = this->getHardwareTime("");
            /* Tolerate fast SW processing (software can process buffers ahead of
             * real-time); only flag a genuine clock stall/reset (>1 s backwards). */
            if (now < _rx_stream.last_time_ns - 1000000000LL) {
                _rx_stream.time_valid = false;
                if (!_rx_stream.time_warned) {
                    SoapySDR::log(SOAPY_SDR_WARNING,
                        "RX hardware time appears stuck; disabling SOAPY_SDR_HAS_TIME");
                    _rx_stream.time_warned = true;
                }
            }
            flags |= SOAPY_SDR_HAS_TIME;
            if (timeNs < _rx_stream.last_time_ns) {
                timeNs = _rx_stream.last_time_ns;
            } else {
                _rx_stream.last_time_ns = timeNs;
            }
        } else if (!_rx_stream.time_valid) {
            if (_rx_stream.samplerate <= 0.0 && !_rx_stream.time_warned) {
                SoapySDR::log(SOAPY_SDR_WARNING,
                    "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
                _rx_stream.time_warned = true;
            }
        }
        if (_rx_stream.rx_debug) {
            const auto acquire_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - acquire_start).count();
            if (acquire_us >= _rx_stream.rx_debug_threshold_us &&
                debug_log_allowed(_rx_stream.rx_debug, _rx_stream.rx_debug_seq, _rx_stream.rx_debug_limit)) {
                std::fprintf(stderr,
                    "RXDBG acquire dt=%lldus handle=%lld flags=0x%x q_after=%zu hw=%lld sw=%lld user=%lld\n",
                    (long long)acquire_us,
                    (long long)ready_handle,
                    flags,
                    queue_before,
                    (long long)_rx_stream.hw_count,
                    (long long)_rx_stream.sw_count,
                    (long long)_rx_stream.user_count);
            }
        }
        return samples_per_buffer;
    }
#endif
}

/* Release a read buffer after use. */
void SoapyLiteXM2SDR::releaseReadBuffer(
    SoapySDR::Stream */*stream*/,
    size_t handle) {
    assert(handle != (size_t)-1 && "Attempt to release an invalid buffer (e.g., from an overflow).");

#if USE_LITEPCIE
    if (!_rx_stream.free_batch_handles.push(handle))
        throw std::runtime_error("RX free buffer queue overflow.");
    _rx_stream.recv_cv.notify_all();
#elif USE_VFIO
    /* No kernel notification needed; just advance local sw_count. */
    _rx_stream.sw_count = (int64_t)handle + 1;
    m2sdr_vfio_rx_sw_set(_vfio, _rx_stream.sw_count);
#endif
}

/* Acquire a buffer for writing. */
int SoapyLiteXM2SDR::acquireWriteBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    void **buffs,
    const long timeoutUs) {
    if (stream != TX_STREAM) {
        return SOAPY_SDR_STREAM_ERROR;
    }

#if USE_LITEPCIE
    /* Check if there are buffers available. */
    int buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
    assert(buffers_pending <= (int)_dma_mmap_info.dma_tx_buf_count);

    /* If not, check with the DMA engine.
     * Only call litepcie_dma_reader once the DMA is actually running (started
     * in releaseWriteBuffer after the first buffer header is written).  Before
     * that, hw_count is 0 and user_count is 0, so pending stays 0 and we can
     * issue buffers freely up to ring depth without needing hw_count updates. */
    if (_tx_stream.dma_started &&
        (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count) || DETECT_EVERY_UNDERFLOW)) {
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
    }

    /* hw_count can legitimately move ahead of user_count when the hardware has
     * drained the queue. That means "no buffers pending", not "buffer acquire
     * failed". Clamp to zero and let stream-status report real underruns. */
    if (buffers_pending < 0) {
        buffers_pending = 0;
    }

    /* If no buffers available, wait for new buffers to become available. */
    if (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count)) {
        if (timeoutUs == 0) {
            return SOAPY_SDR_TIMEOUT;
        }
        const bool wait_forever = (timeoutUs < 0);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::microseconds(wait_forever ? (long long)1e18 : timeoutUs);
        while (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count)) {
            const auto now = std::chrono::steady_clock::now();
            if (!wait_forever && now >= deadline)
                return SOAPY_SDR_TIMEOUT;
            const long remaining_us = wait_forever ? 1000 :
                std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
            const int ret = poll(&_tx_stream.fds, 1, poll_timeout_ms_slice(remaining_us));
            if (ret < 0) {
                throw std::runtime_error("SoapyLiteXM2SDR::acquireWriteBuffer(): Poll failed, " +
                                         std::string(strerror(errno)) + ".");
            }
            if (ret == 0)
                continue;

            litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
            buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
            if (buffers_pending < 0)
                buffers_pending = 0;
        }
    }

    /* Get the buffer. */
    int buf_offset = _tx_stream.user_count % _dma_mmap_info.dma_tx_buf_count;
    getDirectAccessBufferAddrs(stream, buf_offset, buffs);

    /* Update the DMA counters. */
    handle = _tx_stream.user_count;
    _tx_stream.user_count++;
    return getStreamMTU(stream);
#elif USE_VFIO
    {
    int buffers_pending = (int)(_tx_stream.user_count - _tx_stream.hw_count);

        if (buffers_pending >= (int)_dma_mmap_info.dma_tx_buf_count || DETECT_EVERY_UNDERFLOW) {
            _tx_stream.hw_count = m2sdr_vfio_tx_hw_count(_vfio);
            buffers_pending = (int)(_tx_stream.user_count - _tx_stream.hw_count);
        }

        if (buffers_pending >= (int)_dma_mmap_info.dma_tx_buf_count) {
            if (timeoutUs == 0)
                return SOAPY_SDR_TIMEOUT;
            int free_slots = m2sdr_vfio_tx_wait(_vfio, &_tx_stream.hw_count,
                                                _tx_stream.user_count, timeoutUs);
            if (free_slots <= 0)
                return SOAPY_SDR_TIMEOUT;
            buffers_pending = (int)(_tx_stream.user_count - _tx_stream.hw_count);
        }

        int buf_offset = (int)(_tx_stream.user_count % _dma_mmap_info.dma_tx_buf_count);
        getDirectAccessBufferAddrs(stream, buf_offset, buffs);

        /* Update the DMA counters. */
        handle = _tx_stream.user_count;
        _tx_stream.user_count++;
        return getStreamMTU(stream);
    }
#endif
}

/* Release a write buffer after use. */
void SoapyLiteXM2SDR::releaseWriteBuffer(
    SoapySDR::Stream *stream,
    size_t handle,
    const size_t numElems,
    int &flags,
    const long long timeNs) {
    if (flags & SOAPY_SDR_END_BURST) {
        _tx_stream.burst_end = true;
    }

#if USE_LITEPCIE
    /* Write DMA header: sync word with burst flags (word 0) and TX timestamp (word 1).
     * The TimedTXArbiter in the FPGA reads word 1 and holds samples until
     * time_gen.time >= timeNs. Both values are in nanoseconds. */
    {
        const size_t buf_offset = handle % _dma_mmap_info.dma_tx_buf_count;
        uint8_t *tx_hdr = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
                          (buf_offset * _dma_mmap_info.dma_tx_buf_size);
        const bool hasTime  = (flags & SOAPY_SDR_HAS_TIME) && timeNs > 0;
        const bool endBurst = (flags & SOAPY_SDR_END_BURST);
        uint64_t sync_word = DMA_HEADER_SYNC_WORD;
        if (hasTime)  sync_word |= TX_FLAG_HAS_TIME;
        if (endBurst) sync_word |= TX_FLAG_END_BURST;
        *reinterpret_cast<uint64_t*>(tx_hdr + 0) = htole64(sync_word);
        *reinterpret_cast<uint64_t*>(tx_hdr + 8) = htole64(hasTime ? (uint64_t)timeNs : 0ULL);
        if (_tx_stream.tx_debug_headers && (hasTime || endBurst)) {
            const uint64_t seq = ++_tx_stream.tx_debug_seq;
            if (_tx_stream.tx_debug_limit == 0 || seq <= _tx_stream.tx_debug_limit) {
                const long long now_ns = this->getHardwareTime("");
                std::fprintf(stderr,
                    "TXDBG hdr seq=%llu handle=%llu slot=%llu flags=%s%s numElems=%llu ts=%.6f hw_now=%.6f delta_ms=%+.3f\n",
                    (unsigned long long)seq,
                    (unsigned long long)handle,
                    (unsigned long long)buf_offset,
                    hasTime ? "TIME" : "-",
                    endBurst ? "|EOB" : "",
                    (unsigned long long)numElems,
                    timeNs / 1e9,
                    now_ns / 1e9,
                    (timeNs - now_ns) / 1e6);
            }
        }
    }
#endif

    const size_t mtu = this->getStreamMTU(stream);
    if (numElems < mtu) {
        uint8_t *buf = nullptr;
#if USE_LITEPCIE || USE_VFIO
        const size_t buf_offset = handle % _dma_mmap_info.dma_tx_buf_count;
        buf = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
              (buf_offset * _dma_mmap_info.dma_tx_buf_size) + TX_DMA_HEADER_SIZE;
#endif
        if (buf) {
            const size_t offset_bytes = numElems * _nChannels * _bytesPerComplex;
            const size_t zero_bytes = (mtu - numElems) * _nChannels * _bytesPerComplex;
            std::memset(buf + offset_bytes, 0, zero_bytes);
        }
    }

#if USE_LITEPCIE
    if (!_tx_stream.dma_started) {
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _tx_stream.dma_started = true;
    }
    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = handle + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &mmap_dma_update);
    if (_tx_stream.tx_debug_headers) {
        const uint64_t seq = ++_tx_stream.tx_submit_seq;
        if (_tx_stream.tx_debug_limit == 0 || seq <= _tx_stream.tx_debug_limit) {
            const long long now_ns = this->getHardwareTime("");
            std::fprintf(stderr,
                "TXDBG submit seq=%llu handle=%llu sw_count=%llu hw_now=%.6f dma_hw=%lld dma_sw=%lld\n",
                (unsigned long long)seq,
                (unsigned long long)handle,
                (unsigned long long)mmap_dma_update.sw_count,
                now_ns / 1e9,
                (long long)_tx_stream.hw_count,
                (long long)_tx_stream.sw_count);
        }
    }
#elif USE_VFIO
    /* VFIO loop-mode: ring is pre-programmed; track sw_count locally only. */
    _tx_stream.sw_count = (int64_t)handle + 1;
#endif
}

/* Interleave CF32 samples. */
void SoapyLiteXM2SDR::interleaveCF32(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    const float *samples_cf32 = reinterpret_cast<const float*>(src) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 2) {
        int16_t *dst_int16 = reinterpret_cast<int16_t*>(dst) + (offset * 2 * _samplesPerComplex);
        for (uint32_t i = 0; i < len; i++) {
            dst_int16[0] = static_cast<int16_t>(samples_cf32[0] * _samplesScaling); /* I. */
            dst_int16[1] = static_cast<int16_t>(samples_cf32[1] * _samplesScaling); /* Q. */
            samples_cf32 += 2;
            dst_int16 += _nChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        int8_t *dst_int8 = reinterpret_cast<int8_t*>(dst) + (offset * 2 * _samplesPerComplex);
        for (uint32_t i = 0; i < len; i++) {
            dst_int8[0] = static_cast<int8_t>(samples_cf32[0] * (_samplesScaling)); /* I. */
            dst_int8[1] = static_cast<int8_t>(samples_cf32[1] * (_samplesScaling)); /* Q. */
            samples_cf32 += 2;
            dst_int8 += _nChannels * _samplesPerComplex;
        }
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported _bytesPerSample value: %u.", _bytesPerSample);
    }
}

/* Deinterleave CF32 samples. */
void SoapyLiteXM2SDR::deinterleaveCF32(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    float *samples_cf32 = reinterpret_cast<float*>(dst) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 2) {
        const int16_t *src_int16 = reinterpret_cast<const int16_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            /* Mask 12 LSB and sign-extend */
            int16_t i_sample = static_cast<int16_t>(src_int16[0] << 4) >> 4;  /* I. */
            int16_t q_sample = static_cast<int16_t>(src_int16[1] << 4) >> 4;  /* Q. */
            /* Scale to float (-1.0 to 1.0 range) */
            samples_cf32[0] = static_cast<float>(i_sample) / _samplesScaling; /* I. */
            samples_cf32[1] = static_cast<float>(q_sample) / _samplesScaling; /* Q. */
            samples_cf32 += 2;
            src_int16 += _nChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        const int8_t *src_int8 = reinterpret_cast<const int8_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            /* Scale to float (-1.0 to 1.0 range) */
            samples_cf32[0] = static_cast<float>(src_int8[0]) / _samplesScaling; /* I. */
            samples_cf32[1] = static_cast<float>(src_int8[1]) / _samplesScaling; /* Q. */
            samples_cf32 += 2;
            src_int8 += _nChannels * _samplesPerComplex;
        }
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported _bytesPerSample value: %u.", _bytesPerSample);
    }
}

/* Interleave CS16 samples */
void SoapyLiteXM2SDR::interleaveCS16(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    const int16_t *samples_cs16 = reinterpret_cast<const int16_t*>(src) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 2) {
        int16_t *dst_int16 = reinterpret_cast<int16_t*>(dst) + (offset * 2 * _samplesPerComplex);

        for (uint32_t i = 0; i < len; i++) {
            dst_int16[0] = samples_cs16[0]; /* I. */
            dst_int16[1] = samples_cs16[1]; /* Q. */
            samples_cs16 += 2;
            dst_int16 += _nChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        int8_t *dst_int8 = reinterpret_cast<int8_t*>(dst) + (offset * 2 * _samplesPerComplex);

        for (uint32_t i = 0; i < len; i++) {
            dst_int8[0] = samples_cs16[0]; /* I. */
            dst_int8[1] = samples_cs16[1]; /* Q. */
            samples_cs16 += 2;
            dst_int8 += _nChannels * _samplesPerComplex;
        }
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported _bytesPerSample value: %u.", _bytesPerSample);
    }
}

/* Deinterleave CS16 samples */
void SoapyLiteXM2SDR::deinterleaveCS16(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    int16_t *samples_cs16 = reinterpret_cast<int16_t*>(dst) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 2) {
        const int16_t *src_int16 = reinterpret_cast<const int16_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            /* Mask 12 LSB and sign-extend */
            samples_cs16[0] = static_cast<int16_t>(src_int16[0] << 4) >> 4;  /* I. */
            samples_cs16[1] = static_cast<int16_t>(src_int16[1] << 4) >> 4;  /* Q. */
            samples_cs16 += 2;
            src_int16 += _nChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        const int8_t *src_int8 = reinterpret_cast<const int8_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            samples_cs16[0] = static_cast<int16_t>(src_int8[0]); /* I. */
            samples_cs16[1] = static_cast<int16_t>(src_int8[1]); /* Q. */
            samples_cs16 += 2;
            src_int8 += _nChannels * _samplesPerComplex;
        }
    } else {
        printf("Unsupported _bytesPerSample value: %u\n", _bytesPerSample);
    }
}

/* Interleave samples */
void SoapyLiteXM2SDR::interleave(
    const void *src,
    void *dst,
    uint32_t len,
    const std::string &format,
    size_t offset) {
    if (format == SOAPY_SDR_CF32) {
        interleaveCF32(src, dst, len, offset);
    } else if (format == SOAPY_SDR_CS16) {
        interleaveCS16(src, dst, len, offset);
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported format: %s.", format.c_str());
    }
}

/* Deinterleave samples */
void SoapyLiteXM2SDR::deinterleave(
    const void *src,
    void *dst,
    uint32_t len,
    const std::string &format,
    size_t offset) {
    if (format == SOAPY_SDR_CF32) {
        deinterleaveCF32(src, dst, len, offset);
    } else if (format == SOAPY_SDR_CS16) {
        deinterleaveCS16(src, dst, len, offset);
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported format: %s.", format.c_str());
    }
}

/* Read from the RX stream. */
int SoapyLiteXM2SDR::readStream(
    SoapySDR::Stream *stream,
    void *const *buffs,
    const size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs) {
    if (stream != RX_STREAM) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    /* readStream may span multiple hardware packets. */
    size_t returnedElems = numElems;
    const size_t soapy_bytes = _rx_stream.soapy_bytes_per_complex;

    size_t samp_avail = 0;
    int returnedFlags = flags;
    long long returnedTimeNs = timeNs;
    bool have_returned_time = false;

    while (samp_avail < returnedElems) {
        if (_rx_stream.active_handle < 0) {
            size_t handle;
            int newFlags = flags;
            long long newTimeNs = timeNs;
            int ret = this->acquireReadBuffer(
                stream,
                handle,
                (const void **)&_rx_stream.active_buff,
                newFlags,
                newTimeNs,
                (samp_avail > 0) ? 0 : timeoutUs);

            if (ret < 0) {
                if ((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
                    break;
                }
                return ret;
            }

            _rx_stream.active_handle = handle;
            _rx_stream.active_num_elems = ret;
            _rx_stream.active_offset = 0;
            _rx_stream.active_time_ns = newTimeNs;

            if (!have_returned_time && (newFlags & SOAPY_SDR_HAS_TIME)) {
                returnedFlags = newFlags;
                returnedTimeNs = newTimeNs;
                have_returned_time = true;
            }
        }

        const size_t n = std::min(returnedElems - samp_avail, _rx_stream.active_num_elems);
        long long chunk_time_ns = _rx_stream.active_time_ns;
        int chunk_flags = flags;
        if (_rx_stream.time_valid && _rx_stream.time_base_locked) {
            chunk_time_ns += samples_to_ns(_rx_stream.samplerate, _rx_stream.active_offset);
            chunk_flags |= SOAPY_SDR_HAS_TIME;
            if (chunk_time_ns < _rx_stream.last_time_ns) {
                chunk_time_ns = _rx_stream.last_time_ns;
            } else {
                _rx_stream.last_time_ns = chunk_time_ns;
            }
        } else if (_rx_stream.samplerate <= 0.0 && !_rx_stream.time_warned) {
            SoapySDR::log(SOAPY_SDR_WARNING,
                "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
            _rx_stream.time_warned = true;
        }
        if (!have_returned_time && (chunk_flags & SOAPY_SDR_HAS_TIME)) {
            returnedFlags = chunk_flags;
            returnedTimeNs = chunk_time_ns;
            have_returned_time = true;
        }

        for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
            const uint8_t *src = reinterpret_cast<const uint8_t *>(_rx_stream.active_buff) +
                (i * getStreamMTU(stream) + _rx_stream.active_offset) * soapy_bytes;
            std::memcpy(reinterpret_cast<uint8_t *>(buffs[i]) + samp_avail * soapy_bytes,
                        src,
                        n * soapy_bytes);
        }

        _rx_stream.active_num_elems -= n;
        _rx_stream.active_offset += n;
        samp_avail += n;

        if (_rx_stream.active_num_elems == 0) {
            this->releaseReadBuffer(stream, _rx_stream.active_handle);
            _rx_stream.active_handle = -1;
            _rx_stream.active_offset = 0;
            _rx_stream.active_buff = nullptr;
        }
    }

    const size_t totalReturnedElems = samp_avail;

    if (rx_ts_trace_allowed()) {
        std::fprintf(stderr,
            "RXTS read ret=%zu time=%lld flags=0x%x samp_avail=%zu active_handle=%lld active_num=%zu active_off=%zu have_time=%d\n",
            totalReturnedElems,
            (long long)returnedTimeNs,
            returnedFlags,
            samp_avail,
            (long long)_rx_stream.active_handle,
            _rx_stream.active_num_elems,
            _rx_stream.active_offset,
            have_returned_time ? 1 : 0);
    }
    flags = returnedFlags;
    timeNs = returnedTimeNs;
    return static_cast<int>(totalReturnedElems);
}

/* Write to the TX stream. */
int SoapyLiteXM2SDR::writeStream(
    SoapySDR::Stream *stream,
    const void *const *buffs,
    const size_t numElems,
    int &flags,
    const long long timeNs,
    const long timeoutUs) {
    if (stream != TX_STREAM) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    checkTxWriteError();

    const size_t mtu = this->getStreamMTU(stream);
    const size_t soapy_bytes = _tx_stream.soapy_bytes_per_complex;
    const size_t chan_count = _tx_stream.channels.size();

    auto enqueue_packet = [&](size_t packet_elems, int packet_flags, long long packet_time_ns) -> int {
        TXStream::WriteJob job;
        job.numElems = packet_elems;
        job.flags = packet_flags;
        job.timeNs = packet_time_ns;
        job.enqueue_tp = std::chrono::steady_clock::now();
        job.data.resize(chan_count * packet_elems * soapy_bytes);
        for (size_t i = 0; i < chan_count; ++i) {
            const uint8_t *src = _tx_stream.staging_data.data() + i * mtu * soapy_bytes;
            std::memcpy(job.data.data() + i * packet_elems * soapy_bytes,
                        src,
                        packet_elems * soapy_bytes);
        }

        std::unique_lock<std::mutex> lock(_tx_stream.write_mutex);
        auto can_enqueue = [this]() {
            return _tx_stream.write_thread_stop ||
                   (_tx_stream.pending_write_jobs.size() < _tx_stream.write_queue_depth);
        };
        if (timeoutUs == 0) {
            if (!can_enqueue())
                return SOAPY_SDR_TIMEOUT;
        } else if (timeoutUs < 0) {
            _tx_stream.write_cv.wait(lock, can_enqueue);
        } else if (!_tx_stream.write_cv.wait_for(lock, std::chrono::microseconds(timeoutUs), can_enqueue)) {
            return SOAPY_SDR_TIMEOUT;
        }

        if (_tx_stream.write_thread_stop)
            return SOAPY_SDR_STREAM_ERROR;

        _tx_stream.pending_write_jobs.push_back(std::move(job));
        lock.unlock();
        _tx_stream.write_cv.notify_one();
        return static_cast<int>(packet_elems);
    };

    auto flush_staging = [&](int packet_flags) -> int {
        if (_tx_stream.staging_num_elems == 0 && (_tx_stream.staging_flags & SOAPY_SDR_HAS_TIME) == 0 &&
            (_tx_stream.staging_flags & SOAPY_SDR_END_BURST) == 0 && (packet_flags & SOAPY_SDR_END_BURST) == 0) {
            return 0;
        }
        const int ret = enqueue_packet(_tx_stream.staging_num_elems,
                                       _tx_stream.staging_flags | packet_flags,
                                       _tx_stream.staging_time_ns);
        _tx_stream.staging_num_elems = 0;
        _tx_stream.staging_flags = 0;
        _tx_stream.staging_time_ns = 0;
        return ret;
    };

    if (_tx_stream.staging_data.size() != chan_count * mtu * soapy_bytes)
        _tx_stream.staging_data.assign(chan_count * mtu * soapy_bytes, 0);

    if ((flags & SOAPY_SDR_HAS_TIME) && _tx_stream.staging_num_elems > 0) {
        const int ret = flush_staging(0);
        if (ret < 0)
            return ret;
    }

    if (numElems == 0) {
        if (flags & SOAPY_SDR_END_BURST) {
            if (_tx_stream.staging_num_elems > 0)
                return flush_staging(SOAPY_SDR_END_BURST);
            return enqueue_packet(0, flags, timeNs);
        }
        return 0;
    }

    size_t consumed = 0;
    while (consumed < numElems) {
        if (_tx_stream.staging_num_elems == 0) {
            _tx_stream.staging_flags = 0;
            _tx_stream.staging_time_ns = 0;
            if (flags & SOAPY_SDR_HAS_TIME) {
                _tx_stream.staging_flags |= SOAPY_SDR_HAS_TIME;
                _tx_stream.staging_time_ns = timeNs;
                if (_tx_stream.samplerate > 0.0 && consumed > 0)
                    _tx_stream.staging_time_ns += samples_to_ns(_tx_stream.samplerate, consumed);
            }
        }

        const size_t chunk = std::min(mtu - _tx_stream.staging_num_elems, numElems - consumed);
        for (size_t i = 0; i < chan_count; ++i) {
            const uint8_t *src = reinterpret_cast<const uint8_t *>(buffs[i]) + consumed * soapy_bytes;
            uint8_t *dst = _tx_stream.staging_data.data() +
                (i * mtu + _tx_stream.staging_num_elems) * soapy_bytes;
            std::memcpy(dst, src, chunk * soapy_bytes);
        }

        consumed += chunk;
        _tx_stream.staging_num_elems += chunk;

        const bool end_of_input = (consumed == numElems);
        const bool flush_now = (_tx_stream.staging_num_elems == mtu) ||
                               (end_of_input && (flags & SOAPY_SDR_END_BURST));
        if (flush_now) {
            const int packet_flags = (end_of_input && (flags & SOAPY_SDR_END_BURST)) ? SOAPY_SDR_END_BURST : 0;
            const int ret = flush_staging(packet_flags);
            if (ret < 0)
                return consumed > chunk ? static_cast<int>(consumed - chunk) : ret;
        }
    }

    return static_cast<int>(numElems);
}

/* Check the status of the TX/RX streams. */
int SoapyLiteXM2SDR::readStreamStatus(
    SoapySDR::Stream *stream,
    size_t &chanMask,
    int &flags,
    long long &timeNs,
    const long timeoutUs){

    if (stream == RX_STREAM) {
        if (_rx_stream.overflow) {
            _rx_stream.overflow = false;
            SoapySDR::log(SOAPY_SDR_SSI, "O");
            return SOAPY_SDR_OVERFLOW;
        }
    } else if (stream == TX_STREAM) {
        if (_tx_stream.underflow) {
            _tx_stream.underflow = false;
            SoapySDR::log(SOAPY_SDR_SSI, "U");
            return SOAPY_SDR_UNDERFLOW;
        }
#if USE_LITEPCIE && defined(CSR_TIMED_TX_LATE_COUNT_ADDR)
        /* Report late TX packets (timestamp missed) from the TimedTXArbiter. */
        {
            uint32_t late = litex_m2sdr_readl(_fd, CSR_TIMED_TX_LATE_COUNT_ADDR);
            if (late != _tx_stream.last_late_count) {
                _tx_stream.last_late_count = late;
                chanMask = 1;
                flags    = SOAPY_SDR_TIME_ERROR;
                timeNs   = this->getHardwareTime("");
                return SOAPY_SDR_TIME_ERROR;
            }
        }
#endif
#if USE_LITEPCIE && defined(CSR_TIMED_TX_UNDERRUN_COUNT_ADDR)
        /* Report TX underruns (no burst ready) from the TimedTXArbiter. */
        {
            uint32_t underrun = litex_m2sdr_readl(_fd, CSR_TIMED_TX_UNDERRUN_COUNT_ADDR);
            if (underrun != _tx_stream.last_underrun_count) {
                _tx_stream.last_underrun_count = underrun;
                chanMask = 1;
                flags    = SOAPY_SDR_UNDERFLOW;
                timeNs   = this->getHardwareTime("");
                return SOAPY_SDR_UNDERFLOW;
            }
        }
#endif
    } else {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    /* Calculate the timeout duration and exit time. */
    const auto timeout = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(timeoutUs));
    const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

    /* Poll for status events until the timeout expires. */
    while (true) {
        /* Sleep for a fraction of the total timeout. */
        const auto sleepTimeUs = std::min<long>(1000, timeoutUs/10);
        std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeUs));

        /* Check if the timeout has expired. */
        const auto timeNow = std::chrono::high_resolution_clock::now();
        if (exitTime < timeNow) return SOAPY_SDR_TIMEOUT;
    }
}
