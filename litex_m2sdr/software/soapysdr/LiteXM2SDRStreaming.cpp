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

#if USE_LITEETH
#include "liteeth_udp.h"
#endif

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

#if USE_LITEETH
static constexpr size_t VRT_SIGNAL_HEADER_BYTES = 20;
static constexpr size_t VRT_RX_DATA_WORDS_DEFAULT = 256;
static constexpr size_t VRT_RX_PAYLOAD_BYTES_DEFAULT = VRT_RX_DATA_WORDS_DEFAULT * sizeof(uint32_t);
static constexpr size_t VRT_RX_PACKET_BYTES_DEFAULT = VRT_SIGNAL_HEADER_BYTES + VRT_RX_PAYLOAD_BYTES_DEFAULT;

static inline uint32_t read_be32(const uint8_t *p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) <<  8) |
           (static_cast<uint32_t>(p[3]) <<  0);
}

static inline uint64_t read_be64(const uint8_t *p)
{
    return (static_cast<uint64_t>(read_be32(p + 0)) << 32) |
           (static_cast<uint64_t>(read_be32(p + 4)) <<  0);
}
#endif

/* RX DMA Header */
#if USE_LITEPCIE && defined(_RX_DMA_HEADER_TEST)
static constexpr size_t RX_DMA_HEADER_SIZE = 16;
#else
static constexpr size_t RX_DMA_HEADER_SIZE = 0;
#endif

/* TX DMA Header - always 16 bytes on PCIe (sync word + timestamp for TimedTXArbiter). */
#if USE_LITEPCIE
static constexpr size_t TX_DMA_HEADER_SIZE = 16;
#else
static constexpr size_t TX_DMA_HEADER_SIZE = 0;
#endif

/* TX burst control flags embedded in the upper 16 bits of DMA header word 0. */
static constexpr uint64_t TX_FLAG_HAS_TIME  = (1ULL << 63);
static constexpr uint64_t TX_FLAG_END_BURST = (1ULL << 62);

static int poll_timeout_ms_slice(const long remaining_us)
{
    if (remaining_us <= 0)
        return 0;
    const long slice_us = std::min<long>(remaining_us, 1000);
    return std::max<int>(1, static_cast<int>((slice_us + 999) / 1000));
}

static bool debug_log_allowed(bool enabled, std::atomic<uint64_t> &seq, uint64_t limit)
{
    if (!enabled)
        return false;
    const uint64_t cur = ++seq;
    return (limit == 0 || cur <= limit);
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

void SoapyLiteXM2SDR::checkRxReceiveError()
{
#if USE_LITEPCIE
    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
    if (!_rx_stream.recv_error.empty()) {
        const std::string err = _rx_stream.recv_error;
        _rx_stream.recv_error.clear();
        throw std::runtime_error(err);
    }
#endif
}

void SoapyLiteXM2SDR::rxDeliverLoop()
{
#if USE_LITEPCIE
    try {
        for (;;) {
            RXStream::ReadyBuffer ready;
            {
                std::unique_lock<std::mutex> lock(_rx_stream.recv_mutex);
                _rx_stream.recv_cv.wait(lock, [this]() {
                    return _rx_stream.recv_thread_stop || !_rx_stream.pending_ready_buffs.empty();
                });
                if (_rx_stream.recv_thread_stop) {
                    _rx_stream.deliver_thread_running = false;
                    _rx_stream.recv_cv.notify_all();
                    return;
                }
                ready = _rx_stream.pending_ready_buffs.front();
            }

            if (_rx_stream.wallclock_pacing && (_rx_stream.time_valid && ready.timeNs > 0)) {
                for (;;) {
                    {
                        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
                        if (_rx_stream.recv_thread_stop) {
                            _rx_stream.deliver_thread_running = false;
                            _rx_stream.recv_cv.notify_all();
                            return;
                        }
                    }
                    const long long now_ns = this->getHardwareTime("");
                    const long long wait_ns = ready.timeNs - now_ns - _rx_stream.pacing_margin_ns;
                    if (wait_ns <= 0)
                        break;
                    const long long sleep_ns = std::min<long long>(wait_ns, 100000LL);
                    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
                }
            }

            {
                std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
                if (_rx_stream.recv_thread_stop) {
                    _rx_stream.deliver_thread_running = false;
                    _rx_stream.recv_cv.notify_all();
                    return;
                }
                if (_rx_stream.pending_ready_buffs.empty())
                    continue;
                ready = _rx_stream.pending_ready_buffs.front();
                _rx_stream.pending_ready_buffs.pop_front();
                _rx_stream.ready_buffs.push_back(ready);
                _rx_stream.recv_cv.notify_all();
            }
        }
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        if (_rx_stream.recv_error.empty())
            _rx_stream.recv_error = std::string("RX delivery worker failed: ") + e.what();
        _rx_stream.deliver_thread_running = false;
        _rx_stream.recv_cv.notify_all();
    }
#endif
}

void SoapyLiteXM2SDR::rxReceiveLoop()
{
#if USE_LITEPCIE
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

        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        {
            struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
            mmap_dma_update.sw_count = _rx_stream.hw_count;
            checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
        }
        _rx_stream.sw_count = _rx_stream.hw_count;
        _rx_stream.enqueue_count = _rx_stream.hw_count;
        _rx_stream.user_count = _rx_stream.hw_count;
        _rx_stream.time0_ns = _rx_stream.timed_start_pending ? _rx_stream.start_time_ns : this->getHardwareTime("");
        _rx_stream.time0_count = _rx_stream.hw_count;
        _rx_stream.last_time_ns = _rx_stream.time0_ns;
        _rx_stream.dma_writer_started = true;
        _rx_stream.timed_start_pending = false;

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
            const int poll_ret = poll(&_rx_stream.fds, 1, 1);
            const auto poll_end = std::chrono::steady_clock::now();
            if (poll_ret < 0) {
                if (errno == EINTR)
                    continue;
                throw std::runtime_error("RX receive worker poll failed, " + std::string(strerror(errno)) + ".");
            }

            const auto dma_start = std::chrono::steady_clock::now();
            litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
            const auto dma_end = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
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

                    _rx_stream.ready_buffs.clear();
                    _rx_stream.pending_ready_buffs.clear();
                    _rx_stream.pending_dma_handles.clear();
                    _rx_stream.free_batch_handles.clear();
                    for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
                        _rx_stream.free_batch_handles.push_back(i);
                    _rx_stream.enqueue_count = _rx_stream.hw_count;
                    _rx_stream.user_count = _rx_stream.hw_count;
                    _rx_stream.sw_count = _rx_stream.hw_count;
                    _rx_stream.overflow_lost_buffers = lost_buffers;
                    _rx_stream.overflow = true;
                    _rx_stream.time0_ns = this->getHardwareTime("");
                    _rx_stream.time0_count = _rx_stream.user_count;
                    _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
                    _rx_stream.last_time_ns = _rx_stream.time0_ns;
                    _rx_stream.time_warned = false;
                    _rx_stream.recv_cv.notify_all();
                    continue;
                }

                const size_t dma_mtu = _rx_buf_size / (_nChannels * _bytesPerComplex);
                const int64_t newly_completed = _rx_stream.hw_count - _rx_stream.enqueue_count;
                while (_rx_stream.enqueue_count < _rx_stream.hw_count) {
                    _rx_stream.pending_dma_handles.push_back(_rx_stream.enqueue_count);
                    _rx_stream.enqueue_count++;
                }

                const size_t batch_buffers = std::max<size_t>(1, _rx_stream.batch_buffers);
                const size_t total_batch_elems = batch_buffers * dma_mtu;
                const size_t slice_elems = std::max<size_t>(
                    1, std::min(total_batch_elems, _rx_stream.slice_elems ? _rx_stream.slice_elems : total_batch_elems));
                const size_t slice_count = (total_batch_elems + slice_elems - 1) / slice_elems;
                const size_t bytes_per_elem = _nChannels * _bytesPerComplex;
                while (_rx_stream.pending_dma_handles.size() >= batch_buffers &&
                       _rx_stream.free_batch_handles.size() >= slice_count) {
                    const int64_t first_dma_handle = _rx_stream.pending_dma_handles.front();
                    std::vector<int64_t> dma_handles;
                    dma_handles.reserve(batch_buffers);
                    for (size_t i = 0; i < batch_buffers; ++i) {
                        dma_handles.push_back(_rx_stream.pending_dma_handles.front());
                        _rx_stream.pending_dma_handles.pop_front();
                    }

                    int ready_flags = 0;
                    long long ready_time_ns = 0;
#if defined(_RX_DMA_HEADER_TEST)
                    {
                        const int first_offset = first_dma_handle % _dma_mmap_info.dma_rx_buf_count;
                        const uint8_t *header_ptr = reinterpret_cast<const uint8_t *>(_rx_stream.buf) +
                                                    first_offset * _dma_mmap_info.dma_rx_buf_size;
                        uint64_t header = *reinterpret_cast<const uint64_t*>(header_ptr);
                        if (header == DMA_HEADER_SYNC_WORD) {
                            ready_time_ns = static_cast<long long>(*reinterpret_cast<const uint64_t*>(header_ptr + 8));
                            ready_flags |= SOAPY_SDR_HAS_TIME;
                        }
                    }
#endif
                    if (!(ready_flags & SOAPY_SDR_HAS_TIME) && _rx_stream.time_valid) {
                        const double ns = (static_cast<double>(
                                               static_cast<long long>(first_dma_handle - _rx_stream.time0_count) *
                                               static_cast<long long>(dma_mtu)) * 1e9) / _rx_stream.samplerate;
                        ready_time_ns = _rx_stream.time0_ns + static_cast<long long>(std::llround(ns));
                        ready_flags |= SOAPY_SDR_HAS_TIME;
                    }

                    size_t elems_copied = 0;
                    size_t dma_index = 0;
                    size_t dma_elem_offset = 0;
                    while (elems_copied < total_batch_elems) {
                        RXStream::ReadyBuffer ready;
                        ready.handle = static_cast<int64_t>(_rx_stream.free_batch_handles.front());
                        _rx_stream.free_batch_handles.pop_front();
                        ready.flags = ready_flags;
                        ready.numElems = std::min(slice_elems, total_batch_elems - elems_copied);
                        if (ready_flags & SOAPY_SDR_HAS_TIME) {
                            const double ns =
                                (static_cast<double>(elems_copied) * 1e9) / _rx_stream.samplerate;
                            ready.timeNs = ready_time_ns + static_cast<long long>(std::llround(ns));
                        }

                        uint8_t *dst = reinterpret_cast<uint8_t *>(_rx_stream.batch_buf) +
                                       ready.handle * _rx_stream.batch_buf_size;
                        size_t dst_elems = 0;
                        while (dst_elems < ready.numElems) {
                            const int64_t dma_handle = dma_handles[dma_index];
                            const int buf_offset = dma_handle % _dma_mmap_info.dma_rx_buf_count;
                            const uint8_t *src_base = reinterpret_cast<const uint8_t *>(_rx_stream.buf) +
                                                     buf_offset * _dma_mmap_info.dma_rx_buf_size +
                                                     RX_DMA_HEADER_SIZE;
                            const size_t take_elems = std::min(ready.numElems - dst_elems, dma_mtu - dma_elem_offset);
                            const uint8_t *src = src_base + dma_elem_offset * bytes_per_elem;
                            std::memcpy(dst + dst_elems * bytes_per_elem, src, take_elems * bytes_per_elem);
                            dst_elems += take_elems;
                            dma_elem_offset += take_elems;
                            if (dma_elem_offset == dma_mtu) {
                                dma_elem_offset = 0;
                                dma_index++;
                            }
                        }

                        _rx_stream.pending_ready_buffs.push_back(ready);
                        elems_copied += ready.numElems;
                    }

                    const int64_t last_dma_handle = dma_handles.back();
                    if (last_dma_handle + 1 > _rx_stream.sw_count) {
                        struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
                        mmap_dma_update.sw_count = last_dma_handle + 1;
                        checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
                        _rx_stream.sw_count = last_dma_handle + 1;
                    }
                }
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
                _rx_stream.recv_cv.notify_all();
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
#if USE_LITEPCIE
    stopRxReceiveWorker();
    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
    _rx_stream.ready_buffs.clear();
    _rx_stream.pending_ready_buffs.clear();
    _rx_stream.pending_dma_handles.clear();
    _rx_stream.recv_error.clear();
    _rx_stream.recv_thread_stop = false;
    _rx_stream.recv_thread_running = true;
    _rx_stream.deliver_thread_running = true;
    _rx_stream.enqueue_count = 0;
    _rx_stream.overflow_lost_buffers = 0;
    _rx_stream.deliver_thread = std::thread(&SoapyLiteXM2SDR::rxDeliverLoop, this);
    configure_worker_thread(_rx_stream.deliver_thread, "m2sdr-rxd", _rx_stream.worker_rt_prio, _rx_stream.worker_cpu);
    _rx_stream.recv_thread = std::thread(&SoapyLiteXM2SDR::rxReceiveLoop, this);
    configure_worker_thread(_rx_stream.recv_thread, "m2sdr-rx", _rx_stream.worker_rt_prio, _rx_stream.worker_cpu);
#endif
}

void SoapyLiteXM2SDR::stopRxReceiveWorker()
{
#if USE_LITEPCIE
    {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        _rx_stream.recv_thread_stop = true;
    }
    _rx_stream.recv_cv.notify_all();
    if (_rx_stream.deliver_thread.joinable())
        _rx_stream.deliver_thread.join();
    if (_rx_stream.recv_thread.joinable())
        _rx_stream.recv_thread.join();
    {
        std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
        _rx_stream.ready_buffs.clear();
        _rx_stream.pending_ready_buffs.clear();
        _rx_stream.pending_dma_handles.clear();
        _rx_stream.free_batch_handles.clear();
        for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
            _rx_stream.free_batch_handles.push_back(i);
        _rx_stream.recv_thread_running = false;
        _rx_stream.deliver_thread_running = false;
    }
#endif
}

void SoapyLiteXM2SDR::checkTxSubmitError()
{
#if USE_LITEPCIE
    std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
    if (!_tx_stream.submit_error.empty()) {
        const std::string err = _tx_stream.submit_error;
        _tx_stream.submit_error.clear();
        throw std::runtime_error(err);
    }
#endif
}

void SoapyLiteXM2SDR::checkTxWriteError()
{
#if USE_LITEPCIE
    std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
    if (!_tx_stream.write_error.empty()) {
        const std::string err = _tx_stream.write_error;
        _tx_stream.write_error.clear();
        throw std::runtime_error(err);
    }
#endif
}

void SoapyLiteXM2SDR::txSubmitLoop()
{
#if USE_LITEPCIE
    for (;;) {
        size_t handle = 0;
        {
            std::unique_lock<std::mutex> lock(_tx_stream.submit_mutex);
            _tx_stream.submit_cv.wait(lock, [this]() {
                return _tx_stream.submit_thread_stop || !_tx_stream.pending_submit_handles.empty();
            });
            if (_tx_stream.submit_thread_stop && _tx_stream.pending_submit_handles.empty()) {
                _tx_stream.submit_thread_running = false;
                return;
            }
            handle = _tx_stream.pending_submit_handles.front();
            _tx_stream.pending_submit_handles.pop_front();
        }

        try {
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
        } catch (const std::exception &e) {
            std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
            if (_tx_stream.submit_error.empty())
                _tx_stream.submit_error = std::string("TX submit worker failed: ") + e.what();
        }
    }
#endif
}

void SoapyLiteXM2SDR::txWriteLoop()
{
#if USE_LITEPCIE
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
            if (_tx_stream.tx_queue_debug &&
                debug_log_allowed(_tx_stream.tx_queue_debug,
                                  _tx_stream.tx_queue_debug_seq,
                                  _tx_stream.tx_queue_debug_limit)) {
                std::fprintf(stderr, "TXQDBG worker-pre-submit-check q=%zu hw=%lld sw=%lld user=%lld\n",
                    _tx_stream.pending_write_jobs.size(),
                    (long long)_tx_stream.hw_count,
                    (long long)_tx_stream.sw_count,
                    (long long)_tx_stream.user_count);
            }
            checkTxSubmitError();
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
            const size_t bytes_per_elem = _nChannels * _bytesPerComplex;
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
            size_t sent_total = 0;
            while (sent_total < job.numElems) {
                {
                    std::lock_guard<std::mutex> lock(_tx_stream.write_mutex);
                    if (_tx_stream.write_thread_stop) {
                        _tx_stream.write_thread_running = false;
                        _tx_stream.write_cv.notify_all();
                        return;
                    }
                }

                size_t handle = 0;
                void *wr_buffs[4] = {};
                const auto acq_t0 = std::chrono::steady_clock::now();
                const int ret = this->acquireWriteBuffer(TX_STREAM, handle, wr_buffs, 1000);
                const long acq_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - acq_t0).count();
                if (ret == SOAPY_SDR_TIMEOUT) {
                    if (_tx_stream.tx_queue_debug &&
                        debug_log_allowed(_tx_stream.tx_queue_debug,
                                          _tx_stream.tx_queue_debug_seq,
                                          _tx_stream.tx_queue_debug_limit)) {
                        std::fprintf(stderr,
                            "TXQDBG acquire-timeout acq_us=%ld q=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                            acq_us,
                            _tx_stream.pending_write_jobs.size(),
                            (long long)_tx_stream.hw_count,
                            (long long)_tx_stream.sw_count,
                            (long long)_tx_stream.user_count,
                            (long long)(_tx_stream.user_count - _tx_stream.hw_count));
                    }
                    continue;
                }
                if (ret < 0) {
                    throw std::runtime_error("TX write worker acquireWriteBuffer failed: " + std::to_string(ret));
                }
                if (_tx_stream.tx_queue_debug &&
                    acq_us >= _tx_stream.tx_queue_debug_threshold_us &&
                    debug_log_allowed(_tx_stream.tx_queue_debug,
                                      _tx_stream.tx_queue_debug_seq,
                                      _tx_stream.tx_queue_debug_limit)) {
                    std::fprintf(stderr,
                        "TXQDBG acquire acq_us=%ld handle=%zu q=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                        acq_us,
                        handle,
                        _tx_stream.pending_write_jobs.size(),
                        (long long)_tx_stream.hw_count,
                        (long long)_tx_stream.sw_count,
                        (long long)_tx_stream.user_count,
                        (long long)(_tx_stream.user_count - _tx_stream.hw_count));
                }

                const size_t chunk = std::min(mtu, job.numElems - sent_total);
                std::memcpy(wr_buffs[0],
                            job.data.data() + sent_total * bytes_per_elem,
                            chunk * bytes_per_elem);

                int chunk_flags = 0;
                if (sent_total == 0)
                    chunk_flags |= (job.flags & SOAPY_SDR_HAS_TIME);
                if (sent_total + chunk >= job.numElems)
                    chunk_flags |= (job.flags & SOAPY_SDR_END_BURST);

                long long chunk_time_ns = job.timeNs;
                if (_tx_stream.samplerate > 0.0) {
                    const double ns =
                        (static_cast<double>(sent_total) * 1e9) / _tx_stream.samplerate;
                    chunk_time_ns += static_cast<long long>(std::llround(ns));
                }

                if (_tx_stream.tx_queue_debug &&
                    sent_total == 0 &&
                    debug_log_allowed(_tx_stream.tx_queue_debug,
                                      _tx_stream.tx_queue_debug_seq,
                                      _tx_stream.tx_queue_debug_limit)) {
                    const long long now_ns = steady_now_ns();
                    long long hw_now_ns = 0;
                    long long lead_us = 0;
                    bool hw_ok = false;
                    try {
                        hw_now_ns = this->getHardwareTime("");
                        lead_us = (chunk_time_ns - hw_now_ns) / 1000;
                        hw_ok = true;
                    } catch (...) {
                    }
                    std::fprintf(stderr,
                        "TXQDBG submit mono_ns=%lld queued_us=%ld acq_us=%ld handle=%zu numElems=%zu chunk=%zu "
                        "flags=0x%x lead_us=%lld hw_now_ns=%lld ts_ns=%lld q=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                        now_ns,
                        queued_us,
                        acq_us,
                        handle,
                        job.numElems,
                        chunk,
                        chunk_flags,
                        hw_ok ? lead_us : -1LL,
                        hw_ok ? hw_now_ns : -1LL,
                        chunk_time_ns,
                        _tx_stream.pending_write_jobs.size(),
                        (long long)_tx_stream.hw_count,
                        (long long)_tx_stream.sw_count,
                        (long long)_tx_stream.user_count,
                        (long long)(_tx_stream.user_count - _tx_stream.hw_count));
                }

                this->releaseWriteBuffer(TX_STREAM, handle, chunk, chunk_flags, chunk_time_ns);
                sent_total += chunk;
            }
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

void SoapyLiteXM2SDR::startTxSubmitWorker()
{
#if USE_LITEPCIE
    stopTxSubmitWorker();
    std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
    _tx_stream.pending_submit_handles.clear();
    _tx_stream.submit_error.clear();
    _tx_stream.submit_thread_stop = false;
    _tx_stream.submit_thread_running = true;
    if (_tx_stream.tx_queue_debug &&
        debug_log_allowed(_tx_stream.tx_queue_debug,
                          _tx_stream.tx_queue_debug_seq,
                          _tx_stream.tx_queue_debug_limit)) {
        std::fprintf(stderr, "TXQDBG submit-worker-start hw=%lld sw=%lld user=%lld\n",
            (long long)_tx_stream.hw_count,
            (long long)_tx_stream.sw_count,
            (long long)_tx_stream.user_count);
    }
    _tx_stream.submit_thread = std::thread(&SoapyLiteXM2SDR::txSubmitLoop, this);
    configure_worker_thread(_tx_stream.submit_thread, "m2sdr-tx", _tx_stream.worker_rt_prio, _tx_stream.worker_cpu);
#endif
}

void SoapyLiteXM2SDR::stopTxSubmitWorker()
{
#if USE_LITEPCIE
    {
        std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
        _tx_stream.submit_thread_stop = true;
    }
    _tx_stream.submit_cv.notify_all();
    if (_tx_stream.submit_thread.joinable())
        _tx_stream.submit_thread.join();
    {
        std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
        _tx_stream.pending_submit_handles.clear();
        _tx_stream.submit_thread_running = false;
    }
#endif
}

void SoapyLiteXM2SDR::startTxWriteWorker()
{
#if USE_LITEPCIE
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
#if USE_LITEPCIE
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
            throw std::runtime_error("DMA Writer/RX not available (litepcie_dma_init failed).");

        /* Get Buffer and Parameters from RX DMA Writer */
        _rx_stream.buf = _rx_stream.dma.buf_rd;
        _rx_buf_size   = _rx_stream.dma.mmap_dma_info.dma_rx_buf_size - RX_DMA_HEADER_SIZE;
        _rx_buf_count  = _rx_stream.dma.mmap_dma_info.dma_rx_buf_count;

        /* Ensure the DMA is disabled initially to avoid counters being in a bad state. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);

#elif USE_LITEETH
    /* Lazy-init UDP helper (enable RX+TX to mirror DMA symmetry). */
    if (!_udp_inited) {
        const std::string ip   = searchArgs.count("udp_ip")   ? searchArgs.at("udp_ip")
                                : (_deviceArgs.count("udp_ip")   ? _deviceArgs.at("udp_ip")   : "127.0.0.1");
        const bool is_vrt = (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT);
        const uint16_t port = is_vrt
            ? static_cast<uint16_t>(searchArgs.count("vrt_port")
                  ? std::stoul(searchArgs.at("vrt_port"))
                  : (_deviceArgs.count("vrt_port") ? std::stoul(_deviceArgs.at("vrt_port")) : 4991))
            : static_cast<uint16_t>(searchArgs.count("udp_port")
                  ? std::stoul(searchArgs.at("udp_port"))
                  : (_deviceArgs.count("udp_port") ? std::stoul(_deviceArgs.at("udp_port")) : 2345));

        const size_t buf_bytes = is_vrt
                                 ? VRT_RX_PACKET_BYTES_DEFAULT
                                 : ((searchArgs.count("udp_buf_complex")
                                      ? static_cast<size_t>(std::stoul(searchArgs.at("udp_buf_complex")))
                                      : 4096) * _bytesPerComplex * selected_channels.size());
        const size_t buf_count = 2;

        if (liteeth_udp_init(&_udp,
                             /*listen_ip*/  nullptr, /*listen_port*/  port,
                             /*remote_ip*/  ip.c_str(), /*remote_port*/ port,
                             /*rx_enable*/  1, /*tx_enable*/ is_vrt ? 0 : 1,
                             /*buffer_size*/ buf_bytes,
                             /*buffer_count*/buf_count,
                             /*nonblock*/    0) < 0) {
            throw std::runtime_error("UDP init failed.");
        }
        _udp_inited = true;
        SoapySDR::logf(SOAPY_SDR_INFO, "LiteEth %s init: remote=%s:%u",
                       is_vrt ? "VRT/UDP" : "UDP", ip.c_str(), port);
    } else {
        if (_nChannels && selected_channels.size() != _nChannels) {
            throw std::runtime_error("LiteEth UDP buffers already initialized with a different channel count");
        }
    }

    _rx_buf_size  = (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT)
                    ? VRT_RX_PAYLOAD_BYTES_DEFAULT
                    : _udp.buf_size;
    _rx_buf_count = _udp.buf_count;

    _rx_stream.buf = std::malloc(_rx_buf_size * _rx_buf_count);
    if (!_rx_stream.buf) {
        throw std::runtime_error("malloc() failed for RX staging buffer.");
    }
#endif

        _rx_stream.opened = true;
        _rx_stream.format = format;
        _rx_stream.remainderHandle = -1;
        _rx_stream.remainderSamps  = 0;
        _rx_stream.remainderOffset = 0;

        /* Log the selected RX channels for debugging */
        if (selected_channels.size() == 1) {
            SoapySDR_logf(SOAPY_SDR_INFO, "RX setupStream: Selected channel %zu", selected_channels[0]);
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "RX setupStream: Selected channels %zu, %zu",
                          selected_channels[0], selected_channels[1]);
        }

        _rx_stream.channels = selected_channels;
        _nChannels = _rx_stream.channels.size();
#if USE_LITEPCIE
        {
            size_t batch_buffers = std::max<size_t>(1, _rx_stream.batch_buffers);
            const size_t dma_mtu = _rx_buf_size / (_nChannels * _bytesPerComplex);
            size_t slice_elems = _rx_stream.slice_elems;
            if (!_rx_stream.batch_buffers_explicit && _rx_stream.samplerate > 0.0 && dma_mtu > 0.0) {
                const double target_samples = 0.0001 * _rx_stream.samplerate; /* ~100 us like LimeSuiteNG */
                const size_t auto_buffers = std::max<size_t>(
                    1, static_cast<size_t>(std::floor(target_samples / static_cast<double>(dma_mtu))));
                batch_buffers = std::clamp<size_t>(auto_buffers, 1, 4);
                _rx_stream.batch_buffers = batch_buffers;
            }
            if (!_rx_stream.slice_elems_explicit) {
                slice_elems = std::max<size_t>(1, std::min<size_t>(dma_mtu, 480));
                _rx_stream.slice_elems = slice_elems;
            } else {
                slice_elems = std::max<size_t>(1, std::min<size_t>(slice_elems, batch_buffers * dma_mtu));
            }
            const size_t slice_stride = slice_elems * _nChannels * _bytesPerComplex;
            const size_t slice_count = std::max<size_t>(1, (batch_buffers * dma_mtu + slice_elems - 1) / slice_elems);
            // Size the staged queue from the requested latency budget rather than mirroring the full DMA ring.
            // Mirroring the ring creates tens of milliseconds of elasticity, which lets the consumer fall behind
            // and then catch up in bursts. Keep enough buffers for at least two full batches plus a small floor.
            size_t batch_count = std::max<size_t>(32, slice_count * 2);
            if (_rx_stream.samplerate > 0.0) {
                const double batch_ms = (1000.0 * static_cast<double>(slice_elems)) /
                                        _rx_stream.samplerate;
                if (batch_ms > 0.0) {
                    const size_t queue_batches = static_cast<size_t>(
                        std::ceil(_rx_stream.batch_queue_ms / batch_ms));
                    batch_count = std::max(batch_count, std::max<size_t>(32, queue_batches));
                }
            }
            _rx_stream.batch_buf = std::malloc(slice_stride * batch_count);
            if (!_rx_stream.batch_buf) {
                throw std::runtime_error("malloc() failed for RX batch staging buffer.");
            }
            _rx_stream.batch_buf_size = slice_stride;
            _rx_stream.batch_buf_count = batch_count;
            _rx_stream.free_batch_handles.clear();
            for (size_t i = 0; i < batch_count; ++i)
                _rx_stream.free_batch_handles.push_back(i);
            SoapySDR_logf(SOAPY_SDR_INFO,
                          "RX staging batches=%zu slice_stride=%zu bytes slice_elems=%zu (~%.1f ms total)",
                          batch_count,
                          slice_stride,
                          slice_elems,
                          (_rx_stream.samplerate > 0.0)
                              ? (1000.0 * (static_cast<double>(batch_count * slice_elems)) /
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
#elif USE_LITEETH
    if (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT) {
        throw std::runtime_error("Soapy TX streaming is not supported in eth_mode=vrt");
    }
    if (!_udp_inited) {
        const std::string ip   = searchArgs.count("udp_ip")   ? searchArgs.at("udp_ip")
                                : (_deviceArgs.count("udp_ip")   ? _deviceArgs.at("udp_ip")   : "127.0.0.1");
        const uint16_t    port = searchArgs.count("udp_port") ? static_cast<uint16_t>(std::stoul(searchArgs.at("udp_port")))
                                : (_deviceArgs.count("udp_port") ? static_cast<uint16_t>(std::stoul(_deviceArgs.at("udp_port"))) : 2345);

        const size_t buf_complex = searchArgs.count("udp_buf_complex")
                                   ? static_cast<size_t>(std::stoul(searchArgs.at("udp_buf_complex")))
                                   : 4096;

        const size_t chs       = selected_channels.size();
        const size_t buf_bytes = buf_complex * _bytesPerComplex * chs;
        const size_t buf_count = 2;

        if (liteeth_udp_init(&_udp,
                             /*listen_ip*/  nullptr, /*listen_port*/  port,
                             /*remote_ip*/  ip.c_str(), /*remote_port*/ port,
                             /*rx_enable*/  1, /*tx_enable*/ 1,
                             /*buffer_size*/ buf_bytes,
                             /*buffer_count*/buf_count,
                             /*nonblock*/    0) < 0) {
            throw std::runtime_error("UDP init failed.");
        }
        _udp_inited = true;
        SoapySDR::logf(SOAPY_SDR_INFO, "LiteEth UDP init: remote=%s:%u", ip.c_str(), port);
    } else {
        if (_nChannels && selected_channels.size() != _nChannels) {
            throw std::runtime_error("LiteEth UDP buffers already initialized with a different channel count");
        }
    }

    _tx_buf_size  = _udp.buf_size;
    _tx_buf_count = _udp.buf_count;

    _tx_stream.buf = std::malloc(_tx_buf_size * _tx_buf_count);
    if (!_tx_stream.buf) {
        throw std::runtime_error("malloc() failed for TX staging buffer.");
    }
#endif

        _tx_stream.opened = true;
        _tx_stream.format = format;
        _tx_stream.remainderHandle = -1;
        _tx_stream.remainderSamps  = 0;
        _tx_stream.remainderOffset = 0;

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

    /* AD9361 Port Control 2t2r timing enable */
    struct ad9361_phy_platform_data *pd = ad9361_phy->pdata;
    pd->port_ctrl.pp_conf[0] &= ~(1 << 2);
    if (_nChannels == 2)
        pd->port_ctrl.pp_conf[0] |= (1 << 2);

    ad9361_set_no_ch_mode(ad9361_phy, _nChannels);

    return direction == SOAPY_SDR_RX ? RX_STREAM : TX_STREAM;
}

/* Close the specified stream and release associated resources. */
void SoapyLiteXM2SDR::closeStream(SoapySDR::Stream *stream) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (stream == RX_STREAM) {
#if USE_LITEPCIE
        stopRxReceiveWorker();
        litepcie_dma_cleanup(&_rx_stream.dma);
        _rx_stream.buf = NULL;
        std::free(_rx_stream.batch_buf);
        _rx_stream.batch_buf = NULL;
        _rx_stream.batch_buf_size = 0;
        _rx_stream.batch_buf_count = 0;
#elif USE_LITEETH
        std::free(_rx_stream.buf);
        _rx_stream.buf = NULL;
#endif
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
#if USE_LITEPCIE
        stopTxSubmitWorker();
#endif
#if USE_LITEPCIE
        litepcie_dma_cleanup(&_tx_stream.dma);
        _tx_stream.buf = NULL;
#elif USE_LITEETH
        std::free(_tx_stream.buf);
        _tx_stream.buf = NULL;
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
#elif USE_LITEETH
        /* Crossbar Demux: Select Ethernet streaming */
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_DEMUX_SEL_ADDR, 1);
#ifdef CSR_ETH_RX_MODE_ADDR
        litex_m2sdr_writel(_fd, CSR_ETH_RX_MODE_ADDR,
                           (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT) ? 2 : 1);
#endif
        /* UDP helper is ready; nothing to start explicitly. */
#endif
        _rx_stream.user_count = 0;
        _rx_stream.burst_end = false;
        _rx_stream.time0_ns = this->getHardwareTime("");
        _rx_stream.time0_count = _rx_stream.user_count;
        _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
        _rx_stream.timed_start_pending = false;
        _rx_stream.start_time_ns = 0;
        _rx_stream.last_time_ns = _rx_stream.time0_ns;
        _rx_stream.time_warned = false;
        _rx_stream.dma_writer_started = false;
#if USE_LITEPCIE
        {
            std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
            _rx_stream.ready_buffs.clear();
            _rx_stream.pending_dma_handles.clear();
            _rx_stream.free_batch_handles.clear();
            for (size_t i = 0; i < _rx_stream.batch_buf_count; ++i)
                _rx_stream.free_batch_handles.push_back(i);
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
#if USE_LITEPCIE
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
#elif USE_LITEETH
        /* Crossbar Mux: Select Ethernet streaming */
        litex_m2sdr_writel(_fd, CSR_CROSSBAR_MUX_SEL_ADDR, 1);
        /* No explicit start; pacing handled by client cadence if needed. */
        _tx_stream.user_count = 0;
#endif
        _tx_stream.pendingWriteBufs.clear();
        _tx_stream.burst_end = false;
        _tx_stream.dma_started = false;
        _tx_stream.tx_debug_seq = 0;
        _tx_stream.tx_submit_seq = 0;
        startTxSubmitWorker();
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
#if USE_LITEPCIE
        stopRxReceiveWorker();
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
#elif USE_LITEETH
        /* Flush/disable Ethernet RX branch when available. */
#ifdef CSR_ETH_RX_MODE_ADDR
        litex_m2sdr_writel(_fd, CSR_ETH_RX_MODE_ADDR, 0);
#endif
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
        stopTxSubmitWorker();
        /* Disable the DMA engine for TX. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
#elif USE_LITEETH
        /* No-op for UDP helper. */
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
        /* Each sample is 2 * Complex{Int16}. */
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
#if USE_LITEETH
    return _tx_buf_count;
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
        if (_rx_stream.batch_buf) {
            buffs[0] = (char *)_rx_stream.batch_buf + handle * _rx_stream.batch_buf_size;
        } else {
            buffs[0] = (char *)_rx_stream.buf + handle * _dma_mmap_info.dma_rx_buf_size + RX_DMA_HEADER_SIZE;
        }
#else
        buffs[0] = (char *)_rx_stream.buf + handle * _rx_buf_size;
#endif
    } else if (stream == TX_STREAM) {
#if USE_LITEPCIE
        buffs[0] = (char *)_tx_stream.buf + handle * _dma_mmap_info.dma_tx_buf_size + TX_DMA_HEADER_SIZE;
#else
        buffs[0] = (char *)_tx_stream.buf + handle * _tx_buf_size;
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

static constexpr uint64_t DMA_HEADER_SYNC_WORD = 0x5aa55aa55aa55aa5ULL;

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

#if USE_LITEETH
    /* Pump UDP helper once with caller timeout (ms). */
    liteeth_udp_process(&_udp, static_cast<int>(timeoutUs / 1000));

    int avail = liteeth_udp_buffers_available_read(&_udp);
    if (avail <= 0) {
        return SOAPY_SDR_TIMEOUT;
    }
    /* Drop older buffers under load to keep the most recent samples. */
    while (avail > 1) {
        (void)liteeth_udp_next_read_buffer(&_udp);
        avail--;
    }

    uint8_t *src = liteeth_udp_next_read_buffer(&_udp);
    if (!src) {
        return SOAPY_SDR_TIMEOUT;
    }

    if (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT) {
        if (_udp.buf_size < VRT_SIGNAL_HEADER_BYTES) {
            return SOAPY_SDR_STREAM_ERROR;
        }
        const uint32_t common = read_be32(src + 0);
        const uint32_t packet_type = (common >> 28) & 0xF;
        const uint32_t packet_words = (common & 0xFFFF);
        if (packet_type != 0x1 || packet_words < 5) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "Invalid/unsupported VRT RX packet (type=%u words=%u)",
                          packet_type, packet_words);
            return SOAPY_SDR_STREAM_ERROR;
        }
        size_t payload_bytes = static_cast<size_t>(packet_words - 5) * sizeof(uint32_t);
        if (payload_bytes > (_udp.buf_size - VRT_SIGNAL_HEADER_BYTES))
            payload_bytes = _udp.buf_size - VRT_SIGNAL_HEADER_BYTES;
        if (payload_bytes > _rx_buf_size)
            payload_bytes = _rx_buf_size;

        const uint32_t tsi_type = (common >> 22) & 0x3;
        const uint32_t tsf_type = (common >> 20) & 0x3;
        if (tsi_type != 0 || tsf_type != 0) {
            const uint64_t tsi = read_be32(src + 8);
            const uint64_t tsf = read_be64(src + 12);
            timeNs = static_cast<long long>(tsi * 1000000000ULL + (tsf / 1000ULL));
            flags |= SOAPY_SDR_HAS_TIME;
        }

        std::memcpy(_rx_stream.buf, src + VRT_SIGNAL_HEADER_BYTES, payload_bytes);
        buffs[0] = _rx_stream.buf;
        handle   = 0;
        return static_cast<int>(payload_bytes / (_nChannels * _bytesPerComplex));
    }

    /* Stage into RX buffer; remainder/deinterleave pipeline expects a flat buffer. */
    std::memcpy(_rx_stream.buf, src, _rx_buf_size);
    buffs[0] = _rx_stream.buf;
    handle   = 0; /* dummy for LiteEth path */
    return getStreamMTU(stream);
#elif USE_LITEPCIE
    const auto acquire_start = std::chrono::steady_clock::now();
    checkRxReceiveError();
    const auto wait_for_due_time = [&](const long long due_time_ns) -> int {
        if (!_rx_stream.wallclock_pacing || !(_rx_stream.time_valid && due_time_ns > 0))
            return 0;
        const bool wait_forever = (timeoutUs < 0);
        const auto deadline = wait_forever
            ? std::chrono::steady_clock::time_point::max()
            : (acquire_start + std::chrono::microseconds(timeoutUs));
        while (true) {
            const long long now_ns = this->getHardwareTime("");
            const long long wait_ns = due_time_ns - now_ns - _rx_stream.pacing_margin_ns;
            if (wait_ns <= 0)
                return 0;
            if (!wait_forever && std::chrono::steady_clock::now() >= deadline)
                return SOAPY_SDR_TIMEOUT;
            const long long sleep_ns = std::min<long long>(wait_ns, 100000LL);
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
    };
    RXStream::ReadyBuffer ready;
    size_t queue_before = 0;
    {
        std::unique_lock<std::mutex> lock(_rx_stream.recv_mutex);
        queue_before = _rx_stream.ready_buffs.size();
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
            if (timeoutUs == 0)
                return SOAPY_SDR_TIMEOUT;

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
        ready = _rx_stream.ready_buffs.front();
        _rx_stream.ready_buffs.pop_front();
        queue_before = _rx_stream.ready_buffs.size();
    }

    handle = static_cast<size_t>(ready.handle);
    _rx_stream.user_count++;
    flags |= ready.flags;
    timeNs = ready.timeNs;

    /* Get the buffer. */
    {
        getDirectAccessBufferAddrs(stream, handle, (void **)buffs);
        const size_t samples_per_buffer = ready.numElems ? ready.numElems : getStreamMTU(stream);
        if (_rx_stream.time_valid && !(flags & SOAPY_SDR_HAS_TIME)) {
            timeNs = _rx_stream.time0_ns +
                     samples_to_ns(_rx_stream.samplerate,
                                   static_cast<long long>(_rx_stream.user_count - _rx_stream.time0_count) *
                                   static_cast<long long>(samples_per_buffer));
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
        const int pace_ret = wait_for_due_time(timeNs);
        if (pace_ret != 0) {
            releaseReadBuffer(stream, handle);
            return pace_ret;
        }
        if (_rx_stream.rx_debug) {
            const auto acquire_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - acquire_start).count();
            if (acquire_us >= _rx_stream.rx_debug_threshold_us &&
                debug_log_allowed(_rx_stream.rx_debug, _rx_stream.rx_debug_seq, _rx_stream.rx_debug_limit)) {
                std::fprintf(stderr,
                    "RXDBG acquire dt=%lldus handle=%lld flags=0x%x q_after=%zu hw=%lld sw=%lld user=%lld\n",
                    (long long)acquire_us,
                    (long long)ready.handle,
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
    std::lock_guard<std::mutex> lock(_rx_stream.recv_mutex);
    _rx_stream.free_batch_handles.push_back(handle);
    _rx_stream.recv_cv.notify_all();
#elif USE_LITEETH
    (void)handle; /* UDP slot was advanced by next_read_buffer(). */
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
    checkTxSubmitError();
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
        long waited_us = 0;
        while (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count)) {
            const bool wait_forever = (timeoutUs < 0);
            const long remaining_us = wait_forever ? 1000 : (timeoutUs - waited_us);
            if (!wait_forever && remaining_us <= 0)
                return SOAPY_SDR_TIMEOUT;
            const int ret = poll(&_tx_stream.fds, 1, poll_timeout_ms_slice(remaining_us));
            if (ret < 0) {
                throw std::runtime_error("SoapyLiteXM2SDR::acquireWriteBuffer(): Poll failed, " +
                                         std::string(strerror(errno)) + ".");
            } else if (ret == 0) {
                if (!wait_forever)
                    waited_us += 1000;
                checkTxSubmitError();
                continue;
            }

            litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
            buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
            if (buffers_pending < 0)
                buffers_pending = 0;
            checkTxSubmitError();
        }
    }

    /* Get the buffer. */
    int buf_offset = _tx_stream.user_count % _dma_mmap_info.dma_tx_buf_count;
    getDirectAccessBufferAddrs(stream, buf_offset, buffs);

    /* Update the DMA counters. */
    handle = _tx_stream.user_count;
    _tx_stream.user_count++;
    return getStreamMTU(stream);
#elif USE_LITEETH
    uint8_t *dst = liteeth_udp_next_write_buffer(&_udp);
    if (!dst) {
        return SOAPY_SDR_TIMEOUT;
    }
    buffs[0] = dst;
    handle   = _tx_stream.user_count++;
    _tx_stream.pendingWriteBufs[handle] = dst;
    (void)timeoutUs;
    return getStreamMTU(stream);
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
    checkTxSubmitError();
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
#if USE_LITEPCIE
        const size_t buf_offset = handle % _dma_mmap_info.dma_tx_buf_count;
        buf = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
              (buf_offset * _dma_mmap_info.dma_tx_buf_size) + TX_DMA_HEADER_SIZE;
#elif USE_LITEETH
        auto it = _tx_stream.pendingWriteBufs.find(handle);
        if (it != _tx_stream.pendingWriteBufs.end()) {
            buf = it->second;
            _tx_stream.pendingWriteBufs.erase(it);
        } else {
            buf = reinterpret_cast<uint8_t*>(_tx_stream.remainderBuff);
        }
#endif
        if (buf) {
            const size_t offset_bytes = numElems * _nChannels * _bytesPerComplex;
            const size_t zero_bytes = (mtu - numElems) * _nChannels * _bytesPerComplex;
            std::memset(buf + offset_bytes, 0, zero_bytes);
        }
    }

#if USE_LITEPCIE
    {
        std::lock_guard<std::mutex> lock(_tx_stream.submit_mutex);
        _tx_stream.pending_submit_handles.push_back(handle);
    }
    _tx_stream.submit_cv.notify_one();
#elif USE_LITEETH
    if (numElems >= mtu) {
        auto it = _tx_stream.pendingWriteBufs.find(handle);
        if (it != _tx_stream.pendingWriteBufs.end()) {
            _tx_stream.pendingWriteBufs.erase(it);
        }
    }
    if (liteeth_udp_write_submit(&_udp) < 0) {
        _tx_stream.underflow = true;
        SoapySDR_logf(SOAPY_SDR_ERROR, "UDP write_submit failed.");
    }
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

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous read, process that first. */
    if (_rx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_rx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _rx_stream.remainderOffset *  _nChannels *_bytesPerComplex;

        if (n < returnedElems) {
            samp_avail = n;
        }

        if (_rx_stream.time_valid) {
            timeNs = _rx_stream.remainderTimeNs +
                     samples_to_ns(_rx_stream.samplerate, _rx_stream.remainderOffset);
            flags |= SOAPY_SDR_HAS_TIME;
            if (timeNs < _rx_stream.last_time_ns) {
                timeNs = _rx_stream.last_time_ns;
            } else {
                _rx_stream.last_time_ns = timeNs;
            }
        } else if (_rx_stream.samplerate <= 0.0 && !_rx_stream.time_warned) {
            SoapySDR::log(SOAPY_SDR_WARNING,
                "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
            _rx_stream.time_warned = true;
        }

        /* Read out channels from the remainder buffer. */
        for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
            const uint32_t chan = _rx_stream.channels[i];
            this->deinterleave(
                _rx_stream.remainderBuff + (remainderOffset + chan * _bytesPerComplex),
                buffs[i],
                n,
                _rx_stream.format,
                0
            );
        }
        _rx_stream.remainderSamps -= n;
        _rx_stream.remainderOffset += n;

        if (_rx_stream.remainderSamps == 0) {
            this->releaseReadBuffer(stream, _rx_stream.remainderHandle);
            _rx_stream.remainderHandle = -1;
            _rx_stream.remainderOffset = 0;
        }

        if (n == returnedElems) {
            return returnedElems;
        }
    }

    /* Acquire a new read buffer from the DMA engine / UDP helper. */
    size_t handle;
    int ret = this->acquireReadBuffer(
        stream,
        handle,
        (const void **)&_rx_stream.remainderBuff,
        flags,
        timeNs,
        timeoutUs);

    if (ret < 0) {
        if ((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
            return samp_avail;
        }
        return ret;
    }

    _rx_stream.remainderHandle = handle;
    _rx_stream.remainderSamps = ret;
    _rx_stream.remainderTimeNs = timeNs;

    const size_t n = std::min((returnedElems - samp_avail), _rx_stream.remainderSamps);

    if (_rx_stream.time_valid) {
        timeNs = _rx_stream.remainderTimeNs +
                 samples_to_ns(_rx_stream.samplerate, _rx_stream.remainderOffset);
        flags |= SOAPY_SDR_HAS_TIME;
        if (timeNs < _rx_stream.last_time_ns) {
            timeNs = _rx_stream.last_time_ns;
        } else {
            _rx_stream.last_time_ns = timeNs;
        }
    } else if (_rx_stream.samplerate <= 0.0 && !_rx_stream.time_warned) {
        SoapySDR::log(SOAPY_SDR_WARNING,
            "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
        _rx_stream.time_warned = true;
    }

    /* Read out channels from the new buffer. */
    for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
        const uint32_t chan = _rx_stream.channels[i];
        this->deinterleave(
            _rx_stream.remainderBuff + (chan * _bytesPerComplex),
            buffs[i],
            n,
            _rx_stream.format,
            samp_avail
        );
    }
    _rx_stream.remainderSamps -= n;
    _rx_stream.remainderOffset += n;

    if (_rx_stream.remainderSamps == 0) {
        this->releaseReadBuffer(stream, _rx_stream.remainderHandle);
        _rx_stream.remainderHandle = -1;
        _rx_stream.remainderOffset = 0;
    }

    return returnedElems;
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

#if USE_LITEPCIE
    checkTxWriteError();
    checkTxSubmitError();

    TXStream::WriteJob job;
    job.numElems = numElems;
    job.flags = flags;
    job.timeNs = timeNs;
    job.enqueue_tp = std::chrono::steady_clock::now();
    job.data.resize(numElems * _nChannels * _bytesPerComplex, 0);

    for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
        this->interleave(
            buffs[i],
            job.data.data() + (_tx_stream.channels[i] * _bytesPerComplex),
            numElems,
            _tx_stream.format,
            0
        );
    }

    std::unique_lock<std::mutex> lock(_tx_stream.write_mutex);
    auto can_enqueue = [this]() {
        return _tx_stream.write_thread_stop ||
               (_tx_stream.pending_write_jobs.size() < _tx_stream.write_queue_depth);
    };
    if (timeoutUs == 0) {
        if (!can_enqueue()) {
            if (_tx_stream.tx_queue_debug &&
                debug_log_allowed(_tx_stream.tx_queue_debug,
                                  _tx_stream.tx_queue_debug_seq,
                                  _tx_stream.tx_queue_debug_limit)) {
                std::fprintf(stderr,
                    "TXQDBG enqueue-timeout timeoutUs=%ld q=%zu depth=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                    timeoutUs,
                    _tx_stream.pending_write_jobs.size(),
                    _tx_stream.write_queue_depth,
                    (long long)_tx_stream.hw_count,
                    (long long)_tx_stream.sw_count,
                    (long long)_tx_stream.user_count,
                    (long long)(_tx_stream.user_count - _tx_stream.hw_count));
            }
            return SOAPY_SDR_TIMEOUT;
        }
    } else if (timeoutUs < 0) {
        _tx_stream.write_cv.wait(lock, can_enqueue);
    } else {
        if (!_tx_stream.write_cv.wait_for(lock, std::chrono::microseconds(timeoutUs), can_enqueue)) {
            if (_tx_stream.tx_queue_debug &&
                debug_log_allowed(_tx_stream.tx_queue_debug,
                                  _tx_stream.tx_queue_debug_seq,
                                  _tx_stream.tx_queue_debug_limit)) {
                std::fprintf(stderr,
                    "TXQDBG enqueue-wait-timeout timeoutUs=%ld q=%zu depth=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
                    timeoutUs,
                    _tx_stream.pending_write_jobs.size(),
                    _tx_stream.write_queue_depth,
                    (long long)_tx_stream.hw_count,
                    (long long)_tx_stream.sw_count,
                    (long long)_tx_stream.user_count,
                    (long long)(_tx_stream.user_count - _tx_stream.hw_count));
            }
            return SOAPY_SDR_TIMEOUT;
        }
    }

    if (_tx_stream.write_thread_stop)
        return SOAPY_SDR_STREAM_ERROR;

    _tx_stream.pending_write_jobs.push_back(std::move(job));
    if (_tx_stream.tx_queue_debug &&
        debug_log_allowed(_tx_stream.tx_queue_debug,
                          _tx_stream.tx_queue_debug_seq,
                          _tx_stream.tx_queue_debug_limit)) {
        const long long now_ns = steady_now_ns();
        std::fprintf(stderr,
            "TXQDBG enqueue mono_ns=%lld numElems=%zu flags=0x%x q=%zu depth=%zu hw=%lld sw=%lld user=%lld pending=%lld\n",
            now_ns,
            numElems,
            flags,
            _tx_stream.pending_write_jobs.size(),
            _tx_stream.write_queue_depth,
            (long long)_tx_stream.hw_count,
            (long long)_tx_stream.sw_count,
            (long long)_tx_stream.user_count,
            (long long)(_tx_stream.user_count - _tx_stream.hw_count));
    }
    lock.unlock();
    _tx_stream.write_cv.notify_one();
    return static_cast<int>(numElems);
#else

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous write, process that first. */
    if (_tx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_tx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _tx_stream.remainderOffset * _nChannels * _bytesPerComplex;

        if (n < returnedElems) {
            samp_avail = n;
        }

        /* Write out channels to the remainder buffer. */
        for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
            this->interleave(
                buffs[i],
                _tx_stream.remainderBuff + remainderOffset + (_tx_stream.channels[i] * _bytesPerComplex),
                n,
                _tx_stream.format,
                0
            );
        }
        /* UDP path: no mid-buffer send; submission happens in releaseWriteBuffer(). */
        _tx_stream.remainderSamps -= n;
        _tx_stream.remainderOffset += n;

        if (_tx_stream.remainderSamps == 0) {
            this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset, flags, timeNs);
            _tx_stream.remainderHandle = -1;
            _tx_stream.remainderOffset = 0;
        }

        if (n == returnedElems) {
            return returnedElems;
        }
    }

    /* Acquire a new write buffer from the DMA engine / UDP helper. */
    size_t handle;

    int ret = this->acquireWriteBuffer(
        stream,
        handle,
        (void **)&_tx_stream.remainderBuff,
        timeoutUs);
    if (ret < 0) {
        if ((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
            return samp_avail;
        }
        return ret;
    }

    _tx_stream.remainderHandle = handle;
    _tx_stream.remainderSamps = ret;

    const size_t n = std::min((returnedElems - samp_avail), _tx_stream.remainderSamps);

    /* Write out channels to the new buffer. */
    for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
        this->interleave(
            buffs[i],
            _tx_stream.remainderBuff + (_tx_stream.channels[i] * _bytesPerComplex),
            n,
            _tx_stream.format,
            samp_avail
        );
    }
    /* UDP path: no mid-buffer send; submission happens in releaseWriteBuffer(). */
    _tx_stream.remainderSamps -= n;
    _tx_stream.remainderOffset += n;

    if (_tx_stream.remainderSamps == 0) {
        this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset, flags, timeNs);
        _tx_stream.remainderHandle = -1;
        _tx_stream.remainderOffset = 0;
    }

    return returnedElems;
#endif
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
