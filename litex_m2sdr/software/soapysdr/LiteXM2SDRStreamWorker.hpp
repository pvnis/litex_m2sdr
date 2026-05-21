/*
 * SoapySDR driver for the LiteX M2SDR.
 *
 * Worker-thread streaming primitives:
 *   - PacketsFIFO<T>: SPSC lock-free queue with bounded-wait push/pop,
 *     adapted from LimeSuiteNG (Apache-2.0).
 *   - StreamPacket: DMA-buffer-sized packet carrying a fixed sample
 *     payload plus metadata exchanged between the SoapySDR caller and
 *     the per-direction worker thread.
 *   - Thread-priority helper modelled on Lime's threadHelper.
 *
 * Copyright (c) 2021-2026 Enjoy Digital.
 * Adapted from LimeSuiteNG: Copyright (c) Lime Microsystems.
 * SPDX-License-Identifier: Apache-2.0
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

#ifdef __unix__
#include <pthread.h>
#include <sched.h>
#endif

namespace litex_m2sdr {

/* Packet pool depth per direction. 16 packets at ~8 KB each is ~128 KB
 * per stream, which decouples the SoapySDR caller from the DMA pump
 * over typical Linux scheduling jitter (~1 ms) at the sample rates this
 * board supports without burning meaningful memory.
 */
constexpr size_t kWorkerPktCount = 16;

enum class ThreadPriority : uint8_t {
    LOWEST,
    LOW,
    BELOW_NORMAL,
    NORMAL,
    ABOVE_NORMAL,
    HIGH,
    HIGHEST,
};

enum class ThreadPolicy : uint8_t {
    DEFAULT,
    REALTIME,    /* SCHED_RR */
    PREEMPTIVE,  /* SCHED_FIFO */
};

inline int setOsThreadPriority(ThreadPriority priority, ThreadPolicy policy, std::thread *t) {
#ifdef __unix__
    if (!t) return -1;
    int sched_policy = SCHED_OTHER;
    switch (policy) {
        case ThreadPolicy::DEFAULT:    sched_policy = SCHED_OTHER; break;
        case ThreadPolicy::REALTIME:   sched_policy = SCHED_RR;    break;
        case ThreadPolicy::PREEMPTIVE: sched_policy = SCHED_FIFO;  break;
    }
    const int prio_min = sched_get_priority_min(sched_policy);
    const int prio_max = sched_get_priority_max(sched_policy);
    if (prio_min < 0 || prio_max < 0) return -1;
    sched_param sch{};
    sch.sched_priority = prio_min +
        static_cast<int>(((prio_max - prio_min) /
                          static_cast<float>(ThreadPriority::HIGHEST)) *
                         static_cast<int>(priority));
    return pthread_setschedparam(t->native_handle(), sched_policy, &sch);
#else
    (void)priority; (void)policy; (void)t;
    return 0;
#endif
}

/* Single-producer single-consumer bounded queue.
 *
 * Adapted from LimeSuiteNG/src/streaming/PacketsFIFO.h.
 * One thread may push, another may pop; concurrent multi-producer or
 * multi-consumer use is not supported.
 */
template <class T>
class PacketsFIFO {
public:
    explicit PacketsFIFO(std::size_t fixed_size)
        : _ring_size(fixed_size + 1), _ring(fixed_size + 1) {}

    PacketsFIFO(const PacketsFIFO&) = delete;
    PacketsFIFO& operator=(const PacketsFIFO&) = delete;

    bool empty() const noexcept {
        return _read.load() == _write.load();
    }

    bool full() const noexcept {
        return next(_write.load()) == _read.load();
    }

    std::size_t max_size() const noexcept { return _ring_size - 1; }

    std::size_t size() const noexcept {
        const std::size_t r = _read.load();
        const std::size_t w = _write.load();
        if (r == w) return 0;
        if (w > r) return w - r;
        return _ring_size - r + w;
    }

    bool push(const T &value,
              bool wait = false,
              std::chrono::microseconds timeout = std::chrono::microseconds(250000)) {
        std::unique_lock<std::mutex> lk(_mu);
        const std::size_t w = _write.load();
        const std::size_t w_next = next(w);
        if (w_next == _read.load()) {
            if (!wait) return false;
            if (_can_write.wait_for(lk, timeout) == std::cv_status::timeout)
                return false;
            if (w_next == _read.load()) return false;
        }
        _ring[w] = value;
        _write.store(w_next);
        _can_read.notify_one();
        return true;
    }

    bool pop(T *out,
             bool wait = false,
             std::chrono::microseconds timeout = std::chrono::microseconds(250000)) {
        std::unique_lock<std::mutex> lk(_mu);
        if (empty()) {
            if (!wait) return false;
            if (_can_read.wait_for(lk, timeout) == std::cv_status::timeout)
                return false;
            if (empty()) return false;
        }
        const std::size_t r = _read.load();
        *out = _ring[r];
        _read.store(next(r));
        _can_write.notify_one();
        return true;
    }

    /* Best-effort drain; preserves single-consumer invariant. */
    void clear() noexcept {
        _read.store(_write.load());
        _can_write.notify_all();
    }

    /* Wake any thread blocked in wait_for(); used at shutdown. */
    void wakeAll() noexcept {
        _can_read.notify_all();
        _can_write.notify_all();
    }

private:
    std::size_t next(std::size_t pos) const noexcept {
        return (pos + 1 == _ring_size) ? 0 : pos + 1;
    }

    const std::size_t _ring_size;
    std::vector<T> _ring;
    std::atomic<std::size_t> _read{0};
    std::atomic<std::size_t> _write{0};
    std::mutex _mu;
    std::condition_variable _can_read;
    std::condition_variable _can_write;
};

/* StreamPacket: fixed-capacity sample container exchanged between the
 * SoapySDR caller and the per-direction worker thread.
 *
 * Ownership model:
 *   - All packets live in a single pool allocated at activateStream and
 *     freed at closeStream.
 *   - For RX the worker fills `data` from a DMA buffer and pushes the
 *     packet to `work` for the caller. The caller pushes it back to
 *     `free` when done.
 *   - For TX the caller fills `data` and pushes to `work`; the worker
 *     consumes, copies to the DMA reader buffer, then pushes back to
 *     `free`.
 */
struct StreamPacket {
    uint8_t  *data     = nullptr;  /* heap-allocated to capacity bytes        */
    size_t    capacity = 0;        /* allocated bytes                         */
    uint32_t  samples  = 0;        /* samples per channel in this packet      */
    int       flags    = 0;        /* SoapySDR flags                          */
    int       ret_code = 0;        /* 0=ok, <0=SOAPY_SDR_* (OVERFLOW etc.)    */
    long long timeNs   = 0;        /* sample-aligned timestamp                */
    size_t    index    = 0;        /* stable index into the pool              */
};

} // namespace litex_m2sdr
