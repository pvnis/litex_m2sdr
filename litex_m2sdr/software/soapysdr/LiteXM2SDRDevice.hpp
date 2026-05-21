/*
 * SoapySDR driver for the LiteX M2SDR.
 *
 * Copyright (c) 2021-2026 Enjoy Digital.
 * SPDX-License-Identifier: Apache-2.0
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <memory>
#include <map>
#include <thread>
#include <vector>
#include <string>
#include <cstdint>

#include "liblitepcie.h"
#include "etherbone.h"
#include "libm2sdr.h"
#include "m2sdr.h"
#include "LiteXM2SDRStreamWorker.hpp"

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>

#if USE_LITEETH
extern "C" {
#include "liteeth_udp.h"
}

enum class SoapyLiteXM2SDREthernetMode {
    UDP = 0,
    VRT = 1,
};
#endif

enum class LoopbackMode { None, Phy, AD9361 };

#define DEBUG

//#define _RX_DMA_HEADER_TEST
//#define _TX_DMA_HEADER_TEST

/* Thresholds above which we switch to 8-bit mode: */
#define LITEPCIE_8BIT_THRESHOLD  61.44e6
#define LITEETH_8BIT_THRESHOLD   20.0e6

#define DLL_EXPORT __attribute__ ((visibility ("default")))

/*
 * LiteX M2SDR specific flags for RX overflow buffer count reporting.
 *
 * When SOAPY_SDR_OVERFLOW is returned from acquireReadBuffer(), the flags
 * parameter contains the number of lost DMA buffers.
 *
 * Check for LITEX_HAS_OVERFLOW_COUNT in flags to determine if the count
 * is available. If set, extract the count using:
 *   int lost_buffers = (flags & LITEX_OVERFLOW_COUNT_MASK) >> LITEX_OVERFLOW_COUNT_SHIFT;
 *
 * This allows applications to calculate the exact number of lost samples:
 *   lost_samples = lost_buffers * samples_per_buffer
 *
 * Bit layout (flags is int, 32 bits):
 *   Bits 0-7:   Standard SoapySDR flags (OVERFLOW, TIMEOUT, etc.)
 *   Bits 8-15:  Reserved
 *   Bit 16:     LITEX_HAS_OVERFLOW_COUNT (SOAPY_SDR_USER_FLAG0)
 *   Bits 17-30: Lost buffer count (14 bits, up to 16K buffers)
 *   Bit 31:     Sign bit (unused)
 */
#ifndef SOAPY_SDR_USER_FLAG0
#define SOAPY_SDR_USER_FLAG0 (1 << 16)
#endif
#define LITEX_HAS_OVERFLOW_COUNT    SOAPY_SDR_USER_FLAG0
#define LITEX_OVERFLOW_COUNT_SHIFT  17
#define LITEX_OVERFLOW_COUNT_MASK   0x7FFE0000  /* 14 bits for count (up to 16K buffers) */

#if USE_LITEPCIE
#define FD_INIT -1
typedef int litex_m2sdr_device_desc_t;
#elif USE_LITEETH
#define FD_INIT NULL
typedef struct eb_connection *litex_m2sdr_device_desc_t;
#endif

static inline uint32_t litex_m2sdr_readl(struct m2sdr_dev *dev, uint32_t addr)
{
    uint32_t val = 0;
    m2sdr_reg_read(dev, addr, &val);
    return val;
}

static inline void litex_m2sdr_writel(struct m2sdr_dev *dev, uint32_t addr, uint32_t val)
{
    m2sdr_reg_write(dev, addr, val);
}

class DLL_EXPORT SoapyLiteXM2SDR : public SoapySDR::Device {
 /**************************************************************************************************
 *                                        PUBLIC
 **************************************************************************************************/
  public:
    SoapyLiteXM2SDR(const SoapySDR::Kwargs &args);
    ~SoapyLiteXM2SDR(void);

    /***********************************************************************************************
    *                                 Channel configuration
    ***********************************************************************************************/
    void channel_configure(const int direction, const size_t channel);

    /***********************************************************************************************
    *                              Identification API
    ***********************************************************************************************/
    std::string getDriverKey(void) const override;
    std::string getHardwareKey(void) const override;

    /***********************************************************************************************
    *                                 Channels API
    ***********************************************************************************************/
    size_t getNumChannels(const int) const override;
    bool getFullDuplex(const int, const size_t) const override;

    /***********************************************************************************************
    *                                  Stream API
    ***********************************************************************************************/
    std::string getNativeStreamFormat(
        const int /*direction*/,
        const size_t /*channel*/,
        double &fullScale) const override {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args) override;

    void closeStream(SoapySDR::Stream *stream) override;

    int activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems) override;

    int deactivateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs) override;

    size_t getStreamMTU(SoapySDR::Stream *stream) const override;

    size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream) override;

    int getDirectAccessBufferAddrs(
        SoapySDR::Stream *stream,
        const size_t handle,
        void **buffs) override;

    int acquireReadBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        const void **buffs,
        int &flags,
        long long &timeNs,
        const long timeoutUs) override;

    void releaseReadBuffer(
        SoapySDR::Stream *stream,
        size_t handle) override;

    int acquireWriteBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        void **buffs,
        const long timeoutUs) override;

    void releaseWriteBuffer(
        SoapySDR::Stream *stream,
        size_t handle,
        const size_t numElems,
        int &flags,
        const long long timeNs = 0) override;

    std::vector<std::string> getStreamFormats(
        const int direction,
        const size_t channel) const override;

    int readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs = 100000) override;

    int writeStream(
        SoapySDR::Stream *stream,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs = 0,
        const long timeoutUs = 100000) override;

    int readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs) override;

    /***********************************************************************************************
    *                                    Antenna API
    ***********************************************************************************************/
    std::vector<std::string> listAntennas(
        const int direction,
        const size_t channel) const override;

    void setAntenna(
        const int direction,
        const size_t channel,
        const std::string &name) override;

    std::string getAntenna(
        const int direction,
        const size_t channel) const override;

    std::map<int, std::map<size_t, std::string>> _cachedAntValues;

    /***********************************************************************************************
    *                               Frontend corrections API
    ***********************************************************************************************/
    bool hasDCOffsetMode(
        const int direction,
        const size_t channel) const override;

    /***********************************************************************************************
    *                                      Gain API
    ***********************************************************************************************/
    std::vector<std::string> listGains(
        const int direction,
        const size_t channel) const override;

    bool hasGainMode(
        const int direction,
        const size_t channel) const override;

    void setGainMode(const int direction,
        const size_t channel,
        const bool automatic) override;

    bool getGainMode(const int direction,
        const size_t channel) const override;

    void setGain(
        int direction,
        size_t channel,
        const double value) override;

    void setGain(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double value) override;

    double getGain(
        const int direction,
        const size_t channel) const override;

    double getGain(
        const int direction,
        const size_t channel,
        const std::string &name) const override;

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel) const override;

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel,
        const std::string &name) const override;

    std::map<int, std::map<size_t, std::map<std::string, double>>> _cachedGainValues;

    /***********************************************************************************************
    *                                      Frequency API
    ***********************************************************************************************/
    void setFrequency(
        int direction,
        size_t channel,
        double frequency,
        const SoapySDR::Kwargs &args) override;

    void setFrequency(
        const int direction,
        const size_t channel,
        const std::string &,
        const double frequency,
        const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

    double getFrequency(
        const int direction,
        const size_t channel,
        const std::string &name) const override;

    std::vector<std::string> listFrequencies(
        const int,
        const size_t) const override;

    SoapySDR::RangeList getFrequencyRange(
        const int,
        const size_t,
        const std::string &) const override;

    std::map<int, std::map<size_t, std::map<std::string, double>>> _cachedFreqValues;

    /***********************************************************************************************
    *                                    Sample Rate  API
    ***********************************************************************************************/
    void setSampleRate(
        const int direction,
        const size_t,
        const double rate) override;

    double getSampleRate(
        const int direction,
        const size_t) const override;

    std::vector<double> listSampleRates(
        const int direction,
        const size_t channel) const override;

    SoapySDR::RangeList getSampleRateRange(
        const int direction,
        const size_t) const override;

    std::map<int, double> _cachedSampleRates;

    /***********************************************************************************************
    *                                    Bandwidth API
    ***********************************************************************************************/
    void setBandwidth(
        const int direction,
        const size_t channel,
        const double bw) override;

    double getBandwidth(
        const int direction,
        const size_t channel) const override;

    SoapySDR::RangeList getBandwidthRange(
        const int direction,
        const size_t channel) const override;

    /***********************************************************************************************
    *                                    Clocking API
    ***********************************************************************************************/
    std::vector<std::string> listTimeSources(void) const override;
    void setTimeSource(const std::string &source) override;
    std::string getTimeSource(void) const override;

    /***********************************************************************************************
    *                                     Time API
    ***********************************************************************************************/
    bool hasHardwareTime(const std::string &) const override;
    long long getHardwareTime(const std::string &) const override;
    void setHardwareTime(const long long timeNs, const std::string &) override;

    /***********************************************************************************************
    *                                    Sensor API
    ***********************************************************************************************/
    std::vector<std::string> listSensors(void) const override;

    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;

    std::string readSensor(const std::string &key) const override;


 /**************************************************************************************************
 *                                        PRIVATE
 **************************************************************************************************/
  private:
    SoapySDR::Kwargs _deviceArgs;
    struct m2sdr_dev *_dev = nullptr;
    SoapySDR::Stream *const TX_STREAM = (SoapySDR::Stream *)0x1;
    SoapySDR::Stream *const RX_STREAM = (SoapySDR::Stream *)0x2;

    struct litepcie_ioctl_mmap_dma_info _dma_mmap_info;
    void *_dma_buf;

    size_t _rx_buf_size = 0;
    size_t _tx_buf_size = 0;
    size_t _rx_buf_count = 0;
    size_t _tx_buf_count = 0;

#if USE_LITEETH
    struct liteeth_udp_ctrl _udp;
    bool _udp_inited = false;
    SoapyLiteXM2SDREthernetMode _eth_mode = SoapyLiteXM2SDREthernetMode::UDP;
#endif

    /* Per-stream worker fan-out (one TX worker, one RX worker).
     *
     * Topology mirrors LimeSuiteNG/TRXLooper:
     *   - work_fifo carries packets in the direction of useful work
     *     (worker -> caller for RX, caller -> worker for TX).
     *   - free_fifo recycles packet slots back to the producer side.
     *   - pkt_pool owns the heap-allocated sample buffers and is the
     *     authoritative answer to getDirectAccessBufferAddrs().
     * Stats counters are atomics so readStreamStatus can be lock-free.
     */
    struct Stream {
        Stream() :
            opened(false),
            buf(nullptr),
            hw_count(0), sw_count(0), user_count(0),
            remainderHandle(-1), remainderSamps(0),
            remainderOffset(0), remainderBuff(nullptr),
            format(),
            worker_running(false),
            pkt_capacity(0) {}

        bool opened;
        void *buf;
        struct pollfd fds{};
        int64_t hw_count, sw_count, user_count;

        int32_t remainderHandle;
        size_t remainderSamps;
        size_t remainderOffset;
        int8_t* remainderBuff;
        /* Aggregated burst metadata for the currently-open remainder
         * buffer. The first writeStream into a fresh buffer seeds
         * remainder_flags/remainder_timeNs; later writeStream calls
         * into the same buffer can only OR additional flag bits in
         * (END_BURST in particular) but do not overwrite HAS_TIME's
         * timestamp - the buffer fires from the time of its first
         * write, and downstream samples flow in untimed order. */
        int       remainder_flags  = 0;
        long long remainder_timeNs = 0;
        std::string format;
        std::vector<size_t> channels;
#if USE_LITEPCIE
        struct litepcie_dma_ctrl dma;
#endif

        /* Worker thread + lock-free FIFOs (one direction each). */
        std::thread worker;
        std::atomic<bool> worker_running;
        std::vector<litex_m2sdr::StreamPacket> pkt_pool;
        size_t pkt_capacity;
        std::unique_ptr<litex_m2sdr::PacketsFIFO<litex_m2sdr::StreamPacket*>> work_fifo;
        std::unique_ptr<litex_m2sdr::PacketsFIFO<litex_m2sdr::StreamPacket*>> free_fifo;

        /* Stats updated by the worker, read by readStreamStatus.
         * "edge" mirrors the last-reported value so we can return a
         * single Soapy status event per fault occurrence.
         */
        std::atomic<uint64_t> overflow_count{0};
        std::atomic<uint64_t> underflow_count{0};
        std::atomic<uint64_t> late_count{0};
        uint64_t reported_overflow_count = 0;
        uint64_t reported_underflow_count = 0;
        uint64_t reported_late_count = 0;

        /* Per-commit TX-timing slip (commit_ns - target_ns), updated
         * after each HAS_TIME commit. Positive = TX fired late.
         * Negative = TX fired ahead of target (should be rare).
         * Stored as a small ring so a test harness can grab the
         * recent N samples without per-commit polling.
         */
        static constexpr size_t TX_SLIP_RING = 256;
        std::array<std::atomic<int64_t>, TX_SLIP_RING> tx_slip_ring{};
        std::atomic<uint64_t> tx_slip_count{0};
    };

    struct RXStream: Stream {
        double gain[2]{};
        bool   gainMode[2]{};
        double iqbalance[2]{};
        double samplerate = 0.0;
        double bandwidth  = 0.0;
        double frequency  = 0.0;
        std::string antenna[2];

        bool burst_end = false;
        bool time_valid = false;
        /* time0_ns is anchored on the first DMA buffer the worker
         * actually acquires (not at activate). For continuous ADC
         * streaming that's microseconds after activate; for bursty
         * paths like PHY/AD9361 loopback, it's when the first burst
         * arrives. Linear extrapolation from there gives sample-
         * accurate timestamps in both regimes.
         */
        bool time_anchored = false;
        long long time0_ns = 0;
        int64_t time0_count = 0;
        long long remainderTimeNs = 0;
        long long last_time_ns = 0;
        long long last_hw_time_ns = 0;  /* last getHardwareTime() observed */
        bool time_warned = false;
        uint32_t hdr_trace_count = 0;

        /* Timed RX activation is implemented Lime-style in software:
         * RX starts immediately, then readStream drops/trim packets
         * until the first returned sample is exactly the requested one.
         */
        bool timed_start_pending = false;
        long long timed_start_ns = 0;
        long long timed_start_sample = 0;

        /* Re-framer for the RX PCIe byte stream. LitePCIe ring-buffer
         * boundaries are not packet boundaries, so the worker accumulates
         * raw DMA bytes here until it can extract one complete stream
         * packet payload.
         */
        std::vector<uint8_t> rx_reframe_buf;
        std::vector<uint8_t> rx_payload_buf;
    };

    struct TXStream: Stream {
        double gain[2]{};
        double iqbalance[2]{};
        double samplerate = 0.0;
        double bandwidth  = 0.0;
        double frequency  = 0.0;
        std::string antenna[2];

        bool   burst_end   = false;
        int32_t burst_samps = 0;
        std::map<size_t, uint8_t*> pendingWriteBufs;

        /* Cursor for HAS_TIME-clearing on consumed buffers. The FPGA
         * TX DMA reader runs in LOOP_PROG_N=1 (loop mode), so each
         * ring slot is re-read each time the descriptor table cycles
         * past it. If a buffer was submitted with HAS_TIME=1, every
         * subsequent re-read by the FPGA would push another stale
         * descriptor into the arbiter's ts_fifo and (because the
         * timestamp is now in the past) increment late_count. To
         * make HAS_TIME single-use, _dmaAcquireWrite clears the
         * HAS_TIME / END_BURST flag bits on every buffer slot whose
         * index is below hw_count but at-or-above cleared_count, as
         * soon as the kernel updates hw_count.
         */
        int64_t cleared_count = 0;

        /* True once we have called litepcie_dma_reader(enable=1) for
         * this stream. The FPGA TX reader is held *off* until the
         * first user buffer has been fully written into ring slot 0,
         * because the LOOP-mode reader would otherwise immediately
         * latch slot 0's stale uninitialised content (header word 0
         * = 0) as an untimed descriptor and complete a full ~45 ms
         * ring cycle before our HAS_TIME=1 header lands - by which
         * point _clearConsumedTXHeaders has already wiped it.
         */
        bool reader_enabled = false;
    };

    RXStream _rx_stream;
    TXStream _tx_stream;
    std::vector<std::string> _rx_antennas;
    std::vector<std::string> _tx_antennas;
    int _rx_agc_mode = 0;
    std::string _ad9361_fir_profile = "legacy"; /* legacy | bypass | match | wide */

    void interleaveCF32(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void deinterleaveCF32(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void interleaveCS16(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void deinterleaveCS16(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void interleaveCS8(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void deinterleaveCS8(
        const void *src,
        void *dst,
        uint32_t len,
        size_t offset);

    void interleave(
        const void *src,
        void *dst,
        uint32_t len,
        const std::string &format,
        size_t offset);

    void deinterleave(
        const void *src,
        void *dst,
        uint32_t len,
        const std::string &format,
        size_t offset);

    void setSampleMode();

    /* DMA-side acquire/release used by the worker threads.
     * These are the "raw" implementations: they talk directly to the
     * LitePCIe DMA buffers (or the LiteEth UDP helper) and are the
     * only code paths that touch hw/sw/user_count or pendingWriteBufs.
     * The public Soapy acquire/release APIs are now FIFO operations
     * against pkt_pool.
     */
    int  _dmaAcquireRead(size_t &handle, uint8_t **buf, int &flags,
                         long long &timeNs, const long timeoutUs);
    void _dmaReleaseRead(size_t handle);
    int  _dmaAcquireWrite(size_t &handle, uint8_t **buf, const long timeoutUs);
    void _dmaReleaseWrite(size_t handle, size_t numElems,
                          int flags, long long timeNs);
    /* Walk _tx_stream.cleared_count up to hw_count, clearing the
     * HAS_TIME / END_BURST bits in each consumed buffer's header so
     * the LOOP-mode DMA reader doesn't push stale "late" descriptors
     * the next time the ring cycles past those slots. */
    void _clearConsumedTXHeaders();

    /* Worker thread bodies. Run until worker_running is cleared. */
    void rxWorkerLoop();
    void txWorkerLoop();

    /* Pool lifecycle: allocate StreamPacket buffers + seed FIFOs at
     * activateStream, tear down at closeStream.
     */
    void _spawnWorker(Stream &s, bool is_rx);
    void _stopWorker(Stream &s);
    void _freePktPool(Stream &s);

    static LoopbackMode parseLoopbackMode(const SoapySDR::Kwargs &args);
    static const char  *loopbackModeToString(LoopbackMode mode);

    const char *dir2Str(const int direction) const {
        return (direction == SOAPY_SDR_RX) ? "RX" : "TX";
    }

    litex_m2sdr_device_desc_t _fd;
    struct ad9361_rf_phy *ad9361_phy;
    uint8_t _spi_id = 0;

    uint32_t _bitMode           = 16;
    uint32_t _oversampling      = 0;
    uint32_t _nChannels         = 2;
    /* FPGA frame layout invariant: each DMA beat carries four 16-bit
     * fields (IA, QA, IB, QB) even in 1T1R mode (the disabled channel
     * fields stay zero). Stride/MTU arithmetic must always treat the
     * frame as two-channel; _nChannels only governs which channels the
     * SoapySDR caller sees.
     */
    static constexpr uint32_t _frameChannels = 2;
    uint32_t _samplesPerComplex = 2;
    uint32_t _bytesPerSample    = 2;
    uint32_t _bytesPerComplex   = 4;
    float    _samplesScaling    = 2047.0f;
    float    _rateMult          = 1.0f;
    LoopbackMode _loopbackMode  = LoopbackMode::None;

    // register protection
    std::mutex _mutex;
};
