/*
 * SoapySDR driver for the LiteX M2SDR.
 *
 * Copyright (c) 2021-2026 Enjoy Digital.
 * SPDX-License-Identifier: Apache-2.0
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#pragma once

#include <chrono>
#include <mutex>
#include <thread>
#include <deque>
#include <condition_variable>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <cstdint>
#include <atomic>

#include "liblitepcie.h"
#include "libm2sdr.h"

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>

#if USE_VFIO
extern "C" {
#include "../user/libm2sdr_vfio/vfio_priv.h"
struct m2sdr_dev;
int m2sdr_open(struct m2sdr_dev **dev_out, const char *device_identifier);
void m2sdr_close(struct m2sdr_dev *dev);
const char *m2sdr_strerror(int err);
int m2sdr_reg_read(struct m2sdr_dev *dev, uint32_t addr, uint32_t *val);
int m2sdr_reg_write(struct m2sdr_dev *dev, uint32_t addr, uint32_t val);
struct m2sdr_vfio *m2sdr_get_vfio(struct m2sdr_dev *dev);
}
#endif

#define DEBUG

//#define _TX_DMA_HEADER_TEST

/* Thresholds above which we switch to 8-bit mode: */
#define LITEPCIE_8BIT_THRESHOLD  61.44e6

#define DLL_EXPORT __attribute__ ((visibility ("default")))

template<typename T>
class SPSCQueue {
  public:
    void reset(size_t capacity)
    {
        ring_size_ = capacity + 1;
        buffer_.assign(ring_size_, T{});
        read_pos_.store(0, std::memory_order_relaxed);
        write_pos_.store(0, std::memory_order_relaxed);
    }

    void clear()
    {
        const size_t write = write_pos_.load(std::memory_order_acquire);
        read_pos_.store(write, std::memory_order_release);
    }

    bool push(const T &value)
    {
        const size_t write = write_pos_.load(std::memory_order_relaxed);
        const size_t next = advance(write);
        if (next == read_pos_.load(std::memory_order_acquire))
            return false;
        buffer_[write] = value;
        write_pos_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T &value)
    {
        const size_t read = read_pos_.load(std::memory_order_relaxed);
        if (read == write_pos_.load(std::memory_order_acquire))
            return false;
        value = buffer_[read];
        read_pos_.store(advance(read), std::memory_order_release);
        return true;
    }

    bool empty() const
    {
        return read_pos_.load(std::memory_order_acquire) == write_pos_.load(std::memory_order_acquire);
    }

    size_t size() const
    {
        const size_t read = read_pos_.load(std::memory_order_acquire);
        const size_t write = write_pos_.load(std::memory_order_acquire);
        if (write >= read)
            return write - read;
        return ring_size_ - read + write;
    }

  private:
    size_t advance(size_t pos) const
    {
        return (pos + 1 == ring_size_) ? 0 : pos + 1;
    }

    size_t ring_size_ = 0;
    std::vector<T> buffer_;
    std::atomic<size_t> read_pos_{0};
    std::atomic<size_t> write_pos_{0};
};

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
#define litex_m2sdr_writel(_fd, _addr, _val) litepcie_writel(_fd, _addr, _val)
#define litex_m2sdr_readl(_fd, _addr) litepcie_readl(_fd, _addr)
typedef int litex_m2sdr_device_desc_t;
#elif USE_VFIO
#define FD_INIT nullptr
typedef struct m2sdr_dev *litex_m2sdr_device_desc_t;
static inline uint32_t litex_m2sdr_readl(litex_m2sdr_device_desc_t dev, uint32_t addr)
{
    uint32_t val = 0;
    (void)m2sdr_reg_read(dev, addr, &val);
    return val;
}
static inline void litex_m2sdr_writel(litex_m2sdr_device_desc_t dev, uint32_t addr, uint32_t val)
{
    (void)m2sdr_reg_write(dev, addr, val);
}
#endif

class DLL_EXPORT SoapyLiteXM2SDR : public SoapySDR::Device {
 /**************************************************************************************************
 *                                        PUBLIC
 **************************************************************************************************/
  public:
    enum class LoopbackMode {
        None,
        Phy,
        RficBist,
        TxRxCrossbar,
        Dma,
    };

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
    static constexpr const char *kLoopbackArg              = "loopback";
    static constexpr const char *kLoopbackModeArg          = "loopback_mode";
    static constexpr const char *kLoopbackModeNone         = "none";
    static constexpr const char *kLoopbackModePhy          = "phy";
    static constexpr const char *kLoopbackModeRficBist     = "rfic_bist";
    static constexpr const char *kLoopbackModeTxRxCrossbar = "txrx_crossbar";
    static constexpr const char *kLoopbackModeDma          = "dma";

    SoapySDR::Kwargs _deviceArgs;
    SoapySDR::Stream *const TX_STREAM = (SoapySDR::Stream *)0x1;
    SoapySDR::Stream *const RX_STREAM = (SoapySDR::Stream *)0x2;

    struct litepcie_ioctl_mmap_dma_info _dma_mmap_info;
    void *_dma_buf;

#if USE_VFIO
    struct m2sdr_vfio *_vfio = nullptr;
#endif

    size_t _rx_buf_size = 0;
    size_t _tx_buf_size = 0;
    size_t _rx_buf_count = 0;
    size_t _tx_buf_count = 0;

    struct Stream {
        Stream() :
            opened(false),
            buf(nullptr),
            hw_count(0), sw_count(0), user_count(0),
            format() {}

        bool opened;
        void *buf;
        struct pollfd fds{};
        int64_t hw_count, sw_count, user_count;
        std::string format;
        std::vector<size_t> channels;
#if USE_LITEPCIE
        struct litepcie_dma_ctrl dma;
#endif
    };

    struct RXStream: Stream {
        struct Packet {
            std::vector<uint8_t> data;
            int flags = 0;
            long long timeNs = 0;
            int64_t sampleIndex = 0;
            size_t numElems = 0;
        };

        double gain[2]{};
        bool   gainMode[2]{};
        double iqbalance[2]{};
        double samplerate = 0.0;
        double bandwidth  = 0.0;
        double frequency  = 0.0;
        std::string antenna[2];

        bool overflow  = false;
        bool burst_end = false;
        bool time_valid = false;
        bool timed_start_pending = false;
        long long time0_ns = 0;
        std::chrono::steady_clock::time_point time0_steady = std::chrono::steady_clock::now();
        long long start_time_ns = 0;
        int64_t time0_count = 0;
        bool time_base_locked = false;
        long long last_time_ns = 0;
        bool time_warned = false;
        int32_t active_handle = -1;
        size_t active_num_elems = 0;
        size_t active_offset = 0;
        int8_t *active_buff = nullptr;
        long long active_time_ns = 0;
        bool rx_debug = false;
        uint64_t rx_debug_limit = 0;
        long rx_debug_threshold_us = 0;
        std::atomic<uint64_t> rx_debug_seq{0};
        uint64_t continuity_resyncs = 0;
        long long max_lag_ns = 2000000;
        size_t packet_words = 128;
        bool packet_words_explicit = false;
        size_t frame_cycles = 0;
        size_t packet_elems = 0;
        size_t packets_per_dma = 0;
        size_t dma_payload_elems = 0;
        size_t batch_buffers = 1;
        bool batch_buffers_explicit = false;
        size_t slice_elems = 0;
        bool slice_elems_explicit = false;
        double batch_queue_ms = 0.20;
        int worker_rt_prio = -1;
        int worker_cpu = -1;
        size_t batch_buf_size = 0;
        size_t batch_buf_count = 0;
        size_t soapy_bytes_per_complex = 0;
        std::vector<Packet> packet_pool;

#if USE_LITEPCIE || USE_VFIO
        std::mutex recv_mutex;
        std::condition_variable recv_cv;
        SPSCQueue<size_t> ready_buffs;
        std::deque<int64_t> pending_dma_handles;
        SPSCQueue<size_t> free_batch_handles;
        std::thread recv_thread;
        bool recv_thread_stop = false;
        bool recv_thread_running = false;
        std::string recv_error;
        int64_t enqueue_count = 0;
        int64_t overflow_lost_buffers = 0;
#endif
    };

    struct TXStream: Stream {
        struct WriteJob {
            std::vector<uint8_t> data;
            size_t numElems = 0;
            int flags = 0;
            long long timeNs = 0;
            std::chrono::steady_clock::time_point enqueue_tp{};
        };

        double gain[2]{};
        double iqbalance[2]{};
        double samplerate = 0.0;
        double bandwidth  = 0.0;
        double frequency  = 0.0;
        std::string antenna[2];

        bool underflow = false;

        bool   burst_end   = false;
        int32_t burst_samps = 0;
        size_t soapy_bytes_per_complex = 0;
        size_t staging_num_elems = 0;
        int staging_flags = 0;
        long long staging_time_ns = 0;
        std::vector<uint8_t> staging_data;

        /* Tracks the last-seen late/underrun counts from TimedTXArbiter CSRs. */
        uint32_t last_late_count    = 0;
        uint32_t last_underrun_count = 0;

        /* DMA reader is not started until the first releaseWriteBuffer so that
         * the FPGA never reads a zeroed buffer header (ts=0) before the real
         * sync-word/timestamp has been written, which would cause the
         * TimedTXArbiter to pass the burst immediately (ts=0 < now). */
        bool dma_started = false;

        bool tx_debug_headers = false;
        uint64_t tx_debug_limit = 0;
        std::atomic<uint64_t> tx_debug_seq{0};
        std::atomic<uint64_t> tx_submit_seq{0};
        int worker_rt_prio = -1;
        int worker_cpu = -1;
        size_t write_queue_depth = 128;
        bool tx_queue_debug = false;
        long tx_queue_debug_threshold_us = 50;
        uint64_t tx_queue_debug_limit = 0;
        std::atomic<uint64_t> tx_queue_debug_seq{0};

#if USE_LITEPCIE || USE_VFIO
        std::mutex write_mutex;
        std::condition_variable write_cv;
        std::deque<WriteJob> pending_write_jobs;
        std::thread write_thread;
        bool write_thread_stop = false;
        bool write_thread_running = false;
        std::string write_error;
#endif
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
    void startTxWriteWorker();
    void stopTxWriteWorker();
    void txWriteLoop();
    void checkTxWriteError();
    void startRxReceiveWorker();
    void stopRxReceiveWorker();
    void rxReceiveLoop();
    void checkRxReceiveError();

    static constexpr const char *kTransportName =
#if USE_VFIO
        "vfio";
#else
        "litepcie";
#endif

    const char *dir2Str(const int direction) const {
        return (direction == SOAPY_SDR_RX) ? "RX" : "TX";
    }

    size_t getSoapyBytesPerComplex(const std::string &format) const;
    static LoopbackMode parseLoopbackMode(const SoapySDR::Kwargs &args);
    static const char *loopbackModeToString(LoopbackMode mode);

    litex_m2sdr_device_desc_t _fd;
    struct ad9361_rf_phy *ad9361_phy;
    uint8_t _spi_id = 0;
    bool _pcie_dma_synchronizer_bypass = true;

    uint32_t _bitMode           = 16;
    uint32_t _oversampling      = 0;
    uint32_t _nChannels         = 2;
    uint32_t _samplesPerComplex = 2;
    uint32_t _bytesPerSample    = 2;
    uint32_t _bytesPerComplex   = 4;
    float    _samplesScaling    = 2047.0f;
    float    _rateMult          = 1.0f;

    // register protection
    std::mutex _mutex;
};
