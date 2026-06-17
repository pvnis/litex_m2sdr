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
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <unistd.h>
#include <limits>

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

/* TX/RX Stream Headers
 *
 * TX still places one 16-byte header at the start of each host DMA
 * buffer. RX uses a Lime/Myriad-RF style stream packet: the FPGA
 * inserts the same 16-byte header at a fixed cadence in the continuous
 * sample stream and the PCIe worker re-frames around it. Format:
 *
 *   Word 0 (8 bytes, little-endian):
 *     bit 63    : HAS_TIME    (this frame opens a timed burst)
 *     bit 62    : END_BURST   (this frame closes the current burst)
 *     bits 61..0: sync constant (TX_HEADER_SYNC_VALUE)
 *
 *   Word 1 (8 bytes, little-endian):
 *     64-bit sample-count timestamp. For TX this is when the burst
 *     should start firing. For RX this is when the first sample in
 *     the buffer was captured. Generated/consumed by the FPGA-side
 *     SampleCounter; ns conversion happens at the SoapySDR boundary.
 */
#if USE_LITEPCIE
static constexpr size_t TX_DMA_HEADER_SIZE = 16;
static constexpr size_t RX_DMA_HEADER_SIZE = 16;
#else
static constexpr size_t TX_DMA_HEADER_SIZE = 0;
static constexpr size_t RX_DMA_HEADER_SIZE = 0;
#endif

/* Lime/Myriad-RF style stream packetization: fixed 4096-byte packets
 * with a 16-byte metadata header followed by 4080 bytes of RF payload.
 * M2SDR keeps a 64-bit sync word in byte 0 so the PCIe worker can
 * recover packet boundaries from the raw DMA byte stream.
 */
static constexpr size_t RX_STREAM_PACKET_SIZE    = 4096;
static constexpr size_t RX_STREAM_PAYLOAD_BYTES  = RX_STREAM_PACKET_SIZE - RX_DMA_HEADER_SIZE;
static constexpr size_t RX_STREAM_PAYLOAD_WORDS  = RX_STREAM_PAYLOAD_BYTES / sizeof(uint64_t);

static constexpr uint64_t TX_HEADER_SYNC_VALUE = 0x0aa5'5aa5'5aa5'5aa5ULL;
static constexpr uint64_t TX_HEADER_SYNC_MASK  = 0x3fff'ffff'ffff'ffffULL;
static constexpr uint64_t RX_HEADER_SYNC_VALUE = 0x5aa5'5aa5'5aa5'5aa5ULL;
static constexpr uint64_t TX_FLAG_HAS_TIME     = 1ULL << 63;
static constexpr uint64_t TX_FLAG_END_BURST    = 1ULL << 62;


static bool m2sdr_env_tx_cs16_to_sc12_enabled()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_CS16_TO_SC12");
    return v && *v &&
           std::strcmp(v, "0") != 0 &&
           std::strcmp(v, "false") != 0 &&
           std::strcmp(v, "FALSE") != 0 &&
           std::strcmp(v, "off") != 0 &&
           std::strcmp(v, "OFF") != 0 &&
           std::strcmp(v, "no") != 0 &&
           std::strcmp(v, "NO") != 0;
}

static inline int16_t m2sdr_cs16_to_sc12_word(int16_t x)
{
    /*
     * Soapy CS16 convention is full-scale int16.  The M2SDR/AD9361 sample
     * lane is signed 12-bit stored in a 16-bit word.  Divide by 16 to map
     * [-32768, 32767] -> [-2048, 2047] without low-12-bit wrapping.
     */
    return static_cast<int16_t>(static_cast<int32_t>(x) / 16);
}



static bool m2sdr_env_tx_idle_fill_enabled()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_IDLE_FILL");
    return v && *v &&
           std::strcmp(v, "0") != 0 &&
           std::strcmp(v, "false") != 0 &&
           std::strcmp(v, "FALSE") != 0 &&
           std::strcmp(v, "off") != 0 &&
           std::strcmp(v, "OFF") != 0 &&
           std::strcmp(v, "no") != 0 &&
           std::strcmp(v, "NO") != 0;
}

static bool m2sdr_env_tx_dedup_time_enabled()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_DEDUP_TIME");
    if (!v || !*v)
        return true;
    return std::strcmp(v, "0") != 0 &&
           std::strcmp(v, "false") != 0 &&
           std::strcmp(v, "FALSE") != 0 &&
           std::strcmp(v, "off") != 0 &&
           std::strcmp(v, "OFF") != 0 &&
           std::strcmp(v, "no") != 0 &&
           std::strcmp(v, "NO") != 0;
}



static bool m2sdr_env_tx_rms_log_enabled()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_RMS_LOG");
    return v && *v && std::strcmp(v, "0") != 0 &&
           std::strcmp(v, "false") != 0 &&
           std::strcmp(v, "FALSE") != 0 &&
           std::strcmp(v, "off") != 0;
}



static void m2sdr_dump_tx_input_once(
    const void *buf,
    const size_t numElems,
    const std::string &format,
    const size_t channel_index,
    const size_t inputOffsetElems,
    const int flags,
    const long long timeNs,
    const size_t frameChannels,
    const size_t inputBytesPerComplex,
    const double sampleRate)
{
    const char *path = std::getenv("M2SDR_SOAPY_TX_DUMP_PATH");
    if (!path || !*path || !buf || numElems == 0)
        return;

    const char *limit_s = std::getenv("M2SDR_SOAPY_TX_DUMP_SAMPLES");
    const size_t limit = limit_s && *limit_s ? std::strtoul(limit_s, nullptr, 10) : 2000000;

    static size_t dumped_samples = 0;
    static size_t dumped_records = 0;
    static bool announced = false;

    if (dumped_samples >= limit)
        return;

    const size_t n = std::min(numElems, limit - dumped_samples);
    const size_t dumpBytesPerComplex = 2 * sizeof(int16_t); /* pre-dump is always CS16 IQ on disk */
    const size_t offset_bytes = dumped_samples * dumpBytesPerComplex;

    double sum2 = 0.0;
    double peak = 0.0;
    double mean_i = 0.0;
    double mean_q = 0.0;

    std::ofstream out(path, std::ios::binary | std::ios::app);
    if (!out.good())
        return;

    if (format == SOAPY_SDR_CS16) {
        const int16_t *x = static_cast<const int16_t *>(buf) + inputOffsetElems * 2;
        for (size_t i = 0; i < n; ++i) {
            const double re = x[2*i + 0] / 32768.0;
            const double im = x[2*i + 1] / 32768.0;
            const double mag = std::sqrt(re*re + im*im);
            sum2 += re*re + im*im;
            peak = std::max(peak, mag);
            mean_i += re;
            mean_q += im;
        }
        out.write(reinterpret_cast<const char *>(x), n * dumpBytesPerComplex);
    } else if (format == SOAPY_SDR_CF32) {
        const float *x = static_cast<const float *>(buf) + inputOffsetElems * 2;
        std::vector<int16_t> tmp(2 * n);
        for (size_t i = 0; i < n; ++i) {
            const double re0 = x[2*i + 0];
            const double im0 = x[2*i + 1];
            const double mag = std::sqrt(re0*re0 + im0*im0);
            sum2 += re0*re0 + im0*im0;
            peak = std::max(peak, mag);
            mean_i += re0;
            mean_q += im0;

            const double re = std::max(-1.0, std::min(1.0, re0));
            const double im = std::max(-1.0, std::min(1.0, im0));
            tmp[2*i + 0] = (int16_t)std::lrint(re * 32767.0);
            tmp[2*i + 1] = (int16_t)std::lrint(im * 32767.0);
        }
        out.write(reinterpret_cast<const char *>(tmp.data()), tmp.size() * sizeof(int16_t));
    } else {
        return;
    }

    if (!out.good())
        return;

    const double denom = std::max<size_t>(n, 1);
    const double rms = std::sqrt(sum2 / denom);
    mean_i /= denom;
    mean_q /= denom;

    const std::string meta_path = std::string(path) + ".meta.jsonl";
    std::ofstream meta(meta_path, std::ios::app);
    if (meta.good()) {
        meta
            << "{"
            << "\"seq\":" << dumped_records
            << ",\"offset_bytes\":" << offset_bytes
            << ",\"numElems\":" << n
            << ",\"requestedNumElems\":" << numElems
            << ",\"format\":\"" << format << "\""
            << ",\"flags\":" << flags
            << ",\"timeNs\":" << timeNs
            << ",\"sampleRate\":" << sampleRate
            << ",\"channel\":" << channel_index
            << ",\"inputOffsetElems\":" << inputOffsetElems
            << ",\"frameChannels\":" << frameChannels
            << ",\"bytesPerComplex\":" << dumpBytesPerComplex
            << ",\"inputBytesPerComplex\":" << inputBytesPerComplex
            << ",\"rms\":" << rms
            << ",\"peak\":" << peak
            << ",\"mean_i\":" << mean_i
            << ",\"mean_q\":" << mean_q
            << "}\n";
    }

    dumped_samples += n;
    ++dumped_records;

    if (!announced) {
        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "TX_DUMP_PROBE path=%s meta=%s format=%s channel=%zu limit_samples=%zu",
            path, meta_path.c_str(), format.c_str(), channel_index, limit);
        announced = true;
    }

    if (dumped_samples >= limit) {
        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "TX_DUMP_PROBE complete path=%s dumped_samples=%zu records=%zu",
            path, dumped_samples, dumped_records);
    }
}


static size_t m2sdr_env_tx_dma_dump_buffers()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_DMA_DUMP_BUFFERS");
    if (!v || !*v)
        return 16;

    const long n = std::strtol(v, nullptr, 10);
    if (n <= 0)
        return 0;
    if (n > 4096)
        return 4096;
    return static_cast<size_t>(n);
}


static void m2sdr_dump_tx_dma_buffer_once(
    const uint8_t *tx_buffer,
    const size_t write_size,
    const size_t handle,
    const size_t numElems,
    const int flags,
    const long long timeNs,
    const size_t frameChannels,
    const size_t bytesPerComplex,
    const double sampleRate,
    const bool zeroCopy)
{
    const char *path = std::getenv("M2SDR_SOAPY_TX_DMA_DUMP_PATH");
    if (!path || !*path || !tx_buffer || write_size == 0)
        return;

    const size_t limit = m2sdr_env_tx_dma_dump_buffers();
    if (limit == 0)
        return;

    static size_t dumped_buffers = 0;
    static bool announced = false;

    if (dumped_buffers >= limit)
        return;

    std::ofstream out(path, std::ios::binary | std::ios::app);
    if (!out.good())
        return;

    out.write(reinterpret_cast<const char *>(tx_buffer), write_size);
    if (!out.good())
        return;

    const uint64_t header0 = (write_size >= 8)
        ? *reinterpret_cast<const uint64_t *>(tx_buffer + 0)
        : 0;
    const uint64_t header1 = (write_size >= 16)
        ? *reinterpret_cast<const uint64_t *>(tx_buffer + 8)
        : 0;

    char header0_hex[32];
    char header1_hex[32];
    std::snprintf(header0_hex, sizeof(header0_hex), "0x%016llx",
                  static_cast<unsigned long long>(header0));
    std::snprintf(header1_hex, sizeof(header1_hex), "0x%016llx",
                  static_cast<unsigned long long>(header1));

    const size_t payload_bytes = write_size > TX_DMA_HEADER_SIZE
        ? write_size - TX_DMA_HEADER_SIZE
        : 0;
    const size_t logical_payload_bytes = numElems * frameChannels * bytesPerComplex;
    const size_t zero_padding_bytes = payload_bytes > logical_payload_bytes
        ? payload_bytes - logical_payload_bytes
        : 0;

    const std::string meta_path = std::string(path) + ".meta.jsonl";
    std::ofstream meta(meta_path, std::ios::app);
    if (meta.good()) {
        meta
            << "{"
            << "\"seq\":" << dumped_buffers
            << ",\"handle\":" << handle
            << ",\"write_size\":" << write_size
            << ",\"payload_bytes\":" << payload_bytes
            << ",\"numElems\":" << numElems
            << ",\"logical_payload_bytes\":" << logical_payload_bytes
            << ",\"zero_padding_bytes\":" << zero_padding_bytes
            << ",\"flags\":" << flags
            << ",\"timeNs\":" << timeNs
            << ",\"timestamp_samples\":" << static_cast<unsigned long long>(header1)
            << ",\"header0\":\"" << header0_hex << "\""
            << ",\"header1\":\"" << header1_hex << "\""
            << ",\"frameChannels\":" << frameChannels
            << ",\"bytesPerComplex\":" << bytesPerComplex
            << ",\"sampleRate\":" << sampleRate
            << ",\"zeroCopy\":" << (zeroCopy ? "true" : "false")
            << "}\n";
    }

    if (!announced) {
        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "TX_DMA_DUMP_PROBE path=%s meta=%s buffer_bytes=%zu limit_buffers=%zu",
            path, meta_path.c_str(), write_size, limit);
        announced = true;
    }

    ++dumped_buffers;

    if (dumped_buffers >= limit) {
        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "TX_DMA_DUMP_PROBE complete path=%s dumped_buffers=%zu",
            path, dumped_buffers);
    }
}



static void m2sdr_log_tx_input_rms_once(
    const void *buf,
    const size_t numElems,
    const std::string &format,
    const size_t channel_index,
    const int flags,
    const long long timeNs)
{
    static size_t logged = 0;
    if (!m2sdr_env_tx_rms_log_enabled())
        return;

    const char *limit_s = std::getenv("M2SDR_SOAPY_TX_RMS_LOG_LIMIT");
    const size_t limit = limit_s && *limit_s ? std::strtoul(limit_s, nullptr, 10) : 64;
    if (logged >= limit)
        return;

    if (!buf || numElems == 0)
        return;

    const size_t n = std::min<size_t>(numElems, 4096);
    double sum2 = 0.0;
    double peak = 0.0;
    double mean_i = 0.0;
    double mean_q = 0.0;

    if (format == SOAPY_SDR_CF32) {
        const float *x = static_cast<const float *>(buf);
        for (size_t i = 0; i < n; ++i) {
            const double re = x[2*i + 0];
            const double im = x[2*i + 1];
            const double p = re*re + im*im;
            sum2 += p;
            peak = std::max(peak, std::sqrt(p));
            mean_i += re;
            mean_q += im;
        }
    } else if (format == SOAPY_SDR_CS16) {
        const int16_t *x = static_cast<const int16_t *>(buf);
        for (size_t i = 0; i < n; ++i) {
            const double re = x[2*i + 0] / 32768.0;
            const double im = x[2*i + 1] / 32768.0;
            const double p = re*re + im*im;
            sum2 += p;
            peak = std::max(peak, std::sqrt(p));
            mean_i += re;
            mean_q += im;
        }
    } else {
        SoapySDR_logf(
            SOAPY_SDR_INFO,
            "TX_RMS_PROBE[%zu] ch=%zu format=%s numElems=%zu flags=0x%x timeNs=%lld unsupported_format",
            logged, channel_index, format.c_str(), numElems, flags, timeNs);
        ++logged;
        return;
    }

    const double rms = std::sqrt(sum2 / std::max<size_t>(n, 1));
    mean_i /= std::max<size_t>(n, 1);
    mean_q /= std::max<size_t>(n, 1);

    SoapySDR_logf(
        SOAPY_SDR_INFO,
        "TX_RMS_PROBE[%zu] ch=%zu format=%s numElems=%zu n=%zu rms=%.6g peak=%.6g mean_i=%.6g mean_q=%.6g flags=0x%x timeNs=%lld",
        logged, channel_index, format.c_str(), numElems, n, rms, peak, mean_i, mean_q, flags, timeNs);

    ++logged;
}


static long long m2sdr_env_tx_time_offset_ns()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_TIME_OFFSET_NS");
    if (!v || !*v)
        return 0;
    return std::strtoll(v, nullptr, 10);
}


static long m2sdr_env_tx_copy_prime_buffers()
{
    const char *v = std::getenv("M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS");
    if (!v || !*v)
        return 8;
    const long n = std::strtol(v, nullptr, 10);
    if (n < 1)
        return 1;
    if (n > 16)
        return 16;
    return n;
}


static bool m2sdr_env_disables_litepcie_zero_copy()
{
    const char *v = std::getenv("M2SDR_LITEPCIE_ZERO_COPY");
    if (!v || !*v)
        return false;

    return std::strcmp(v, "0") == 0 ||
           std::strcmp(v, "false") == 0 ||
           std::strcmp(v, "FALSE") == 0 ||
           std::strcmp(v, "off") == 0 ||
           std::strcmp(v, "OFF") == 0 ||
           std::strcmp(v, "no") == 0 ||
           std::strcmp(v, "NO") == 0;
}

static inline long long ns_to_samples(double sample_rate, long long ns)
{
    if (sample_rate <= 0.0) return 0;
    const long double samples =
        (static_cast<long double>(ns) * static_cast<long double>(sample_rate)) / 1000000000.0L;
    return static_cast<long long>(std::llround(samples));
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
        _rx_stream.dma.zero_copy  = m2sdr_env_disables_litepcie_zero_copy() ? 0 : 1;
        if (!_rx_stream.dma.zero_copy) {
            SoapySDR_logf(SOAPY_SDR_WARNING,
                "LitePCIe RX zero-copy disabled by M2SDR_LITEPCIE_ZERO_COPY=0; using read() copy path");
        }
        if (litepcie_dma_init(&_rx_stream.dma, "", _rx_stream.dma.zero_copy) < 0)
            throw std::runtime_error("DMA Writer/RX not available (litepcie_dma_init failed).");

        /* Get Buffer and Parameters from RX DMA Writer */
        _rx_stream.buf = _rx_stream.dma.buf_rd;
        _rx_buf_size   = RX_STREAM_PAYLOAD_BYTES;
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
        if (format == SOAPY_SDR_CS8) {
            _bitMode = 8;
            setSampleMode();
        }
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
        /*
         * In TX copy-mode, do not enable the FPGA reader from
         * litepcie_dma_process() during acquire.  Copy-mode must first stage a
         * completed userspace buffer, write() it into the kernel, and only then
         * start the reader after a small priming queue exists.  Otherwise the
         * FPGA reader can run ahead of reader_sw_count and the kernel reports
         * "Writing too late".
         */
        _tx_stream.dma.reader_enable = 0;
        _tx_stream.dma.writer_enable = 0;
        _tx_stream.dma.zero_copy  = m2sdr_env_disables_litepcie_zero_copy() ? 0 : 1;
        if (!_tx_stream.dma.zero_copy) {
            SoapySDR_logf(SOAPY_SDR_WARNING,
                "LitePCIe TX zero-copy disabled by M2SDR_LITEPCIE_ZERO_COPY=0; using write() copy path");
        }
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
        if (format == SOAPY_SDR_CS8) {
            _bitMode = 8;
            setSampleMode();
        }
        _tx_stream.remainderHandle = -1;
        _tx_stream.remainderSamps  = 0;
        _tx_stream.remainderOffset = 0;

        _tx_stream.channels = selected_channels;
        _nChannels = _tx_stream.channels.size();
    } else {
        throw std::runtime_error("Invalid direction.");
    }

    /* Configure 2T2R/1T1R mode (PHY) without clobbering other control bits such as loopback. */
    {
        uint32_t phy_ctrl = litex_m2sdr_readl(_dev, CSR_AD9361_PHY_CONTROL_ADDR);
#ifdef CSR_AD9361_PHY_CONTROL_MODE_OFFSET
        phy_ctrl &= ~((uint32_t)(((1u << CSR_AD9361_PHY_CONTROL_MODE_SIZE) - 1) << CSR_AD9361_PHY_CONTROL_MODE_OFFSET));
        phy_ctrl |= ((_nChannels == 1 ? 1u : 0u) << CSR_AD9361_PHY_CONTROL_MODE_OFFSET);
#else
        phy_ctrl = (phy_ctrl & ~1u) | (_nChannels == 1 ? 1u : 0u);
#endif
        litex_m2sdr_writel(_dev, CSR_AD9361_PHY_CONTROL_ADDR, phy_ctrl);
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

    /* ad9361_set_no_ch_mode() rewrites AD9361 channel-mode state. Many
     * Soapy clients configure rate/frequency/gain before setupStream(), so
     * reapply the cached RF settings here after the final channel mode is
     * known. Without this, TX descriptors can stream while the RF datapath
     * stays effectively muted until the application happens to set RF again.
     *
     * For sample rate and bandwidth, the AD9361 has a single value shared
     * by TX and RX. We honor the direction being currently set up - the
     * stream the caller is actively configuring - rather than blindly
     * preferring TX. Otherwise an RX-only application that never set TX
     * rate would force the chip back to the 30.72 MSPS init-time default
     * and silently run at the wrong rate.
     */
    {
        const double rate = (direction == SOAPY_SDR_TX)
            ? _tx_stream.samplerate
            : _rx_stream.samplerate;
        if (rate > 0.0) {
            int rc = m2sdr_set_sample_rate(_dev, static_cast<int64_t>(rate / _rateMult));
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "m2sdr_set_sample_rate(reapply after channel mode) failed: %s",
                    m2sdr_strerror(rc));
            }
            setSampleMode();
        }

        const double bw = (direction == SOAPY_SDR_TX)
            ? _tx_stream.bandwidth
            : _rx_stream.bandwidth;
        if (bw > 0.0) {
            int rc = m2sdr_set_bandwidth(_dev, static_cast<int64_t>(bw));
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "m2sdr_set_bandwidth(reapply after channel mode) failed: %s",
                    m2sdr_strerror(rc));
            }
        }

        if (_tx_stream.frequency > 0.0) {
            int rc = m2sdr_set_frequency(_dev, M2SDR_TX,
                static_cast<uint64_t>(_tx_stream.frequency));
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "m2sdr_set_frequency(TX reapply after channel mode) failed: %s",
                    m2sdr_strerror(rc));
            }
        }
        if (_rx_stream.frequency > 0.0) {
            int rc = m2sdr_set_frequency(_dev, M2SDR_RX,
                static_cast<uint64_t>(_rx_stream.frequency));
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "m2sdr_set_frequency(RX reapply after channel mode) failed: %s",
                    m2sdr_strerror(rc));
            }
        }

        for (size_t ch = 0; ch < 2; ++ch) {
            int rc = m2sdr_set_tx_att(_dev, _tx_stream.gain[ch]);
            if (rc != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                    "m2sdr_set_tx_att(ch%zu reapply after channel mode) failed: %s",
                    ch, m2sdr_strerror(rc));
            }
        }
    }

    return direction == SOAPY_SDR_RX ? RX_STREAM : TX_STREAM;
}

/* Close the specified stream and release associated resources. */
void SoapyLiteXM2SDR::closeStream(SoapySDR::Stream *stream) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (stream == RX_STREAM) {
        /* Ensure the worker thread is stopped before tearing down DMA;
         * the worker holds raw pointers into _rx_stream.buf.
         */
        _stopWorker(_rx_stream);
        _freePktPool(_rx_stream);
#if USE_LITEPCIE
        litepcie_dma_cleanup(&_rx_stream.dma);
        _rx_stream.buf = NULL;
#elif USE_LITEETH
        std::free(_rx_stream.buf);
        _rx_stream.buf = NULL;
#endif
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
        _stopWorker(_tx_stream);
        _freePktPool(_tx_stream);
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
        /* Keep activation lightweight: Soapy setters apply RF/rate/gain
         * changes immediately, and timed activation needs this path to
         * complete well inside the caller's future start window.
         */
#if USE_LITEPCIE
        /* Re-assert synchronizer bypass on every activate. The kernel's
         * litepcie_dma_reader_stop / writer_stop clear bypass to 0
         * whenever both DMA directions become idle (see kernel/main.c),
         * so a fresh activate after a prior deactivate would otherwise
         * leave bypass=0. That holds the TX header extractor (and any
         * other consumer of ~synchronizer.synced) in reset, stalling
         * the whole TX pipeline on the next run. */
#ifdef CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR
        litex_m2sdr_writel(_dev, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 1);
#endif
        /* Enable the 16-byte per-buffer RX header (sync + sample-count
         * timestamp written by the FPGA HeaderInserter). The default
         * CSR state is enable=1, header_enable=0; we need bit1 set so
         * each RX frame carries its capture timestamp. */
#ifdef CSR_HEADER_RX_CONTROL_ADDR
        litex_m2sdr_writel(_dev, CSR_HEADER_RX_CONTROL_ADDR,
            (1u << CSR_HEADER_RX_CONTROL_ENABLE_OFFSET) |
            (1u << CSR_HEADER_RX_CONTROL_HEADER_ENABLE_OFFSET));
#endif
#ifdef CSR_HEADER_RX_FRAME_CYCLES_ADDR
        litex_m2sdr_writel(_dev, CSR_HEADER_RX_FRAME_CYCLES_ADDR,
            static_cast<uint32_t>(RX_STREAM_PAYLOAD_WORDS));
#endif
        /* Crossbar Demux: Select PCIe streaming */
        litex_m2sdr_writel(_dev, CSR_CROSSBAR_DEMUX_SEL_ADDR, 0);
        /* Configure the DMA engine for RX, but don't enable it yet. */
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
#elif USE_LITEETH
        /* Crossbar Demux: Select Ethernet streaming */
        litex_m2sdr_writel(_dev, CSR_CROSSBAR_DEMUX_SEL_ADDR, 1);
#ifdef CSR_ETH_RX_MODE_ADDR
        litex_m2sdr_writel(_dev, CSR_ETH_RX_MODE_ADDR,
                           (_eth_mode == SoapyLiteXM2SDREthernetMode::VRT) ? 2 : 1);
#endif
        /* UDP helper is ready; nothing to start explicitly. */
#endif
        /* Start the worker at the kernel's current hw_count so we read
         * the freshest DMA buffer next, not buffer 0 which may hold
         * stale data from a previous session of the same fd. Without
         * this, the worker churns through the entire ring of historical
         * buffers before reaching live samples. For the LiteEth path
         * (which doesn't expose a counter) hw_count stays 0, matching
         * the prior behaviour.
         */
        _rx_stream.user_count = _rx_stream.hw_count;
        _rx_stream.burst_end = false;
        _rx_stream.time0_ns = this->getHardwareTime("");
        _rx_stream.time0_count = _rx_stream.user_count;
        _rx_stream.time_valid = (_rx_stream.samplerate > 0.0);
        _rx_stream.time_anchored = false;  /* lazy-init on first acquire */
        _rx_stream.last_time_ns = _rx_stream.time0_ns;
        _rx_stream.time_warned = false;
        _rx_stream.hdr_trace_count = 0;
        _rx_stream.remainderHandle = -1;
        _rx_stream.remainderSamps = 0;
        _rx_stream.remainderOffset = 0;
        _rx_stream.remainderTimeNs = 0;
        _rx_stream.rx_reframe_buf.clear();
        _rx_stream.rx_payload_buf.clear();
        _rx_stream.timed_start_pending = false;
        _rx_stream.timed_start_ns = 0;
        _rx_stream.timed_start_sample = 0;
        if (flags & SOAPY_SDR_HAS_TIME) {
            if (_rx_stream.samplerate <= 0.0) {
                SoapySDR::log(SOAPY_SDR_ERROR,
                    "RX timed activation requested before RX sample rate was set");
                return SOAPY_SDR_NOT_SUPPORTED;
            }
            _rx_stream.timed_start_pending = true;
            _rx_stream.timed_start_ns = timeNs;
            _rx_stream.timed_start_sample = ns_to_samples(_rx_stream.samplerate, timeNs);
            SoapySDR::logf(SOAPY_SDR_INFO,
                "RX timed activation requested for %lld ns (%lld samples); aligning in readStream",
                (long long)timeNs,
                (long long)_rx_stream.timed_start_sample);
        }

        /* Spin up the RX worker thread; it pumps DMA -> packet FIFO
         * asynchronously so readStream() pulls from the FIFO without
         * blocking on poll().
         */
        _spawnWorker(_rx_stream, /*is_rx=*/true);

    /* TX */
    } else if (stream == TX_STREAM) {
        /* RF/rate/gain settings are already applied by the Soapy setters;
         * activateStream only arms the DMA/worker pipeline.
         */
#if USE_LITEPCIE
        /* Re-assert synchronizer bypass (see RX branch above for why -
         * the kernel clears bypass when both DMA directions stop, so
         * we have to re-set it every activate or the TX header
         * extractor stays held in reset). */
#ifdef CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR
        litex_m2sdr_writel(_dev, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 1);
#endif
        /* Enable the 16-byte per-buffer TX header. The HeaderExtractor
         * strips it before the sample stream reaches the TimedTXArbiter,
         * which uses the flags + timestamp word to gate sample release.
         */
#ifdef CSR_HEADER_TX_CONTROL_ADDR
        litex_m2sdr_writel(_dev, CSR_HEADER_TX_CONTROL_ADDR,
            (1u << CSR_HEADER_TX_CONTROL_ENABLE_OFFSET) |
            (1u << CSR_HEADER_TX_CONTROL_HEADER_ENABLE_OFFSET));
#endif
        /* Crossbar Mux: Select PCIe streaming */
        litex_m2sdr_writel(_dev, CSR_CROSSBAR_MUX_SEL_ADDR, 0);
        /* Configure the DMA engine for TX, but don't enable it yet. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
        /* The kernel reuses the mmap TX ring across sessions. If a previous
         * process exited before the idle cleanup cleared consumed headers, the
         * LOOP-mode DMA reader can replay stale HAS_TIME descriptors as soon
         * as it starts, causing startup-only late drops. With the reader
         * stopped, it is safe to sanitize every slot before first release. */
        _clearAllTXHeaderFlags();
        /* Newer gateware exposes a real timed-TX reset that flushes the
         * arbiter FIFOs/FSM, not just the status counters. Use it at the
         * last quiet point before the first DMA release; old bitstreams keep
         * the counter reset as a harmless fallback. */
#if defined(CSR_TIMED_TX_RESET_ADDR)
        litex_m2sdr_writel(_dev, CSR_TIMED_TX_RESET_ADDR, 1);
#elif defined(CSR_TIMED_TX_RESET_COUNTS_ADDR)
        litex_m2sdr_writel(_dev, CSR_TIMED_TX_RESET_COUNTS_ADDR, 1);
#endif
        _tx_stream.user_count = _tx_stream.hw_count;
        _tx_stream.cleared_count = _tx_stream.hw_count;
        _tx_stream.reader_enabled = false;
#elif USE_LITEETH
        /* Crossbar Mux: Select Ethernet streaming */
        litex_m2sdr_writel(_dev, CSR_CROSSBAR_MUX_SEL_ADDR, 1);
        /* No explicit start; pacing handled by client cadence if needed. */
        _tx_stream.user_count = 0;
#endif
        _tx_stream.pendingWriteBufs.clear();
        _tx_stream.burst_end = false;
        if (flags & SOAPY_SDR_HAS_TIME) {
            SoapySDR::logf(SOAPY_SDR_DEBUG,
                "TX timed activation requested for %lld ns; timing is enforced on buffer submission",
                (long long)timeNs);
        }

        /* Spin up the TX worker thread; the caller's writeStream()
         * now enqueues into the worker's packet FIFO and the worker
         * drives DMA + the HAS_TIME wait off the user thread.
         */
        _spawnWorker(_tx_stream, /*is_rx=*/false);
    }

#ifdef CSR_AD9361_CH_EN_ADDR
    {
        uint32_t ch_en = (_nChannels >= 2) ? 0x3 : 0x1;
        litex_m2sdr_writel(_dev, CSR_AD9361_CH_EN_ADDR, ch_en);
    }
#endif

    return 0;
}

/* Deactivate the specified stream (disable DMA engine). */
int SoapyLiteXM2SDR::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs) {
    if (stream == RX_STREAM) {
        /* Tag any remaining packets as end-of-burst and stop the
         * worker before disabling DMA so it doesn't race the teardown.
         */
        _rx_stream.burst_end = true;
        _stopWorker(_rx_stream);
        /* Disable the DMA engine for RX. */
#if USE_LITEPCIE
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
        _rx_stream.timed_start_ns = 0;
        _rx_stream.timed_start_sample = 0;
        if (flags & SOAPY_SDR_HAS_TIME) {
            SoapySDR::logf(SOAPY_SDR_WARNING,
                "RX timed deactivation requested for %lld ns; stopping immediately",
                (long long)timeNs);
        }
    } else if (stream == TX_STREAM) {
        _stopWorker(_tx_stream);
#if USE_LITEPCIE
        /* Disable the DMA engine for TX. */
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _tx_stream.reader_enabled = false;
#if defined(CSR_TIMED_TX_RESET_ADDR)
        litex_m2sdr_writel(_dev, CSR_TIMED_TX_RESET_ADDR, 1);
#elif defined(CSR_TIMED_TX_RESET_COUNTS_ADDR)
        litex_m2sdr_writel(_dev, CSR_TIMED_TX_RESET_COUNTS_ADDR, 1);
#endif
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
        return _rx_buf_size / (_frameChannels * _bytesPerComplex);
    } else if (stream == TX_STREAM) {
        return _tx_buf_size / (_frameChannels * _bytesPerComplex);
    } else {
        throw std::runtime_error("SoapySDR::getStreamMTU(): Invalid stream.");
    }
}

/* Retrieve the number of direct access buffers available for a stream.
 *
 * With worker-thread streaming the buffers callers can directly index
 * are the StreamPacket sample buffers in the worker pool. We return
 * the pool size; if the worker is not yet started (stream not
 * activated) we fall back to the underlying DMA buffer count so old
 * code that calls this between setup and activate still gets a sane
 * answer.
 */
size_t SoapyLiteXM2SDR::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
    if (stream == RX_STREAM) {
        if (!_rx_stream.pkt_pool.empty()) return _rx_stream.pkt_pool.size();
        return _rx_buf_count;
    } else if (stream == TX_STREAM) {
        if (!_tx_stream.pkt_pool.empty()) return _tx_stream.pkt_pool.size();
#if USE_LITEETH
        return _tx_buf_count;
#else
        return _dma_mmap_info.dma_tx_buf_count;
#endif
    } else {
        throw std::runtime_error("SoapySDR::getNumDirectAccessBuffers(): Invalid stream.");
    }
}

/* Retrieve buffer addresses for a direct access buffer.
 *
 * Once the worker is running, the handle refers to the StreamPacket
 * pool index. Before activation we still expose the raw DMA buffer
 * addresses to keep diagnostics tooling that probes prior to
 * activateStream working.
 */
int SoapyLiteXM2SDR::getDirectAccessBufferAddrs(
    SoapySDR::Stream *stream,
    const size_t handle,
    void **buffs) {
    if (stream == RX_STREAM) {
        if (!_rx_stream.pkt_pool.empty() && handle < _rx_stream.pkt_pool.size()) {
            buffs[0] = _rx_stream.pkt_pool[handle].data;
            return 0;
        }
#if USE_LITEPCIE
        buffs[0] = (char *)_rx_stream.buf + handle * _dma_mmap_info.dma_rx_buf_size + RX_DMA_HEADER_SIZE;
#else
        buffs[0] = (char *)_rx_stream.buf + handle * _rx_buf_size;
#endif
    } else if (stream == TX_STREAM) {
        if (!_tx_stream.pkt_pool.empty() && handle < _tx_stream.pkt_pool.size()) {
            buffs[0] = _tx_stream.pkt_pool[handle].data;
            return 0;
        }
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

#define DETECT_EVERY_OVERFLOW  true  /* Detect overflow every time it occurs. */
#define DETECT_EVERY_UNDERFLOW true  /* Detect underflow every time it occurs. */

static inline long long samples_to_ns(double sample_rate, long long samples)
{
    if (sample_rate <= 0.0) {
        return 0;
    }
    const long double ns =
        (static_cast<long double>(samples) * 1000000000.0L) / static_cast<long double>(sample_rate);
    return static_cast<long long>(std::llround(ns));
}

/* Acquire a buffer for reading (worker FIFO consumer).
 *
 * The RX worker thread is the sole owner of the DMA buffer ring and
 * stages each captured DMA buffer into a StreamPacket which it pushes
 * to `work_fifo`. This call pops the next packet (blocking up to
 * timeoutUs) and exposes its payload via `buffs[0]`. `handle` is the
 * pool index so the caller can return the packet via releaseReadBuffer.
 *
 * Overflow handling: the worker stamps the packet with ret_code =
 * SOAPY_SDR_OVERFLOW. We recycle that packet immediately and return -1
 * to the caller with handle=-1, matching the prior synchronous contract
 * (callers must not releaseReadBuffer on overflow).
 */
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
    if (!_rx_stream.work_fifo || !_rx_stream.free_fifo) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(timeoutUs > 0 ? timeoutUs : 0);
    auto remaining_timeout_us = [&]() -> long {
        if (timeoutUs <= 0) return timeoutUs;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return 0;
        return static_cast<long>(
            std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count());
    };

    size_t dropped_packets = 0;
    long long dropped_samples = 0;

    while (true) {
        litex_m2sdr::StreamPacket *pkt = nullptr;
        if (!_rx_stream.work_fifo->pop(&pkt,
                                       /*wait=*/true,
                                       std::chrono::microseconds(remaining_timeout_us()))) {
            return SOAPY_SDR_TIMEOUT;
        }

        if (pkt->ret_code < 0) {
            /* Worker reported an error (typically OVERFLOW). Return the
             * carrier packet to the free pool so the worker can reuse it,
             * mark the handle invalid, and propagate the code+flags.
             */
            const int code = pkt->ret_code;
            flags |= pkt->flags;
            timeNs = pkt->timeNs;
            pkt->ret_code = 0;
            pkt->samples  = 0;
            pkt->flags    = 0;
            _rx_stream.free_fifo->push(pkt, /*wait=*/true);
            handle = (size_t)-1;
            return code;
        }

        size_t packet_offset = 0;
        uint32_t packet_samps = pkt->samples;
        long long packet_timeNs = pkt->timeNs;
        int packet_flags = pkt->flags;

        if (_rx_stream.timed_start_pending) {
            if (!_rx_stream.time_valid || !(packet_flags & SOAPY_SDR_HAS_TIME)) {
                _rx_stream.free_fifo->push(pkt, /*wait=*/true);
                SoapySDR::log(SOAPY_SDR_ERROR,
                    "RX timed activation cannot align because RX packets have no timestamp");
                return SOAPY_SDR_STREAM_ERROR;
            }

            const long long packet_start_sample = ns_to_samples(_rx_stream.samplerate, packet_timeNs);
            const long long packet_end_sample = packet_start_sample + static_cast<long long>(packet_samps);
            const long long target_sample = _rx_stream.timed_start_sample;

            if (packet_end_sample <= target_sample) {
                dropped_packets++;
                dropped_samples += static_cast<long long>(packet_samps);
                pkt->samples = 0;
                pkt->flags = 0;
                _rx_stream.free_fifo->push(pkt, /*wait=*/true);
                continue;
            }

            if (packet_start_sample > target_sample) {
                _rx_stream.free_fifo->push(pkt, /*wait=*/true);
                _rx_stream.timed_start_pending = false;
                timeNs = samples_to_ns(_rx_stream.samplerate, target_sample);
                flags |= SOAPY_SDR_HAS_TIME;
                handle = (size_t)-1;
                SoapySDR::logf(SOAPY_SDR_WARNING,
                    "RX timed activation missed target: requested=%lld samples first=%lld samples delta=%lld samples",
                    (long long)target_sample,
                    (long long)packet_start_sample,
                    (long long)(packet_start_sample - target_sample));
                return SOAPY_SDR_TIME_ERROR;
            }

            packet_offset = static_cast<size_t>(target_sample - packet_start_sample);
            packet_samps -= static_cast<uint32_t>(packet_offset);
            packet_timeNs = samples_to_ns(_rx_stream.samplerate, target_sample);
            _rx_stream.timed_start_pending = false;
            SoapySDR::logf(SOAPY_SDR_INFO,
                "RX timed activation aligned: requested=%lld samples packet_start=%lld offset=%zu dropped_packets=%zu dropped_samples=%lld",
                (long long)target_sample,
                (long long)packet_start_sample,
                packet_offset,
                dropped_packets,
                (long long)dropped_samples);
        }

        const size_t byte_offset = packet_offset * _frameChannels * _bytesPerComplex;
        handle    = pkt->index;
        buffs[0]  = pkt->data + byte_offset;
        flags    |= packet_flags;
        timeNs    = packet_timeNs;
        return static_cast<int>(packet_samps);
    }
}

/* Release a read buffer after use: return the StreamPacket to the
 * worker's free pool. Callers must not release a packet returned with
 * handle = -1 (overflow); the worker already reclaimed it.
 */
void SoapyLiteXM2SDR::releaseReadBuffer(
    SoapySDR::Stream */*stream*/,
    size_t handle) {
    assert(handle != (size_t)-1 && "Attempt to release an invalid buffer (e.g., from an overflow).");
    if (!_rx_stream.free_fifo) return;
    if (handle >= _rx_stream.pkt_pool.size()) return;
    _rx_stream.free_fifo->push(&_rx_stream.pkt_pool[handle], /*wait=*/true);
}

/* Acquire a buffer for writing (worker FIFO producer).
 *
 * Pops a free StreamPacket from the TX worker's free pool, exposes its
 * payload to the caller, and stamps the caller's handle with the pool
 * index. The caller fills `buffs[0]` then calls releaseWriteBuffer().
 *
 * The DMA pump (litepcie reader) is owned exclusively by the TX worker
 * thread; this call no longer touches DMA counters or pollfds.
 */
int SoapyLiteXM2SDR::acquireWriteBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    void **buffs,
    const long timeoutUs) {
    if (stream != TX_STREAM) {
        return SOAPY_SDR_STREAM_ERROR;
    }
    if (!_tx_stream.free_fifo || !_tx_stream.work_fifo) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    litex_m2sdr::StreamPacket *pkt = nullptr;
    if (!_tx_stream.free_fifo->pop(&pkt,
                                   /*wait=*/true,
                                   std::chrono::microseconds(timeoutUs))) {
        return SOAPY_SDR_TIMEOUT;
    }
    handle    = pkt->index;
    buffs[0]  = pkt->data;
    /* Caller-owned metadata is set on release(). */
    pkt->samples = 0;
    pkt->flags   = 0;
    pkt->timeNs  = 0;
    pkt->ret_code = 0;

    /* Report any pending underflow back to the caller. The worker
     * increments underflow_count when the DMA queue empties; surface
     * it here so callers using the direct API see one underflow
     * notification per occurrence, then consume the counter so we
     * don't repeat-report the same underflow on every subsequent
     * acquireWriteBuffer. readStreamStatus also reads/consumes the
     * counter independently. Callers who want to do nothing with the
     * underflow info can ignore the return code; the buffer is still
     * available to them (we already popped it from free_fifo).
     */
    const uint64_t cur = _tx_stream.underflow_count.load(std::memory_order_relaxed);
    if (cur > _tx_stream.reported_underflow_count) {
        _tx_stream.reported_underflow_count = cur;
        /* Push the packet back: caller will retry, and won't get a
         * "phantom" buffer that they then have no way to free. */
        _tx_stream.free_fifo->push(pkt, /*wait=*/true);
        return SOAPY_SDR_UNDERFLOW;
    }
    return static_cast<int>(this->getStreamMTU(stream));
}

/* Release a write buffer after use: push the StreamPacket into the
 * worker's work queue so the TX worker can copy it into the DMA buffer,
 * wait for HAS_TIME alignment, and commit to hardware.
 *
 * The blocking wait for hardware time previously done here is now
 * performed inside the worker so writeStream() returns immediately.
 */
void SoapyLiteXM2SDR::releaseWriteBuffer(
    SoapySDR::Stream *stream,
    size_t handle,
    const size_t numElems,
    int &flags,
    const long long timeNs) {
    (void)stream;
    if (!_tx_stream.work_fifo) return;
    if (handle >= _tx_stream.pkt_pool.size()) return;

    litex_m2sdr::StreamPacket *pkt = &_tx_stream.pkt_pool[handle];
    pkt->samples = static_cast<uint32_t>(numElems);
    pkt->flags   = flags;
    pkt->timeNs  = timeNs;
    pkt->ret_code = 0;
    _tx_stream.work_fifo->push(pkt, /*wait=*/true);
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
            dst_int16 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        int8_t *dst_int8 = reinterpret_cast<int8_t*>(dst) + (offset * 2 * _samplesPerComplex);
        for (uint32_t i = 0; i < len; i++) {
            dst_int8[0] = static_cast<int8_t>(samples_cf32[0] * (_samplesScaling)); /* I. */
            dst_int8[1] = static_cast<int8_t>(samples_cf32[1] * (_samplesScaling)); /* Q. */
            samples_cf32 += 2;
            dst_int8 += _frameChannels * _samplesPerComplex;
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
            src_int16 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        const int8_t *src_int8 = reinterpret_cast<const int8_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            /* Scale to float (-1.0 to 1.0 range) */
            samples_cf32[0] = static_cast<float>(src_int8[0]) / _samplesScaling; /* I. */
            samples_cf32[1] = static_cast<float>(src_int8[1]) / _samplesScaling; /* Q. */
            samples_cf32 += 2;
            src_int8 += _frameChannels * _samplesPerComplex;
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

        const bool cs16_to_sc12 = m2sdr_env_tx_cs16_to_sc12_enabled();

        for (uint32_t i = 0; i < len; i++) {
            if (cs16_to_sc12) {
                dst_int16[0] = m2sdr_cs16_to_sc12_word(samples_cs16[0]); /* I. */
                dst_int16[1] = m2sdr_cs16_to_sc12_word(samples_cs16[1]); /* Q. */
            } else {
                dst_int16[0] = samples_cs16[0]; /* I. */
                dst_int16[1] = samples_cs16[1]; /* Q. */
            }
            samples_cs16 += 2;
            dst_int16 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        int8_t *dst_int8 = reinterpret_cast<int8_t*>(dst) + (offset * 2 * _samplesPerComplex);

        for (uint32_t i = 0; i < len; i++) {
            dst_int8[0] = samples_cs16[0]; /* I. */
            dst_int8[1] = samples_cs16[1]; /* Q. */
            samples_cs16 += 2;
            dst_int8 += _frameChannels * _samplesPerComplex;
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
            src_int16 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 1) {
        const int8_t *src_int8 = reinterpret_cast<const int8_t*>(src);

        for (uint32_t i = 0; i < len; i++) {
            samples_cs16[0] = static_cast<int16_t>(src_int8[0]); /* I. */
            samples_cs16[1] = static_cast<int16_t>(src_int8[1]); /* Q. */
            samples_cs16 += 2;
            src_int8 += _frameChannels * _samplesPerComplex;
        }
    } else {
        printf("Unsupported _bytesPerSample value: %u\n", _bytesPerSample);
    }
}

/* Interleave CS8 samples */
void SoapyLiteXM2SDR::interleaveCS8(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    const int8_t *samples_cs8 = reinterpret_cast<const int8_t*>(src) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 1) {
        int8_t *dst_int8 = reinterpret_cast<int8_t*>(dst) + (offset * 2 * _samplesPerComplex);
        for (uint32_t i = 0; i < len; i++) {
            dst_int8[0] = samples_cs8[0]; /* I. */
            dst_int8[1] = samples_cs8[1]; /* Q. */
            samples_cs8 += 2;
            dst_int8 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 2) {
        int16_t *dst_int16 = reinterpret_cast<int16_t*>(dst) + (offset * 2 * _samplesPerComplex);
        for (uint32_t i = 0; i < len; i++) {
            dst_int16[0] = static_cast<int16_t>(samples_cs8[0]) << 4; /* I. */
            dst_int16[1] = static_cast<int16_t>(samples_cs8[1]) << 4; /* Q. */
            samples_cs8 += 2;
            dst_int16 += _frameChannels * _samplesPerComplex;
        }
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported _bytesPerSample value: %u.", _bytesPerSample);
    }
}

/* Deinterleave CS8 samples */
void SoapyLiteXM2SDR::deinterleaveCS8(
    const void *src,
    void *dst,
    uint32_t len,
    size_t offset) {
    int8_t *samples_cs8 = reinterpret_cast<int8_t*>(dst) + (offset * _samplesPerComplex);

    if (_bytesPerSample == 1) {
        const int8_t *src_int8 = reinterpret_cast<const int8_t*>(src);
        for (uint32_t i = 0; i < len; i++) {
            samples_cs8[0] = src_int8[0]; /* I. */
            samples_cs8[1] = src_int8[1]; /* Q. */
            samples_cs8 += 2;
            src_int8 += _frameChannels * _samplesPerComplex;
        }
    } else if (_bytesPerSample == 2) {
        const int16_t *src_int16 = reinterpret_cast<const int16_t*>(src);
        for (uint32_t i = 0; i < len; i++) {
            samples_cs8[0] = static_cast<int8_t>(src_int16[0] >> 4); /* I. */
            samples_cs8[1] = static_cast<int8_t>(src_int16[1] >> 4); /* Q. */
            samples_cs8 += 2;
            src_int16 += _frameChannels * _samplesPerComplex;
        }
    } else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unsupported _bytesPerSample value: %u.", _bytesPerSample);
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
    } else if (format == SOAPY_SDR_CS8) {
        interleaveCS8(src, dst, len, offset);
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
    } else if (format == SOAPY_SDR_CS8) {
        deinterleaveCS8(src, dst, len, offset);
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

    flags = 0;
    timeNs = 0;

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));
    if (returnedElems == 0) {
        return 0;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(timeoutUs > 0 ? timeoutUs : 0);
    auto remaining_timeout_us = [&]() -> long {
        if (timeoutUs <= 0) return timeoutUs;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return 0;
        return static_cast<long>(
            std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count());
    };

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous read, process that first. */
    if (_rx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_rx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _rx_stream.remainderOffset * _frameChannels * _bytesPerComplex;

        if (n < returnedElems) {
            samp_avail = n;
        }

        if (_rx_stream.time_valid) {
            timeNs = _rx_stream.remainderTimeNs +
                     samples_to_ns(_rx_stream.samplerate, _rx_stream.remainderOffset);
            flags |= SOAPY_SDR_HAS_TIME;
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

    /* Acquire a new read buffer. When timed activation is pending, emulate
     * LimeSDR's contract in software: discard early packets and trim the
     * first packet that overlaps the requested sample.
     */
    int ret = 0;
    size_t dropped_packets = 0;
    long long dropped_samples = 0;
    while (true) {
        size_t handle;
        int acquire_flags = 0;
        long long acquire_timeNs = 0;
        int acquire_ret = this->acquireReadBuffer(
            stream,
            handle,
            (const void **)&_rx_stream.remainderBuff,
            acquire_flags,
            acquire_timeNs,
            remaining_timeout_us());

        if (acquire_ret < 0) {
            if ((acquire_ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)) {
                return samp_avail;
            }
            flags |= acquire_flags;
            timeNs = acquire_timeNs;
            return acquire_ret;
        }

        uint32_t packet_offset = 0;
        uint32_t packet_samps = static_cast<uint32_t>(acquire_ret);
        if (_rx_stream.timed_start_pending) {
            if (!_rx_stream.time_valid || !(acquire_flags & SOAPY_SDR_HAS_TIME)) {
                this->releaseReadBuffer(stream, handle);
                SoapySDR::log(SOAPY_SDR_ERROR,
                    "RX timed activation cannot align because RX packets have no timestamp");
                return SOAPY_SDR_STREAM_ERROR;
            }

            const long long packet_start_sample = ns_to_samples(_rx_stream.samplerate, acquire_timeNs);
            const long long packet_end_sample = packet_start_sample + static_cast<long long>(packet_samps);
            const long long target_sample = _rx_stream.timed_start_sample;

            if (packet_end_sample <= target_sample) {
                dropped_packets++;
                dropped_samples += static_cast<long long>(packet_samps);
                this->releaseReadBuffer(stream, handle);
                continue;
            }

            if (packet_start_sample > target_sample) {
                this->releaseReadBuffer(stream, handle);
                _rx_stream.timed_start_pending = false;
                timeNs = samples_to_ns(_rx_stream.samplerate, target_sample);
                flags |= SOAPY_SDR_HAS_TIME;
                SoapySDR::logf(SOAPY_SDR_WARNING,
                    "RX timed activation missed target: requested=%lld samples first=%lld samples delta=%lld samples",
                    (long long)target_sample,
                    (long long)packet_start_sample,
                    (long long)(packet_start_sample - target_sample));
                return SOAPY_SDR_TIME_ERROR;
            }

            packet_offset = static_cast<uint32_t>(target_sample - packet_start_sample);
            packet_samps -= packet_offset;
            _rx_stream.timed_start_pending = false;
            SoapySDR::logf(SOAPY_SDR_INFO,
                "RX timed activation aligned: requested=%lld samples packet_start=%lld offset=%u dropped_packets=%zu dropped_samples=%lld",
                (long long)target_sample,
                (long long)packet_start_sample,
                packet_offset,
                dropped_packets,
                (long long)dropped_samples);
        }

        _rx_stream.remainderHandle = handle;
        _rx_stream.remainderSamps = packet_samps;
        _rx_stream.remainderOffset = packet_offset;
        _rx_stream.remainderTimeNs = acquire_timeNs;
        flags |= acquire_flags;
        timeNs = acquire_timeNs;
        ret = static_cast<int>(packet_samps);
        break;
    }

    (void)ret;
    const size_t n = std::min((returnedElems - samp_avail), _rx_stream.remainderSamps);
    const uint32_t remainderOffset = _rx_stream.remainderOffset * _frameChannels * _bytesPerComplex;

    if (_rx_stream.time_valid) {
        timeNs = _rx_stream.remainderTimeNs +
                 samples_to_ns(_rx_stream.samplerate, _rx_stream.remainderOffset);
        flags |= SOAPY_SDR_HAS_TIME;
    } else if (_rx_stream.samplerate <= 0.0 && !_rx_stream.time_warned) {
        SoapySDR::log(SOAPY_SDR_WARNING,
            "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
        _rx_stream.time_warned = true;
    }

    /* Read out channels from the new buffer. */
    for (size_t i = 0; i < _rx_stream.channels.size(); i++) {
        const uint32_t chan = _rx_stream.channels[i];
        this->deinterleave(
            _rx_stream.remainderBuff + (remainderOffset + chan * _bytesPerComplex),
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

    return samp_avail + n;
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

    /* Determine the number of samples to return, respecting the MTU. */
    size_t returnedElems = std::min(numElems, this->getStreamMTU(stream));

    size_t samp_avail = 0;

    /* If there's a remainder buffer from a previous write, process that first. */
    if (_tx_stream.remainderHandle >= 0) {
        const size_t n = std::min(_tx_stream.remainderSamps, returnedElems);
        const uint32_t remainderOffset = _tx_stream.remainderOffset * _frameChannels * _bytesPerComplex;

        if (n < returnedElems) {
            samp_avail = n;
        }

        /* Write out channels to the remainder buffer. */
        for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
            m2sdr_log_tx_input_rms_once(
                buffs[i],
                n,
                _tx_stream.format,
                _tx_stream.channels[i],
                flags,
                timeNs
            );
            m2sdr_dump_tx_input_once(
                buffs[i],
                n,
                _tx_stream.format,
                _tx_stream.channels[i],
                0,
                flags,
                timeNs,
                _frameChannels,
                _bytesPerComplex,
                _tx_stream.samplerate
            );
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

        /* Merge late-arriving flag bits (END_BURST may land on the
         * last partial write that closes a buffer; HAS_TIME must NOT
         * overwrite a timestamp set by an earlier writeStream that
         * opened this buffer — the buffer fires from the first ts). */
        if ((flags & SOAPY_SDR_HAS_TIME) &&
            !(_tx_stream.remainder_flags & SOAPY_SDR_HAS_TIME)) {
            _tx_stream.remainder_flags  |= SOAPY_SDR_HAS_TIME;
            _tx_stream.remainder_timeNs = timeNs;
        }
        _tx_stream.remainder_flags |= (flags & SOAPY_SDR_END_BURST);

        if (_tx_stream.remainderSamps == 0) {
            int merged_flags = _tx_stream.remainder_flags;
            this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset,
                                     merged_flags, _tx_stream.remainder_timeNs);
            _tx_stream.remainderHandle = -1;
            _tx_stream.remainderOffset = 0;
            _tx_stream.remainder_flags = 0;
            _tx_stream.remainder_timeNs = 0;
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
    /* First writeStream into this fresh buffer: its flags/timeNs
     * become the buffer's metadata. Subsequent writes into the same
     * buffer may OR in END_BURST but cannot rewrite HAS_TIME's ts. */
    _tx_stream.remainder_flags  = flags;
    _tx_stream.remainder_timeNs = timeNs;

    const size_t n = std::min((returnedElems - samp_avail), _tx_stream.remainderSamps);

    /* Write out channels to the new buffer. */
    for (size_t i = 0; i < _tx_stream.channels.size(); i++) {
        m2sdr_log_tx_input_rms_once(
            buffs[i],
            n,
            _tx_stream.format,
            _tx_stream.channels[i],
            flags,
            timeNs
        );
            m2sdr_dump_tx_input_once(
                buffs[i],
                n,
                _tx_stream.format,
                _tx_stream.channels[i],
                samp_avail,
                flags,
                timeNs,
                _frameChannels,
                _bytesPerComplex,
                _tx_stream.samplerate
            );
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
        int merged_flags = _tx_stream.remainder_flags;
        this->releaseWriteBuffer(stream, _tx_stream.remainderHandle, _tx_stream.remainderOffset,
                                 merged_flags, _tx_stream.remainder_timeNs);
        _tx_stream.remainderHandle = -1;
        _tx_stream.remainderOffset = 0;
        _tx_stream.remainder_flags = 0;
        _tx_stream.remainder_timeNs = 0;
    }

    return returnedElems;
}

/* Check the status of the TX/RX streams.
 *
 * Worker threads maintain monotonically-increasing atomic counters for
 * overflow (RX), underflow (TX), and late (TX timed submission slipped
 * past the requested timestamp). Compare against the "reported" value
 * stamped in this method to surface exactly one Soapy event per fault
 * occurrence.
 */
int SoapyLiteXM2SDR::readStreamStatus(
    SoapySDR::Stream *stream,
    size_t &/*chanMask*/,
    int &flags,
    long long &/*timeNs*/,
    const long timeoutUs) {

    auto poll_once = [&]() -> int {
        if (stream == RX_STREAM) {
            const uint64_t cur = _rx_stream.overflow_count.load(std::memory_order_relaxed);
            if (cur > _rx_stream.reported_overflow_count) {
                _rx_stream.reported_overflow_count = cur;
                SoapySDR::log(SOAPY_SDR_SSI, "O");
                return SOAPY_SDR_OVERFLOW;
            }
            return 0;
        }
        if (stream == TX_STREAM) {
            const uint64_t under = _tx_stream.underflow_count.load(std::memory_order_relaxed);
            if (under > _tx_stream.reported_underflow_count) {
                _tx_stream.reported_underflow_count = under;
                SoapySDR::log(SOAPY_SDR_SSI, "U");
                return SOAPY_SDR_UNDERFLOW;
            }
            const uint64_t late = _tx_stream.late_count.load(std::memory_order_relaxed);
            if (late > _tx_stream.reported_late_count) {
                _tx_stream.reported_late_count = late;
                SoapySDR::log(SOAPY_SDR_SSI, "L");
                flags |= SOAPY_SDR_HAS_TIME;
                return SOAPY_SDR_TIME_ERROR;
            }
            return 0;
        }
        return SOAPY_SDR_NOT_SUPPORTED;
    };

    int ret = poll_once();
    if (ret != 0) return ret;

    const auto exitTime = std::chrono::high_resolution_clock::now() +
        std::chrono::microseconds(timeoutUs);
    while (std::chrono::high_resolution_clock::now() < exitTime) {
        const long sleepUs = std::min<long>(1000, std::max<long>(100, timeoutUs / 10));
        std::this_thread::sleep_for(std::chrono::microseconds(sleepUs));
        ret = poll_once();
        if (ret != 0) return ret;
    }
    return SOAPY_SDR_TIMEOUT;
}

/* ===========================================================================
 *                          DMA-side helpers (worker)
 *
 * Only the per-direction worker thread is allowed to call these. They
 * are the lifted bodies of the previous synchronous acquire/release
 * implementations, adapted to a plain pointer/handle interface (no
 * SoapySDR::Stream*) and stripped of the public direct-access plumbing.
 * ===========================================================================
 */

int SoapyLiteXM2SDR::_dmaAcquireRead(
    size_t &handle,
    uint8_t **buf,
    int &flags,
    long long &timeNs,
    const long timeoutUs) {
    if (_rx_stream.burst_end)
        flags |= SOAPY_SDR_END_BURST;

#if USE_LITEETH
    liteeth_udp_process(&_udp, static_cast<int>(timeoutUs / 1000));

    int avail = liteeth_udp_buffers_available_read(&_udp);
    if (avail <= 0) {
        return SOAPY_SDR_TIMEOUT;
    }
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
            SoapySDR_logf(SOAPY_SDR_WARNING,
                "Invalid/unsupported VRT RX packet (type=%u words=%u)",
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

        *buf = src + VRT_SIGNAL_HEADER_BYTES;
        handle = 0;
        return static_cast<int>(payload_bytes / (_frameChannels * _bytesPerComplex));
    }

    *buf = src;
    handle = 0;
    return getStreamMTU(RX_STREAM);

#elif USE_LITEPCIE
    const auto find_sync = [](const std::vector<uint8_t> &bytes) -> size_t {
        if (bytes.size() < sizeof(uint64_t)) return std::string::npos;
        for (size_t pos = 0; pos + sizeof(uint64_t) <= bytes.size(); pos++) {
            uint64_t word = 0;
            std::memcpy(&word, bytes.data() + pos, sizeof(word));
            if (word == RX_HEADER_SYNC_VALUE) return pos;
        }
        return std::string::npos;
    };

    auto append_dma_buffer = [&]() -> int {
        /*
         * Non-zero-copy mode does not expose the kernel DMA ring through
         * _rx_stream.buf. It must drive liblitepcie's read() path, then pull
         * populated userspace buffers with litepcie_dma_next_read_buffer().
         */
        if (!_rx_stream.dma.zero_copy) {
            _rx_stream.dma.writer_enable = 1;
            litepcie_dma_process(&_rx_stream.dma);

            auto &q = _rx_stream.rx_reframe_buf;
            size_t appended = 0;

            while (true) {
                char *copy_buf = litepcie_dma_next_read_buffer(&_rx_stream.dma);
                if (!copy_buf)
                    break;

                const uint8_t *raw = reinterpret_cast<const uint8_t *>(copy_buf);
                q.insert(q.end(), raw, raw + _rx_stream.dma.mmap_dma_info.dma_rx_buf_size);
                appended++;
            }

            return appended ? 0 : SOAPY_SDR_TIMEOUT;
        }

        int buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
        assert(buffers_available >= 0);
        if (buffers_available == 0 || DETECT_EVERY_OVERFLOW) {
            litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
            buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
        }
        if (buffers_available == 0) {
            if (timeoutUs == 0) return SOAPY_SDR_TIMEOUT;
            int ret = poll(&_rx_stream.fds, 1, timeoutUs / 1000);
            if (ret < 0) {
                throw std::runtime_error("_dmaAcquireRead: Poll failed, " +
                                         std::string(strerror(errno)) + ".");
            } else if (ret == 0) {
                return SOAPY_SDR_TIMEOUT;
            }
            litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
            buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
            assert(buffers_available > 0);
        }

        if ((_rx_stream.hw_count - _rx_stream.sw_count) >
            ((int64_t)_dma_mmap_info.dma_rx_buf_count / 2)) {
            const int64_t lost_buffers = _rx_stream.hw_count - _rx_stream.sw_count;
            struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
            mmap_dma_update.sw_count = _rx_stream.hw_count;
            checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
            _rx_stream.user_count = _rx_stream.hw_count;
            _rx_stream.sw_count   = _rx_stream.hw_count;
            _rx_stream.rx_reframe_buf.clear();
            _rx_stream.rx_payload_buf.clear();
            flags |= SOAPY_SDR_END_ABRUPT;
            _rx_stream.time0_ns      = this->getHardwareTime("");
            _rx_stream.time0_count   = _rx_stream.user_count;
            _rx_stream.time_valid    = (_rx_stream.samplerate > 0.0);
            _rx_stream.time_anchored = false;
            _rx_stream.last_time_ns  = _rx_stream.time0_ns;
            _rx_stream.time_warned   = false;
            {
                const int64_t max_count = (LITEX_OVERFLOW_COUNT_MASK >> LITEX_OVERFLOW_COUNT_SHIFT);
                const int64_t clamped = (lost_buffers > max_count) ? max_count : lost_buffers;
                flags |= LITEX_HAS_OVERFLOW_COUNT;
                flags |= ((int)clamped << LITEX_OVERFLOW_COUNT_SHIFT) & LITEX_OVERFLOW_COUNT_MASK;
            }
            return SOAPY_SDR_OVERFLOW;
        }

        const int buf_offset = _rx_stream.user_count % _dma_mmap_info.dma_rx_buf_count;
        const uint8_t *raw = reinterpret_cast<const uint8_t *>(_rx_stream.buf) +
                             buf_offset * _dma_mmap_info.dma_rx_buf_size;

        const char *hdr_trace_env = std::getenv("M2SDR_SOAPY_RX_HDR_TRACE");
        const uint32_t hdr_trace_limit = hdr_trace_env ? static_cast<uint32_t>(std::strtoul(hdr_trace_env, nullptr, 0)) : 0;
        if (hdr_trace_limit && _rx_stream.hdr_trace_count < hdr_trace_limit) {
            uint64_t header_word0 = 0;
            uint64_t ts_samples = 0;
            std::memcpy(&header_word0, raw, sizeof(header_word0));
            std::memcpy(&ts_samples, raw + 8, sizeof(ts_samples));
            int sync_offset = -1;
            const size_t scan_words = _dma_mmap_info.dma_rx_buf_size / sizeof(uint64_t);
            const uint64_t *words = reinterpret_cast<const uint64_t *>(raw);
            for (size_t i = 0; i < scan_words; i++) {
                if (words[i] == RX_HEADER_SYNC_VALUE) {
                    sync_offset = static_cast<int>(i * sizeof(uint64_t));
                    break;
                }
            }
            SoapySDR_logf(SOAPY_SDR_INFO,
                "RX raw[%u]: user=%lld hw=%lld sw=%lld off=%d word0=0x%016llx ts=%llu sync_off=%d",
                _rx_stream.hdr_trace_count,
                (long long)_rx_stream.user_count,
                (long long)_rx_stream.hw_count,
                (long long)_rx_stream.sw_count,
                buf_offset,
                (unsigned long long)header_word0,
                (unsigned long long)ts_samples,
                sync_offset);
            _rx_stream.hdr_trace_count++;
        }

        auto &q = _rx_stream.rx_reframe_buf;
        q.insert(q.end(), raw, raw + _dma_mmap_info.dma_rx_buf_size);

        const size_t raw_handle = static_cast<size_t>(_rx_stream.user_count);
        _rx_stream.user_count++;
        _dmaReleaseRead(raw_handle);
        return 0;
    };

    auto &q = _rx_stream.rx_reframe_buf;
    while (_rx_stream.worker_running.load(std::memory_order_acquire)) {
        const size_t sync_pos = find_sync(q);
        if (sync_pos != std::string::npos) {
            if (sync_pos != 0) {
                q.erase(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(sync_pos));
            }
            if (q.size() >= RX_STREAM_PACKET_SIZE) {
                uint64_t header_word0 = 0;
                uint64_t ts_samples = 0;
                std::memcpy(&header_word0, q.data(), sizeof(header_word0));
                std::memcpy(&ts_samples, q.data() + 8, sizeof(ts_samples));

                if (header_word0 != RX_HEADER_SYNC_VALUE) {
                    q.erase(q.begin());
                    continue;
                }

                _rx_stream.rx_payload_buf.resize(RX_STREAM_PAYLOAD_BYTES);
                std::memcpy(_rx_stream.rx_payload_buf.data(),
                            q.data() + RX_DMA_HEADER_SIZE,
                            RX_STREAM_PAYLOAD_BYTES);
                q.erase(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(RX_STREAM_PACKET_SIZE));

                if (_rx_stream.samplerate > 0.0) {
                    timeNs = samples_to_ns(_rx_stream.samplerate,
                                           static_cast<long long>(ts_samples));
                    flags |= SOAPY_SDR_HAS_TIME;
                } else if (!_rx_stream.time_warned) {
                    SoapySDR::log(SOAPY_SDR_WARNING,
                        "RX sample rate not set; not providing SOAPY_SDR_HAS_TIME");
                    _rx_stream.time_warned = true;
                }

                *buf = _rx_stream.rx_payload_buf.data();
                handle = static_cast<size_t>(-1);
                return static_cast<int>(RX_STREAM_PAYLOAD_BYTES /
                                        (_frameChannels * _bytesPerComplex));
            }
        } else if (q.size() > sizeof(uint64_t) - 1) {
            /* Keep enough trailing bytes to detect a sync word that spans
             * two DMA buffers; in practice LitePCIe gives us 64-bit aligned
             * cuts, but this keeps the parser honest.
             */
            q.erase(q.begin(), q.end() - static_cast<std::ptrdiff_t>(sizeof(uint64_t) - 1));
        }

        const int ret = append_dma_buffer();
        if (ret < 0) {
            handle = static_cast<size_t>(-1);
            return ret;
        }
    }

    handle = static_cast<size_t>(-1);
    return SOAPY_SDR_TIMEOUT;

#endif
}

void SoapyLiteXM2SDR::_dmaReleaseRead(size_t handle) {
#if USE_LITEPCIE
    /*
     * RX copy-mode uses liblitepcie's read(fd, dma.buf_rd, ...) path from
     * litepcie_dma_process(), not mmap writer-counter updates. In this mode
     * _rx_stream.dma.buf_rd is userspace calloc() storage, and advancing mmap
     * writer counters is stale zero-copy bookkeeping.
     */
    if (!_rx_stream.dma.zero_copy) {
        return;
    }

    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = handle + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
#elif USE_LITEETH
    (void)handle;
#endif
}

void SoapyLiteXM2SDR::_clearAllTXHeaderFlags() {
#if USE_LITEPCIE
    static constexpr uint64_t TX_HEADER_FLAG_MASK =
        TX_FLAG_HAS_TIME | TX_FLAG_END_BURST;
    for (size_t i = 0; i < _dma_mmap_info.dma_tx_buf_count; ++i) {
        uint64_t *hdr0 = reinterpret_cast<uint64_t*>(
            reinterpret_cast<uint8_t*>(_tx_stream.buf) +
            i * _dma_mmap_info.dma_tx_buf_size);
        *hdr0 &= ~TX_HEADER_FLAG_MASK;
    }
#endif
}

void SoapyLiteXM2SDR::_clearConsumedTXHeaders() {
#if USE_LITEPCIE
    /* Once hw_count has advanced past a buffer index, the FPGA TX DMA
     * reader has read that buffer at least once. Clear the HAS_TIME /
     * END_BURST flag bits in the first 8 bytes of that buffer's
     * header so subsequent re-reads (LOOP_PROG_N=1 keeps cycling the
     * descriptor ring) latch has_time=0 and don't push stale-timed
     * descriptors into the arbiter's ts_fifo.
     *
     * We leave the 64-bit sync word's lower bits and the per-buffer
     * 8-byte timestamp untouched; the FPGA only cares about
     * bit 63 (HAS_TIME) and bit 62 (END_BURST) for arbitration.
     *
     * Race: there is a window between sw_count advance (in
     * _dmaReleaseWrite) and the FPGA's first read of the buffer
     * where we must NOT clear, or we'd race the FPGA latch. The
     * hw_count > buffer_index check guarantees the FPGA finished
     * reading the buffer at least once, so the descriptor was
     * already latched. Clearing now affects only the *next* time
     * the ring cycles to this slot.
     */
    static constexpr uint64_t TX_HEADER_FLAG_MASK =
        TX_FLAG_HAS_TIME | TX_FLAG_END_BURST;
    while (_tx_stream.cleared_count < _tx_stream.hw_count) {
        const int64_t idx = _tx_stream.cleared_count;
        const size_t buf_offset = idx % _dma_mmap_info.dma_tx_buf_count;
        uint64_t *hdr0 = reinterpret_cast<uint64_t*>(
            reinterpret_cast<uint8_t*>(_tx_stream.buf) +
            buf_offset * _dma_mmap_info.dma_tx_buf_size);
        *hdr0 &= ~TX_HEADER_FLAG_MASK;
        _tx_stream.cleared_count = idx + 1;
    }
#endif
}

int SoapyLiteXM2SDR::_dmaAcquireWrite(
    size_t &handle,
    uint8_t **buf,
    const long timeoutUs) {
#if USE_LITEPCIE
    /*
     * TX copy-mode uses liblitepcie write() path.
     *
     * In non-zero-copy mode litepcie_dma_init() allocates dma.buf_wr with
     * calloc(); it is not a kernel mmap ring. Therefore advancing the mmap
     * DMA reader sw_count does not deliver these userspace bytes to the FPGA.
     * The actual delivery mechanism is litepcie_dma_process(), which performs
     * write(fd, dma->buf_wr, DMA_BUFFER_TOTAL_SIZE) when POLLOUT is ready.
     */
    if (!_tx_stream.dma.zero_copy) {
        /*
         * Copy-mode staging: return a userspace slot directly.  The completed
         * slot is copied into the kernel in _dmaReleaseWrite() with write().
         * Do not call litepcie_dma_process() here, because that can enable the
         * FPGA reader and flush stale/previous userspace contents before this
         * buffer's header and payload are filled.
         */
        const size_t buf_offset = _tx_stream.user_count %
                                  _tx_stream.dma.mmap_dma_info.dma_tx_buf_count;
        uint8_t *tx_buffer = reinterpret_cast<uint8_t*>(_tx_stream.dma.buf_wr) +
                             buf_offset * _tx_stream.dma.mmap_dma_info.dma_tx_buf_size;

        *reinterpret_cast<uint64_t*>(tx_buffer + 0) = TX_HEADER_SYNC_VALUE;
        *reinterpret_cast<uint64_t*>(tx_buffer + 8) = 0;

        *buf = tx_buffer + TX_DMA_HEADER_SIZE;
        handle = _tx_stream.user_count++;
        (void)timeoutUs;
        return static_cast<int>(getStreamMTU(TX_STREAM));
    }

    /* Note on LitePCIe DMA reader mode: the kernel hardwires
     * LOOP_PROG_N=1, which makes the reader cycle the 16-entry
     * descriptor ring indefinitely. We can't switch to PROG mode here
     * because the kernel only loads the descriptor table once at start
     * - in PROG mode the reader would stall after 16 buffers and the
     * driver has no plumbing to refill descriptors. Instead, the
     * driver clears the HAS_TIME bit from per-buffer headers once the
     * FPGA has consumed them (hw_count advanced past their index), so
     * re-reads of the ring see has_time=0 and the TimedTXArbiter
     * doesn't spuriously flag stale bursts as late. See
     * _clearConsumedTXHeaders below.
     */
    int buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
    assert(buffers_pending <= (int)_dma_mmap_info.dma_tx_buf_count);
    /* Only refresh hw_count once the FPGA reader is running; before
     * the first releaseWrite, hw_count is 0 and the kernel would
     * actually *start* the reader if we passed enable=1 here. We
     * want the reader held off until slot 0 has our HAS_TIME=1
     * header in place. */
    if (_tx_stream.reader_enabled &&
        (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count) || DETECT_EVERY_UNDERFLOW)) {
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _clearConsumedTXHeaders();
        buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
        if (buffers_pending < 0) {
            _clearAllTXHeaderFlags();
            _tx_stream.user_count = _tx_stream.hw_count;
            _tx_stream.cleared_count = _tx_stream.hw_count;
            buffers_pending = 0;
        }
    }
    if (buffers_pending == ((int64_t)_dma_mmap_info.dma_tx_buf_count)) {
        if (timeoutUs == 0) return SOAPY_SDR_TIMEOUT;
        int ret = poll(&_tx_stream.fds, 1, timeoutUs / 1000);
        if (ret < 0) {
            throw std::runtime_error("_dmaAcquireWrite: Poll failed, " +
                                     std::string(strerror(errno)) + ".");
        } else if (ret == 0) {
            return SOAPY_SDR_TIMEOUT;
        }
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _clearConsumedTXHeaders();
        buffers_pending = _tx_stream.user_count - _tx_stream.hw_count;
        assert(buffers_pending < ((int64_t)_dma_mmap_info.dma_tx_buf_count));
    }

    const int buf_offset = _tx_stream.user_count % _dma_mmap_info.dma_tx_buf_count;
    *buf = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
           buf_offset * _dma_mmap_info.dma_tx_buf_size + TX_DMA_HEADER_SIZE;
    handle = _tx_stream.user_count;
    _tx_stream.user_count++;

    /* Initialise the 16-byte header to a no-flags sync word; the real
     * flags and timestamp are written in _dmaReleaseWrite where we
     * know HAS_TIME/END_BURST/timeNs from the caller. Writing a valid
     * sync now means the FPGA-side TimedTXArbiter sees a clean
     * untimed-burst marker if the buffer is ever read before release
     * (it shouldn't be, but defence in depth).
     */
    {
        uint8_t *tx_buffer = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
                             buf_offset * _dma_mmap_info.dma_tx_buf_size;
        *reinterpret_cast<uint64_t*>(tx_buffer + 0) = TX_HEADER_SYNC_VALUE;
        *reinterpret_cast<uint64_t*>(tx_buffer + 8) = 0;
    }

    /* `buffers_pending < 0` happens routinely in LOOP_PROG_N=1 mode:
     * the FPGA DMA reader cycles the descriptor ring continuously, so
     * hw_count outruns user_count any time the user isn't actively
     * submitting new data. This is NOT a sample underrun — the FPGA
     * is just re-reading stale (HAS_TIME-cleared) buffers, which the
     * TimedTXArbiter treats as untimed no-ops.
     *
     * The authoritative TX underflow signal for timed-TX use is the
     * TimedTXArbiter's late_count / underrun_count CSRs, surfaced
     * through readStreamStatus. Don't double-report at the DMA level.
     */
    return static_cast<int>(getStreamMTU(TX_STREAM));

#elif USE_LITEETH
    uint8_t *dst = liteeth_udp_next_write_buffer(&_udp);
    if (!dst) {
        return SOAPY_SDR_TIMEOUT;
    }
    *buf = dst;
    handle = _tx_stream.user_count++;
    _tx_stream.pendingWriteBufs[handle] = dst;
    (void)timeoutUs;
    return static_cast<int>(getStreamMTU(TX_STREAM));
#endif
}

void SoapyLiteXM2SDR::_dmaReleaseWrite(
    size_t handle,
    size_t numElems,
    int flags,
    long long timeNs) {
    /* Write the real per-buffer header so the FPGA-side TimedTXArbiter
     * sees the burst flags and the sample-count timestamp before the
     * buffer becomes visible to the DMA reader (sw_count advance
     * below). No driver-side spin-wait: the FPGA arbiter gates sample
     * release in the rfic clock domain against the SampleCounter, which
     * is the source of truth for sample-accurate TX timing.
     */
    if (flags & SOAPY_SDR_END_BURST) {
        _tx_stream.burst_end = true;
    }

    /*
     * Some clients submit continuation buffers with SOAPY_SDR_HAS_TIME and
     * the exact same timestamp as the burst-opening buffer.  The FPGA timed
     * arbiter interprets HAS_TIME as a new timed descriptor, so duplicate
     * timestamps can reopen the same burst and poison timing.  Keep HAS_TIME
     * only on the first buffer for a given timestamp; following buffers are
     * ordinary continuation payload until a new timestamp appears.
     */
    static long long last_has_time_ns = std::numeric_limits<long long>::min();
    if (m2sdr_env_tx_dedup_time_enabled() &&
        (flags & SOAPY_SDR_HAS_TIME) &&
        timeNs == last_has_time_ns) {
        flags &= ~SOAPY_SDR_HAS_TIME;
    } else if (flags & SOAPY_SDR_HAS_TIME) {
        last_has_time_ns = timeNs;
    }

#if USE_LITEPCIE
    {
        const size_t buf_offset = handle % _dma_mmap_info.dma_tx_buf_count;
        uint8_t *tx_buffer = reinterpret_cast<uint8_t*>(_tx_stream.buf) +
                             buf_offset * _dma_mmap_info.dma_tx_buf_size;
        uint64_t hdr = TX_HEADER_SYNC_VALUE;
        if (flags & SOAPY_SDR_HAS_TIME)   hdr |= TX_FLAG_HAS_TIME;
        if (flags & SOAPY_SDR_END_BURST)  hdr |= TX_FLAG_END_BURST;
        *reinterpret_cast<uint64_t*>(tx_buffer + 0) = hdr;
        const long long ts_samples = (flags & SOAPY_SDR_HAS_TIME)
            ? ns_to_samples(_tx_stream.samplerate, timeNs)
            : 0;
        *reinterpret_cast<uint64_t*>(tx_buffer + 8) =
            static_cast<uint64_t>(ts_samples);
    }
#endif

    const size_t mtu = this->getStreamMTU(TX_STREAM);
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
        }
#endif
        if (buf) {
            const size_t offset_bytes = numElems * _frameChannels * _bytesPerComplex;
            const size_t zero_bytes = (mtu - numElems) * _frameChannels * _bytesPerComplex;
            std::memset(buf + offset_bytes, 0, zero_bytes);
        }
    }

#if USE_LITEPCIE
    /*
     * TX copy-mode delivery: the payload lives in userspace calloc() storage,
     * so the completed single DMA buffer must be copied into the kernel here,
     * after its header and payload are valid.  Only start the FPGA reader once
     * a small priming queue is present.
     */
    if (!_tx_stream.dma.zero_copy) {
        const size_t buf_offset = handle % _tx_stream.dma.mmap_dma_info.dma_tx_buf_count;
        const size_t write_size = _tx_stream.dma.mmap_dma_info.dma_tx_buf_size;
        uint8_t *tx_buffer = reinterpret_cast<uint8_t*>(_tx_stream.dma.buf_wr) +
                             buf_offset * write_size;

        /*
         * Optional post-DMA-frame diagnostic dump.  At this point the Soapy
         * input samples have already been interleaved/framed, the 16-byte TX
         * DMA header has been finalized, and partial buffers have been
         * zero-padded.  This is the exact buffer copy-mode hands to the
         * LitePCIe kernel driver with write().
         */
        m2sdr_dump_tx_dma_buffer_once(
            tx_buffer,
            write_size,
            handle,
            numElems,
            flags,
            timeNs,
            _frameChannels,
            _bytesPerComplex,
            _tx_stream.samplerate,
            /*zeroCopy=*/false);

        ssize_t written = 0;
        while (written < static_cast<ssize_t>(write_size)) {
            ssize_t ret = ::write(_fd, tx_buffer + written, write_size - written);
            if (ret < 0) {
                if (errno == EINTR)
                    continue;
                SoapySDR_logf(SOAPY_SDR_ERROR,
                    "LitePCIe TX copy-mode write failed: errno=%d (%s)",
                    errno, std::strerror(errno));
                return;
            }
            if (ret == 0) {
                SoapySDR_logf(SOAPY_SDR_ERROR,
                    "LitePCIe TX copy-mode write returned 0 before completing buffer");
                return;
            }
            written += ret;
        }

        const long prime = m2sdr_env_tx_copy_prime_buffers();
        if (!_tx_stream.reader_enabled &&
            _tx_stream.user_count >= static_cast<int64_t>(prime)) {
            litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
            _tx_stream.reader_enabled = true;
            SoapySDR_logf(SOAPY_SDR_INFO,
                "LitePCIe TX copy-mode reader enabled after priming %ld buffers",
                prime);
        }

        return;
    }

    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = handle + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &mmap_dma_update);
    /* First releaseWrite after activation: start the FPGA DMA reader
     * now that slot 0 (and any preceding slots from this submission)
     * carry valid user headers. The kernel won't actually start a
     * reader that's already running, so subsequent calls are no-ops. */
    if (!_tx_stream.reader_enabled) {
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
        _tx_stream.reader_enabled = true;
    }
#elif USE_LITEETH
    if (numElems >= mtu) {
        auto it = _tx_stream.pendingWriteBufs.find(handle);
        if (it != _tx_stream.pendingWriteBufs.end()) {
            _tx_stream.pendingWriteBufs.erase(it);
        }
    }
    if (liteeth_udp_write_submit(&_udp) < 0) {
        _tx_stream.underflow_count.fetch_add(1, std::memory_order_relaxed);
        SoapySDR_logf(SOAPY_SDR_ERROR, "UDP write_submit failed.");
    }
#endif
}

/* ===========================================================================
 *                          Worker thread bodies
 * ===========================================================================
 */

void SoapyLiteXM2SDR::rxWorkerLoop() {
    (void)litex_m2sdr::setOsThreadPriority(
        litex_m2sdr::ThreadPriority::HIGH,
        litex_m2sdr::ThreadPolicy::REALTIME, nullptr);

    while (_rx_stream.worker_running.load(std::memory_order_acquire)) {
        litex_m2sdr::StreamPacket *pkt = nullptr;
        if (!_rx_stream.free_fifo->pop(&pkt,
                                       /*wait=*/true,
                                       std::chrono::milliseconds(50))) {
            continue;
        }

        size_t handle = 0;
        uint8_t *dma_buf = nullptr;
        int flags = 0;
        long long timeNs = 0;
        const int ret = _dmaAcquireRead(handle, &dma_buf, flags, timeNs,
                                        /*timeoutUs=*/50000);

        if (ret == SOAPY_SDR_TIMEOUT) {
            _rx_stream.free_fifo->push(pkt, /*wait=*/true);
            continue;
        }
        if (ret == SOAPY_SDR_OVERFLOW) {
            _rx_stream.overflow_count.fetch_add(1, std::memory_order_relaxed);
            pkt->ret_code = SOAPY_SDR_OVERFLOW;
            pkt->samples  = 0;
            pkt->flags    = flags;
            pkt->timeNs   = timeNs;
            _rx_stream.work_fifo->push(pkt, /*wait=*/true);
            continue;
        }
        if (ret < 0) {
            pkt->ret_code = ret;
            pkt->samples  = 0;
            pkt->flags    = flags;
            pkt->timeNs   = timeNs;
            _rx_stream.work_fifo->push(pkt, /*wait=*/true);
            continue;
        }

        const size_t bytes = static_cast<size_t>(ret) *
                             _frameChannels * _bytesPerComplex;
        if (bytes > pkt->capacity) {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                "RX worker: DMA returned %zu bytes > pkt capacity %zu",
                bytes, pkt->capacity);
            if (handle != static_cast<size_t>(-1)) {
                _dmaReleaseRead(handle);
            }
            pkt->ret_code = SOAPY_SDR_STREAM_ERROR;
            pkt->samples  = 0;
            _rx_stream.work_fifo->push(pkt, /*wait=*/true);
            continue;
        }
        std::memcpy(pkt->data, dma_buf, bytes);
        pkt->samples  = static_cast<uint32_t>(ret);
        pkt->flags    = flags;
        pkt->timeNs   = timeNs;
        pkt->ret_code = 0;
        if (handle != static_cast<size_t>(-1)) {
            _dmaReleaseRead(handle);
        }
        _rx_stream.work_fifo->push(pkt, /*wait=*/true);
    }
}

void SoapyLiteXM2SDR::txWorkerLoop() {
    (void)litex_m2sdr::setOsThreadPriority(
        litex_m2sdr::ThreadPriority::HIGH,
        litex_m2sdr::ThreadPolicy::REALTIME, nullptr);

    while (_tx_stream.worker_running.load(std::memory_order_acquire)) {
        litex_m2sdr::StreamPacket *pkt = nullptr;
        const bool tx_idle_fill =
#if USE_LITEPCIE
            m2sdr_env_tx_idle_fill_enabled() &&
            !_tx_stream.dma.zero_copy &&
            _tx_stream.reader_enabled;
#else
            false;
#endif

        if (!_tx_stream.work_fifo->pop(&pkt,
                                       /*wait=*/!tx_idle_fill,
                                       tx_idle_fill ? std::chrono::microseconds(0)
                                                    : std::chrono::milliseconds(50))) {
#if USE_LITEPCIE
            /*
             * Copy-mode idle fill: OCUDU submits sparse/discontinuous timed
             * buffers.  Once the FPGA DMA reader is running, leaving it idle
             * lets reader_hw_count outrun reader_sw_count, so the next real
             * write() triggers kernel "Writing too late" messages.  Feed
             * zero, no-HAS_TIME DMA frames during gaps; write() naturally
             * back-pressures when the kernel queue is full.
             */
            if (tx_idle_fill) {
                size_t idle_handle = 0;
                uint8_t *idle_dma_buf = nullptr;
                const int idle_ret = _dmaAcquireWrite(idle_handle, &idle_dma_buf,
                                                      /*timeoutUs=*/0);
                if (idle_ret > 0 && idle_dma_buf) {
                    const size_t mtu_bytes =
                        static_cast<size_t>(this->getStreamMTU(TX_STREAM)) *
                        _frameChannels * _bytesPerComplex;
                    std::memset(idle_dma_buf, 0, mtu_bytes);
                    _dmaReleaseWrite(idle_handle,
                                     this->getStreamMTU(TX_STREAM),
                                     /*flags=*/0,
                                     /*timeNs=*/0);
                }
                continue;
            }

            /* Idle tick: poll hw_count and clear HAS_TIME on any
             * buffer the FPGA has finished reading. Only meaningful
             * once the reader is actually running - before then,
             * hw_count is 0 and clearing has nothing to do.
             */
            if (_tx_stream.reader_enabled) {
                litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
                _clearConsumedTXHeaders();
            }
#endif
            continue;
        }

        size_t handle = 0;
        uint8_t *dma_buf = nullptr;
        int ret = SOAPY_SDR_TIMEOUT;
        while (_tx_stream.worker_running.load(std::memory_order_acquire)) {
            ret = _dmaAcquireWrite(handle, &dma_buf, /*timeoutUs=*/200000);
            if (ret != SOAPY_SDR_TIMEOUT)
                break;
        }
        if (!_tx_stream.worker_running.load(std::memory_order_acquire)) {
            _tx_stream.free_fifo->push(pkt, /*wait=*/true);
            break;
        }
        if (ret < 0 && ret != SOAPY_SDR_UNDERFLOW) {
            _tx_stream.free_fifo->push(pkt, /*wait=*/true);
            continue;
        }
        if (ret == SOAPY_SDR_UNDERFLOW) {
            _tx_stream.underflow_count.fetch_add(1, std::memory_order_relaxed);
        }

        const size_t mtu_bytes = static_cast<size_t>(this->getStreamMTU(TX_STREAM)) *
                                 _frameChannels * _bytesPerComplex;
        const size_t bytes = std::min<size_t>(
            static_cast<size_t>(pkt->samples) * _frameChannels * _bytesPerComplex,
            mtu_bytes);
        if (bytes > 0) {
            std::memcpy(dma_buf, pkt->data, bytes);
        }
        if (bytes < mtu_bytes) {
            std::memset(dma_buf + bytes, 0, mtu_bytes - bytes);
        }

        long long release_timeNs = pkt->timeNs;
        if ((pkt->flags & SOAPY_SDR_HAS_TIME) && pkt->timeNs > 0) {
            const long long tx_time_offset_ns = m2sdr_env_tx_time_offset_ns();
            release_timeNs = pkt->timeNs + tx_time_offset_ns;
            const long long now = this->getHardwareTime("");
            const int64_t slip = static_cast<int64_t>(now - release_timeNs);
            const uint64_t slip_count =
                _tx_stream.tx_slip_count.fetch_add(1, std::memory_order_acq_rel);
            _tx_stream.tx_slip_ring[slip_count % TXStream::TX_SLIP_RING].store(
                slip, std::memory_order_relaxed);
            if (const char *trace_env = std::getenv("M2SDR_SOAPY_TX_SLIP_TRACE")) {
                if (std::string_view(trace_env) != "0") {
                    size_t trace_limit = 256;
                    if (const char *limit_env = std::getenv("M2SDR_SOAPY_TX_SLIP_TRACE_LIMIT")) {
                        trace_limit = static_cast<size_t>(std::max<long>(0, std::strtol(limit_env, nullptr, 10)));
                    }
                    if (slip_count < trace_limit) {
                        SoapySDR::logf(SOAPY_SDR_INFO,
                            "TX timed commit slip: seq=%llu samples=%u flags=0x%x target_ns=%lld release_ns=%lld offset_ns=%lld now_ns=%lld slip_ns=%lld",
                            (unsigned long long)slip_count,
                            pkt->samples,
                            pkt->flags,
                            (long long)pkt->timeNs,
                            (long long)release_timeNs,
                            (long long)tx_time_offset_ns,
                            (long long)now,
                            (long long)slip);
                    }
                }
            }
            if (now > release_timeNs) {
                _tx_stream.late_count.fetch_add(1, std::memory_order_relaxed);
            }
        }
        _dmaReleaseWrite(handle, pkt->samples, pkt->flags, release_timeNs);
        _tx_stream.free_fifo->push(pkt, /*wait=*/true);
    }
}

/* ===========================================================================
 *                          Worker lifecycle
 * ===========================================================================
 */

void SoapyLiteXM2SDR::_spawnWorker(Stream &s, bool is_rx) {
    const size_t capacity = is_rx ? _rx_buf_size : _tx_buf_size;
    _freePktPool(s);
    s.pkt_capacity = capacity;
    s.pkt_pool.resize(litex_m2sdr::kWorkerPktCount);
    s.work_fifo = std::make_unique<litex_m2sdr::PacketsFIFO<litex_m2sdr::StreamPacket*>>(
        litex_m2sdr::kWorkerPktCount);
    s.free_fifo = std::make_unique<litex_m2sdr::PacketsFIFO<litex_m2sdr::StreamPacket*>>(
        litex_m2sdr::kWorkerPktCount);
    for (size_t i = 0; i < litex_m2sdr::kWorkerPktCount; ++i) {
        s.pkt_pool[i].data     = new uint8_t[capacity];
        s.pkt_pool[i].capacity = capacity;
        s.pkt_pool[i].index    = i;
        s.pkt_pool[i].samples  = 0;
        s.pkt_pool[i].flags    = 0;
        s.pkt_pool[i].timeNs   = 0;
        s.pkt_pool[i].ret_code = 0;
        std::memset(s.pkt_pool[i].data, 0, capacity);
        s.free_fifo->push(&s.pkt_pool[i]);
    }
    s.overflow_count.store(0);
    s.underflow_count.store(0);
    s.late_count.store(0);
    s.reported_overflow_count  = 0;
    s.reported_underflow_count = 0;
    s.reported_late_count      = 0;
    s.tx_slip_count.store(0);
    for (auto &slip : s.tx_slip_ring) {
        slip.store(0, std::memory_order_relaxed);
    }

    s.worker_running.store(true, std::memory_order_release);
    if (is_rx) {
        s.worker = std::thread([this]() { this->rxWorkerLoop(); });
    } else {
        s.worker = std::thread([this]() { this->txWorkerLoop(); });
    }
}

void SoapyLiteXM2SDR::_stopWorker(Stream &s) {
    if (!s.worker.joinable()) return;
    s.worker_running.store(false, std::memory_order_release);
    if (s.work_fifo) s.work_fifo->wakeAll();
    if (s.free_fifo) s.free_fifo->wakeAll();
    s.worker.join();
}

void SoapyLiteXM2SDR::_freePktPool(Stream &s) {
    for (auto &p : s.pkt_pool) {
        delete[] p.data;
        p.data = nullptr;
    }
    s.pkt_pool.clear();
    s.work_fifo.reset();
    s.free_fifo.reset();
    s.pkt_capacity = 0;
}
