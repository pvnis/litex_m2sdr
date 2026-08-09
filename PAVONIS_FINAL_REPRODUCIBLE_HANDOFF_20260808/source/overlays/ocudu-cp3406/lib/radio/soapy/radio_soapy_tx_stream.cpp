// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_tx_stream.h"
#include "radio_soapy_tx_deadline.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader_view.h"
#include "ocudu/ocuduvec/conversion.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Types.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace ocudu;

/// Async status poll timeout in microseconds (1 ms).
static constexpr long RECV_ASYNC_TIMEOUT_US = 1000;
static constexpr float SCALING_FACTOR_CI16_TO_CF = std::numeric_limits<int16_t>::max();

/// Samples-to-nanoseconds helper.
static inline long long samples_to_ns(uint64_t samples, double srate_hz)
{
  return static_cast<long long>(static_cast<double>(samples) * 1e9 / srate_hz);
}

static bool env_requests_cf32_tx()
{
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_FORMAT")) {
    const std::string_view value(env);
    if (value == "cf32" || value == "CF32" || value == "fc32" || value == "FC32" || value == "SOAPY_SDR_CF32") {
      return true;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_CF32")) {
    return std::string_view(env) != "0";
  }
  return false;
}

static const char* getenv_or_empty(const char* name)
{
  const char* value = std::getenv(name);
  return (value != nullptr) ? value : "";
}

static uint64_t getenv_u64_or_zero(const char* name)
{
  const char* value = std::getenv(name);
  if ((value == nullptr) || (value[0] == '\0')) {
    return 0;
  }
  return static_cast<uint64_t>(std::strtoull(value, nullptr, 10));
}

static std::vector<uint64_t> getenv_u64_list(const char* name)
{
  std::vector<uint64_t> values;
  const char* value = std::getenv(name);
  if ((value == nullptr) || (value[0] == '\0')) {
    return values;
  }

  const char* ptr = value;
  while (*ptr != '\0') {
    while ((*ptr == ' ') || (*ptr == '\t') || (*ptr == ',') || (*ptr == ';')) {
      ++ptr;
    }
    if (*ptr == '\0') {
      break;
    }
    char* end = nullptr;
    const uint64_t parsed = static_cast<uint64_t>(std::strtoull(ptr, &end, 10));
    if (end == ptr) {
      break;
    }
    values.push_back(parsed);
    ptr = end;
  }
  return values;
}

static unsigned getenv_unsigned_or_default(const char* name, unsigned default_value)
{
  const uint64_t value = getenv_u64_or_zero(name);
  return (value > 0) ? static_cast<unsigned>(std::min<uint64_t>(value, std::numeric_limits<unsigned>::max()))
                     : default_value;
}

static int getenv_int_or_default(const char* name, int default_value)
{
  const char* value = std::getenv(name);
  if ((value == nullptr) || (value[0] == '\0')) {
    return default_value;
  }
  return static_cast<int>(std::strtol(value, nullptr, 10));
}

static bool env_enabled(const char* name)
{
  const char* value = std::getenv(name);
  return (value != nullptr) && (value[0] != '\0') && (std::string_view(value) != "0");
}

static bool getenv_bool_or_default(const char* name, bool default_value)
{
  const char* value = std::getenv(name);
  if ((value == nullptr) || (value[0] == '\0')) {
    return default_value;
  }
  const std::string_view view(value);
  return (view != "0") && (view != "false") && (view != "False") && (view != "FALSE") && (view != "no") &&
         (view != "NO") && (view != "off") && (view != "OFF");
}

static unsigned getenv_unsigned_allow_zero_or_default(const char* name, unsigned default_value)
{
  const char* value = std::getenv(name);
  if ((value == nullptr) || (value[0] == '\0')) {
    return default_value;
  }
  return static_cast<unsigned>(
      std::min<uint64_t>(static_cast<uint64_t>(std::strtoull(value, nullptr, 10)), std::numeric_limits<unsigned>::max()));
}

static int16_t clamp_i16(int value)
{
  return static_cast<int16_t>(std::max<int>(std::numeric_limits<int16_t>::min(),
                                            std::min<int>(std::numeric_limits<int16_t>::max(), value)));
}

static uint32_t pavonis_marker_hash(uint32_t seed, uint64_t index)
{
  uint32_t x = seed ^ static_cast<uint32_t>(index) ^ static_cast<uint32_t>(index >> 32);
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

static ci16_t pavonis_marker_sample(uint32_t seed, uint64_t index, int amp)
{
  const uint32_t x = pavonis_marker_hash(seed, index);
  return ci16_t((x & 0x1U) ? amp : -amp, (x & 0x2U) ? amp : -amp);
}

static void write_tx_readback_sidecar(radio_soapy_device& device,
                                      const char*         path,
                                      unsigned            stream_id,
                                      size_t              mtu,
                                      double              srate_hz,
                                      unsigned            nof_channels,
                                      bool                configured_discontinuous_tx,
                                      bool                discontinuous_tx,
                                      long                write_timeout_us,
                                      unsigned            tx_force_chunk_samples,
                                      bool                tx_suppress_empty_eob,
                                      bool                tx_force_continuous_stream,
                                      bool                tx_skip_stream_io,
                                      bool                tx_skip_stream_setup,
                                      bool                tx_skip_setupstream_only,
                                      uint64_t            tx_reanchor_interval_samples,
                                      bool                tx_all_timed_chunks,
                                      bool                tx_timeout_retry_enabled,
                                      unsigned            tx_timeout_retry_max_attempts,
                                      long                tx_timeout_retry_timeout_us,
                                      bool                tx_timeout_carry_enabled,
                                      uint64_t            tx_timeout_carry_max_samples,
                                      unsigned            tx_timeout_carry_drain_max_writes,
                                      long                tx_timeout_carry_write_timeout_us,
                                      bool                tx_timeout_carry_deadline_enabled,
                                      bool                tx_timeout_carry_deadline_drop_enabled,
                                      uint64_t            tx_timeout_carry_max_lag_samples,
                                      long                tx_timeout_carry_deadline_guard_us,
                                      long                tx_timeout_carry_deadline_max_write_timeout_us,
                                      bool                tx_deadline_write_timeout_enabled,
                                      long                tx_deadline_write_guard_us,
                                      long                tx_deadline_write_max_timeout_us,
                                      bool                tx_deadline_write_inside_guard_enabled,
                                      bool                tx_deadline_write_carry_on_timeout_enabled,
                                      long                tx_deadline_write_direct_poll_us,
                                      bool                tx_cf32_format,
                                      bool                tx_marker_enabled,
                                      uint64_t            tx_marker_configured_start_ts,
                                      uint64_t            tx_marker_start_ts,
                                      uint64_t            tx_marker_delay_samples,
                                      bool                tx_marker_dynamic_start,
                                      bool                tx_marker_start_selected,
                                      bool                tx_marker_sequence_mode,
                                      unsigned            tx_marker_count,
                                      unsigned            tx_marker_len,
                                      int                 tx_marker_amp,
                                      unsigned            tx_marker_port,
                                      uint32_t            tx_marker_seed)
{
  if ((path == nullptr) || (path[0] == '\0')) {
    return;
  }

  FILE* sidecar = std::fopen(path, "w");
  if (sidecar == nullptr) {
    return;
  }

  long long hw_now_ns = 0;
  const bool hw_ok = device.get_hardware_time(hw_now_ns);
  std::fprintf(sidecar, "PAVONIS_SOAPY_TX_READBACK_PATH=1\n");
  std::fprintf(sidecar, "stream_id=%u\n", stream_id);
  std::fprintf(sidecar, "mtu=%zu\n", mtu);
  std::fprintf(sidecar, "srate_hz=%.9f\n", srate_hz);
  std::fprintf(sidecar, "nof_channels=%u\n", nof_channels);
  std::fprintf(sidecar, "configured_discontinuous_tx=%u\n", configured_discontinuous_tx ? 1U : 0U);
  std::fprintf(sidecar, "discontinuous_tx=%u\n", discontinuous_tx ? 1U : 0U);
  std::fprintf(sidecar, "write_timeout_us=%ld\n", write_timeout_us);
  std::fprintf(sidecar, "tx_force_chunk_samples=%u\n", tx_force_chunk_samples);
  std::fprintf(sidecar, "tx_suppress_empty_eob=%u\n", tx_suppress_empty_eob ? 1U : 0U);
  std::fprintf(sidecar, "tx_force_continuous_stream=%u\n", tx_force_continuous_stream ? 1U : 0U);
  std::fprintf(sidecar, "tx_skip_stream_io=%u\n", tx_skip_stream_io ? 1U : 0U);
  std::fprintf(sidecar, "tx_skip_stream_setup=%u\n", tx_skip_stream_setup ? 1U : 0U);
  std::fprintf(sidecar, "tx_skip_setupstream_only=%u\n", tx_skip_setupstream_only ? 1U : 0U);
  std::fprintf(sidecar, "tx_reanchor_enabled=%u\n", tx_reanchor_interval_samples > 0 ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_reanchor_interval_samples=%llu\n",
               static_cast<unsigned long long>(tx_reanchor_interval_samples));
  std::fprintf(sidecar, "tx_all_timed_chunks=%u\n", tx_all_timed_chunks ? 1U : 0U);
  std::fprintf(sidecar, "tx_timeout_retry_enabled=%u\n", tx_timeout_retry_enabled ? 1U : 0U);
  std::fprintf(sidecar, "tx_timeout_retry_max_attempts=%u\n", tx_timeout_retry_max_attempts);
  std::fprintf(sidecar, "tx_timeout_retry_timeout_us=%ld\n", tx_timeout_retry_timeout_us);
  std::fprintf(sidecar, "tx_timeout_carry_enabled=%u\n", tx_timeout_carry_enabled ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_timeout_carry_max_samples=%llu\n",
               static_cast<unsigned long long>(tx_timeout_carry_max_samples));
  std::fprintf(sidecar, "tx_timeout_carry_drain_max_writes=%u\n", tx_timeout_carry_drain_max_writes);
  std::fprintf(sidecar, "tx_timeout_carry_write_timeout_us=%ld\n", tx_timeout_carry_write_timeout_us);
  std::fprintf(sidecar, "tx_timeout_carry_deadline_enabled=%u\n", tx_timeout_carry_deadline_enabled ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_timeout_carry_deadline_drop_enabled=%u\n",
               tx_timeout_carry_deadline_drop_enabled ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_timeout_carry_max_lag_samples=%llu\n",
               static_cast<unsigned long long>(tx_timeout_carry_max_lag_samples));
  std::fprintf(sidecar, "tx_timeout_carry_deadline_guard_us=%ld\n", tx_timeout_carry_deadline_guard_us);
  std::fprintf(sidecar,
               "tx_timeout_carry_deadline_max_write_timeout_us=%ld\n",
               tx_timeout_carry_deadline_max_write_timeout_us);
  std::fprintf(sidecar, "tx_deadline_write_timeout_enabled=%u\n", tx_deadline_write_timeout_enabled ? 1U : 0U);
  std::fprintf(sidecar, "tx_deadline_write_guard_us=%ld\n", tx_deadline_write_guard_us);
  std::fprintf(sidecar, "tx_deadline_write_max_timeout_us=%ld\n", tx_deadline_write_max_timeout_us);
  std::fprintf(sidecar,
               "tx_deadline_write_inside_guard_enabled=%u\n",
               tx_deadline_write_inside_guard_enabled ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_deadline_write_carry_on_timeout_enabled=%u\n",
               tx_deadline_write_carry_on_timeout_enabled ? 1U : 0U);
  std::fprintf(sidecar, "tx_deadline_write_direct_poll_us=%ld\n", tx_deadline_write_direct_poll_us);
  std::fprintf(sidecar, "tx_cf32_format=%u\n", tx_cf32_format ? 1U : 0U);
  std::fprintf(sidecar, "tx_format=%s\n", tx_cf32_format ? "CF32" : "CS16");
  std::fprintf(sidecar, "tx_marker_enabled=%u\n", tx_marker_enabled ? 1U : 0U);
  std::fprintf(sidecar,
               "tx_marker_configured_start_ts=%llu\n",
               static_cast<unsigned long long>(tx_marker_configured_start_ts));
  std::fprintf(sidecar, "tx_marker_start_ts=%llu\n", static_cast<unsigned long long>(tx_marker_start_ts));
  std::fprintf(sidecar,
               "tx_marker_delay_samples=%llu\n",
               static_cast<unsigned long long>(tx_marker_delay_samples));
  std::fprintf(sidecar, "tx_marker_dynamic_start=%u\n", tx_marker_dynamic_start ? 1U : 0U);
  std::fprintf(sidecar, "tx_marker_start_selected=%u\n", tx_marker_start_selected ? 1U : 0U);
  std::fprintf(sidecar, "tx_marker_sequence_mode=%u\n", tx_marker_sequence_mode ? 1U : 0U);
  std::fprintf(sidecar, "tx_marker_count=%u\n", tx_marker_count);
  std::fprintf(sidecar, "tx_marker_len=%u\n", tx_marker_len);
  std::fprintf(sidecar, "tx_marker_amp=%d\n", tx_marker_amp);
  std::fprintf(sidecar, "tx_marker_port=%u\n", tx_marker_port);
  std::fprintf(sidecar, "tx_marker_seed=%u\n", tx_marker_seed);
  std::fprintf(sidecar, "ci16_to_cf32_scale=%.9f\n", SCALING_FACTOR_CI16_TO_CF);
  std::fprintf(sidecar, "hardware_time_ok=%u\n", hw_ok ? 1U : 0U);
  std::fprintf(sidecar, "hardware_time_ns=%lld\n", hw_ok ? hw_now_ns : -1LL);
  std::fprintf(sidecar, "env_OCUDU_SOAPY_TX_FORMAT=%s\n", getenv_or_empty("OCUDU_SOAPY_TX_FORMAT"));
  std::fprintf(sidecar, "env_OCUDU_SOAPY_TX_CF32=%s\n", getenv_or_empty("OCUDU_SOAPY_TX_CF32"));
  std::fprintf(sidecar, "env_M2SDR_SOAPY_TX_CS16_TO_SC12=%s\n", getenv_or_empty("M2SDR_SOAPY_TX_CS16_TO_SC12"));
  std::fprintf(sidecar, "env_M2SDR_SOAPY_TX_TIME_OFFSET_NS=%s\n", getenv_or_empty("M2SDR_SOAPY_TX_TIME_OFFSET_NS"));
  std::fprintf(sidecar, "env_OCUDU_SOAPY_TX_SUMMARY=%s\n", getenv_or_empty("OCUDU_SOAPY_TX_SUMMARY"));
  std::fprintf(sidecar, "env_OCUDU_SOAPY_TX_SUPPRESS_EMPTY_EOB=%s\n", getenv_or_empty("OCUDU_SOAPY_TX_SUPPRESS_EMPTY_EOB"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_SKIP_STREAM_IO=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_SKIP_STREAM_IO"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_SKIP_STREAM_SETUP=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_SKIP_STREAM_SETUP"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_SKIP_SETUPSTREAM_ONLY=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_SKIP_SETUPSTREAM_ONLY"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_RETRY=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_RETRY"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_RETRY_MAX=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_RETRY_MAX"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_RETRY_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_RETRY_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_SAMPLES=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_SAMPLES"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_DRAIN_WRITES=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DRAIN_WRITES"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_WRITE_TIMEOUT_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_WRITE_TIMEOUT_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_DROP=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_DROP"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_LAG_SAMPLES=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_LAG_SAMPLES"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_GUARD_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_GUARD_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_MAX_WRITE_TIMEOUT_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_MAX_WRITE_TIMEOUT_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_TIMEOUT=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_TIMEOUT"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_GUARD_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_GUARD_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_INSIDE_GUARD=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_INSIDE_GUARD"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_CARRY_ON_TIMEOUT=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_CARRY_ON_TIMEOUT"));
  std::fprintf(sidecar,
               "env_OCUDU_SOAPY_TX_DEADLINE_WRITE_DIRECT_POLL_US=%s\n",
               getenv_or_empty("OCUDU_SOAPY_TX_DEADLINE_WRITE_DIRECT_POLL_US"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_ENABLE=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_ENABLE"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_START_TS=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_START_TS"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_DELAY_SAMPLES=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_DELAY_SAMPLES"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_DELAY_SEQUENCE_SAMPLES=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_DELAY_SEQUENCE_SAMPLES"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_DELAYS_SAMPLES=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_DELAYS_SAMPLES"));
  std::fprintf(sidecar,
               "env_PAVONIS_SOAPY_TX_MARKER_META_PATH=%s\n",
               getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_META_PATH"));
  std::fprintf(sidecar, "channel\trate_ok\trate_hz\tfreq_ok\tfreq_hz\tgain_ok\tgain_db\tbw_ok\tbw_hz\n");
  for (unsigned ch = 0; ch != nof_channels; ++ch) {
    double rate_Hz = -1.0;
    double freq_Hz = -1.0;
    double gain_dB = -1.0;
    double bandwidth_Hz = -1.0;
    const bool rate_ok = device.get_sample_rate(SOAPY_SDR_TX, ch, rate_Hz);
    const bool freq_ok = device.get_frequency(SOAPY_SDR_TX, ch, freq_Hz);
    const bool gain_ok = device.get_gain(SOAPY_SDR_TX, ch, gain_dB);
    const bool bandwidth_ok = device.get_bandwidth(SOAPY_SDR_TX, ch, bandwidth_Hz);
    std::fprintf(sidecar,
                 "%u\t%u\t%.9f\t%u\t%.9f\t%u\t%.9f\t%u\t%.9f\n",
                 ch,
                 rate_ok ? 1U : 0U,
                 rate_ok ? rate_Hz : -1.0,
                 freq_ok ? 1U : 0U,
                 freq_ok ? freq_Hz : -1.0,
                 gain_ok ? 1U : 0U,
                 gain_ok ? gain_dB : -1.0,
                 bandwidth_ok ? 1U : 0U,
                 bandwidth_ok ? bandwidth_Hz : -1.0);
  }
  std::fclose(sidecar);
}

radio_soapy_tx_stream::radio_soapy_tx_stream(radio_soapy_device&       device_,
                                               SoapySDR::Stream*         stream_,
                                               const stream_description& desc,
                                               task_executor&            async_executor_,
                                               radio_event_notifier&     notifier_) :
  stream_id(desc.id),
  async_executor(async_executor_),
  notifier(notifier_),
  device(device_),
  stream(stream_),
  srate_hz(desc.srate_hz),
  nof_channels(desc.nof_channels),
  discontinuous_tx(desc.discontinuous_tx),
  power_ramping_buffer(desc.nof_channels, 0),
  logger(ocudulog::fetch_basic_logger("RF"))
{
  ocudu_assert(std::isnormal(srate_hz) && srate_hz > 0.0, "Invalid sampling rate {}.", srate_hz);

  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TRACE")) {
    tx_trace_enabled = std::string_view(env) != "0";
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TRACE_WRITES")) {
    tx_trace_writes_enabled = std::string_view(env) != "0";
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_SUMMARY")) {
    tx_summary_enabled = std::string_view(env) != "0";
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_SUPPRESS_EMPTY_EOB")) {
    tx_suppress_empty_eob = std::string_view(env) != "0";
  }
  tx_force_continuous_stream = env_enabled("PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM");
  if (tx_force_continuous_stream) {
    discontinuous_tx = false;
  }
  tx_skip_stream_io = env_enabled("PAVONIS_SOAPY_TX_SKIP_STREAM_IO");
  tx_skip_stream_setup = env_enabled("PAVONIS_SOAPY_TX_SKIP_STREAM_SETUP");
  tx_skip_setupstream_only = env_enabled("PAVONIS_SOAPY_TX_SKIP_SETUPSTREAM_ONLY");
  tx_reanchor_interval_samples = getenv_u64_or_zero("PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES");
  tx_all_timed_chunks = env_enabled("PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS");
  ocudu_assert((stream != nullptr) || ((tx_skip_stream_setup || tx_skip_setupstream_only) && tx_skip_stream_io),
               "TX stream must not be null unless a TX setupStream skip mode and "
               "PAVONIS_SOAPY_TX_SKIP_STREAM_IO are both enabled.");
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_SUMMARY_PERIOD")) {
    const long value = std::strtol(env, nullptr, 10);
    tx_summary_period = value > 0 ? static_cast<unsigned>(value) : 0;
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TRACE_US")) {
    tx_trace_threshold_us = std::strtol(env, nullptr, 10);
    if (tx_trace_threshold_us <= 0) {
      tx_trace_threshold_us = 100;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_WRITE_TIMEOUT_US")) {
    write_timeout_us = std::strtol(env, nullptr, 10);
    if (write_timeout_us < 0) {
      write_timeout_us = 0;
    }
  }
  tx_timeout_retry_enabled = getenv_bool_or_default("OCUDU_SOAPY_TX_TIMEOUT_RETRY", tx_timeout_retry_enabled);
  tx_timeout_retry_max_attempts =
      getenv_unsigned_allow_zero_or_default("OCUDU_SOAPY_TX_TIMEOUT_RETRY_MAX", tx_timeout_retry_max_attempts);
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TIMEOUT_RETRY_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value > 0) {
      tx_timeout_retry_timeout_us = value;
    }
  }
  if (tx_timeout_retry_timeout_us < 1000) {
    tx_timeout_retry_timeout_us = 1000;
  }
  tx_timeout_carry_enabled = getenv_bool_or_default("OCUDU_SOAPY_TX_TIMEOUT_CARRY", tx_timeout_carry_enabled);
  tx_timeout_carry_max_samples =
      getenv_u64_or_zero("OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_SAMPLES");
  if (tx_timeout_carry_max_samples == 0) {
    tx_timeout_carry_max_samples = 1152000;
  }
  tx_timeout_carry_drain_max_writes =
      getenv_unsigned_or_default("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DRAIN_WRITES", tx_timeout_carry_drain_max_writes);
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TIMEOUT_CARRY_WRITE_TIMEOUT_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_timeout_carry_write_timeout_us = value;
    }
  }
  tx_timeout_carry_deadline_enabled =
      getenv_bool_or_default("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE", tx_timeout_carry_deadline_enabled);
  tx_timeout_carry_deadline_drop_enabled =
      getenv_bool_or_default("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_DROP",
                             tx_timeout_carry_deadline_drop_enabled);
  tx_timeout_carry_max_lag_samples =
      getenv_u64_or_zero("OCUDU_SOAPY_TX_TIMEOUT_CARRY_MAX_LAG_SAMPLES");
  if (tx_timeout_carry_max_lag_samples == 0) {
    tx_timeout_carry_max_lag_samples =
        std::max<uint64_t>(1, static_cast<uint64_t>(srate_hz / 1000.0)); // 1 ms by default.
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_GUARD_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_timeout_carry_deadline_guard_us = value;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TIMEOUT_CARRY_DEADLINE_MAX_WRITE_TIMEOUT_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_timeout_carry_deadline_max_write_timeout_us = value;
    }
  }
  tx_deadline_write_timeout_enabled =
      getenv_bool_or_default("OCUDU_SOAPY_TX_DEADLINE_WRITE_TIMEOUT", tx_deadline_write_timeout_enabled);
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_DEADLINE_WRITE_GUARD_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_deadline_write_guard_us = value;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_deadline_write_max_timeout_us = value;
    }
  }
  tx_deadline_write_inside_guard_enabled =
      env_enabled("OCUDU_SOAPY_TX_DEADLINE_WRITE_INSIDE_GUARD");
  if (const char* env = std::getenv("M2SDR_SOAPY_TX_TIME_OFFSET_NS")) {
    tx_deadline_write_time_offset_ns = std::strtoll(env, nullptr, 10);
  }
  tx_deadline_write_carry_on_timeout_enabled =
      getenv_bool_or_default("OCUDU_SOAPY_TX_DEADLINE_WRITE_CARRY_ON_TIMEOUT",
                             tx_deadline_write_carry_on_timeout_enabled);
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_DEADLINE_WRITE_DIRECT_POLL_US")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_deadline_write_direct_poll_us = value;
    }
  }
  tx_deadline_break_trace_enabled = env_enabled("PAVONIS_SOAPY_TX_DEADLINE_BREAK_TRACE");
  tx_deadline_break_trace_limit =
      getenv_unsigned_allow_zero_or_default("PAVONIS_SOAPY_TX_DEADLINE_BREAK_TRACE_LIMIT",
                                            tx_deadline_break_trace_limit);
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_FORCE_CHUNK_SAMPLES")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value > 0) {
      tx_force_chunk_samples = static_cast<unsigned>(value);
    }
  }
  tx_cf32_format = env_requests_cf32_tx();
  init_tx_marker();

  if (stream != nullptr) {
    mtu = device.get_stream_mtu(stream);
  } else {
    mtu = static_cast<size_t>(getenv_u64_or_zero("PAVONIS_SOAPY_TX_SKIP_STREAM_SETUP_MTU"));
    if (mtu == 0) {
      mtu = 1022;
    }
  }
  init_tx_write_dump();

  write_tx_readback_sidecar(device,
                            std::getenv("PAVONIS_SOAPY_TX_READBACK_PATH"),
                            stream_id,
                            mtu,
                            srate_hz,
                            nof_channels,
                            desc.discontinuous_tx,
                            discontinuous_tx,
                            write_timeout_us,
                            tx_force_chunk_samples,
                            tx_suppress_empty_eob,
                            tx_force_continuous_stream,
                            tx_skip_stream_io,
                            tx_skip_stream_setup,
                            tx_skip_setupstream_only,
                            tx_reanchor_interval_samples,
                            tx_all_timed_chunks,
                            tx_timeout_retry_enabled,
                            tx_timeout_retry_max_attempts,
                            tx_timeout_retry_timeout_us,
                            tx_timeout_carry_enabled,
                            tx_timeout_carry_max_samples,
                            tx_timeout_carry_drain_max_writes,
                            tx_timeout_carry_write_timeout_us,
                            tx_timeout_carry_deadline_enabled,
                            tx_timeout_carry_deadline_drop_enabled,
                            tx_timeout_carry_max_lag_samples,
                            tx_timeout_carry_deadline_guard_us,
                            tx_timeout_carry_deadline_max_write_timeout_us,
                            tx_deadline_write_timeout_enabled,
                            tx_deadline_write_guard_us,
                            tx_deadline_write_max_timeout_us,
                            tx_deadline_write_inside_guard_enabled,
                            tx_deadline_write_carry_on_timeout_enabled,
                            tx_deadline_write_direct_poll_us,
                            tx_cf32_format,
                            tx_marker_enabled,
                            tx_marker_configured_start_ts,
                            tx_marker_start_ts,
                            tx_marker_delay_samples,
                            tx_marker_dynamic_start,
                            tx_marker_start_selected,
                            tx_marker_sequence_mode,
                            static_cast<unsigned>(tx_marker_events.size()),
                            tx_marker_len,
                            tx_marker_amp,
                            tx_marker_port,
                            tx_marker_seed);

  if (tx_summary_enabled) {
    fmt::print(stderr,
               "Soapy TX summary enabled: stream={} mtu={} period={} force_chunk={} "
               "suppress_empty_eob={} force_continuous_stream={} reanchor_interval_samples={} all_timed_chunks={} "
               "timeout_retry={} timeout_retry_max={} timeout_retry_us={} "
               "timeout_carry={} timeout_carry_max_samples={} timeout_carry_drain_writes={} timeout_carry_write_timeout_us={} "
               "timeout_carry_deadline={} timeout_carry_deadline_drop={} timeout_carry_max_lag_samples={} "
               "timeout_carry_deadline_guard_us={} "
               "timeout_carry_deadline_max_write_timeout_us={} "
               "deadline_write_timeout={} deadline_write_guard_us={} deadline_write_max_timeout_us={} "
               "deadline_write_inside_guard={} "
               "deadline_write_time_offset_ns={} "
               "deadline_write_carry_on_timeout={} deadline_write_direct_poll_us={} "
               "skip_stream_io={} skip_stream_setup={} skip_setupstream_only={} "
               "write_timeout_us={} tx_format={}\n",
               stream_id,
               mtu,
               tx_summary_period,
               tx_force_chunk_samples,
               tx_suppress_empty_eob,
               tx_force_continuous_stream,
               static_cast<unsigned long long>(tx_reanchor_interval_samples),
               tx_all_timed_chunks,
               tx_timeout_retry_enabled,
               tx_timeout_retry_max_attempts,
               tx_timeout_retry_timeout_us,
               tx_timeout_carry_enabled,
               static_cast<unsigned long long>(tx_timeout_carry_max_samples),
               tx_timeout_carry_drain_max_writes,
               tx_timeout_carry_write_timeout_us,
               tx_timeout_carry_deadline_enabled,
               tx_timeout_carry_deadline_drop_enabled,
               static_cast<unsigned long long>(tx_timeout_carry_max_lag_samples),
               tx_timeout_carry_deadline_guard_us,
               tx_timeout_carry_deadline_max_write_timeout_us,
               tx_deadline_write_timeout_enabled,
               tx_deadline_write_guard_us,
               tx_deadline_write_max_timeout_us,
               tx_deadline_write_inside_guard_enabled,
               tx_deadline_write_time_offset_ns,
               tx_deadline_write_carry_on_timeout_enabled,
               tx_deadline_write_direct_poll_us,
               tx_skip_stream_io,
               tx_skip_stream_setup,
               tx_skip_setupstream_only,
               write_timeout_us,
               tx_cf32_format ? "CF32" : "CS16");
  }

  if (tx_force_continuous_stream) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM enabled: stream={} configured_discontinuous={} "
               "effective_discontinuous={} writeStream timing/EOB flags will be stripped.\n",
               stream_id,
               desc.discontinuous_tx ? 1U : 0U,
               discontinuous_tx ? 1U : 0U);
  }

  if (tx_reanchor_interval_samples > 0) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES enabled: stream={} interval_samples={}\n",
               stream_id,
               static_cast<unsigned long long>(tx_reanchor_interval_samples));
  }

  if (tx_all_timed_chunks) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS enabled: stream={} every non-empty data chunk will carry HAS_TIME.\n",
               stream_id);
  }

  if (tx_timeout_retry_enabled && (tx_timeout_retry_max_attempts > 0)) {
    fmt::print(stderr,
               "OCUDU_SOAPY_TX_TIMEOUT_RETRY enabled: stream={} max_attempts={} retry_timeout_us={}.\n",
               stream_id,
               tx_timeout_retry_max_attempts,
               tx_timeout_retry_timeout_us);
  }

  if (tx_timeout_carry_enabled) {
    fmt::print(stderr,
               "OCUDU_SOAPY_TX_TIMEOUT_CARRY enabled: stream={} max_samples={} drain_writes={} write_timeout_us={} "
               "deadline={} deadline_drop={} max_lag_samples={} deadline_guard_us={} deadline_max_write_timeout_us={}.\n",
               stream_id,
               static_cast<unsigned long long>(tx_timeout_carry_max_samples),
               tx_timeout_carry_drain_max_writes,
               tx_timeout_carry_write_timeout_us,
               tx_timeout_carry_deadline_enabled,
               tx_timeout_carry_deadline_drop_enabled,
               static_cast<unsigned long long>(tx_timeout_carry_max_lag_samples),
               tx_timeout_carry_deadline_guard_us,
               tx_timeout_carry_deadline_max_write_timeout_us);
  }

  if (tx_deadline_write_timeout_enabled) {
    fmt::print(stderr,
               "OCUDU_SOAPY_TX_DEADLINE_WRITE_TIMEOUT enabled: stream={} guard_us={} max_timeout_us={} "
               "inside_guard={} carry_on_timeout={} direct_poll_us={}.\n",
               stream_id,
               tx_deadline_write_guard_us,
               tx_deadline_write_max_timeout_us,
               tx_deadline_write_inside_guard_enabled,
               tx_deadline_write_carry_on_timeout_enabled,
               tx_deadline_write_direct_poll_us);
  }

  if (tx_deadline_break_trace_enabled) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_DEADLINE_BREAK_TRACE active: stream={} limit={} event-triggered metadata only.\n",
               stream_id,
               tx_deadline_break_trace_limit);
  }

  if (tx_skip_stream_io) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_SKIP_STREAM_IO enabled: stream={} TX activate/write/deactivate calls will be skipped.\n",
               stream_id);
  }

  if (tx_skip_stream_setup) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_SKIP_STREAM_SETUP enabled: stream={} no Soapy TX stream was set up; logical MTU={}.\n",
               stream_id,
               mtu);
  }
  if (tx_skip_setupstream_only) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_SKIP_SETUPSTREAM_ONLY enabled: stream={} TX channel config ran, no Soapy TX stream was set up; logical MTU={}.\n",
               stream_id,
               mtu);
  }

  if (tx_cf32_format) {
    fmt::print(stderr,
               "OCUDU_SOAPY_TX_FORMAT_CF32 enabled: stream={} scale={} mtu={} channels={}\n",
               stream_id,
               SCALING_FACTOR_CI16_TO_CF,
               mtu,
               nof_channels);
  }

  if (tx_marker_enabled) {
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_MARKER enabled: stream={} start_ts={} configured_start_ts={} "
               "dynamic_start={} delay_samples={} sequence_mode={} count={} len={} amp={} port={} seed={} meta={}\n",
               stream_id,
               tx_marker_start_ts,
               tx_marker_configured_start_ts,
               tx_marker_dynamic_start ? 1U : 0U,
               tx_marker_delay_samples,
               tx_marker_sequence_mode ? 1U : 0U,
               tx_marker_events.size(),
               tx_marker_len,
               tx_marker_amp,
               tx_marker_port,
               tx_marker_seed,
               tx_marker_meta_path != nullptr ? tx_marker_meta_path : "");
  }

  if (discontinuous_tx && desc.power_ramping_us > 0.0f) {
    power_ramping_nof_samples =
        static_cast<unsigned>(srate_hz * static_cast<double>(desc.power_ramping_us) / 1e6);
    // Align to MTU.
    power_ramping_nof_samples = (power_ramping_nof_samples / static_cast<unsigned>(mtu)) * static_cast<unsigned>(mtu);

    double aligned_us = static_cast<double>(power_ramping_nof_samples) * 1e6 / srate_hz;
    fmt::print("SoapySDR TX: power ramping guard aligned to {} samples ({:.1f} us).\n",
               power_ramping_nof_samples,
               aligned_us);

    power_ramping_buffer.resize(power_ramping_nof_samples);
    for (unsigned ch = 0; ch != nof_channels; ++ch) {
      ocuduvec::zero(power_ramping_buffer.get_writer()[ch]);
    }
    if (tx_cf32_format) {
      power_ramping_cf32_buffer.assign(static_cast<size_t>(nof_channels) * power_ramping_nof_samples, cf_t{});
    }
  }

  state_fsm.init_successful();
}

void radio_soapy_tx_stream::recv_async_msg()
{
  size_t    chan_mask = 0;
  int       flags     = 0;
  long long time_ns   = 0;

  int ret = device.read_stream_status(stream, chan_mask, flags, time_ns, RECV_ASYNC_TIMEOUT_US);

  radio_event_notifier::event_description event = {.stream_id  = stream_id,
                                                   .channel_id = std::nullopt,
                                                   .source     = radio_event_source::TRANSMIT,
                                                   .type       = radio_event_type::UNDEFINED,
                                                   .timestamp  = std::nullopt};

  if (ret == SOAPY_SDR_TIME_ERROR) {
    event.type            = radio_event_type::LATE;
    event.timestamp       = static_cast<uint64_t>(time_ns * srate_hz / 1e9);
    state_fsm.async_event_late_underflow(time_ns);
  } else if (ret == SOAPY_SDR_UNDERFLOW) {
    event.type            = radio_event_type::UNDERFLOW;
    state_fsm.async_event_late_underflow(time_ns);
  }
  // ret == 0 is timeout (no event) - ignore.

  if (event.type != radio_event_type::UNDEFINED) {
    notifier.on_radio_rt_event(event);
  }
}

void radio_soapy_tx_stream::run_recv_async_msg()
{
  auto token = stop_control.get_token();
  if (OCUDU_UNLIKELY(token.is_stop_requested())) {
    return;
  }

  recv_async_msg();

  report_error_if_not(async_executor.defer([this, tk = std::move(token)]() { run_recv_async_msg(); }),
                      "Unable to run SoapySDR async TX stream task");
}

void radio_soapy_tx_stream::record_tx_logical(bool is_empty, unsigned nof_samples, int flags)
{
  if (!tx_summary_enabled) {
    return;
  }

  ++tx_summary.logical_transmits;
  tx_summary.stream_samples += nof_samples;
  if (is_empty) {
    ++tx_summary.empty_logical_transmits;
  } else {
    ++tx_summary.data_logical_transmits;
    tx_summary.data_samples += nof_samples;
  }
  if ((flags & SOAPY_SDR_HAS_TIME) != 0) {
    ++tx_summary.logical_has_time;
  }
  if ((flags & SOAPY_SDR_END_BURST) != 0) {
    ++tx_summary.logical_eob;
  }
}

void radio_soapy_tx_stream::record_tx_write(unsigned requested,
                                            int      ret,
                                            int      flags,
                                            bool     data_write,
                                            bool     final_data_chunk,
                                            bool     power_ramp_write,
                                            bool     empty_eob_write,
                                            bool     stop_flush_write,
                                            long long time_ns)
{
  if (!tx_summary_enabled) {
    return;
  }

  const bool first_write = tx_summary.write_calls == 0;
  ++tx_summary.write_calls;
  if (data_write) {
    ++tx_summary.data_write_calls;
    tx_summary.data_requested_samples += requested;
    if (tx_all_timed_chunks && (requested > 0)) {
      ++tx_summary.all_timed_writes;
    }
    if (final_data_chunk) {
      ++tx_summary.final_data_chunks;
    } else {
      ++tx_summary.nonfinal_data_chunks;
    }
  }
  if (power_ramp_write) {
    ++tx_summary.power_ramp_write_calls;
  }
  if (empty_eob_write) {
    ++tx_summary.empty_eob_write_calls;
  }
  if (stop_flush_write) {
    ++tx_summary.stop_flush_write_calls;
  }

  tx_summary.requested_samples += requested;
  tx_summary.min_requested = first_write ? requested : std::min<uint64_t>(tx_summary.min_requested, requested);
  tx_summary.max_requested = std::max<uint64_t>(tx_summary.max_requested, requested);
  if (requested == 0) {
    ++tx_summary.zero_requested_writes;
  }

  if (ret >= 0) {
    const uint64_t returned = static_cast<uint64_t>(ret);
    const bool     first_return = tx_summary.return_observations == 0;
    ++tx_summary.return_observations;
    tx_summary.returned_samples += returned;
    if (data_write) {
      tx_summary.data_returned_samples += returned;
    }
    tx_summary.min_returned = first_return ? returned : std::min<uint64_t>(tx_summary.min_returned, returned);
    tx_summary.max_returned = std::max<uint64_t>(tx_summary.max_returned, returned);
    if (returned == 0) {
      ++tx_summary.zero_return_writes;
    }
    if (requested > 0 && returned < requested) {
      ++tx_summary.partial_return_writes;
    }
    if (returned > requested) {
      ++tx_summary.over_return_writes;
    }
  } else if (ret == SOAPY_SDR_TIMEOUT) {
    ++tx_summary.timeout_writes;
  } else {
    ++tx_summary.error_writes;
  }

  const bool has_time = (flags & SOAPY_SDR_HAS_TIME) != 0;
  const bool eob      = (flags & SOAPY_SDR_END_BURST) != 0;
  if (has_time) {
    const bool first_timed = tx_summary.has_time_writes == 0;
    ++tx_summary.has_time_writes;
    if (time_ns > 0) {
      ++tx_summary.has_time_positive_ns;
    } else if (time_ns == 0) {
      ++tx_summary.has_time_zero_ns;
    } else {
      ++tx_summary.has_time_negative_ns;
    }
    tx_summary.has_time_min_ns = first_timed ? time_ns : std::min(tx_summary.has_time_min_ns, time_ns);
    tx_summary.has_time_max_ns = first_timed ? time_ns : std::max(tx_summary.has_time_max_ns, time_ns);
  }
  if (eob) {
    ++tx_summary.eob_writes;
  }
  if (has_time && eob) {
    ++tx_summary.both_flag_writes;
  }
  if (!has_time && !eob) {
    ++tx_summary.zero_flag_writes;
  }
}

void radio_soapy_tx_stream::log_tx_summary(std::string_view reason)
{
  if (!tx_summary_enabled) {
    return;
  }

  auto sample_delta = [](uint64_t lhs, uint64_t rhs) -> long long {
    return static_cast<long long>(lhs) - static_cast<long long>(rhs);
  };

  fmt::print(stderr,
             "Soapy TX summary: reason={} stream={} logical={} data_logical={} empty_logical={} "
             "logical_has_time={} logical_eob={} data_samples={} stream_samples={} writes={} data_writes={} power_ramp_writes={} "
             "empty_eob_writes={} suppressed_empty_eob={} stop_flush_writes={} "
             "samples_req={} samples_ret={} data_samples_req={} data_samples_ret={} "
             "write_deficit={} data_write_deficit={} logical_data_deficit={} stream_deficit={} req_min={} "
             "req_max={} ret_min={} ret_max={} zero_req={} zero_ret={} has_time_writes={} reanchor_writes={} "
             "all_timed_writes={} eob_writes={} "
             "has_time_pos_ns={} has_time_zero_ns={} has_time_neg_ns={} has_time_min_ns={} has_time_max_ns={} "
             "both_flags={} zero_flags={} final_data_chunks={} nonfinal_data_chunks={} partial_ret={} "
             "over_ret={} timeouts={} errors={} timeout_retry_attempts={} timeout_retry_successes={} "
             "timeout_retry_exhausted={} timeout_retry_recovered_samples={} timeout_retry_abandoned_samples={} "
             "timeout_carry_queued_chunks={} timeout_carry_queued_samples={} "
             "timeout_carry_drained_chunks={} timeout_carry_drained_samples={} "
             "timeout_carry_drain_writes={} timeout_carry_drain_timeouts={} timeout_carry_drain_errors={} "
             "timeout_carry_partial_drains={} timeout_carry_overflow_chunks={} timeout_carry_overflow_samples={} "
             "timeout_carry_abandoned_samples={} timeout_carry_pending_chunks={} timeout_carry_pending_samples={} "
             "timeout_carry_max_pending_samples={} timeout_carry_first_queue_logical={} "
             "timeout_carry_first_overflow_logical={} timeout_carry_first_lag_logical={} "
             "timeout_carry_deadline_limited_writes={} "
             "timeout_carry_deadline_breaks={} timeout_carry_deadline_hwtime_failures={} "
             "timeout_carry_deadline_lag_events={} timeout_carry_deadline_max_observed_lag_samples={} "
             "timeout_carry_deadline_would_drop_samples={} "
             "timeout_carry_deadline_dropped_chunks={} timeout_carry_deadline_partial_drops={} "
	             "timeout_carry_deadline_dropped_samples={} "
	             "deadline_write_limited_writes={} deadline_write_breaks={} deadline_write_hwtime_failures={} "
	             "deadline_write_timeouts={} deadline_write_carry_timeouts={} "
	             "deadline_write_abandoned_samples={} deadline_write_max_selected_timeout_us={} "
	             "deadline_write_first_timeout_logical={} deadline_write_inside_guard_attempts={} "
	             "deadline_write_inside_guard_successes={} deadline_write_inside_guard_timeouts={} "
	             "deadline_write_inside_guard_recovered_samples={} "
             "mtu={} force_chunk={} suppress_empty_eob={} "
             "force_continuous_stream={} all_timed_chunks={} skip_stream_io={} skip_stream_setup={} skip_setupstream_only={} "
             "write_timeout_us={} timeout_retry={} timeout_retry_max={} timeout_retry_us={} "
             "timeout_carry={} timeout_carry_max_samples={} timeout_carry_drain_writes_cfg={} timeout_carry_write_timeout_us={} "
             "timeout_carry_deadline={} timeout_carry_deadline_drop={} timeout_carry_max_lag_samples={} "
             "timeout_carry_deadline_guard_us={} "
	             "timeout_carry_deadline_max_write_timeout_us={} "
	             "deadline_write_timeout={} deadline_write_guard_us={} deadline_write_max_timeout_us={} "
	             "deadline_write_inside_guard={} "
	             "deadline_write_time_offset_ns={} "
	             "deadline_write_carry_on_timeout={} deadline_write_direct_poll_us={} "
	             "discontinuous={} tx_format={}\n",
             reason,
             stream_id,
             tx_summary.logical_transmits,
             tx_summary.data_logical_transmits,
             tx_summary.empty_logical_transmits,
             tx_summary.logical_has_time,
             tx_summary.logical_eob,
             tx_summary.data_samples,
             tx_summary.stream_samples,
             tx_summary.write_calls,
             tx_summary.data_write_calls,
             tx_summary.power_ramp_write_calls,
             tx_summary.empty_eob_write_calls,
             tx_summary.suppressed_empty_eob,
             tx_summary.stop_flush_write_calls,
             tx_summary.requested_samples,
             tx_summary.returned_samples,
             tx_summary.data_requested_samples,
             tx_summary.data_returned_samples,
             sample_delta(tx_summary.requested_samples, tx_summary.returned_samples),
             sample_delta(tx_summary.data_requested_samples, tx_summary.data_returned_samples),
             sample_delta(tx_summary.data_samples, tx_summary.data_returned_samples),
             sample_delta(tx_summary.stream_samples, tx_summary.returned_samples),
             tx_summary.min_requested,
             tx_summary.max_requested,
             tx_summary.min_returned,
             tx_summary.max_returned,
             tx_summary.zero_requested_writes,
             tx_summary.zero_return_writes,
             tx_summary.has_time_writes,
             tx_summary.reanchor_writes,
             tx_summary.all_timed_writes,
             tx_summary.eob_writes,
             tx_summary.has_time_positive_ns,
             tx_summary.has_time_zero_ns,
             tx_summary.has_time_negative_ns,
             tx_summary.has_time_min_ns,
             tx_summary.has_time_max_ns,
             tx_summary.both_flag_writes,
             tx_summary.zero_flag_writes,
             tx_summary.final_data_chunks,
             tx_summary.nonfinal_data_chunks,
             tx_summary.partial_return_writes,
             tx_summary.over_return_writes,
             tx_summary.timeout_writes,
             tx_summary.error_writes,
             tx_summary.timeout_retry_attempts,
             tx_summary.timeout_retry_successes,
             tx_summary.timeout_retry_exhausted,
             tx_summary.timeout_retry_recovered_samples,
             tx_summary.timeout_retry_abandoned_samples,
             tx_summary.timeout_carry_queued_chunks,
             tx_summary.timeout_carry_queued_samples,
             tx_summary.timeout_carry_drained_chunks,
             tx_summary.timeout_carry_drained_samples,
             tx_summary.timeout_carry_drain_writes,
             tx_summary.timeout_carry_drain_timeouts,
             tx_summary.timeout_carry_drain_errors,
             tx_summary.timeout_carry_partial_drains,
             tx_summary.timeout_carry_overflow_chunks,
             tx_summary.timeout_carry_overflow_samples,
             tx_summary.timeout_carry_abandoned_samples,
             tx_pending_chunks.size(),
             tx_pending_samples,
             tx_summary.timeout_carry_max_pending_samples,
             tx_summary.timeout_carry_first_queue_logical,
             tx_summary.timeout_carry_first_overflow_logical,
             tx_summary.timeout_carry_first_lag_logical,
             tx_summary.timeout_carry_deadline_limited_writes,
             tx_summary.timeout_carry_deadline_breaks,
             tx_summary.timeout_carry_deadline_hwtime_failures,
             tx_summary.timeout_carry_deadline_lag_events,
             tx_summary.timeout_carry_deadline_max_observed_lag_samples,
             tx_summary.timeout_carry_deadline_would_drop_samples,
             tx_summary.timeout_carry_deadline_dropped_chunks,
             tx_summary.timeout_carry_deadline_partial_drops,
             tx_summary.timeout_carry_deadline_dropped_samples,
             tx_summary.deadline_write_limited_writes,
	             tx_summary.deadline_write_breaks,
	             tx_summary.deadline_write_hwtime_failures,
	             tx_summary.deadline_write_timeouts,
	             tx_summary.deadline_write_carry_timeouts,
	             tx_summary.deadline_write_abandoned_samples,
	             tx_summary.deadline_write_max_selected_timeout_us,
	             tx_summary.deadline_write_first_timeout_logical,
	             tx_summary.deadline_write_inside_guard_attempts,
	             tx_summary.deadline_write_inside_guard_successes,
	             tx_summary.deadline_write_inside_guard_timeouts,
	             tx_summary.deadline_write_inside_guard_recovered_samples,
             mtu,
             tx_force_chunk_samples,
             tx_suppress_empty_eob,
             tx_force_continuous_stream,
             tx_all_timed_chunks,
             tx_skip_stream_io,
             tx_skip_stream_setup,
             tx_skip_setupstream_only,
             write_timeout_us,
             tx_timeout_retry_enabled,
             tx_timeout_retry_max_attempts,
             tx_timeout_retry_timeout_us,
             tx_timeout_carry_enabled,
             static_cast<unsigned long long>(tx_timeout_carry_max_samples),
             tx_timeout_carry_drain_max_writes,
             tx_timeout_carry_write_timeout_us,
             tx_timeout_carry_deadline_enabled,
             tx_timeout_carry_deadline_drop_enabled,
             static_cast<unsigned long long>(tx_timeout_carry_max_lag_samples),
	             tx_timeout_carry_deadline_guard_us,
	             tx_timeout_carry_deadline_max_write_timeout_us,
	             tx_deadline_write_timeout_enabled,
	             tx_deadline_write_guard_us,
	             tx_deadline_write_max_timeout_us,
	             tx_deadline_write_inside_guard_enabled,
	             tx_deadline_write_time_offset_ns,
	             tx_deadline_write_carry_on_timeout_enabled,
	             tx_deadline_write_direct_poll_us,
	             discontinuous_tx,
             tx_cf32_format ? "CF32" : "CS16");
  tx_summary.last_report_logical = tx_summary.logical_transmits;
}

void radio_soapy_tx_stream::maybe_log_tx_summary()
{
  if (!tx_summary_enabled || tx_summary_period == 0) {
    return;
  }
  if (tx_summary.logical_transmits - tx_summary.last_report_logical >= tx_summary_period) {
    log_tx_summary("periodic");
  }
}

bool radio_soapy_tx_stream::queue_tx_pending_chunk(
    const std::array<span<const ci16_t>, RADIO_MAX_NOF_CHANNELS>& src,
    unsigned nof_samples,
    int flags,
    bool final_data_chunk,
    long long time_ns,
    uint64_t logical_ts,
    unsigned logical_offset)
{
  if (nof_samples == 0) {
    return true;
  }

  if (tx_pending_samples + nof_samples > tx_timeout_carry_max_samples) {
    if (tx_summary_enabled) {
      if (tx_summary.timeout_carry_first_overflow_logical == 0) {
        tx_summary.timeout_carry_first_overflow_logical = tx_summary.logical_transmits;
      }
      ++tx_summary.timeout_carry_overflow_chunks;
      tx_summary.timeout_carry_overflow_samples += nof_samples;
      tx_summary.timeout_carry_abandoned_samples += nof_samples;
    }
    logger.warning("SoapySDR TX: timeout-carry queue overflow stream={} chunk_samples={} pending_samples={} "
                   "max_samples={}; abandoning chunk.",
                   stream_id,
                   nof_samples,
                   tx_pending_samples,
                   tx_timeout_carry_max_samples);
    return false;
  }

  tx_pending_chunk chunk;
  chunk.nof_samples      = nof_samples;
  chunk.flags            = flags;
  chunk.final_data_chunk = final_data_chunk;
  chunk.time_ns          = time_ns;
  chunk.logical_ts       = logical_ts;
  chunk.logical_offset   = logical_offset;
  chunk.samples.resize(static_cast<size_t>(nof_channels) * nof_samples);
  for (unsigned ch = 0; ch != nof_channels; ++ch) {
    const span<const ci16_t> channel_src = src[ch];
    if (channel_src.size() < nof_samples) {
      if (tx_summary_enabled) {
        tx_summary.timeout_carry_abandoned_samples += nof_samples;
      }
      logger.warning("SoapySDR TX: timeout-carry queue source too short stream={} channel={} chunk_samples={} "
                     "source_samples={}; abandoning chunk.",
                     stream_id,
                     ch,
                     nof_samples,
                     channel_src.size());
      return false;
    }
    std::copy(channel_src.begin(),
              channel_src.begin() + nof_samples,
              chunk.samples.begin() + static_cast<size_t>(ch) * nof_samples);
  }

  tx_pending_chunks.push_back(std::move(chunk));
  tx_pending_samples += nof_samples;
  if (tx_summary_enabled) {
    if (tx_summary.timeout_carry_first_queue_logical == 0) {
      tx_summary.timeout_carry_first_queue_logical = tx_summary.logical_transmits;
    }
    ++tx_summary.timeout_carry_queued_chunks;
    tx_summary.timeout_carry_queued_samples += nof_samples;
    tx_summary.timeout_carry_max_pending_samples =
        std::max<uint64_t>(tx_summary.timeout_carry_max_pending_samples, tx_pending_samples);
  }
  return true;
}

void radio_soapy_tx_stream::trim_tx_pending_front(unsigned accepted_samples)
{
  if (tx_pending_chunks.empty() || accepted_samples == 0) {
    return;
  }

  tx_pending_chunk& chunk = tx_pending_chunks.front();
  if (accepted_samples >= chunk.nof_samples) {
    tx_pending_samples -= chunk.nof_samples;
    tx_pending_chunks.pop_front();
    return;
  }

  const unsigned remaining = chunk.nof_samples - accepted_samples;
  std::vector<ci16_t> trimmed(static_cast<size_t>(nof_channels) * remaining);
  for (unsigned ch = 0; ch != nof_channels; ++ch) {
    const ci16_t* src_begin = chunk.samples.data() + static_cast<size_t>(ch) * chunk.nof_samples + accepted_samples;
    std::copy(src_begin, src_begin + remaining, trimmed.begin() + static_cast<size_t>(ch) * remaining);
  }
  chunk.samples.swap(trimmed);
  chunk.nof_samples = remaining;
  chunk.time_ns += samples_to_ns(accepted_samples, srate_hz);
  chunk.logical_offset += accepted_samples;
  if (((chunk.flags & SOAPY_SDR_HAS_TIME) != 0) && !tx_all_timed_chunks) {
    chunk.flags &= ~SOAPY_SDR_HAS_TIME;
  }
  tx_pending_samples -= accepted_samples;
}

long radio_soapy_tx_stream::select_tx_pending_write_timeout(long long deadline_time_ns)
{
  long selected_timeout_us = tx_timeout_carry_write_timeout_us;
  if (!tx_timeout_carry_deadline_enabled || deadline_time_ns <= 0) {
    return selected_timeout_us;
  }

  long long hw_now_ns = 0;
  if (!device.get_hardware_time(hw_now_ns)) {
    if (tx_summary_enabled) {
      ++tx_summary.timeout_carry_deadline_hwtime_failures;
    }
    return selected_timeout_us;
  }

  const long long guarded_lead_us = ((deadline_time_ns - hw_now_ns) / 1000) - tx_timeout_carry_deadline_guard_us;
  if (guarded_lead_us <= 0) {
    if (tx_summary_enabled) {
      ++tx_summary.timeout_carry_deadline_breaks;
    }
    return -1;
  }

  long deadline_timeout_us = tx_timeout_carry_deadline_max_write_timeout_us;
  if (deadline_timeout_us <= 0) {
    deadline_timeout_us = write_timeout_us > 0 ? write_timeout_us : 1000;
  }
  deadline_timeout_us = static_cast<long>(std::min<long long>(deadline_timeout_us, guarded_lead_us));
  deadline_timeout_us = std::max<long>(1, deadline_timeout_us);

  if (selected_timeout_us > 0) {
    selected_timeout_us = std::min(selected_timeout_us, deadline_timeout_us);
  } else {
    selected_timeout_us = deadline_timeout_us;
  }

  if (tx_summary_enabled && selected_timeout_us != tx_timeout_carry_write_timeout_us) {
    ++tx_summary.timeout_carry_deadline_limited_writes;
  }
  return selected_timeout_us;
}

long radio_soapy_tx_stream::select_tx_data_write_timeout(long long deadline_time_ns,
                                                         bool&     deadline_controlled,
                                                         bool&     inside_guard_attempt)
{
  deadline_controlled = false;
  inside_guard_attempt = false;
  if (!tx_deadline_write_timeout_enabled || deadline_time_ns <= 0) {
    return write_timeout_us;
  }

  long long hw_now_ns = 0;
  if (!device.get_hardware_time(hw_now_ns)) {
    if (tx_summary_enabled) {
      ++tx_summary.deadline_write_hwtime_failures;
    }
    return write_timeout_us;
  }

  deadline_controlled = true;
  long long effective_deadline_time_ns = deadline_time_ns;
  if (tx_deadline_write_time_offset_ns > 0 &&
      deadline_time_ns > std::numeric_limits<long long>::max() - tx_deadline_write_time_offset_ns) {
    effective_deadline_time_ns = std::numeric_limits<long long>::max();
  } else if (tx_deadline_write_time_offset_ns < 0 &&
             deadline_time_ns < std::numeric_limits<long long>::min() - tx_deadline_write_time_offset_ns) {
    effective_deadline_time_ns = std::numeric_limits<long long>::min();
  } else {
    effective_deadline_time_ns += tx_deadline_write_time_offset_ns;
  }
  const radio_soapy_tx_deadline_selection selection =
      select_radio_soapy_tx_deadline_timeout(effective_deadline_time_ns,
                                             hw_now_ns,
                                             tx_deadline_write_guard_us,
                                             tx_deadline_write_max_timeout_us,
                                             tx_deadline_write_inside_guard_enabled);
  if (selection.deadline_expired) {
    if (tx_deadline_break_trace_enabled) {
      tx_deadline_break_hw_now_ns = hw_now_ns;
      tx_deadline_break_requested_deadline_ns = deadline_time_ns;
      tx_deadline_break_effective_deadline_ns = effective_deadline_time_ns;
      tx_deadline_break_guarded_lead_us = selection.guarded_lead_us;
    }
    if (tx_summary_enabled) {
      ++tx_summary.deadline_write_breaks;
    }
    return -1;
  }

  inside_guard_attempt = selection.inside_guard;
  if (inside_guard_attempt && tx_summary_enabled) {
    ++tx_summary.deadline_write_inside_guard_attempts;
  }
  if (inside_guard_attempt && tx_deadline_break_trace_enabled) {
    tx_deadline_break_hw_now_ns = hw_now_ns;
    tx_deadline_break_requested_deadline_ns = deadline_time_ns;
    tx_deadline_break_effective_deadline_ns = effective_deadline_time_ns;
    tx_deadline_break_guarded_lead_us = selection.guarded_lead_us;
  }

  long selected_timeout_us = selection.timeout_us;
  if (tx_deadline_write_carry_on_timeout_enabled) {
    long direct_poll_us = tx_deadline_write_direct_poll_us > 0 ? tx_deadline_write_direct_poll_us : write_timeout_us;
    if (direct_poll_us <= 0) {
      direct_poll_us = 1;
    }
    selected_timeout_us = std::min(selected_timeout_us, direct_poll_us);
  }
  selected_timeout_us = std::max<long>(1, selected_timeout_us);

  if (tx_summary_enabled) {
    tx_summary.deadline_write_max_selected_timeout_us =
        std::max<uint64_t>(tx_summary.deadline_write_max_selected_timeout_us,
                           static_cast<uint64_t>(selected_timeout_us));
    if (selected_timeout_us != write_timeout_us) {
      ++tx_summary.deadline_write_limited_writes;
    }
  }
  return selected_timeout_us;
}

uint64_t radio_soapy_tx_stream::trim_tx_pending_for_lag(uint64_t current_start_ts)
{
  if (!tx_timeout_carry_enabled || !tx_timeout_carry_deadline_enabled || tx_timeout_carry_max_lag_samples == 0) {
    return 0;
  }

  uint64_t dropped_samples = 0;
  while (!tx_pending_chunks.empty()) {
    tx_pending_chunk& chunk = tx_pending_chunks.front();
    const uint64_t chunk_start_ts = chunk.logical_ts + chunk.logical_offset;
    if (current_start_ts <= chunk_start_ts + tx_timeout_carry_max_lag_samples) {
      break;
    }

    const uint64_t observed_lag_samples = current_start_ts - chunk_start_ts;
    const uint64_t stale_samples = current_start_ts - tx_timeout_carry_max_lag_samples - chunk_start_ts;
    const unsigned drop_samples =
        static_cast<unsigned>(std::min<uint64_t>(static_cast<uint64_t>(chunk.nof_samples), stale_samples));
    if (drop_samples == 0) {
      break;
    }

    const bool full_chunk_drop = drop_samples >= chunk.nof_samples;
    if (tx_summary_enabled) {
      if (tx_summary.timeout_carry_first_lag_logical == 0) {
        tx_summary.timeout_carry_first_lag_logical = tx_summary.logical_transmits;
      }
      ++tx_summary.timeout_carry_deadline_lag_events;
      tx_summary.timeout_carry_deadline_max_observed_lag_samples =
          std::max<uint64_t>(tx_summary.timeout_carry_deadline_max_observed_lag_samples, observed_lag_samples);
      if (!tx_timeout_carry_deadline_drop_enabled) {
        tx_summary.timeout_carry_deadline_would_drop_samples += drop_samples;
      }
    }

    if (!tx_timeout_carry_deadline_drop_enabled) {
      if (tx_trace_enabled) {
        logger.info("Soapy TX trace: stream={} timeout_carry_deadline_lag current_ts={} chunk_ts={} "
                    "observed_lag_samples={} max_lag_samples={} would_drop_samples={}",
                    stream_id,
                    current_start_ts,
                    chunk_start_ts,
                    observed_lag_samples,
                    tx_timeout_carry_max_lag_samples,
                    drop_samples);
      }
      break;
    }

    if (tx_summary_enabled) {
      if (full_chunk_drop) {
        ++tx_summary.timeout_carry_deadline_dropped_chunks;
      } else {
        ++tx_summary.timeout_carry_deadline_partial_drops;
      }
      tx_summary.timeout_carry_deadline_dropped_samples += drop_samples;
      tx_summary.timeout_carry_abandoned_samples += drop_samples;
    }
    if (tx_trace_enabled) {
      logger.info("Soapy TX trace: stream={} timeout_carry_deadline_drop samples={} current_ts={} chunk_ts={} "
                  "max_lag_samples={}",
                  stream_id,
                  drop_samples,
                  current_start_ts,
                  chunk_start_ts,
                  tx_timeout_carry_max_lag_samples);
    }
    trim_tx_pending_front(drop_samples);
    dropped_samples += drop_samples;
  }
  return dropped_samples;
}

unsigned radio_soapy_tx_stream::drain_tx_pending_chunks(unsigned max_writes, long long deadline_time_ns)
{
  if (!tx_timeout_carry_enabled || max_writes == 0) {
    return 0;
  }

  unsigned writes = 0;
  while (!tx_pending_chunks.empty() && writes < max_writes) {
    tx_pending_chunk& chunk = tx_pending_chunks.front();
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> buffs = {};
    const void* dump_src_ptr = nullptr;

    if (tx_cf32_format) {
      tx_pending_cf32_conversion_buffer.resize(static_cast<size_t>(nof_channels) * chunk.nof_samples);
      for (unsigned ch = 0; ch != nof_channels; ++ch) {
        const ci16_t* src_ptr = chunk.samples.data() + static_cast<size_t>(ch) * chunk.nof_samples;
        cf_t* dst_ptr = tx_pending_cf32_conversion_buffer.data() + static_cast<size_t>(ch) * chunk.nof_samples;
        ocuduvec::convert(span<cf_t>(dst_ptr, chunk.nof_samples), span<const ci16_t>(src_ptr, chunk.nof_samples), SCALING_FACTOR_CI16_TO_CF);
        buffs[ch] = dst_ptr;
      }
      if (tx_write_dump_port < nof_channels) {
        dump_src_ptr = tx_pending_cf32_conversion_buffer.data() + static_cast<size_t>(tx_write_dump_port) * chunk.nof_samples;
      }
    } else {
      for (unsigned ch = 0; ch != nof_channels; ++ch) {
        const ci16_t* src_ptr = chunk.samples.data() + static_cast<size_t>(ch) * chunk.nof_samples;
        buffs[ch] = src_ptr;
      }
      if (tx_write_dump_port < nof_channels) {
        dump_src_ptr = chunk.samples.data() + static_cast<size_t>(tx_write_dump_port) * chunk.nof_samples;
      }
    }

    const int requested_flags = chunk.flags;
    int flags_after = requested_flags;
    const long carry_timeout_us = select_tx_pending_write_timeout(deadline_time_ns);
    if (carry_timeout_us < 0) {
      break;
    }
    const int ret = device.write_stream(stream, buffs.data(), chunk.nof_samples, flags_after, chunk.time_ns, carry_timeout_us);
    ++writes;
    if (tx_summary_enabled) {
      ++tx_summary.timeout_carry_drain_writes;
    }
    record_tx_write(chunk.nof_samples, ret, requested_flags, true, chunk.final_data_chunk, false, false, false, chunk.time_ns);
    dump_tx_write_cf32("carry",
                       dump_src_ptr,
                       tx_cf32_format,
                       chunk.nof_samples,
                       ret,
                       requested_flags,
                       flags_after,
                       chunk.time_ns,
                       chunk.logical_ts,
                       chunk.logical_offset);

    if (ret == static_cast<int>(chunk.nof_samples)) {
      if (tx_summary_enabled) {
        ++tx_summary.timeout_carry_drained_chunks;
        tx_summary.timeout_carry_drained_samples += chunk.nof_samples;
      }
      trim_tx_pending_front(chunk.nof_samples);
      continue;
    }

    if (ret > 0) {
      if (tx_summary_enabled) {
        ++tx_summary.timeout_carry_partial_drains;
        tx_summary.timeout_carry_drained_samples += static_cast<uint64_t>(ret);
      }
      trim_tx_pending_front(static_cast<unsigned>(ret));
      continue;
    }

    if (ret == SOAPY_SDR_TIMEOUT) {
      if (tx_summary_enabled) {
        ++tx_summary.timeout_carry_drain_timeouts;
      }
      break;
    }

    if (tx_summary_enabled) {
      ++tx_summary.timeout_carry_drain_errors;
      tx_summary.timeout_carry_abandoned_samples += chunk.nof_samples;
    }
    logger.warning("SoapySDR TX: timeout-carry drain failed ret={} stream={}; abandoning {} pending samples.",
                   ret,
                   stream_id,
                   chunk.nof_samples);
    trim_tx_pending_front(chunk.nof_samples);
    break;
  }
  return writes;
}

void radio_soapy_tx_stream::init_tx_write_dump()
{
  tx_write_dump_cf32_path = std::getenv("PAVONIS_SOAPY_TX_WRITE_DUMP_CF32_PATH");
  if ((tx_write_dump_cf32_path == nullptr) || (tx_write_dump_cf32_path[0] == '\0')) {
    tx_write_dump_cf32_path = nullptr;
    return;
  }

  tx_write_dump_meta_path   = std::getenv("PAVONIS_SOAPY_TX_WRITE_DUMP_META_PATH");
  tx_write_dump_max_samples = getenv_u64_or_zero("PAVONIS_SOAPY_TX_WRITE_DUMP_MAX_SAMPLES");
  if (const char* env = std::getenv("PAVONIS_SOAPY_TX_WRITE_DUMP_PORT")) {
    const long value = std::strtol(env, nullptr, 10);
    if (value >= 0) {
      tx_write_dump_port = static_cast<unsigned>(value);
    }
  }

  if (FILE* dump = std::fopen(tx_write_dump_cf32_path, "wb")) {
    std::fclose(dump);
  } else {
    tx_write_dump_cf32_path = nullptr;
    return;
  }

  if ((tx_write_dump_meta_path != nullptr) && (tx_write_dump_meta_path[0] != '\0')) {
    if (FILE* meta = std::fopen(tx_write_dump_meta_path, "w")) {
      std::fprintf(meta, "PAVONIS_SOAPY_TX_WRITE_DUMP=1\n");
      std::fprintf(meta, "stream_id=%u\n", stream_id);
      std::fprintf(meta, "srate_hz=%.9f\n", srate_hz);
      std::fprintf(meta, "nof_channels=%u\n", nof_channels);
      std::fprintf(meta, "dump_port=%u\n", tx_write_dump_port);
      std::fprintf(meta, "dump_format=CF32\n");
      std::fprintf(meta, "tx_format=%s\n", tx_cf32_format ? "CF32" : "CS16");
      std::fprintf(meta, "ci16_to_cf32_scale=%.9f\n", SCALING_FACTOR_CI16_TO_CF);
      std::fprintf(meta, "max_samples=%llu\n", static_cast<unsigned long long>(tx_write_dump_max_samples));
      std::fprintf(meta,
                   "row\tkind\tlogical_ts\tlogical_offset\ttime_ns\trequested\tret\tflags_before\tflags_after"
                   "\tdumped_samples\ttotal_dumped_samples\n");
      std::fclose(meta);
    } else {
      tx_write_dump_meta_path = nullptr;
    }
  }
}

void radio_soapy_tx_stream::init_tx_marker()
{
  tx_marker_enabled = env_enabled("PAVONIS_SOAPY_TX_MARKER_ENABLE");
  tx_marker_configured_start_ts = getenv_u64_or_zero("PAVONIS_SOAPY_TX_MARKER_START_TS");
  tx_marker_len = getenv_unsigned_or_default("PAVONIS_SOAPY_TX_MARKER_LEN", tx_marker_len);
  tx_marker_amp = std::max(1, std::min<int>(32767, getenv_int_or_default("PAVONIS_SOAPY_TX_MARKER_AMP", tx_marker_amp)));
  tx_marker_port = getenv_unsigned_or_default("PAVONIS_SOAPY_TX_MARKER_PORT", tx_marker_port);
  tx_marker_seed = getenv_unsigned_or_default("PAVONIS_SOAPY_TX_MARKER_SEED", tx_marker_seed);
  tx_marker_meta_path = std::getenv("PAVONIS_SOAPY_TX_MARKER_META_PATH");
  if ((tx_marker_meta_path != nullptr) && (tx_marker_meta_path[0] == '\0')) {
    tx_marker_meta_path = nullptr;
  }

  tx_marker_events.clear();
  std::vector<uint64_t> delay_sequence = getenv_u64_list("PAVONIS_SOAPY_TX_MARKER_DELAY_SEQUENCE_SAMPLES");
  if (delay_sequence.empty()) {
    delay_sequence = getenv_u64_list("PAVONIS_SOAPY_TX_MARKER_DELAYS_SAMPLES");
  }

  if (!delay_sequence.empty()) {
    tx_marker_sequence_mode = true;
    tx_marker_events.reserve(delay_sequence.size());
    for (size_t i = 0; i != delay_sequence.size(); ++i) {
      tx_marker_event marker;
      marker.delay_samples = delay_sequence[i];
      marker.dynamic_start = tx_marker_configured_start_ts == 0;
      marker.start_selected = !marker.dynamic_start;
      marker.start_ts = marker.dynamic_start ? 0 : (tx_marker_configured_start_ts + marker.delay_samples);
      marker.seed = tx_marker_seed + static_cast<uint32_t>(i);
      tx_marker_events.push_back(marker);
    }
  } else {
    tx_marker_sequence_mode = false;
    tx_marker_start_ts = tx_marker_configured_start_ts;
    tx_marker_start_selected = tx_marker_start_ts != 0;
    if (const char* delay_env = std::getenv("PAVONIS_SOAPY_TX_MARKER_DELAY_SAMPLES")) {
      if (delay_env[0] != '\0') {
        tx_marker_delay_samples = static_cast<uint64_t>(std::strtoull(delay_env, nullptr, 10));
        tx_marker_dynamic_start = tx_marker_start_ts == 0;
      }
    }
    if (tx_marker_start_selected || tx_marker_dynamic_start) {
      tx_marker_event marker;
      marker.start_ts = tx_marker_start_ts;
      marker.delay_samples = tx_marker_delay_samples;
      marker.dynamic_start = tx_marker_dynamic_start;
      marker.start_selected = tx_marker_start_selected;
      marker.seed = tx_marker_seed;
      tx_marker_events.push_back(marker);
    }
  }
  sync_primary_tx_marker_fields();

  if (tx_marker_enabled && (tx_marker_events.empty() || (tx_marker_len == 0))) {
    tx_marker_enabled = false;
  }

  if (tx_marker_enabled && (tx_marker_meta_path != nullptr)) {
    if (FILE* meta = std::fopen(tx_marker_meta_path, "w")) {
      std::fprintf(meta, "PAVONIS_SOAPY_TX_MARKER=1\n");
      std::fprintf(meta, "stream_id=%u\n", stream_id);
      std::fprintf(meta,
                   "configured_start_ts=%llu\n",
                   static_cast<unsigned long long>(tx_marker_configured_start_ts));
      std::fprintf(meta, "start_ts=%llu\n", static_cast<unsigned long long>(tx_marker_start_ts));
      std::fprintf(meta,
                   "start_mode=%s\n",
                   tx_marker_sequence_mode
                       ? (tx_marker_dynamic_start ? "first_data_delay_sequence" : "absolute_sequence")
                       : (tx_marker_dynamic_start ? "first_data_delay" : "absolute"));
      std::fprintf(meta,
                   "delay_samples=%llu\n",
                   static_cast<unsigned long long>(tx_marker_delay_samples));
      std::fprintf(meta, "start_selected=%u\n", tx_marker_start_selected ? 1U : 0U);
      std::fprintf(meta, "sequence_mode=%u\n", tx_marker_sequence_mode ? 1U : 0U);
      std::fprintf(meta, "marker_count=%zu\n", tx_marker_events.size());
      std::fprintf(meta,
                   "delay_sequence_samples=%s\n",
                   getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_DELAY_SEQUENCE_SAMPLES"));
      std::fprintf(meta,
                   "delays_samples=%s\n",
                   getenv_or_empty("PAVONIS_SOAPY_TX_MARKER_DELAYS_SAMPLES"));
      std::fprintf(meta, "len=%u\n", tx_marker_len);
      std::fprintf(meta, "amp=%d\n", tx_marker_amp);
      std::fprintf(meta, "port=%u\n", tx_marker_port);
      std::fprintf(meta, "seed=%u\n", tx_marker_seed);
      std::fprintf(meta,
                   "row\tmarker_index\tmarker_seed\tchannel\tchunk_start_ts\tchunk_time_ns\tlogical_offset\toverlay_start_ts\toverlay_end_ts"
                   "\tmarker_offset\tsamples\n");
      std::fclose(meta);
    } else {
      tx_marker_meta_path = nullptr;
    }
  }
}

void radio_soapy_tx_stream::sync_primary_tx_marker_fields()
{
  if (tx_marker_events.empty()) {
    tx_marker_start_ts = tx_marker_configured_start_ts;
    tx_marker_delay_samples = 0;
    tx_marker_dynamic_start = false;
    tx_marker_start_selected = tx_marker_start_ts != 0;
    return;
  }

  const tx_marker_event& first = tx_marker_events.front();
  tx_marker_start_ts = first.start_ts;
  tx_marker_delay_samples = first.delay_samples;
  tx_marker_dynamic_start = first.dynamic_start;
  tx_marker_start_selected = first.start_selected;
}

void radio_soapy_tx_stream::write_tx_marker_selection_meta(unsigned marker_index,
                                                           const tx_marker_event& marker,
                                                           uint64_t chunk_start_ts,
                                                           long long chunk_time_ns,
                                                           unsigned logical_offset)
{
  if (tx_marker_meta_path == nullptr) {
    return;
  }
  if (FILE* meta = std::fopen(tx_marker_meta_path, "a")) {
    if (marker_index == 0) {
      std::fprintf(meta,
                   "selected_start_ts=%llu\n",
                   static_cast<unsigned long long>(marker.start_ts));
      std::fprintf(meta,
                   "selected_first_chunk_start_ts=%llu\n",
                   static_cast<unsigned long long>(chunk_start_ts));
      std::fprintf(meta, "selected_first_chunk_time_ns=%lld\n", chunk_time_ns);
      std::fprintf(meta, "selected_first_chunk_logical_offset=%u\n", logical_offset);
    }
    std::fprintf(meta,
                 "selected_marker_%u_start_ts=%llu\n",
                 marker_index,
                 static_cast<unsigned long long>(marker.start_ts));
    std::fprintf(meta,
                 "selected_marker_%u_delay_samples=%llu\n",
                 marker_index,
                 static_cast<unsigned long long>(marker.delay_samples));
    std::fprintf(meta, "selected_marker_%u_seed=%u\n", marker_index, static_cast<unsigned>(marker.seed));
    std::fprintf(meta,
                 "selected_marker_%u_first_chunk_start_ts=%llu\n",
                 marker_index,
                 static_cast<unsigned long long>(chunk_start_ts));
    std::fprintf(meta, "selected_marker_%u_first_chunk_time_ns=%lld\n", marker_index, chunk_time_ns);
    std::fprintf(meta, "selected_marker_%u_first_chunk_logical_offset=%u\n", marker_index, logical_offset);
    std::fclose(meta);
  }
}

void radio_soapy_tx_stream::write_tx_marker_meta(unsigned marker_index,
                                                 const tx_marker_event& marker,
                                                 unsigned channel,
                                                 uint64_t chunk_start_ts,
                                                 long long chunk_time_ns,
                                                 unsigned logical_offset,
                                                 uint64_t overlay_start_ts,
                                                 uint64_t overlay_end_ts)
{
  if ((tx_marker_meta_path == nullptr) || (overlay_end_ts <= overlay_start_ts)) {
    return;
  }
  if (FILE* meta = std::fopen(tx_marker_meta_path, "a")) {
    std::fprintf(meta,
                 "%llu\t%u\t%u\t%u\t%llu\t%lld\t%u\t%llu\t%llu\t%llu\t%llu\n",
                 static_cast<unsigned long long>(tx_marker_rows++),
                 marker_index,
                 static_cast<unsigned>(marker.seed),
                 channel,
                 static_cast<unsigned long long>(chunk_start_ts),
                 chunk_time_ns,
                 logical_offset,
                 static_cast<unsigned long long>(overlay_start_ts),
                 static_cast<unsigned long long>(overlay_end_ts),
                 static_cast<unsigned long long>(overlay_start_ts - marker.start_ts),
                 static_cast<unsigned long long>(overlay_end_ts - overlay_start_ts));
    std::fclose(meta);
  }
}

void radio_soapy_tx_stream::select_dynamic_tx_markers(uint64_t chunk_start_ts,
                                                      long long chunk_time_ns,
                                                      unsigned logical_offset)
{
  bool selected_any = false;
  for (size_t marker_index = 0; marker_index != tx_marker_events.size(); ++marker_index) {
    tx_marker_event& marker = tx_marker_events[marker_index];
    if (marker.start_selected || !marker.dynamic_start) {
      continue;
    }
    marker.start_ts = chunk_start_ts + marker.delay_samples;
    marker.start_selected = true;
    write_tx_marker_selection_meta(static_cast<unsigned>(marker_index), marker, chunk_start_ts, chunk_time_ns, logical_offset);
    fmt::print(stderr,
               "PAVONIS_SOAPY_TX_MARKER selected dynamic start: stream={} marker_index={} start_ts={} "
               "first_chunk_start_ts={} delay_samples={} len={} seed={}\n",
               stream_id,
               marker_index,
               marker.start_ts,
               chunk_start_ts,
               marker.delay_samples,
               tx_marker_len,
               marker.seed);
    selected_any = true;
  }
  if (selected_any) {
    sync_primary_tx_marker_fields();
  }
}

span<const ci16_t> radio_soapy_tx_stream::maybe_apply_tx_marker(span<const ci16_t> src,
                                                                uint64_t           chunk_start_ts,
                                                                unsigned           channel,
                                                                long long          chunk_time_ns,
                                                                unsigned           logical_offset)
{
  if (!tx_marker_enabled || (channel != tx_marker_port) || src.empty() || tx_marker_events.empty()) {
    return src;
  }

  const uint64_t chunk_end_ts = chunk_start_ts + static_cast<uint64_t>(src.size());
  select_dynamic_tx_markers(chunk_start_ts, chunk_time_ns, logical_offset);

  bool copied = false;
  for (size_t marker_index = 0; marker_index != tx_marker_events.size(); ++marker_index) {
    const tx_marker_event& marker = tx_marker_events[marker_index];
    if (!marker.start_selected) {
      continue;
    }
    const uint64_t marker_end_ts = marker.start_ts + static_cast<uint64_t>(tx_marker_len);
    const uint64_t overlay_start_ts = std::max<uint64_t>(chunk_start_ts, marker.start_ts);
    const uint64_t overlay_end_ts = std::min<uint64_t>(chunk_end_ts, marker_end_ts);
    if (overlay_end_ts <= overlay_start_ts) {
      continue;
    }

    if (!copied) {
      tx_marker_ci16_buffer.resize(src.size());
      std::copy(src.begin(), src.end(), tx_marker_ci16_buffer.begin());
      copied = true;
    }
    for (uint64_t ts = overlay_start_ts; ts != overlay_end_ts; ++ts) {
      const size_t local_idx = static_cast<size_t>(ts - chunk_start_ts);
      const uint64_t marker_idx = ts - marker.start_ts;
      const ci16_t marker_sample = pavonis_marker_sample(marker.seed, marker_idx, tx_marker_amp);
      const ci16_t current = tx_marker_ci16_buffer[local_idx];
      tx_marker_ci16_buffer[local_idx] =
          ci16_t(clamp_i16(static_cast<int>(current.real()) + static_cast<int>(marker_sample.real())),
                 clamp_i16(static_cast<int>(current.imag()) + static_cast<int>(marker_sample.imag())));
    }

    write_tx_marker_meta(static_cast<unsigned>(marker_index),
                         marker,
                         channel,
                         chunk_start_ts,
                         chunk_time_ns,
                         logical_offset,
                         overlay_start_ts,
                         overlay_end_ts);
  }

  if (!copied) {
    return src;
  }
  return span<const ci16_t>(tx_marker_ci16_buffer.data(), tx_marker_ci16_buffer.size());
}

void radio_soapy_tx_stream::dump_tx_write_cf32(std::string_view kind,
                                               const void*      src,
                                               bool             src_is_cf32,
                                               unsigned         requested,
                                               int              ret,
                                               int              flags_before,
                                               int              flags_after,
                                               long long        time_ns,
                                               uint64_t         logical_ts,
                                               unsigned         logical_offset)
{
  if ((tx_write_dump_cf32_path == nullptr) || (tx_write_dump_port >= nof_channels) || (src == nullptr)) {
    return;
  }

  const unsigned accepted_samples = (ret > 0) ? std::min<unsigned>(requested, static_cast<unsigned>(ret)) : 0U;
  unsigned       dump_samples     = accepted_samples;
  if (tx_write_dump_max_samples != 0) {
    if (tx_write_dump_samples >= tx_write_dump_max_samples) {
      dump_samples = 0;
    } else {
      const uint64_t remaining = tx_write_dump_max_samples - tx_write_dump_samples;
      dump_samples = static_cast<unsigned>(std::min<uint64_t>(dump_samples, remaining));
    }
  }

  unsigned actual_dump_samples = 0;
  if (dump_samples > 0) {
    if (FILE* dump = std::fopen(tx_write_dump_cf32_path, "ab")) {
      if (src_is_cf32) {
        actual_dump_samples = static_cast<unsigned>(std::fwrite(src, sizeof(cf_t), dump_samples, dump));
      } else {
        tx_write_dump_cf32_buffer.resize(dump_samples);
        span<const ci16_t> chunk_ci16(static_cast<const ci16_t*>(src), dump_samples);
        span<cf_t>         chunk_cf32(tx_write_dump_cf32_buffer.data(), dump_samples);
        ocuduvec::convert(chunk_cf32, chunk_ci16, SCALING_FACTOR_CI16_TO_CF);
        actual_dump_samples =
            static_cast<unsigned>(std::fwrite(tx_write_dump_cf32_buffer.data(), sizeof(cf_t), dump_samples, dump));
      }
      std::fclose(dump);
    }
  }

  tx_write_dump_samples += actual_dump_samples;

  if ((tx_write_dump_meta_path != nullptr) && (tx_write_dump_meta_path[0] != '\0')) {
    if (FILE* meta = std::fopen(tx_write_dump_meta_path, "a")) {
      std::fprintf(meta,
                   "%llu\t%.*s\t%llu\t%u\t%lld\t%u\t%d\t%d\t%d\t%u\t%llu\n",
                   static_cast<unsigned long long>(tx_write_dump_rows),
                   static_cast<int>(kind.size()),
                   kind.data(),
                   static_cast<unsigned long long>(logical_ts),
                   logical_offset,
                   time_ns,
                   requested,
                   ret,
                   flags_before,
                   flags_after,
                   actual_dump_samples,
                   static_cast<unsigned long long>(tx_write_dump_samples));
      std::fclose(meta);
    }
  }
  ++tx_write_dump_rows;
}

void radio_soapy_tx_stream::transmit(const baseband_gateway_buffer_reader&        data,
                                      const baseband_gateway_transmitter_metadata& tx_md)
{
  const auto tx_start_tp = std::chrono::steady_clock::now();
  long long  tx_deadline_break_entry_gap_us = -1;
  uint64_t   tx_deadline_break_prev_md_ts_snapshot = 0;
  if (tx_deadline_break_trace_enabled) {
    if (tx_deadline_break_prev_start_valid) {
      tx_deadline_break_entry_gap_us =
          std::chrono::duration_cast<std::chrono::microseconds>(tx_start_tp - tx_deadline_break_prev_start_tp).count();
      tx_deadline_break_prev_md_ts_snapshot = tx_deadline_break_prev_md_ts;
    }
    tx_deadline_break_prev_start_tp = tx_start_tp;
    tx_deadline_break_prev_md_ts = tx_md.ts;
    tx_deadline_break_prev_start_valid = true;
  }
  auto token = stop_control.get_token();
  if (OCUDU_UNLIKELY(token.is_stop_requested())) {
    return;
  }

  const bool tx_start_padding = tx_md.tx_start.has_value();
  const bool tx_end_padding   = tx_md.tx_end.has_value();

  // Compute TX timestamp in nanoseconds.
  long long time_ns = samples_to_ns(tx_md.ts, srate_hz);
  if (discontinuous_tx && tx_start_padding) {
    time_ns = samples_to_ns(tx_md.ts + static_cast<baseband_gateway_timestamp>(tx_md.tx_start.value()), srate_hz);
  }

  int  flags    = 0;
  bool transmit = false;
  if (discontinuous_tx) {
    transmit = state_fsm.on_transmit(flags, time_ns, tx_md.is_empty, tx_end_padding);
  } else {
    transmit = state_fsm.on_transmit(flags, time_ns, false, false);
  }

  if (!transmit) {
    return;
  }

  const int logical_flags = flags;
  if (tx_force_continuous_stream) {
    flags = 0;
  }

  const bool is_sob = (flags & SOAPY_SDR_HAS_TIME) != 0;
  const bool is_eob = (flags & SOAPY_SDR_END_BURST) != 0;

  // Notify start of burst.
  if (is_sob) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::START_OF_BURST,
                                .timestamp  = static_cast<uint64_t>(time_ns * srate_hz / 1e9)});

    // Transmit power-ramping zeros before the burst.
    if (discontinuous_tx && power_ramping_nof_samples > 0) {
      const unsigned tx_gap_samples =
          static_cast<unsigned>((time_ns - last_tx_time_ns) * srate_hz / 1e9);
      const unsigned min_gap = static_cast<unsigned>(srate_hz / 100000.0); // 10 us

      if (tx_gap_samples > min_gap) {
        unsigned nof_pad = std::min(power_ramping_nof_samples, tx_gap_samples - min_gap);
        if (nof_pad > 0) {
          long long pad_time_ns = time_ns - samples_to_ns(nof_pad, srate_hz);
          std::array<const void*, RADIO_MAX_NOF_CHANNELS> pad_buffs = {};
          for (unsigned ch = 0; ch != nof_channels; ++ch) {
            if (tx_cf32_format) {
              pad_buffs[ch] = power_ramping_cf32_buffer.data() + static_cast<size_t>(ch) * power_ramping_nof_samples;
            } else {
              pad_buffs[ch] = power_ramping_buffer.get_reader()[ch].data();
            }
          }
          int       pad_flags           = SOAPY_SDR_HAS_TIME;
          const int requested_pad_flags = pad_flags;
          const int ret = device.write_stream(stream, pad_buffs.data(), nof_pad, pad_flags, pad_time_ns, write_timeout_us);
          record_tx_write(nof_pad, ret, requested_pad_flags, false, false, true, false, false, pad_time_ns);
          const void* pad_dump_ptr = (tx_write_dump_port < nof_channels) ? pad_buffs[tx_write_dump_port] : nullptr;
          dump_tx_write_cf32("pad",
                             pad_dump_ptr,
                             tx_cf32_format,
                             nof_pad,
                             ret,
                             requested_pad_flags,
                             pad_flags,
                             pad_time_ns,
                             static_cast<uint64_t>(tx_md.ts),
                             0);
          if (ret != static_cast<int>(nof_pad)) {
            if (ret == SOAPY_SDR_TIMEOUT) {
              logger.warning("SoapySDR TX: power ramping writeStream timeout after {} us; expected {} samples.",
                             write_timeout_us,
                             nof_pad);
            } else {
              logger.warning("SoapySDR TX: power ramping writeStream failed ret={} expected={}.", ret, nof_pad);
            }
            maybe_log_tx_summary();
            return;
          }

          // The actual burst is no longer the start-of-burst for SoapySDR
          // (we already opened the burst above with the first padding write).
          flags &= ~SOAPY_SDR_HAS_TIME;
        }
      }
    }
  }

  // Notify end of burst (before sending, so the scheduler knows).
  if (is_eob) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::END_OF_BURST,
                                .timestamp  = static_cast<uint64_t>(time_ns * srate_hz / 1e9)});
  }

  // Determine the sample range within the buffer.
  const unsigned data_start =
      (discontinuous_tx && tx_start_padding) ? tx_md.tx_start.value() : 0;
  const unsigned data_nof_samples =
      (discontinuous_tx && tx_end_padding)
          ? tx_md.tx_end.value() - data_start
          : data.get_nof_samples() - data_start;
  const unsigned logical_nof_samples = (tx_md.is_empty && discontinuous_tx) ? 0U : data_nof_samples;
  record_tx_logical(tx_md.is_empty, logical_nof_samples, logical_flags);

  if (tx_skip_stream_io) {
    if (tx_trace_enabled) {
      logger.info("Soapy TX trace: stream={} skip_stream_io logical_samples={} flags=0x{:x} ts={} tx_time_ns={}",
                  stream_id,
                  tx_md.is_empty ? 0U : data_nof_samples,
                  logical_flags,
                  tx_md.ts,
                  time_ns);
    }
    maybe_log_tx_summary();
    return;
  }

  if (tx_md.is_empty && discontinuous_tx) {
    // Mirror the UHD path: empty discontinuous buffers still need to propagate
    // an end-of-burst marker to the device.
    if (is_eob) {
      if (tx_suppress_empty_eob) {
        if (tx_summary_enabled) {
          ++tx_summary.suppressed_empty_eob;
        }
        if (tx_trace_enabled) {
          logger.info("Soapy TX trace: stream={} suppressed_empty_eob flags=0x{:x} ts={} tx_time_ns={}",
                      stream_id,
                      SOAPY_SDR_END_BURST,
                      tx_md.ts,
                      time_ns);
        }
        maybe_log_tx_summary();
        return;
      }
      std::array<const void*, RADIO_MAX_NOF_CHANNELS> dummy_buffs = {};
      int eob_flags = SOAPY_SDR_END_BURST;
      const int requested_eob_flags = eob_flags;
      const int ret = device.write_stream(stream, dummy_buffs.data(), 0, eob_flags, time_ns, write_timeout_us);
      record_tx_write(0, ret, requested_eob_flags, false, false, false, true, false, time_ns);
      if (ret < 0) {
        if (ret == SOAPY_SDR_TIMEOUT) {
          logger.warning("SoapySDR TX: empty EOB writeStream timeout after {} us for stream {}.",
                         write_timeout_us,
                         stream_id);
        } else {
          logger.warning("SoapySDR TX: empty EOB writeStream failed ret={} for stream {}.", ret, stream_id);
        }
      } else if (tx_trace_enabled) {
        logger.info("Soapy TX trace: stream={} empty_eob ret={} flags=0x{:x} ts={} tx_time_ns={}",
                    stream_id,
                    ret,
                    SOAPY_SDR_END_BURST,
                    tx_md.ts,
                    time_ns);
      }
    }
    maybe_log_tx_summary();
    return;
  }

  if (tx_timeout_carry_enabled && !tx_pending_chunks.empty()) {
    drain_tx_pending_chunks(tx_timeout_carry_drain_max_writes, time_ns);
  }

  unsigned sent_total = 0;
  unsigned chunks     = 0;
  bool     carry_queue_remaining = false;
  while (sent_total < data_nof_samples) {
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> rd_buffs = {};
    std::array<span<const ci16_t>, RADIO_MAX_NOF_CHANNELS> chunk_ci16_spans = {};
    const unsigned remaining = data_nof_samples - sent_total;
    const unsigned mtu_samples = static_cast<unsigned>(mtu);
    unsigned       max_chunk_samples = mtu_samples;
    if (tx_force_chunk_samples > 0) {
      max_chunk_samples = (max_chunk_samples > 0) ? std::min(max_chunk_samples, tx_force_chunk_samples)
                                                  : tx_force_chunk_samples;
    }
    const unsigned nof_chunk_samples =
        (max_chunk_samples > 0) ? std::min(remaining, max_chunk_samples) : remaining;
    const bool is_final_chunk = nof_chunk_samples == remaining;
    const long long chunk_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);
    const uint64_t chunk_start_ts = static_cast<uint64_t>(tx_md.ts) + data_start + sent_total;

    if (tx_cf32_format) {
      tx_cf32_conversion_buffer.resize(static_cast<size_t>(nof_channels) * nof_chunk_samples);
    }
    const void* dump_src_ptr    = nullptr;
    bool        dump_src_is_cf32 = tx_cf32_format;
    for (unsigned ch = 0; ch != nof_channels; ++ch) {
      span<const ci16_t> chunk_ci16 = data[ch].subspan(data_start + sent_total, nof_chunk_samples);
      span<const ci16_t> tx_chunk_ci16 =
          maybe_apply_tx_marker(chunk_ci16, chunk_start_ts, ch, chunk_time_ns, data_start + sent_total);
      chunk_ci16_spans[ch] = tx_chunk_ci16;
      if (tx_cf32_format) {
        cf_t*      dst_ptr = tx_cf32_conversion_buffer.data() + static_cast<size_t>(ch) * nof_chunk_samples;
        span<cf_t> chunk_cf32(dst_ptr, nof_chunk_samples);
        ocuduvec::convert(chunk_cf32, tx_chunk_ci16, SCALING_FACTOR_CI16_TO_CF);
        rd_buffs[ch] = chunk_cf32.data();
      } else {
        rd_buffs[ch] = tx_chunk_ci16.data();
      }
      if (ch == tx_write_dump_port) {
        dump_src_ptr = rd_buffs[ch];
      }
    }

    int chunk_flags = 0;
    if (sent_total == 0) {
      chunk_flags |= (flags & SOAPY_SDR_HAS_TIME);
    }
    if (tx_all_timed_chunks && !tx_md.is_empty) {
      chunk_flags |= SOAPY_SDR_HAS_TIME;
    }
    bool reanchor_due = false;
    if ((tx_reanchor_interval_samples > 0) && !tx_md.is_empty) {
      if (!tx_reanchor_origin_selected) {
        tx_reanchor_origin_selected = true;
        tx_reanchor_origin_ts = chunk_start_ts;
        tx_reanchor_next_ts = tx_reanchor_origin_ts + tx_reanchor_interval_samples;
        if (tx_reanchor_next_ts <= tx_reanchor_origin_ts) {
          tx_reanchor_next_ts = std::numeric_limits<uint64_t>::max();
        }
      }
      if (chunk_start_ts >= tx_reanchor_next_ts) {
        reanchor_due = true;
        while (chunk_start_ts >= tx_reanchor_next_ts) {
          if (tx_reanchor_next_ts > std::numeric_limits<uint64_t>::max() - tx_reanchor_interval_samples) {
            tx_reanchor_next_ts = std::numeric_limits<uint64_t>::max();
            break;
          }
          tx_reanchor_next_ts += tx_reanchor_interval_samples;
        }
      }
    }
    if (reanchor_due) {
      chunk_flags |= SOAPY_SDR_HAS_TIME;
      ++tx_summary.reanchor_writes;
    }
    if (is_final_chunk) {
      chunk_flags |= (flags & SOAPY_SDR_END_BURST);
    }

    if (tx_timeout_carry_enabled && !tx_pending_chunks.empty()) {
      trim_tx_pending_for_lag(chunk_start_ts);
    }

    if (carry_queue_remaining || (tx_timeout_carry_enabled && !tx_pending_chunks.empty())) {
      const bool queued = queue_tx_pending_chunk(chunk_ci16_spans,
                                                nof_chunk_samples,
                                                chunk_flags,
                                                is_final_chunk,
                                                chunk_time_ns,
                                                static_cast<uint64_t>(tx_md.ts),
                                                data_start + sent_total);
      if (!queued) {
        if (tx_summary_enabled && remaining > nof_chunk_samples) {
          tx_summary.timeout_carry_abandoned_samples += remaining - nof_chunk_samples;
        }
        maybe_log_tx_summary();
        return;
      }
      sent_total += nof_chunk_samples;
      ++chunks;
      carry_queue_remaining = true;
      continue;
    }

    const int requested_chunk_flags = chunk_flags;
    int       chunk_flags_after     = requested_chunk_flags;
    int       ret                   = 0;
    unsigned  timeout_retry_attempts = 0;
    bool      timeout_retry_started  = false;
    bool      deadline_write_expired    = false;
    while (true) {
      int        attempt_flags      = requested_chunk_flags;
      bool       attempt_deadline_controlled = false;
      bool       attempt_inside_guard = false;
      const long attempt_timeout_us = (timeout_retry_attempts == 0)
                                          ? select_tx_data_write_timeout(
                                                chunk_time_ns, attempt_deadline_controlled, attempt_inside_guard)
                                          : tx_timeout_retry_timeout_us;
      if (attempt_timeout_us < 0) {
        ret = SOAPY_SDR_TIMEOUT;
        deadline_write_expired = true;
        chunk_flags_after = attempt_flags;
        break;
      }
      ret = device.write_stream(stream, rd_buffs.data(), nof_chunk_samples, attempt_flags, chunk_time_ns, attempt_timeout_us);
      chunk_flags_after = attempt_flags;
      if (attempt_inside_guard && tx_summary_enabled) {
        if (ret > 0) {
          ++tx_summary.deadline_write_inside_guard_successes;
          tx_summary.deadline_write_inside_guard_recovered_samples += static_cast<uint64_t>(ret);
        } else if (ret == SOAPY_SDR_TIMEOUT) {
          ++tx_summary.deadline_write_inside_guard_timeouts;
        }
      }
      if ((ret == SOAPY_SDR_TIMEOUT) && attempt_deadline_controlled) {
        if (tx_summary_enabled) {
          if (tx_summary.deadline_write_first_timeout_logical == 0) {
            tx_summary.deadline_write_first_timeout_logical = tx_summary.logical_transmits;
          }
          ++tx_summary.deadline_write_timeouts;
        }
        if (tx_deadline_write_carry_on_timeout_enabled && tx_timeout_carry_enabled) {
          if (tx_summary_enabled) {
            ++tx_summary.deadline_write_carry_timeouts;
          }
        } else {
          deadline_write_expired = true;
        }
      }

      if ((ret != SOAPY_SDR_TIMEOUT) || tx_timeout_carry_enabled || !tx_timeout_retry_enabled ||
          (timeout_retry_attempts >= tx_timeout_retry_max_attempts)) {
        break;
      }
      timeout_retry_started = true;
      ++timeout_retry_attempts;
      ++tx_summary.timeout_retry_attempts;
    }

    if (timeout_retry_started) {
      if (ret > 0) {
        ++tx_summary.timeout_retry_successes;
        tx_summary.timeout_retry_recovered_samples += static_cast<uint64_t>(ret);
      } else {
        if (ret == SOAPY_SDR_TIMEOUT) {
          ++tx_summary.timeout_retry_exhausted;
        }
        tx_summary.timeout_retry_abandoned_samples += static_cast<uint64_t>(data_nof_samples - sent_total);
      }
    }

    record_tx_write(nof_chunk_samples, ret, requested_chunk_flags, true, is_final_chunk, false, false, false, chunk_time_ns);
    dump_tx_write_cf32("data",
                       dump_src_ptr,
                       dump_src_is_cf32,
                       nof_chunk_samples,
                       ret,
                       requested_chunk_flags,
                       chunk_flags_after,
                       chunk_time_ns,
                       static_cast<uint64_t>(tx_md.ts),
                       data_start + sent_total);

    if ((ret == SOAPY_SDR_TIMEOUT) && tx_timeout_carry_enabled && !deadline_write_expired) {
      const bool queued = queue_tx_pending_chunk(chunk_ci16_spans,
                                                nof_chunk_samples,
                                                requested_chunk_flags,
                                                is_final_chunk,
                                                chunk_time_ns,
                                                static_cast<uint64_t>(tx_md.ts),
                                                data_start + sent_total);
      if (!queued) {
        if (tx_summary_enabled && remaining > nof_chunk_samples) {
          tx_summary.timeout_carry_abandoned_samples += remaining - nof_chunk_samples;
        }
        maybe_log_tx_summary();
        return;
      }
      sent_total += nof_chunk_samples;
      ++chunks;
      carry_queue_remaining = true;
      continue;
    }

    if ((ret == SOAPY_SDR_TIMEOUT) && deadline_write_expired) {
      if (tx_summary_enabled) {
        tx_summary.deadline_write_abandoned_samples += static_cast<uint64_t>(data_nof_samples - sent_total);
        if (remaining > nof_chunk_samples) {
          tx_summary.timeout_carry_abandoned_samples += remaining - nof_chunk_samples;
        }
      }
      if (tx_deadline_break_trace_enabled && (tx_deadline_break_trace_rows < tx_deadline_break_trace_limit)) {
        const long long tx_to_break_us =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tx_start_tp)
                .count();
        logger.warning(
            "PAVONIS_SOAPY_TX_DEADLINE_BREAK_TRACE event=break row={} stream={} logical={} md_ts={} "
            "prev_md_ts={} entry_gap_us={} tx_to_break_us={} data_samples={} sent_samples={} chunk_samples={} "
            "chunk_ts={} requested_deadline_ns={} effective_deadline_ns={} hw_now_ns={} raw_lead_us={} "
            "guard_us={} guarded_lead_us={}",
            tx_deadline_break_trace_rows,
            stream_id,
            tx_summary.logical_transmits,
            tx_md.ts,
            tx_deadline_break_prev_md_ts_snapshot,
            tx_deadline_break_entry_gap_us,
            tx_to_break_us,
            data_nof_samples,
            sent_total,
            nof_chunk_samples,
            chunk_start_ts,
            tx_deadline_break_requested_deadline_ns,
            tx_deadline_break_effective_deadline_ns,
            tx_deadline_break_hw_now_ns,
            (tx_deadline_break_effective_deadline_ns - tx_deadline_break_hw_now_ns) / 1000,
            tx_deadline_write_guard_us,
            tx_deadline_break_guarded_lead_us);
        ++tx_deadline_break_trace_rows;
      }
      logger.warning("SoapySDR TX: deadline-aware write timeout for stream {} after waiting up to chunk deadline; "
                     "abandoning {} samples instead of queueing stale timed data.",
                     stream_id,
                     data_nof_samples - sent_total);
      maybe_log_tx_summary();
      return;
    }

    if (ret <= 0) {
      if (ret == SOAPY_SDR_TIMEOUT) {
        if (timeout_retry_started) {
          logger.warning("SoapySDR TX: writeStream timeout after {} retry attempts (first_timeout_us={} "
                         "retry_timeout_us={}) for stream {}; abandoning {} samples.",
                         timeout_retry_attempts,
                         write_timeout_us,
                         tx_timeout_retry_timeout_us,
                         stream_id,
                         data_nof_samples - sent_total);
        } else {
          logger.warning("SoapySDR TX: writeStream timeout after {} us for stream {}.", write_timeout_us, stream_id);
        }
      } else {
        logger.warning("SoapySDR TX: writeStream failed ret={} for stream {}.", ret, stream_id);
      }
      maybe_log_tx_summary();
      return;
    }

    if (tx_trace_writes_enabled) {
      logger.info("Soapy TX write trace: stream={} chunk={} offset={} requested={} ret={} remaining={} flags=0x{:x} "
                  "final={} ts={} tx_time_ns={}",
                  stream_id,
                  chunks,
                  sent_total,
                  nof_chunk_samples,
                  ret,
                  remaining,
                  requested_chunk_flags,
                  is_final_chunk,
                  tx_md.ts,
                  chunk_time_ns);
    }

    sent_total += static_cast<unsigned>(ret);
    ++chunks;
  }

  last_tx_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);

  if (tx_trace_enabled) {
    const long dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tx_start_tp)
                           .count();
    if (dt_us >= tx_trace_threshold_us) {
      long long hw_now_ns = 0;
      long long lead_us   = 0;
      bool      hw_ok     = false;
      if (device.get_hardware_time(hw_now_ns)) {
        lead_us = (time_ns - hw_now_ns) / 1000;
        hw_ok   = true;
      }
      logger.info("Soapy TX trace: stream={} samples={} chunks={} flags=0x{:x} ts={} empty={} dt={}us hw_now_ns={} "
                  "tx_time_ns={} lead_us={}",
                  stream_id,
                  data_nof_samples,
                  chunks,
                  flags,
                  tx_md.ts,
                  tx_md.is_empty,
                  dt_us,
                  hw_ok ? hw_now_ns : -1LL,
                  time_ns,
                  hw_ok ? lead_us : -1LL);
    }
  }
  maybe_log_tx_summary();
}

void radio_soapy_tx_stream::start()
{
  stop_control.reset();
  if (tx_skip_stream_io) {
    logger.info("PAVONIS_SOAPY_TX_SKIP_STREAM_IO enabled: not activating TX stream {}.", stream_id);
    return;
  }
  if (!device.activate_stream(stream)) {
    logger.error("Error: failed to activate TX stream {}. {}", stream_id, device.get_error_message());
    return;
  }
  report_error_if_not(async_executor.defer([this, token = stop_control.get_token()]() { run_recv_async_msg(); }),
                      "Unable to start SoapySDR TX async task");
}

void radio_soapy_tx_stream::stop()
{
  stop_control.stop();

  if (tx_timeout_carry_enabled && !tx_skip_stream_io && !tx_pending_chunks.empty()) {
    const unsigned stop_drain_stride = std::max(1U, tx_timeout_carry_drain_max_writes);
    unsigned       stop_drain_remaining = stop_drain_stride * 16U;
    while (!tx_pending_chunks.empty() && (stop_drain_remaining > 0)) {
      const unsigned drain_budget = std::min(stop_drain_stride, stop_drain_remaining);
      const unsigned writes       = drain_tx_pending_chunks(drain_budget);
      if (writes == 0) {
        break;
      }
      stop_drain_remaining -= writes;
    }
    if (!tx_pending_chunks.empty()) {
      logger.warning("SoapySDR TX: timeout-carry stop drain left {} chunks and {} samples pending on stream {}.",
                     tx_pending_chunks.size(),
                     tx_pending_samples,
                     stream_id);
    }
  }

  if (state_fsm.on_stop() && !tx_force_continuous_stream && !tx_skip_stream_io) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::END_OF_BURST,
                                .timestamp  = std::nullopt});

    // Send a zero-length end-of-burst to flush.
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> dummy_buffs = {};
    int flush_flags = SOAPY_SDR_END_BURST;
    const int requested_flush_flags = flush_flags;
    const int ret = device.write_stream(stream, dummy_buffs.data(), 0, flush_flags, 0, write_timeout_us);
    record_tx_write(0, ret, requested_flush_flags, false, false, false, false, true, 0);
  }

  log_tx_summary("stop");

  if (tx_deadline_break_trace_enabled) {
    logger.info("PAVONIS_SOAPY_TX_DEADLINE_BREAK_TRACE event=stop stream={} rows={} limit={}",
                stream_id,
                tx_deadline_break_trace_rows,
                tx_deadline_break_trace_limit);
  }

  if (tx_skip_stream_io) {
    logger.info("PAVONIS_SOAPY_TX_SKIP_STREAM_IO enabled: not deactivating TX stream {}.", stream_id);
    return;
  }

  if (!device.deactivate_stream(stream)) {
    logger.error("Error: failed to deactivate TX stream {}. {}", stream_id, device.get_error_message());
  }
}
