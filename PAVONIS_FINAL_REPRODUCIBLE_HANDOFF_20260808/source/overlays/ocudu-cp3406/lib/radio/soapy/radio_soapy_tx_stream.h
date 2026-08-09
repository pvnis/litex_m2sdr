// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "ocudu/adt/complex.h"
#include "radio_soapy_device.h"
#include "radio_soapy_exception_handler.h"
#include "radio_soapy_tx_stream_fsm.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_dynamic.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "ocudu/radio/radio_configuration.h"
#include "ocudu/radio/radio_event_notifier.h"
#include "ocudu/support/executors/task_executor.h"
#include "ocudu/support/synchronization/stop_event.h"
#include <SoapySDR/Device.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <string_view>
#include <vector>

namespace ocudu {

/// Implements baseband_gateway_transmitter using SoapySDR zero-copy write buffers.
class radio_soapy_tx_stream : public baseband_gateway_transmitter, public soapy_exception_handler
{
  unsigned              stream_id;
  task_executor&        async_executor;
  radio_event_notifier& notifier;
  radio_soapy_device&   device;
  SoapySDR::Stream*     stream = nullptr;
  size_t                mtu    = 0;
  double                srate_hz;
  unsigned              nof_channels;
  bool                  discontinuous_tx;
  unsigned              power_ramping_nof_samples = 0;
  long long             last_tx_time_ns           = 0;
  long                  write_timeout_us          = 200;
  /// Pre-zeroed power ramping buffer (CI16 samples per channel).
  baseband_gateway_buffer_dynamic power_ramping_buffer;
  /// Pre-zeroed CF32 power ramping buffer used only by the env-gated CF32 TX path.
  std::vector<cf_t>                power_ramping_cf32_buffer;
  /// Temporary conversion buffer used only by the env-gated CF32 TX path.
  std::vector<cf_t>                tx_cf32_conversion_buffer;
  radio_soapy_tx_stream_fsm       state_fsm;
  rt_stop_event_source            stop_control;
  ocudulog::basic_logger&         logger;
  bool                            tx_trace_enabled        = false;
  bool                            tx_trace_writes_enabled = false;
  bool                            tx_summary_enabled      = false;
  bool                            tx_suppress_empty_eob   = false;
  bool                            tx_force_continuous_stream = false;
  bool                            tx_skip_stream_io        = false;
  bool                            tx_skip_stream_setup     = false;
  bool                            tx_skip_setupstream_only = false;
  uint64_t                        tx_reanchor_interval_samples = 0;
  bool                            tx_all_timed_chunks      = false;
  bool                            tx_timeout_retry_enabled = true;
  unsigned                        tx_timeout_retry_max_attempts = 64;
  long                            tx_timeout_retry_timeout_us = 1000;
  bool                            tx_timeout_carry_enabled = true;
  uint64_t                        tx_timeout_carry_max_samples = 1152000;
  unsigned                        tx_timeout_carry_drain_max_writes = 64;
  long                            tx_timeout_carry_write_timeout_us = 0;
  bool                            tx_timeout_carry_deadline_enabled = true;
  bool                            tx_timeout_carry_deadline_drop_enabled = false;
  uint64_t                        tx_timeout_carry_max_lag_samples = 0;
  long                            tx_timeout_carry_deadline_guard_us = 2000;
  long                            tx_timeout_carry_deadline_max_write_timeout_us = 1000;
  bool                            tx_deadline_write_timeout_enabled = false;
  long                            tx_deadline_write_guard_us = 2000;
  long                            tx_deadline_write_max_timeout_us = 50000;
  bool                            tx_deadline_write_inside_guard_enabled = false;
  long long                       tx_deadline_write_time_offset_ns = 0;
  bool                            tx_deadline_write_carry_on_timeout_enabled = false;
  long                            tx_deadline_write_direct_poll_us = 0;
  bool                            tx_deadline_break_trace_enabled          = false;
  unsigned                        tx_deadline_break_trace_limit            = 64;
  unsigned                        tx_deadline_break_trace_rows             = 0;
  bool                            tx_deadline_break_prev_start_valid       = false;
  std::chrono::steady_clock::time_point tx_deadline_break_prev_start_tp;
  baseband_gateway_timestamp      tx_deadline_break_prev_md_ts             = 0;
  long long                       tx_deadline_break_hw_now_ns               = 0;
  long long                       tx_deadline_break_requested_deadline_ns  = 0;
  long long                       tx_deadline_break_effective_deadline_ns  = 0;
  long long                       tx_deadline_break_guarded_lead_us         = 0;
  bool                            tx_reanchor_origin_selected = false;
  uint64_t                        tx_reanchor_origin_ts = 0;
  uint64_t                        tx_reanchor_next_ts = 0;
  long                            tx_trace_threshold_us   = 100;
  unsigned                        tx_summary_period       = 10000;
  unsigned                        tx_force_chunk_samples  = 0;
  bool                            tx_cf32_format          = false;
  const char*                     tx_write_dump_cf32_path = nullptr;
  const char*                     tx_write_dump_meta_path = nullptr;
  uint64_t                        tx_write_dump_max_samples = 0;
  uint64_t                        tx_write_dump_samples     = 0;
  uint64_t                        tx_write_dump_rows        = 0;
  unsigned                        tx_write_dump_port        = 0;
  std::vector<cf_t>               tx_write_dump_cf32_buffer;
  bool                            tx_marker_enabled         = false;
  uint64_t                        tx_marker_configured_start_ts = 0;
  uint64_t                        tx_marker_start_ts        = 0;
  uint64_t                        tx_marker_delay_samples   = 0;
  bool                            tx_marker_dynamic_start   = false;
  bool                            tx_marker_start_selected  = false;
  bool                            tx_marker_sequence_mode   = false;
  unsigned                        tx_marker_len             = 4096;
  int                             tx_marker_amp             = 1024;
  unsigned                        tx_marker_port            = 0;
  uint32_t                        tx_marker_seed            = 1647;
  const char*                     tx_marker_meta_path       = nullptr;
  uint64_t                        tx_marker_rows            = 0;
  std::vector<ci16_t>             tx_marker_ci16_buffer;

  struct tx_pending_chunk {
    std::vector<ci16_t> samples;
    unsigned            nof_samples       = 0;
    int                 flags             = 0;
    bool                final_data_chunk  = false;
    long long           time_ns           = 0;
    uint64_t            logical_ts        = 0;
    unsigned            logical_offset    = 0;
  };
  std::deque<tx_pending_chunk> tx_pending_chunks;
  uint64_t                     tx_pending_samples = 0;
  std::vector<cf_t>            tx_pending_cf32_conversion_buffer;

  struct tx_marker_event {
    uint64_t start_ts      = 0;
    uint64_t delay_samples = 0;
    bool     dynamic_start = false;
    bool     start_selected = false;
    uint32_t seed          = 0;
  };
  std::vector<tx_marker_event> tx_marker_events;

  struct tx_write_summary_stats {
    uint64_t logical_transmits       = 0;
    uint64_t data_logical_transmits  = 0;
    uint64_t empty_logical_transmits = 0;
    uint64_t logical_has_time        = 0;
    uint64_t logical_eob             = 0;
    uint64_t data_samples            = 0;
    uint64_t stream_samples          = 0;
    uint64_t write_calls             = 0;
    uint64_t data_write_calls        = 0;
    uint64_t power_ramp_write_calls  = 0;
    uint64_t empty_eob_write_calls   = 0;
    uint64_t suppressed_empty_eob    = 0;
    uint64_t stop_flush_write_calls  = 0;
    uint64_t requested_samples       = 0;
    uint64_t returned_samples        = 0;
    uint64_t data_requested_samples  = 0;
    uint64_t data_returned_samples   = 0;
    uint64_t return_observations     = 0;
    uint64_t min_requested           = 0;
    uint64_t max_requested           = 0;
    uint64_t min_returned            = 0;
    uint64_t max_returned            = 0;
    uint64_t zero_requested_writes   = 0;
    uint64_t zero_return_writes      = 0;
    uint64_t has_time_writes         = 0;
    uint64_t reanchor_writes         = 0;
    uint64_t all_timed_writes        = 0;
    uint64_t has_time_positive_ns    = 0;
    uint64_t has_time_zero_ns        = 0;
    uint64_t has_time_negative_ns    = 0;
    long long has_time_min_ns        = 0;
    long long has_time_max_ns        = 0;
    uint64_t eob_writes              = 0;
    uint64_t both_flag_writes        = 0;
    uint64_t zero_flag_writes        = 0;
    uint64_t final_data_chunks       = 0;
    uint64_t nonfinal_data_chunks    = 0;
    uint64_t partial_return_writes   = 0;
    uint64_t over_return_writes      = 0;
    uint64_t timeout_writes          = 0;
    uint64_t error_writes            = 0;
    uint64_t timeout_retry_attempts  = 0;
    uint64_t timeout_retry_successes = 0;
    uint64_t timeout_retry_exhausted = 0;
    uint64_t timeout_retry_recovered_samples = 0;
    uint64_t timeout_retry_abandoned_samples = 0;
    uint64_t timeout_carry_queued_chunks = 0;
    uint64_t timeout_carry_queued_samples = 0;
    uint64_t timeout_carry_drained_chunks = 0;
    uint64_t timeout_carry_drained_samples = 0;
    uint64_t timeout_carry_drain_writes = 0;
    uint64_t timeout_carry_drain_timeouts = 0;
    uint64_t timeout_carry_drain_errors = 0;
    uint64_t timeout_carry_partial_drains = 0;
    uint64_t timeout_carry_overflow_chunks = 0;
    uint64_t timeout_carry_overflow_samples = 0;
    uint64_t timeout_carry_abandoned_samples = 0;
    uint64_t timeout_carry_max_pending_samples = 0;
    uint64_t timeout_carry_first_queue_logical = 0;
    uint64_t timeout_carry_first_overflow_logical = 0;
    uint64_t timeout_carry_first_lag_logical = 0;
    uint64_t timeout_carry_deadline_limited_writes = 0;
    uint64_t timeout_carry_deadline_breaks = 0;
    uint64_t timeout_carry_deadline_hwtime_failures = 0;
    uint64_t timeout_carry_deadline_lag_events = 0;
    uint64_t timeout_carry_deadline_max_observed_lag_samples = 0;
    uint64_t timeout_carry_deadline_would_drop_samples = 0;
    uint64_t timeout_carry_deadline_dropped_chunks = 0;
    uint64_t timeout_carry_deadline_partial_drops = 0;
    uint64_t timeout_carry_deadline_dropped_samples = 0;
    uint64_t deadline_write_limited_writes = 0;
    uint64_t deadline_write_breaks = 0;
    uint64_t deadline_write_hwtime_failures = 0;
    uint64_t deadline_write_timeouts = 0;
    uint64_t deadline_write_carry_timeouts = 0;
    uint64_t deadline_write_abandoned_samples = 0;
    uint64_t deadline_write_max_selected_timeout_us = 0;
    uint64_t deadline_write_first_timeout_logical = 0;
    uint64_t deadline_write_inside_guard_attempts = 0;
    uint64_t deadline_write_inside_guard_successes = 0;
    uint64_t deadline_write_inside_guard_timeouts = 0;
    uint64_t deadline_write_inside_guard_recovered_samples = 0;
    uint64_t last_report_logical     = 0;
  } tx_summary;

  void recv_async_msg();
  void run_recv_async_msg();
  void record_tx_logical(bool is_empty, unsigned nof_samples, int flags);
  bool queue_tx_pending_chunk(const std::array<span<const ci16_t>, RADIO_MAX_NOF_CHANNELS>& src,
                              unsigned nof_samples,
                              int flags,
                              bool final_data_chunk,
                              long long time_ns,
                              uint64_t logical_ts,
                              unsigned logical_offset);
  long     select_tx_pending_write_timeout(long long deadline_time_ns);
  long     select_tx_data_write_timeout(long long deadline_time_ns,
                                        bool&     deadline_controlled,
                                        bool&     inside_guard_attempt);
  uint64_t trim_tx_pending_for_lag(uint64_t current_start_ts);
  unsigned drain_tx_pending_chunks(unsigned max_writes, long long deadline_time_ns = 0);
  void trim_tx_pending_front(unsigned accepted_samples);
  void record_tx_write(unsigned requested,
                       int      ret,
                       int      flags,
                       bool     data_write,
                       bool     final_data_chunk,
                       bool     power_ramp_write,
                       bool     empty_eob_write,
                       bool     stop_flush_write,
                       long long time_ns);
  void log_tx_summary(std::string_view reason);
  void maybe_log_tx_summary();
  void init_tx_write_dump();
  void init_tx_marker();
  void sync_primary_tx_marker_fields();
  void select_dynamic_tx_markers(uint64_t chunk_start_ts, long long chunk_time_ns, unsigned logical_offset);
  span<const ci16_t> maybe_apply_tx_marker(span<const ci16_t> src,
                                           uint64_t           chunk_start_ts,
                                           unsigned           channel,
                                           long long          chunk_time_ns,
                                           unsigned           logical_offset);
  void write_tx_marker_meta(unsigned marker_index,
                            const tx_marker_event& marker,
                            unsigned channel,
                            uint64_t chunk_start_ts,
                            long long chunk_time_ns,
                            unsigned logical_offset,
                            uint64_t overlay_start_ts,
                            uint64_t overlay_end_ts);
  void write_tx_marker_selection_meta(unsigned marker_index,
                                      const tx_marker_event& marker,
                                      uint64_t chunk_start_ts,
                                      long long chunk_time_ns,
                                      unsigned logical_offset);
  void dump_tx_write_cf32(std::string_view kind,
                          const void*      src,
                          bool             src_is_cf32,
                          unsigned         requested,
                          int              ret,
                          int              flags_before,
                          int              flags_after,
                          long long        time_ns,
                          uint64_t         logical_ts,
                          unsigned         logical_offset);

public:
  struct stream_description {
    unsigned   id;
    double     srate_hz;
    unsigned   nof_channels;
    bool       discontinuous_tx;
    float      power_ramping_us;
  };

  radio_soapy_tx_stream(radio_soapy_device&       device_,
                        SoapySDR::Stream*          stream_,
                        const stream_description&  desc,
                        task_executor&             async_executor_,
                        radio_event_notifier&      notifier_);

  unsigned get_buffer_size() const { return static_cast<unsigned>(mtu); }

  // See interface for documentation.
  void transmit(const baseband_gateway_buffer_reader&        data,
                const baseband_gateway_transmitter_metadata& metadata) override;

  void start();
  void stop();
};

} // namespace ocudu
