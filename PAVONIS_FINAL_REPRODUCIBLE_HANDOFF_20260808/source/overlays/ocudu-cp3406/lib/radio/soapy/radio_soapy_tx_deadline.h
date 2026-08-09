// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include <algorithm>
#include <limits>

namespace ocudu {

struct radio_soapy_tx_deadline_selection {
  long      timeout_us       = -1;
  long long raw_lead_us      = 0;
  long long guarded_lead_us  = 0;
  bool      inside_guard     = false;
  bool      deadline_expired = true;
};

inline radio_soapy_tx_deadline_selection
select_radio_soapy_tx_deadline_timeout(long long effective_deadline_ns,
                                       long long hw_now_ns,
                                       long      guard_us,
                                       long      max_timeout_us,
                                       bool      allow_inside_guard_attempt)
{
  radio_soapy_tx_deadline_selection result;
  result.raw_lead_us     = (effective_deadline_ns - hw_now_ns) / 1000;
  result.guarded_lead_us = result.raw_lead_us - guard_us;

  long long write_budget_us = result.guarded_lead_us;
  if (write_budget_us <= 0) {
    if (!allow_inside_guard_attempt || result.raw_lead_us <= 0) {
      return result;
    }
    result.inside_guard = true;
    write_budget_us     = result.raw_lead_us;
  }

  const long long timeout_limit_us = max_timeout_us > 0 ? max_timeout_us : write_budget_us;
  const long long selected_us      = std::max<long long>(1, std::min(timeout_limit_us, write_budget_us));
  result.timeout_us = static_cast<long>(
      std::min<long long>(selected_us, static_cast<long long>(std::numeric_limits<long>::max())));
  result.deadline_expired = false;
  return result;
}

} // namespace ocudu
