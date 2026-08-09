// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_tx_deadline.h"
#include <gtest/gtest.h>

using namespace ocudu;

TEST(radio_soapy_tx_deadline, disabled_path_preserves_guard_expiry)
{
  const auto result = select_radio_soapy_tx_deadline_timeout(1048000, 1000000, 100, 50000, false);

  ASSERT_TRUE(result.deadline_expired);
  ASSERT_FALSE(result.inside_guard);
  ASSERT_EQ(result.raw_lead_us, 48);
  ASSERT_EQ(result.guarded_lead_us, -52);
  ASSERT_EQ(result.timeout_us, -1);
}

TEST(radio_soapy_tx_deadline, enabled_path_attempts_inside_guard)
{
  const auto result = select_radio_soapy_tx_deadline_timeout(1091000, 1000000, 100, 50000, true);

  ASSERT_FALSE(result.deadline_expired);
  ASSERT_TRUE(result.inside_guard);
  ASSERT_EQ(result.raw_lead_us, 91);
  ASSERT_EQ(result.guarded_lead_us, -9);
  ASSERT_EQ(result.timeout_us, 91);
}

TEST(radio_soapy_tx_deadline, enabled_path_keeps_expired_deadline_closed)
{
  const auto at_deadline = select_radio_soapy_tx_deadline_timeout(1000000, 1000000, 100, 50000, true);
  const auto past_deadline = select_radio_soapy_tx_deadline_timeout(999000, 1000000, 100, 50000, true);

  ASSERT_TRUE(at_deadline.deadline_expired);
  ASSERT_EQ(at_deadline.timeout_us, -1);
  ASSERT_TRUE(past_deadline.deadline_expired);
  ASSERT_EQ(past_deadline.timeout_us, -1);
}

TEST(radio_soapy_tx_deadline, normal_guarded_path_is_unchanged)
{
  const auto disabled = select_radio_soapy_tx_deadline_timeout(1198000, 1000000, 100, 50000, false);
  const auto enabled  = select_radio_soapy_tx_deadline_timeout(1198000, 1000000, 100, 50000, true);

  ASSERT_FALSE(disabled.deadline_expired);
  ASSERT_FALSE(enabled.deadline_expired);
  ASSERT_FALSE(disabled.inside_guard);
  ASSERT_FALSE(enabled.inside_guard);
  ASSERT_EQ(disabled.timeout_us, 98);
  ASSERT_EQ(enabled.timeout_us, disabled.timeout_us);
}

TEST(radio_soapy_tx_deadline, maximum_timeout_still_caps_write_budget)
{
  const auto result = select_radio_soapy_tx_deadline_timeout(3000000, 1000000, 100, 500, true);

  ASSERT_FALSE(result.deadline_expired);
  ASSERT_FALSE(result.inside_guard);
  ASSERT_EQ(result.timeout_us, 500);
}
