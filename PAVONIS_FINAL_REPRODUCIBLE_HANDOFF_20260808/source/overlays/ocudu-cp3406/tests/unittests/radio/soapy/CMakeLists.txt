# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI

add_executable(radio_soapy_tx_deadline_test radio_soapy_tx_deadline_test.cpp)
target_include_directories(radio_soapy_tx_deadline_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/radio/soapy)
target_link_libraries(radio_soapy_tx_deadline_test GTest::gtest GTest::gtest_main)
add_test(radio_soapy_tx_deadline_test radio_soapy_tx_deadline_test)
