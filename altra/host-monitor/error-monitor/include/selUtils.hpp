/*
 * Copyright (c) 2021 Ampere Computing LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus.hpp>

#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

namespace ampere
{
namespace sel
{
using namespace phosphor::logging;

const static constexpr u_int8_t IPMI_SEL_OEM_RECORD_TYPE    = 0xC0;
const static constexpr u_int8_t SEL_OEM_DATA_MAX_SIZE       = 12;

const static constexpr char* selLogService  =
                    "xyz.openbmc_project.Logging.IPMI";
const static constexpr char* selLogPath     =
                    "/xyz/openbmc_project/Logging/IPMI";
const static constexpr char* selLogIntf     =
                    "xyz.openbmc_project.Logging.IPMI";
const static constexpr char* selLogMethod   =
                    "IpmiSelAddOem";

/* connection to sdbus */
static std::shared_ptr<sdbusplus::asio::connection> conn;

static void addSelOem( const char* message,
                       const std::vector<uint8_t> &selData)
{
    conn->async_method_call(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                log<level::ERR>("Set: Dbus error: ");
            }
        },
        selLogService,
        selLogPath,
        selLogIntf,
        selLogMethod,
        message,
        selData,
        IPMI_SEL_OEM_RECORD_TYPE);
    usleep(300000);
    return;
}

static int initSelUtil(std::shared_ptr<sdbusplus::asio::connection>& newBus)
{
    conn = newBus;

    return 1;
}

} /* namespace sel */
} /* namespace ampere */
