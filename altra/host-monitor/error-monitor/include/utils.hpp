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

#include <platform_config.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace ampere
{
namespace utils
{
using namespace phosphor::logging;

namespace fs = std::filesystem;
static u_int8_t NUM_SOCKET                          = 2;

std::string hwmonRootDir[2]     = {
        "/sys/bus/platform/devices/smpro-misc.2.auto",
        "/sys/bus/platform/devices/smpro-misc.5.auto"
        };

static std::string getAbsolutePath(u_int8_t socket, std::string fileName)
{
    if (hwmonRootDir[socket] != "")
    {
        return hwmonRootDir[socket] + fileName;
    }

    return "";
}

/** @brief Parsing config JSON file  */
Json parseConfigFile(const std::string configFile)
{
    std::ifstream jsonFile(configFile);

    if (!jsonFile.is_open())
    {
        log<level::ERR>("config JSON file not found",
                        entry("FILENAME = %s", configFile.c_str()));
        throw std::exception{};
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("config readings JSON parser failure",
                        entry("FILENAME = %s", configFile.c_str()));
        throw std::exception{};
    }

    return data;
}

static int parsePlatformConfiguration()
{
    const static u_int8_t MSG_BUFFER_LENGTH   = 128;
    char buff[MSG_BUFFER_LENGTH] = {'\0'};
    auto data = parseConfigFile(AMPERE_PLATFORM_MGMT_CONFIG_FILE);
    std::string desc = "";
    int num = 0;

    num = data.value("number_socket", -1);
    if (num < 1)
    {
        log<level::WARNING>("number_socket configuration is"\
                            "invalid. Using default configuration!");
    }
    else
    {
        NUM_SOCKET = num;
    }

    desc = data.value("s0_errmon_path", "");
    if (desc.empty())
    {
        log<level::WARNING>("s0_errmon_path configuration is invalid."\
                            " Using default configuration!");
    }
    else
    {
        hwmonRootDir[0] = desc;
    }
    snprintf(buff, MSG_BUFFER_LENGTH, "S0 SMPro errmon path: %s\n",
             hwmonRootDir[0].c_str());
    log<level::INFO>(buff);

    desc = data.value("s1_errmon_path", "");
    if (desc.empty())
    {
        log<level::WARNING>("s1_errmon_path configuration is invalid."\
                            "Using default configuration!");
    }
    else
    {
        hwmonRootDir[1] = desc;
    }
    snprintf(buff, MSG_BUFFER_LENGTH, "S1 SMPro errmon path: %s\n",
             hwmonRootDir[1].c_str());
    log<level::INFO>(buff);

    return 0;
}

static int initHwmonRootPath()
{
    bool foundRootPath = false;

    /* parse errmon patch */
    parsePlatformConfiguration();

    for (u_int8_t socket=0; socket < NUM_SOCKET; socket++)
    {
        auto path = fs::path(hwmonRootDir[socket]);
        if (fs::exists(path) && fs::is_directory(path))
        {
            path = fs::path(hwmonRootDir[socket] + "/error_core_ce");
            if (fs::exists(path))
            {
                foundRootPath = true;
                continue;
            }
        }
        hwmonRootDir[socket] = "";
    }
    if (foundRootPath)
    {
        return 1;
    }

    return 0;
}

static u_int64_t parseHexStrToUInt64(std::string str)
{
    char* p;

    u_int64_t n = strtoull(str.c_str(), &p, 16);
    if (*p != 0)
    {
        return 0;
    }

    return n & 0xffffffffffffffff;
}

static u_int32_t parseHexStrToUInt32(std::string str)
{
    char* p;

    long n = strtoul(str.c_str(), &p, 16);
    if (*p != 0)
    {
        return 0;
    }

    return n & 0xffffffff;
}

static u_int16_t parseHexStrToUInt16(std::string str)
{
    char* p;

    long n = strtoul(str.c_str(), &p, 16);
    if (*p != 0)
    {
        return 0;
    }

    return n & 0xffff;
}

static u_int8_t parseHexStrToUInt8(std::string str)
{
    char* p;

    long n = strtoul(str.c_str(), &p, 16);
    if (*p != 0)
    {
        return 0;
    }

    return n & 0xff;
}

static void swap2Byte(std::string& str)
{
    int i, j;
    int len = str.size();

    for (i = 0, j = 1; j < len;)
    {
        std::swap(str[i], str[j]);
        i += 2;
        j += 2;
    }
}

static void reverseStr(std::string& str)
{
    swap2Byte(str);
    int len = str.size();
    for (int i = 0; i < (len/2); i++)
    {
        std::swap(str[i], str[(len-i-1)]);
    }
}

} /* namespace utils */
} /* namespace ampere */
