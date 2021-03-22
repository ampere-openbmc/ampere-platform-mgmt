/*
// Copyright 2021 Ampere Computing LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <boost/algorithm/string.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <platform_config.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>

namespace ampere
{

namespace power
{

namespace fs = std::filesystem;
using Json = nlohmann::json;

constexpr size_t minScpPowerLimit = 90;
constexpr size_t maxScpPowerLimit = 500;

static std::vector<std::string> powerCapPath =  {
    "/sys/bus/i2c/devices/2-004f/1e78a0c0.i2c-bus:smpro@4f:misc/soc_power_limit",
    "/sys/bus/i2c/devices/2-004e/1e78a0c0.i2c-bus:smpro@4e:misc/soc_power_limit"
};

/** @brief Parsing config JSON file  */
Json parseConfigFile(const std::string configFile)
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        std::cerr << "config JSON file not found" << std::endl;
        throw std::exception{};
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        std::cerr << "config readings JSON parser failure" << std::endl;
        throw std::exception{};
    }

    return data;
}

static int parsePlatformConfiguration()
{
    auto data = parseConfigFile(AMPERE_PLATFORM_MGMT_CONFIG_FILE);
    std::string desc = "";

    desc = data.value("s0_misc_path", "");
    if (desc.empty()) {
        std::cerr << "s0_misc_path configuration is invalid. Using default configuration!" << std::endl;
    }
    else {
        powerCapPath[0] =  desc + "soc_power_limit";
    }
    std::cout << "S0 Power Limit path : " << powerCapPath[0] << std::endl;

    desc = data.value("s1_misc_path", "");
    if (desc.empty()) {
        std::cerr << "s1_misc_path configuration is invalid. Using default configuration!" << std::endl;
    }
    else {
        powerCapPath[1] =  desc + "soc_power_limit";
    }
    std::cout << "S1 Power Limit path : " << powerCapPath[1] << std::endl;

    return 0;
}

static std::optional<std::string> getPowerLimitDevPath(unsigned int cpuSocket)
{
    if (cpuSocket >= powerCapPath.size())
        return std::nullopt;

    return powerCapPath[cpuSocket];
}

static uint32_t getScpPowerCap(std::string devPath)
{
    std::ifstream ifs(devPath);
    size_t scpPowerCap = 0;
    ifs >> scpPowerCap;
    ifs.close();
    return scpPowerCap;
}

static void setScpPowerCap(std::string devPath, uint32_t powerCap)
{
    if ((powerCap < minScpPowerLimit ) || (powerCap > maxScpPowerLimit))
        std::cerr << "Pwr Limit need between " << minScpPowerLimit << " and " << maxScpPowerLimit << std::endl;
    std::ofstream ofs(devPath, std::ofstream::out | std::ofstream::trunc);
    ofs << std::hex << powerCap;
    ofs.close();
}

static void setBmcPowerCap(
    std::shared_ptr<sdbusplus::asio::connection>& systemBusConnection,
    uint32_t powerCap)
{
    systemBusConnection->async_method_call(
        [](const boost::system::error_code ec) {
            if (ec)
                std::cerr << "Soc Power Limit Set: Dbus error: " << ec;
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/host0/soc_power",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Control.Power.Soc", "SocPowerLimit",
        std::variant<uint32_t>(powerCap));
}

} // namespace power
} // namespace ampere

int main(int argc, char** argv)
{
    /* Parse platform configuration file */
    ampere::power::parsePlatformConfiguration();

    // Get Power Limit dev path of CPU socket 0
    auto pwrLimitPath = ampere::power::getPowerLimitDevPath(0);
    if (pwrLimitPath == std::nullopt)
    {
        std::cerr << "Unable to get Power Limit dev" << std::endl;
        return -1;
    }

    // Initialize dbus connection
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.Ampere.SocPowerLimit");

    // Update Power Capping value from SCP to BMC settings
    uint32_t scpPowerCap = ampere::power::getScpPowerCap(pwrLimitPath.value());
    ampere::power::setBmcPowerCap(conn, scpPowerCap);

    // Handle BMC settings changed event
    sdbusplus::bus::match::match powerMatch = sdbusplus::bus::match::match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',member='PropertiesChanged',"
        "path='/xyz/openbmc_project/control/host0/soc_power'",
        [pwrLimitPath](sdbusplus::message::message& msg) {
            std::string interfaceName;
            boost::container::flat_map<std::string, std::variant<uint32_t>>
                propertiesList;
            msg.read(interfaceName, propertiesList);
            auto find = propertiesList.find("SocPowerLimit");
            if (find == propertiesList.end())
                return;
            uint32_t bmcPowerCap = std::get<uint32_t>(find->second);
            ampere::power::setScpPowerCap(pwrLimitPath.value(), bmcPowerCap);
        });

    io.run();
    return 0;
}
