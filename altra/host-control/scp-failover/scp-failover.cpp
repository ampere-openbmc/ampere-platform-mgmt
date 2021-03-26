/*
 * Copyright (c) 2020 Ampere Computing LLC
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

#include <systemd/sd-journal.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <gpioplus/chip.hpp>
#include <gpioplus/handle.hpp>
#include <gpioplus/event.hpp>
#include "gpio.hpp"
#include <linux/gpio.h>
#include <nlohmann/json.hpp>
#include <platform_config.hpp>

using namespace ampere;
namespace fs = std::filesystem;
using Json = nlohmann::json;
using namespace phosphor::logging;

#define MSG_BUFFER_LENGTH 128
struct config
{
    uint8_t GpioBmcSelectEepromPin;
    uint8_t GpioS0ScpAuthFailPin;
};

config conf;
Gpio *scpAuthFailgpioPtr;
Gpio *bmcSelectgpioPtr;

static boost::asio::io_service io;
static boost::asio::posix::stream_descriptor s0ScpAuthFailEvent(io);
static std::shared_ptr<sdbusplus::asio::connection> conn;
static bool released_gpio = false;
static bool requested_gpio = true;

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

static int parsePlatformConfiguration(config &conf)
{
    char buff[MSG_BUFFER_LENGTH] = {'\0'};
    auto data = parseConfigFile(AMPERE_PLATFORM_MGMT_CONFIG_FILE);
    int8_t desc = -1;

    desc = data.value("bmc_select_eeprom", -1);
    if (desc < 0)
    {
        snprintf(buff, MSG_BUFFER_LENGTH, "bmc_select_eeprom configuration is invalid. Using default configuration!");
        log<level::WARNING>(buff);
    }
    else
    {
        conf.GpioBmcSelectEepromPin = desc;
    }

    desc = data.value("s0_scp_auth_fail_l", -1);
    if (desc < 0)
    {
        snprintf(buff, MSG_BUFFER_LENGTH, "s0_scp_auth_fail_l configuration is invalid. Using default configuration!");
        log<level::WARNING>(buff);
    }
    else
    {
        conf.GpioS0ScpAuthFailPin = desc;
    }

    return 0;
}

static void requestGPIOs(config conf)
{
    try
    {
        released_gpio = false;
        if (!requested_gpio)
            bmcSelectgpioPtr = new Gpio(conf.GpioBmcSelectEepromPin, true);
        bmcSelectgpioPtr->setValue(Gpio::HIGH);
        sleep(5);
    }
    catch (std::exception& e)
    {
        std::string message = "Cannot get the GPIOs, exit ...";
        sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                        NULL);
    }
}

static void releaseGPIOs(config conf)
{
    try
    {
        released_gpio = true;
        bmcSelectgpioPtr->GpioReleaseHandle();
        requested_gpio = false;
    }
    catch (std::exception& e)
    {
        std::string message = "Cannot release the GPIOs, exit ...";
        sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                        NULL);
    }
}

static void doForceReset()
{
    std::string command =
        "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot";
    conn->async_method_call(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "[Set] Bad D-Bus request error in doForceReset: "
                          << ec;
                return;
            }
        },
        "xyz.openbmc_project.State.Host", "/xyz/openbmc_project/state/host0",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.State.Host", "RequestedHostTransition",
        std::variant<std::string>{command});
}

static int handleSCPAuthFail(sd_event_source* /*es*/, int /*fd*/, uint32_t /*revents*/,
                        void* /*userdata*/)
{
    int i2c_backup_sel_value = -1;
    std::string message = "";
    std::stringstream stream;
    int gpioEventType = scpAuthFailgpioPtr->eventRead();
    bool authFail = gpioEventType == GPIOEVENT_REQUEST_FALLING_EDGE;
    if (authFail)
    {
        try
        {
            if (released_gpio)
            {
                requestGPIOs(conf);
            }
            // Get the GPIO values
            i2c_backup_sel_value = bmcSelectgpioPtr->getValue(true);
            printf("i2c_backup_sel_value: %d\n", i2c_backup_sel_value);

            stream << "scp auth failure signal: "
                   << (i2c_backup_sel_value ? "boot main eeprom"
                                            : "boot failover eeprom")
                   << "(" << i2c_backup_sel_value << ")" << std::endl;

            message = stream.str();
            sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i",
                            LOG_ERR, NULL);
        }
        catch (std::exception& e)
        {
            message = "Cannot get the GPIOs, exit ...";
            sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i",
                            LOG_ERR, NULL);
        }

        // 0 = failover, 1 = main
        if (!i2c_backup_sel_value)
        {
            std::string msg = "scp authentication failure detected, failover "
                              "eeprom boots fail";
            sd_journal_send(
                "MESSAGE=%s", msg.c_str(), "PRIORITY=%i", LOG_ERR,
                "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.AmpereCritical",
                "REDFISH_MESSAGE_ARGS=%s, %s", "SCP", msg.c_str(), NULL);
            releaseGPIOs(conf);
        }
        else
        {
            try
            {
                // switch to failover
                bmcSelectgpioPtr->setValue(Gpio::LOW);
                sleep(1);
                // just add systemd log, no SEL or Redfish log
                message = "scp authentication failure detected, switching to "
                          "failover eeprom";
                sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i",
                                LOG_ERR, NULL);

                // reset the host
                doForceReset();
            }
            catch (std::exception& e)
            {
                message = "auth failure detected, but action failed";
                sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i",
                                LOG_ERR, NULL);
            }
        }
    }

    return 0;
}

inline static sdbusplus::bus::match::match
    startPowerGoodMonitor(std::shared_ptr<sdbusplus::asio::connection> conn)
{
    auto startEventMatcherCallback = [](sdbusplus::message::message& msg) {
        boost::container::flat_map<std::string, std::variant<int>>
            propertiesChanged;
        std::string interfaceName;

        msg.read(interfaceName, propertiesChanged);
        if (propertiesChanged.empty())
        {
            return;
        }

        std::string event = propertiesChanged.begin()->first;
        if (event.empty())
        {
            return;
        }

        if (event == "pgood")
        {
            int value = std::get<int>(propertiesChanged.begin()->second);
            // DC power is off
            if (value == 0)
            {
                try
                {
                    // set back to boot from main
                    requestGPIOs(conf);
                    sleep(1);
                    releaseGPIOs(conf);
                }
                catch (std::exception& e)
                {}
            }
        }
    };

    sdbusplus::bus::match::match startEventMatcher(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',"
        "member='PropertiesChanged',path='/org/openbmc/control/power0',"
        "arg0namespace='org.openbmc.control.Power'",
        std::move(startEventMatcherCallback));

    return startEventMatcher;
}


int main(int argc, char* argv[])
{
    int ret = 0;
    sd_event* event = nullptr;
    int gpioEventFd;

    parsePlatformConfiguration(conf);

    scpAuthFailgpioPtr = new Gpio(conf.GpioS0ScpAuthFailPin);
    bmcSelectgpioPtr = new Gpio(conf.GpioBmcSelectEepromPin, true);

    conn = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::bus::match::match powerGoodMonitor = startPowerGoodMonitor(conn);

    requestGPIOs(conf);

    ret = sd_event_default(&event);
    if (ret < 0)
    {
        std::cout << "sd_event_default rc=" << ret << std::endl;
    }
    EventPtr eventP{event};
    event = nullptr;

    gpioEventFd = scpAuthFailgpioPtr->getEventFd();

    sd_event_source* source = nullptr;
    auto rc =
        sd_event_add_io(eventP.get(), &source, gpioEventFd,
                        EPOLLIN, handleSCPAuthFail, nullptr);
    if (rc < 0)
        std::cout << "add io rc=" << rc << std::endl;

    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    try
    {
        bus.attach_event(eventP.get(), SD_EVENT_PRIORITY_NORMAL);
        ret = sd_event_loop(eventP.get());
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error occurred during the sd_event_loop",
                phosphor::logging::entry("RET=%d", ret));
        }
    }
    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
        ret = -1;
    }

    io.run();
    return 0;
}
