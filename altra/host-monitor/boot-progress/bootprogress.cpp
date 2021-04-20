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

#include "bootprogress.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>

#include <boost/asio.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <nlohmann/json.hpp>
#include <platform_config.hpp>
#include <sdbusplus/asio/object_server.hpp>

namespace bootprogress
{

namespace fs = std::filesystem;
using Json = nlohmann::json;

static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

const std::string BOOT_PROGRESS_PRIMARY_PROC_INIT =
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.PrimaryProcInit";

const std::string BOOT_PROGRESS_PCI_INIT =
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.PCIInit";

const std::string BOOT_PROGRESS_SYSTEM_INIT_COMPLETE =
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.SystemInitComplete";

const std::string BOOT_PROGRESS_OS_START =
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSStart";

void setPropertyInString(std::string state)
{
    conn->async_method_call([](const boost::system::error_code ec)
    {
        if (ec)
        {
            std::cerr << " Cannot set the state:" << std::endl;
            return;
        }
    },
    "xyz.openbmc_project.State.Host",
    "/xyz/openbmc_project/state/host0",
    "org.freedesktop.DBus.Properties",
    "Set",
    "xyz.openbmc_project.State.Boot.Progress",
    "BootProgress", std::variant<std::string>(state));
}

void updateTheProgressDbus(uint32_t progress, bool isOsState)
{
    if (!isOsState)
    {
        if (progress == PRIMARY_PROCESSOR_INITIALIZATION)
        {
            setPropertyInString(BOOT_PROGRESS_PRIMARY_PROC_INIT);
        }
        else if (progress == PCI_BUS_INITIALIZATION_ENUMERATION ||
                 progress == PCI_BUS_INITIALIZATION_ASSIGN_RESOURCES)
        {
            setPropertyInString(BOOT_PROGRESS_PCI_INIT);
        }
        else if (progress == OS_READY_TO_BOOT)
        {
            setPropertyInString(BOOT_PROGRESS_SYSTEM_INIT_COMPLETE);
        }
    }
    else
    {
        setPropertyInString(BOOT_PROGRESS_OS_START);
    }
}

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
        std::cerr << "s0_misc_path configuration is invalid. Using default configuration for BOOT_PROGRESS_FS!" << std::endl;
    }
    else {
        BOOT_PROGRESS_FS = desc + "boot_progress";
    }
    std::cout << "BOOT_PROGRESS_FS : " << BOOT_PROGRESS_FS << std::endl;

    return 0;
}

/*
 * Method to read the system file
 */
static std::vector<uint32_t> readSystemFile(std::string file)
{
    FILE *boot_progress_f;

    uint32_t stage = 0xffffffff;
    uint32_t status = 0xffffffff;
    uint32_t progress = 0xffffffff;
    std::vector <uint32_t> v;

    try
    {
        // Read the boot stage
        boot_progress_f = fopen(file.c_str(), "r");
        if (fscanf(boot_progress_f, "%08x %08x %08x", &stage, &status, &progress))
            v.insert(v.end(), {stage, status, progress});
        fclose(boot_progress_f);
    }
    catch (std::exception &e)
    {
        std::cerr << "cannot read/write the boot progress filesystem" << std::endl;
    }

    return v;
}

static void handleBootProgress()
{
    bool isOSStage = false;
    std::stringstream stream;
    uint32_t bootStage = 0xffffffff;
    uint32_t bootStatus = 0xffffffff;
    uint32_t bootProgress = 0xffffffff;
    std::vector<uint32_t> registerValues;
    boost::container::flat_set<std::string> states;

    std::string bootStateStr[BOOT_STAGE_OS + 1] =
    {
        "SMpro firmware booting",
        "PMpro firmware booting",
        "ATF BL1 firmware booting",
        "DDR initialization",
        "DDR initialization progress",
        "ATF BL2 firmware booting",
        "ATF BL31 firmware booting",
        "ATF BL32 firmware booting",
        "UEFI firmware booting",
        "Os booting",
    };

    while (true)
    {
        try
        {
            registerValues = readSystemFile(BOOT_PROGRESS_FS);
            if (registerValues.empty())
            {
                std::cerr << "cannot read/write the smpro filesystem!!!" << std::endl;
                goto next;
            }

            bootStage = registerValues[0];
            bootStatus = registerValues[1];
            bootProgress = registerValues[2];
        }
        catch (std::exception &e)
        {
            std::cerr << "problem with read/write the smpro filesystem!!!" << std::endl;
            goto next;
        }

        if ((bootStage < BOOT_STAGE_SMPRO) || (bootStage > BOOT_STAGE_OS))
        {
            if (!states.empty())
            {
                states.clear();
            }
            goto next;
        }

        switch (bootStage)
        {
            case BOOT_STAGE_UEFI:
            {
                // Update the progress dbus
                updateTheProgressDbus(bootProgress, isOSStage);

                // if already hit OS stage, just wait
                if (isOSStage)
                {
                    goto next;
                }

                if (bootStatus == BOOT_STATUS_STARTED)
                {
                    for (uint32_t index = BOOT_STAGE_SMPRO; index < bootStage; index++)
                    {
                        stream.str("");
                        stream << bootStateStr[index] + " done" << std::endl;
                        std::string message = stream.str();
                        auto ret = states.insert(message);
                        if (ret.second)
                        {
                            sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                                    "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSBoot.OK",
                                    "REDFISH_MESSAGE_ARGS=%s", message.c_str(), NULL);
                        }
                        usleep(200000);
                    }
                }

                stream.str("");
                stream << bootStateStr[bootStage] << " progress 0x" << std::hex
                        << std::setw(6) << std::setfill('0') << bootProgress << std::endl;
                std::string message = stream.str();
                auto ret = states.insert(message);

                // firstly log the boot progress
                if (ret.second)
                {
                    sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                            "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSBoot.OK",
                            "REDFISH_MESSAGE_ARGS=bootState=0x%x,bootStatus=0x%x,%s",
                            bootStage, bootStatus, message.c_str(), NULL);
                }

                // log the boot failure with progress
                if (bootStatus == BOOT_STATUS_FAILURE)
                {
                    std::string message = bootStateStr[bootStage] + " failed";
                    sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                            "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSPOSTError.Warning",
                            "REDFISH_MESSAGE_ARGS=0x%x,0x%x,0x%x,%s", bootStage, bootStatus,
                            bootProgress, message.c_str(), NULL);
                }

                if (bootStatus == BOOT_STATUS_COMPLETED_OK)
                {
                    stream.str("");
                    stream << bootStateStr[bootStage] + " done" << std::endl;
                    std::string message = stream.str();

                    auto ret = states.insert(message);
                    if (ret.second)
                    {
                      sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                              "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSBoot.OK",
                              "REDFISH_MESSAGE_ARGS=bootState=0x%x,bootStatus=0x%x,%s",
                              bootStage, bootStatus, message.c_str(), NULL);
                    }

                    isOSStage = true;
                }

                break;
            }
            default:
            {
                isOSStage = false;
                // if failure then log all the info boot stage, boot status, boot progress
                if (bootStatus == BOOT_STATUS_FAILURE)
                {
                    std::string message = bootStateStr[bootStage] + " failed";
                    sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                            "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSPOSTError.Warning",
                            "REDFISH_MESSAGE_ARGS=0x%x,0x%x,0x%x,%s", bootStage, bootStatus,
                            bootProgress, message.c_str(), NULL);
                }
                else
                {
                    for (uint32_t index = BOOT_STAGE_SMPRO; index < bootStage; index++)
                    {
                        stream.str("");
                        stream << bootStateStr[index] + " done" << std::endl;
                        std::string message = stream.str();
                        auto ret = states.insert(message);
                        if (ret.second)
                        {
                            sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", LOG_ERR,
                                    "REDFISH_MESSAGE_ID=%s", "OpenBMC.0.1.BIOSBoot.OK",
                                    "REDFISH_MESSAGE_ARGS=%s", message.c_str(), NULL);
                        }
                        usleep(200000);
                    }
                }

                break;
            }
        }

next:
    usleep(200000);
    }
}

}  // namespace bootprogress

int main()
{
    bootprogress::conn = std::make_shared<sdbusplus::asio::connection>(bootprogress::io);

    bootprogress::parsePlatformConfiguration();
    bootprogress::handleBootProgress();

    bootprogress::io.run();
    return 0;
}
