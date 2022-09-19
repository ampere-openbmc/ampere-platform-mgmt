/*
 * Copyright (c) 2021 Ampere Computing LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "internalErrors.hpp"
#include "utils.hpp"
#include "selUtils.hpp"
#include <math.h>

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/timer.hpp>
#include <sdbusplus/asio/sd_event.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>

using namespace phosphor::logging;

namespace ampere
{
namespace ras
{
const static constexpr u_int16_t MAX_MSG_LEN    = 128;
const static constexpr u_int8_t TYPE_TEMP       = 0x03;
const static constexpr u_int8_t TYPE_STATE      = 0x05;
const static constexpr u_int8_t TYPE_OTHER      = 0x12;
const static constexpr u_int8_t TYPE_MEM        = 0x0C;
const static constexpr u_int8_t TYPE_CORE       = 0x07;
const static constexpr u_int8_t TYPE_PCIE       = 0x13;
const static constexpr u_int8_t TYPE_SMPM       = 0xCA;
const static constexpr u_int8_t CE_CORE_IERR    = 139;
const static constexpr u_int8_t UE_CORE_IERR    = 140;
const static constexpr u_int8_t CE_OTHER_IERR   = 141;
const static constexpr u_int8_t UE_OTHER_IERR   = 142;
const static constexpr u_int8_t CE_MEM_IERR     = 151;
const static constexpr u_int8_t UE_MEM_IERR     = 168;
const static constexpr u_int8_t CE_PCIE_IERR    = 191;
const static constexpr u_int8_t UE_PCIE_IERR    = 202;
const static constexpr u_int8_t SMPRO_IERR     = 147;
const static constexpr u_int8_t PMPRO_IERR     = 148;
/* Event ID */
const static constexpr u_int8_t S0_DIMM_HOT         = 160;
const static constexpr u_int8_t S0_VRD_HOT          = 180;
const static constexpr u_int8_t S0_VRD_WARN_FAULT   = 181;
const static constexpr u_int8_t S0_DIMM_2X_REFRESSH = 162;
const static constexpr u_int8_t S1_DIMM_HOT         = 161;
const static constexpr u_int8_t S1_VRD_HOT          = 183;
const static constexpr u_int8_t S1_VRD_WARN_FAULT   = 184;
const static constexpr u_int8_t S1_DIMM_2X_REFRESSH = 163;
/* Direction of RAS Internal errors */
const static constexpr u_int8_t  DIR_ENTER      = 0;
const static constexpr u_int8_t  DIR_EXIT       = 1;
const static constexpr u_int8_t  DIR_ASSERTED   = 0;
const static constexpr u_int8_t  DIR_DEASSERTED = 1;
/* Sub types of RAS Internal errors */
const static constexpr u_int8_t  SMPMPRO_WARNING       = 1;
const static constexpr u_int8_t  SMPMPRO_ERROR         = 2;
const static constexpr u_int8_t  SMPMPRO_ERROR_DATA    = 4;
/* Type of RAS Internal errors */
const static constexpr u_int8_t  SMPRO_IERR_TYPE        = 0;
const static constexpr u_int8_t  PMPRO_IERR_TYPE        = 1;

const static constexpr u_int8_t IERR_SENSOR_SPECIFIC     = 0x71;
const static constexpr u_int8_t TEMP_READ_TYPE          = 0x5;
const static constexpr u_int8_t STATUS_READ_TYPE        = 0x3;

const static constexpr u_int8_t EVENT_DATA_1            = 0x80;
const static constexpr u_int8_t EVENT_DATA_3            = 0x20;

const static constexpr u_int8_t SOC_COMPONENT           = 0x00;
const static constexpr u_int8_t CORE_COMPONENT          = 0x01;
const static constexpr u_int8_t DIMM_COMPONENT          = 0x02;

const static constexpr u_int8_t VRD_1                   = 0x01;
const static constexpr u_int8_t VRD_2                   = 0x02;
const static constexpr u_int8_t VRD_3                   = 0x03;
const static constexpr u_int8_t VRD_4                   = 0x04;

const static constexpr u_int16_t BIT_0                  = 0x0001;
const static constexpr u_int16_t BIT_1                  = 0x0002;
const static constexpr u_int16_t BIT_2                  = 0x0004;
const static constexpr u_int16_t BIT_3                  = 0x0008;
const static constexpr u_int16_t BIT_4                  = 0x0010;
const static constexpr u_int16_t BIT_5                  = 0x0020;
const static constexpr u_int16_t BIT_6                  = 0x0040;
const static constexpr u_int16_t BIT_7                  = 0x0080;
const static constexpr u_int16_t BIT_8                  = 0x0100;
const static constexpr u_int16_t BIT_9                  = 0x0200;
const static constexpr u_int16_t BIT_10                 = 0x0400;
const static constexpr u_int16_t BIT_11                 = 0x0800;

const static constexpr u_int16_t SMPRO_DATA_REG_SIZE    = 16;
const static constexpr u_int8_t AMPERE_IANA_BYTE_1      = 0x3A;
const static constexpr u_int8_t AMPERE_IANA_BYTE_2      = 0xCD;
const static constexpr u_int8_t AMPERE_IANA_BYTE_3      = 0x00;

const static constexpr u_int16_t NUMBER_DIMM_CHANNEL    = 8;

const static constexpr char* AMPERE_REFISH_REGISTRY = "AmpereCritical";

const static constexpr char* createFlagCmd = "touch /tmp/fault_RAS_UE";
const static constexpr char* rmFlagCmd = "rm /tmp/fault_RAS_UE";
const static constexpr char* RASUEFlagPath = "/tmp/fault_RAS_UE";

struct ErrorFields {
    u_int8_t errType;
    u_int8_t subType;
    u_int16_t instance;
    u_int32_t status;
    u_int64_t address;
    u_int64_t misc0;
    u_int64_t misc1;
    u_int64_t misc2;
    u_int64_t misc3;
};

struct InternalFields {
    u_int8_t errType;
    u_int8_t subType;
    u_int8_t imageCode;
    u_int8_t dir;
    u_int8_t location;
    u_int16_t errCode;
    u_int32_t data;
};

struct ErrorData {
    u_int16_t socket;
    u_int8_t intErrorType;
    const char* label;
    u_int8_t errType;
    u_int8_t errNum;
    const char* errName;
    const char* redFishMsgID;
};
/* Error type index of RAS Errors */
enum ErrorTypes{
    errors_core_ue,
    errors_mem_ue,
    errors_pcie_ue,
    errors_other_ue,
    errors_core_ce,
    errors_mem_ce,
    errors_pcie_ce,
    errors_other_ce,
    errors_smpro,
    errors_pmpro
};

ErrorData errorTypeTable[] = {
    {0, errors_core_ue, "errors_core_ue", TYPE_CORE, UE_CORE_IERR,
        "UE_CPU_IError", "CPUError"},
    {0, errors_mem_ue, "errors_mem_ue", TYPE_MEM, UE_MEM_IERR,
        "UE_Memory_IErr", "MemoryECCUncorrectable"},
    {0, errors_pcie_ue, "errors_pcie_ue", TYPE_PCIE, UE_PCIE_IERR,
        "UE_PCIE_IErr", "PCIeFatalUncorrectableInternal"},
    {0, errors_other_ue, "errors_other_ue", TYPE_OTHER, UE_OTHER_IERR,
        "UE_SoC_IErr", "AmpereCritical"},
    {1, errors_core_ue, "errors_core_ue", TYPE_CORE, UE_CORE_IERR,
        "UE_CPU_IError", "CPUError"},
    {1, errors_mem_ue, "errors_mem_ue", TYPE_MEM, UE_MEM_IERR,
        "UE_Memory_IErr", "MemoryECCUncorrectable"},
    {1, errors_pcie_ue, "errors_pcie_ue", TYPE_PCIE, UE_PCIE_IERR,
        "UE_PCIE_IErr", "PCIeFatalUncorrectableInternal"},
    {1, errors_other_ue, "errors_other_ue", TYPE_OTHER, UE_OTHER_IERR,
        "UE_SoC_IErr", "AmpereCritical"},
    {0, errors_core_ce, "errors_core_ce", TYPE_CORE, CE_CORE_IERR,
        "CE_CPU_IError", "CPUError"},
    {0, errors_mem_ce, "errors_mem_ce", TYPE_MEM, CE_MEM_IERR,
        "CE_Memory_IErr", "MemoryECCCorrectable"},
    {0, errors_pcie_ce, "errors_pcie_ce", TYPE_PCIE, CE_PCIE_IERR,
        "CE_PCIE_IErr", "PCIeFatalECRCError"},
    {0, errors_other_ce, "errors_other_ce", TYPE_OTHER, CE_OTHER_IERR,
        "CE_SoC_IErr", "AmpereCritical"},
    {1, errors_core_ce, "errors_core_ce", TYPE_CORE, CE_CORE_IERR,
        "CE_CPU_IError", "CPUError"},
    {1, errors_mem_ce, "errors_mem_ce", TYPE_MEM, CE_MEM_IERR,
        "CE_Memory_IErr", "MemoryECCCorrectable"},
    {1, errors_pcie_ce, "errors_pcie_ce", TYPE_PCIE, CE_PCIE_IERR,
        "CE_PCIE_IErr", "PCIeFatalECRCError"},
    {1, errors_other_ce, "errors_other_ce", TYPE_OTHER, CE_OTHER_IERR,
        "CE_SoC_IErr", "AmpereCritical"},
    {0, errors_smpro, "errors_smpro", TYPE_SMPM, SMPRO_IERR,
        "SMPRO_IErr", "AmpereCritical"},
    {0, errors_pmpro, "errors_pmpro", TYPE_SMPM, PMPRO_IERR,
        "PMPRO_IErr", "AmpereCritical"},
    {1, errors_smpro, "errors_smpro", TYPE_SMPM, SMPRO_IERR,
        "SMPRO_IErr", "AmpereCritical"},
    {1, errors_pmpro, "errors_pmpro", TYPE_SMPM, PMPRO_IERR,
        "PMPRO_IErr", "AmpereCritical"},
};

const static constexpr u_int8_t NUMBER_OF_ERRORS    =
        sizeof(errorTypeTable) / sizeof(ErrorData);

struct ErrorInfo {
    u_int8_t errType;
    u_int8_t subType;
    u_int8_t numPars;
    const char* errName;
    const char* errMsgFormat;
};

std::map<u_int16_t, ErrorInfo> mapOfOccur = {
    {0x0000, {0, 0, 2, "CPM Snoop-Logic", "Socket%s CPM%s"}},
    {0x0001, {0, 1, 2, "CPM Core 0", "Socket%s CPM%s"}},
    {0x0002, {0, 2, 2, "CPM Core 1", "Socket%s CPM%s"}},
    {0x0101, {1, 1, 2, "MCU ERR Record 1 (DRAM CE)", "Socket%s MCU%s"}},
    {0x0102, {1, 2, 2, "MCU ERR Record 2 (DRAM UE)", "Socket%s MCU%s"}},
    {0x0103, {1, 3, 2, "MCU ERR Record 3 (CHI Fault)", "Socket%s MCU%s"}},
    {0x0104, {1, 4, 2, "MCU ERR Record 4 (SRAM CE)", "Socket%s MCU%s"}},
    {0x0105, {1, 5, 2, "MCU ERR 5 (SRAM UE)", "Socket%s MCU%s"}},
    {0x0106, {1, 6, 2, "MCU ERR 6 (DMC recovery)", "Socket%s MCU%s"}},
    {0x0107, {1, 7, 2, "MCU Link ERR", "Socket%s MCU%s"}},
    {0x0200, {2, 0, 2, "Mesh XP", "Socket%s instance:%s"}},
    {0x0201, {2, 1, 2, "Mesh HNI", "Socket%s instance:%s"}},
    {0x0202, {2, 2, 2, "Mesh HNF", "Socket%s instance:%s"}},
    {0x0204, {2, 4, 2, "Mesh CXG", "Socket%s instance:%s"}},
    {0x0300, {3, 0, 2, "2P AER ERR", "Socket%s Link%s"}},
    {0x0400, {4, 0, 2, "2P ALI ERR", "Socket%s Link%s"}},
    {0x0500, {5, 0, 1, "GIC ERR 0", "Socket%s"}},
    {0x0501, {5, 1, 1, "GIC ERR 1", "Socket%s"}},
    {0x0502, {5, 2, 1, "GIC ERR 2", "Socket%s"}},
    {0x0503, {5, 3, 1, "GIC ERR 3", "Socket%s"}},
    {0x0504, {5, 4, 1, "GIC ERR 4", "Socket%s"}},
    {0x0505, {5, 5, 1, "GIC ERR 5", "Socket%s"}},
    {0x0506, {5, 6, 1, "GIC ERR 6", "Socket%s"}},
    {0x0507, {5, 7, 1, "GIC ERR 7", "Socket%s"}},
    {0x0508, {5, 8, 1, "GIC ERR 8", "Socket%s"}},
    {0x0509, {5, 9, 1, "GIC ERR 9", "Socket%s"}},
    {0x050a, {5, 10, 1, "GIC ERR 10", "Socket%s"}},
    {0x050b, {5, 11, 1, "GIC ERR 11", "Socket%s"}},
    {0x050c, {5, 12, 1, "GIC ERR 12", "Socket%s"}},
    {0x0600, {6, 0, 2, "SMMU TBU0", "Socket%s Root complex:%s"}},
    {0x0601, {6, 1, 2, "SMMU TBU1", "Socket%s Root complex:%s"}},
    {0x0602, {6, 2, 2, "SMMU TBU2", "Socket%s Root complex:%s"}},
    {0x0603, {6, 3, 2, "SMMU TBU3", "Socket%s Root complex:%s"}},
    {0x0604, {6, 4, 2, "SMMU TBU4", "Socket%s Root complex:%s"}},
    {0x0605, {6, 5, 2, "SMMU TBU5", "Socket%s Root complex:%s"}},
    {0x0606, {6, 6, 2, "SMMU TBU6", "Socket%s Root complex:%s"}},
    {0x0607, {6, 7, 2, "SMMU TBU7", "Socket%s Root complex:%s"}},
    {0x0608, {6, 8, 2, "SMMU TBU8", "Socket%s Root complex:%s"}},
    {0x0609, {6, 9, 2, "SMMU TBU9", "Socket%s Root complex:%s"}},
    {0x0664, {6, 100, 2, "SMMU TCU", "Socket%s Root complex:%s"}},
    {0x0700, {7, 0, 2, "PCIe AER Root Port", "Socket%s Root complex:%s"}},
    {0x0701, {7, 1, 2, "PCIe AER Device", "Socket%s Root complex:%s"}},
    {0x0800, {8, 0, 2, "PCIe HB RCA", "Socket%s Root complex:%s"}},
    {0x0801, {8, 1, 2, "PCIe HB RCA", "Socket%s Root complex:%s"}},
    {0x0808, {8, 8, 2, "PCIe RASDP Error ", "Socket%s Root complex:%s"}},
    {0x0900, {9, 0, 1, "OCM ERR 0 (ECC Fault)", "Socket%s"}},
    {0x0901, {9, 1, 1, "OCM ERR 1 (ERR Recovery)", "Socket%s"}},
    {0x0902, {9, 2, 1, "OCM ERR 2 (Data Poisoned)", "Socket%s"}},
    {0x0a00, {10, 0, 1, "SMpro ERR 0 (ECC Fault)", "Socket%s"}},
    {0x0a01, {10, 1, 1, "SMpro ERR 1 (ERR Recovery)", "Socket%s"}},
    {0x0a02, {10, 2, 1, "SMpro MPA_ERR", "Socket%s"}},
    {0x0b00, {11, 0, 1, "PMpro ERR 0 (ECC Fault)", "Socket%s"}},
    {0x0b01, {11, 1, 1, "PMpro ERR 1 (ERR Recovery)", "Socket%s"}},
    {0x0b02, {11, 2, 1, "PMpro MPA_ERR", "Socket%s"}},
    {0x0c00, {12, 0, 1, "ATF firmware EL3", "Socket%s"}},
    {0x0c01, {12, 1, 1, "ATF firmware SPM", "Socket%s"}},
    {0x0c02, {12, 2, 1, "ATF firmware Secure Partition ", "Socket%s"}},
    {0x0d00, {13, 0, 1, "SMpro firmware RAS_MSG_ERR", "Socket%s"}},
    {0x0e00, {14, 0, 1, "PMpro firmware RAS_MSG_ERR", "Socket%s"}},
    {0x3f00, {63, 0, 1, "BERT Default", "Socket%s"}},
    {0x3f01, {63, 1, 1, "BERT Watchdog", "Socket%s"}},
    {0x3f02, {63, 2, 1, "BERT ATF Fatal", "Socket%s"}},
    {0x3f03, {63, 3, 1, "BERT SMpro Fatal", "Socket%s"}},
    {0x3f04, {63, 4, 1, "BERT PMpro Fatal", "Socket%s"}},
    {0xffff, {255, 255, 1, "Overflow", "Socket%s"}},
};

const static constexpr u_int16_t MCU_ERR_1_TYPE    = 0x0101;
const static constexpr u_int16_t MCU_ERR_2_TYPE    = 0x0102;

struct EventFields {
    u_int8_t type;
    u_int8_t subType;
    u_int16_t data;
};

struct EventData {
    u_int8_t idx;
    u_int16_t socket;
    u_int8_t intEventType;
    const char* label;
    u_int8_t eventType;
    u_int8_t eventReadType;
    u_int8_t eventNum;
    const char* eventName;
    const char* redFishMsgID;
};

/* Event type index */
enum EventTypes{
    event_vrd_warn_fault,
    event_vrd_hot,
    event_dimm_hot,
    event_dimm_2x_refresh
};

EventData eventTypeTable[] = {
    {0, 0, event_vrd_warn_fault, "event_vrd_warn_fault", TYPE_STATE,
        STATUS_READ_TYPE, S0_VRD_WARN_FAULT,
        "VR_WarnFault", "AmpereWarning"},
    {1, 0, event_vrd_hot, "event_vrd_hot", TYPE_TEMP,
        TEMP_READ_TYPE, S0_VRD_HOT,
        "VR_HOT", "AmpereWarning"},
    {2, 0, event_dimm_hot, "event_dimm_hot", TYPE_TEMP,
        TEMP_READ_TYPE, S0_DIMM_HOT,
        "DIMM_HOT", "AmpereWarning"},
    {3, 1, event_vrd_warn_fault, "event_vrd_warn_fault", TYPE_STATE,
        STATUS_READ_TYPE, S1_VRD_WARN_FAULT,
        "VR_WarnFault", "AmpereWarning"},
    {4, 1, event_vrd_hot, "event_vrd_hot", TYPE_TEMP,
        TEMP_READ_TYPE, S1_VRD_HOT,
        "VR_HOT", "AmpereWarning"},
    {5, 1, event_dimm_hot, "event_dimm_hot", TYPE_TEMP,
        TEMP_READ_TYPE, S1_DIMM_HOT,
        "DIMM_HOT", "AmpereWarning"},
    {6, 0, event_dimm_2x_refresh, "event_dimm_2x_refresh", TYPE_MEM,
        STATUS_READ_TYPE, S0_DIMM_2X_REFRESSH,
        "DIMM_2X_REFRESH_RATE", "AmpereWarning"},
    {7, 1, event_dimm_2x_refresh, "event_dimm_2x_refresh", TYPE_MEM,
        STATUS_READ_TYPE, S1_DIMM_2X_REFRESSH,
        "DIMM_2X_REFRESH_RATE", "AmpereWarning"}
};

const static constexpr u_int8_t NUMBER_OF_EVENTS    =
        sizeof(eventTypeTable) / sizeof(EventData);
u_int16_t curEventMask[NUMBER_OF_EVENTS] = {};

std::unique_ptr<phosphor::Timer> rasTimer
    __attribute__((init_priority(101)));

std::unique_ptr<sdbusplus::bus::match::match> hostStateMatch;

static int logInternalErrorToIpmiSEL(ErrorData data,
                                        InternalFields eFields)
{
    std::vector<uint8_t> eventData(
        ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.errType;
    eventData[4] = data.errNum;
    eventData[5] = (eFields.dir << 7) | IERR_SENSOR_SPECIFIC;
    eventData[6] = ((data.socket & 0x1) << 7) |
                   ((eFields.subType & 0x7) << 4) |
                   (eFields.imageCode & 0xf);
    eventData[7] = eFields.location & 0xff;
    eventData[8] = eFields.errCode & 0xff;
    eventData[9] = (eFields.errCode & 0xff00) >> 8;
    eventData[10] = eFields.data & 0xff;
    eventData[11] = (eFields.data & 0xff00) >> 8;

    ampere::sel::addSelOem("OEM RAS error:", eventData);

    return 1;
}

static int logInternalErrorToRedfish(ErrorData data,
                                        InternalFields eFields)
{
    char redfishMsgID[MAX_MSG_LEN] = {'\0'};
    char redfishMsg[MAX_MSG_LEN] = {'\0'};
    char sLocation[MAX_MSG_LEN] = "Unknown location";
    char sErrorCode[MAX_MSG_LEN] = "Unknown Error";
    char sImage[MAX_MSG_LEN] = "Unknown Image";
    char sDir[MAX_MSG_LEN] = "Unknown Action";
    char redfishComp[MAX_MSG_LEN] = {'\0'};

    if (eFields.location < ampere::internalErrors::NUM_LOCAL_CODES)
    {
        snprintf(sLocation, MAX_MSG_LEN, "%s",
                 ampere::internalErrors::localCodes[eFields.location]);
    }

    if (eFields.imageCode < ampere::internalErrors::NUM_IMAGE_CODES)
    {
        snprintf(sImage, MAX_MSG_LEN, "%s",
                 ampere::internalErrors::imageCodes[eFields.imageCode]);
    }

    if (eFields.errCode < ampere::internalErrors::NUM_ERROR_CODES)
    {
        snprintf(sErrorCode, MAX_MSG_LEN, "%s",
                 ampere::internalErrors::errorCodes[eFields.errCode]\
                 .description);
    }

    if (eFields.dir < ampere::internalErrors::NUM_DIRS)
    {
        snprintf(sDir, MAX_MSG_LEN, "%s",
                 ampere::internalErrors::directions[eFields.dir]);
    }

    snprintf(redfishComp, MAX_MSG_LEN, "S%d_%s: %s %s %s with",
             data.socket, data.errName, sImage, sDir, sLocation);

    if (eFields.subType == SMPMPRO_WARNING)
    {
        snprintf(redfishMsgID, MAX_MSG_LEN,
                 "OpenBMC.0.1.%s.Warning", data.redFishMsgID);
        snprintf(redfishMsg, MAX_MSG_LEN, "Warning %s.", sErrorCode);
    }
    else
    {
        snprintf(redfishMsgID, MAX_MSG_LEN,
                 "OpenBMC.0.1.%s.Critical", data.redFishMsgID);
        if (eFields.subType == SMPMPRO_ERROR)
            snprintf(redfishMsg, MAX_MSG_LEN, "Error %s.", sErrorCode);
        else
            snprintf(redfishMsg, MAX_MSG_LEN, "Error %s, data 0x%08x.",
                     sErrorCode, eFields.data);
    }

    if (data.intErrorType == errors_smpro ||
            data.intErrorType == errors_pmpro)
    {
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redfishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", redfishComp,
                        redfishMsg, NULL);
    }

    return 1;
}

static int parseAndLogInternalErrors(ErrorData data, std::string errLine)
{
    InternalFields errFields;
    std::vector<std::string> result;

    errLine.erase(std::remove(errLine.begin(), errLine.end(), '\n'),
                  errLine.end());
    boost::split(result, errLine, boost::is_any_of(" "));
    if (result.size() < 6)
    {
        return 0;
    }

    if (data.intErrorType == errors_smpro)
    {
        errFields.errType = SMPRO_IERR_TYPE;
    }
    else
    {
        errFields.errType = PMPRO_IERR_TYPE;
    }

    errFields.subType = ampere::utils::parseHexStrToUInt8(result[0]);
    errFields.imageCode = ampere::utils::parseHexStrToUInt8(result[1]);
    errFields.dir = ampere::utils::parseHexStrToUInt8(result[2]);
    errFields.location = ampere::utils::parseHexStrToUInt8(result[3]);
    errFields.errCode = ampere::utils::parseHexStrToUInt16(result[4]);
    errFields.data = ampere::utils::parseHexStrToUInt32(result[5]);

    /* Add SEL log */
    logInternalErrorToIpmiSEL(data, errFields);

    /* Add Redfish log */
    logInternalErrorToRedfish(data, errFields);

    return 1;
}

static int logErrorToRedfish(ErrorData data, ErrorFields eFields)
{
    char redFishMsgID[MAX_MSG_LEN] = {'\0'};
    char redFishMsg[MAX_MSG_LEN] = {'\0'};
    char redFishComp[MAX_MSG_LEN] = {'\0'};
    u_int8_t socket = (eFields.instance & 0xc000) >> 14;
    u_int16_t inst_13_0 = eFields.instance & 0x3fff;
    u_int8_t apiIdx = data.intErrorType;
    u_int16_t temp;
    ErrorInfo eInfo;

    snprintf(redFishMsgID, MAX_MSG_LEN,
             "OpenBMC.0.1.%s.Critical", data.redFishMsgID);
    temp = (eFields.errType << 8) + eFields.subType;
    if (mapOfOccur.size() != 0 && mapOfOccur.count(temp) > 0)
    {
        eInfo = mapOfOccur[temp];
        char str1[4] = {'\0'};
        char str2[6] = {'\0'};
        snprintf(str1, 4, "%d", socket);
        snprintf(str2, 6, "%d", inst_13_0);

        if (eInfo.numPars == 1)
        {
            snprintf(redFishMsg, MAX_MSG_LEN, eInfo.errMsgFormat, str1);
        }
        else if (eInfo.numPars == 2)
        {
            snprintf(redFishMsg, MAX_MSG_LEN, eInfo.errMsgFormat, str1,
                        str2);
        }
        snprintf(redFishComp, MAX_MSG_LEN, "%s", eInfo.errName);
    }

    if (temp == 0xffff)
    {
        char comp[MAX_MSG_LEN] = {'\0'};
        snprintf(redFishMsgID, MAX_MSG_LEN,
                "OpenBMC.0.1.%s.Critical", AMPERE_REFISH_REGISTRY);
        snprintf(comp, MAX_MSG_LEN, "%s: %s", data.errName, redFishComp);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
        return 1;
    }

    if (apiIdx == errors_core_ue || apiIdx == errors_core_ce)
    {
        char sTemp[MAX_MSG_LEN] = {'\0'};
        snprintf(sTemp, MAX_MSG_LEN, "%s: %s %s", data.errName,
                 redFishComp, redFishMsg);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s", sTemp, NULL);
    }
    else if (apiIdx == errors_mem_ue || apiIdx == errors_mem_ce)
    {
        char dimCh[MAX_MSG_LEN] = {'\0'};
        u_int8_t rank = (eFields.address >> 20) & 0xF;
        u_int8_t bank = (int)((eFields.misc0 >> 32) & 0xf);
        u_int8_t row  = (int)((eFields.misc0 >> 10) & 0x3ffff);
        u_int8_t col  = (int)(((eFields.misc0 >> 0) & 0x3ff) << 3);
        char redFishECCMsgID[MAX_MSG_LEN] = {'\0'};

        snprintf(dimCh, MAX_MSG_LEN, "%x", (inst_13_0 & 0x7ff));
        /* Only detect DIMM Idx for MCU_ERROR_1 or MCU_ERROR_2 Type */
        if (temp == MCU_ERR_1_TYPE || temp == MCU_ERR_2_TYPE)
        {
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%d,%s,%d,%d", socket,
                            dimCh, (inst_13_0 & 0x3800) >> 11,
                            rank , NULL);
        }
        else
        {
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%d,%s,%d,%d", socket,
                            dimCh, 0xff, 0xff, NULL);
        }
        if (apiIdx == errors_mem_ue)
        {
            snprintf(redFishECCMsgID, MAX_MSG_LEN,
                    "OpenBMC.0.1.MemoryExtendedECCUEData.Critical");
        }
        else
        {
            snprintf(redFishECCMsgID, MAX_MSG_LEN,
                    "OpenBMC.0.1.MemoryExtendedECCCEData.Warning");
        }
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishECCMsgID,
                        "REDFISH_MESSAGE_ARGS=%d,%d,%d", bank,
                        row, col);
    }
    else if (apiIdx == errors_pcie_ue || apiIdx == errors_pcie_ce)
    {
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%d,%d,%d", socket,
                        inst_13_0, 0, NULL);
    }
    else if (apiIdx == errors_other_ue || apiIdx == errors_other_ce)
    {
        char comp[MAX_MSG_LEN] = {'\0'};
        snprintf(comp, MAX_MSG_LEN, "%s: %s", data.errName, redFishComp);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    if (apiIdx == errors_core_ue || apiIdx == errors_mem_ue ||
            apiIdx == errors_pcie_ue || apiIdx == errors_other_ue)
    {
        if(std::system(createFlagCmd) != 0)
            log<level::INFO>("Cannot create flag RAS UE for fault monitor");
    }
    return 1;
}

static int logErrorToIpmiSEL(ErrorData data, ErrorFields eFields)
{
    std::vector<uint8_t> eventData(
            ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.errType;
    eventData[4] = data.errNum;
    eventData[5] = eFields.errType;
    eventData[6] = eFields.subType;
    eventData[7] = (eFields.instance & 0xff00) >> 8;
    eventData[8] = (eFields.instance & 0xff);

    ampere::sel::addSelOem("OEM RAS error:", eventData);

    return 1;
}

static int parseAndLogErrors(ErrorData data, std::string errLine)
{
    ErrorFields errFields;
    std::vector<std::string> result;

    errLine.erase(std::remove(errLine.begin(), errLine.end(), '\n'),
                  errLine.end());
    boost::split(result, errLine, boost::is_any_of(" "));
    if (result.size() < 5)
    {
        return 0;
    }

    errFields.errType = ampere::utils::parseHexStrToUInt8(result[0]);
    errFields.subType = ampere::utils::parseHexStrToUInt8(result[1]);
    errFields.instance = ampere::utils::parseHexStrToUInt16(result[2]);
    errFields.status = ampere::utils::parseHexStrToUInt32(result[3]);
    errFields.address = ampere::utils::parseHexStrToUInt64(result[4]);
    /* Error type with 48 data bytes */
    if (result.size() >= 9)
    {
        errFields.misc0 = ampere::utils::parseHexStrToUInt64(result[5]);
        errFields.misc1 = ampere::utils::parseHexStrToUInt64(result[6]);
        errFields.misc2 = ampere::utils::parseHexStrToUInt64(result[7]);
        errFields.misc3 = ampere::utils::parseHexStrToUInt64(result[8]);
    }

    /* Error type is Overflowed */
    if (errFields.errType == 0xff && errFields.subType == 0xff)
    {
        errFields.instance = data.socket << 14;
    }

    /* Add Ipmi SEL log*/
    logErrorToIpmiSEL(data, errFields);

    /* Add Redfish log */
    logErrorToRedfish(data, errFields);

    return 1;
}

static int logErrors(ErrorData data, const char *fileName) {
    FILE *fp;
    char* line = NULL;

    /* Read system file */
    fp = fopen(fileName, "r");
    if (!fp)
    {
        return 0;
    }

    size_t len = 0;
    while ((getline(&line, &len, fp)) != -1)
    {
        if (data.intErrorType == errors_smpro ||
            data.intErrorType == errors_pmpro)
        {
            parseAndLogInternalErrors(data, line);
        }
        else
        {
            parseAndLogErrors(data, line);
        }
    }

    fclose(fp);
    if (line)
    {
        free(line);
    }

    return 1;
}

static int logEventDIMMHot(EventData data, EventFields eFields)
{
    std::vector<uint8_t> eventData(
            ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);
    u_int16_t bitMask = 0;
    u_int8_t channel = 0, dimmIdx = 0;
    char redFishMsgID[MAX_MSG_LEN] = {'\0'};
    char redFishMsg[MAX_MSG_LEN] = {'\0'};
    char comp[MAX_MSG_LEN] = {'\0'};
    u_int8_t i = 0;
    u_int16_t currentMask = curEventMask[data.idx];

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.eventType;
    eventData[4] = data.eventNum;
    eventData[6] = 0x1 | EVENT_DATA_1 | EVENT_DATA_3;

    snprintf(redFishMsgID, MAX_MSG_LEN,
             "OpenBMC.0.1.%s.Warning", data.redFishMsgID);
    for (i = 0; i < SMPRO_DATA_REG_SIZE; i++)
    {
        bitMask = pow(2, i);
        channel = i % 8;
        dimmIdx = i / 8;
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM%d of channel %d"\
                 " of Socket %d", data.eventName, dimmIdx, channel,
                 data.socket);

        if ((eFields.data & bitMask) && (!(currentMask & bitMask)))
        {
            eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
            if (dimmIdx == 0)
            {
                eventData[7] = bitMask;
                eventData[8] = 0;
            }
            else
            {
                eventData[7] = 0;
                eventData[8] = bitMask;
            }
            curEventMask[data.idx] = curEventMask[data.idx] | bitMask;
            ampere::sel::addSelOem("OEM RAS error:", eventData);

            snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%s,%s", comp,
                            redFishMsg, NULL);

        }
        else if ((!(eFields.data & bitMask)) && (currentMask & bitMask))
        {
            eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
            if (dimmIdx == 0)
            {
                eventData[7] = bitMask;
                eventData[8] = 0;
            }
            else
            {
                eventData[7] = 0;
                eventData[8] = bitMask;
            }
            curEventMask[data.idx] = curEventMask[data.idx] &
                                     (0xffff - bitMask);
            ampere::sel::addSelOem("OEM RAS error:", eventData);

            snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                            NULL);
        }
    }

    return 1;
}

static int logEventDIMM2xRefresh(EventData data, EventFields eFields)
{
    std::vector<uint8_t> eventData(
            ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);
    u_int16_t bitMask = 0;
    u_int8_t channel = 0;
    char redFishMsgID[MAX_MSG_LEN] = {'\0'};
    char redFishMsg[MAX_MSG_LEN] = {'\0'};
    char comp[MAX_MSG_LEN] = {'\0'};
    u_int16_t currentMask = curEventMask[data.idx];

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.eventType;
    eventData[4] = data.eventNum;
    eventData[6] = 0x1 | EVENT_DATA_1 | EVENT_DATA_3;

    snprintf(redFishMsgID, MAX_MSG_LEN,
             "OpenBMC.0.1.%s.Warning", data.redFishMsgID);
    for (channel = 0; channel < NUMBER_DIMM_CHANNEL; channel++)
    {
        bitMask = pow(2, channel);
        eventData[7] = data.socket;
        eventData[8] = channel;
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM channel %d"\
                 " of Socket %d", data.eventName, channel,
                 data.socket);

        if ((eFields.data & bitMask) && (!(currentMask & bitMask)))
        {
            eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
            curEventMask[data.idx] = curEventMask[data.idx] | bitMask;

            snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
            ampere::sel::addSelOem("OEM RAS error:", eventData);
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                            NULL);
        }
        else if ((!(eFields.data & bitMask)) && (currentMask & bitMask))
        {
            eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
            curEventMask[data.idx] = curEventMask[data.idx] &
                                     (0xffff - bitMask);
            snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
            ampere::sel::addSelOem("OEM RAS error:", eventData);
            sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                            "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                            NULL);
        }
    }

    return 1;
}

static int logEventVrdHot(EventData data, EventFields eFields)
{
    std::vector<uint8_t> eventData(
            ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);
    char redFishMsgID[MAX_MSG_LEN] = {'\0'};
    char redFishMsg[MAX_MSG_LEN] = {'\0'};
    char comp[MAX_MSG_LEN] = {'\0'};
    u_int16_t currentMask = curEventMask[data.idx];

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.eventType;
    eventData[4] = data.eventNum;
    eventData[6] = 0x1 | EVENT_DATA_1 | EVENT_DATA_3;

    snprintf(redFishMsgID, MAX_MSG_LEN,
             "OpenBMC.0.1.%s.Warning", data.redFishMsgID);
    /* SoC VRD hot */
    if ((eFields.data & BIT_0) && (!(currentMask & BIT_0)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (SOC_COMPONENT << 4) | data.socket;
        eventData[8] = 0;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_0;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at SoC_VRD of Socket %d",
                 data.eventName, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_0)) && (currentMask & BIT_0))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (SOC_COMPONENT << 4) | data.socket;
        eventData[8] = 0;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_0);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at SoC_VRD of Socket %d",
                 data.eventName, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD1 fault/warning */
    if ((eFields.data & BIT_4) && (!(currentMask & BIT_4)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_4;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_4)) && (currentMask & BIT_4))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_4);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD2 fault/warning */
    if ((eFields.data & BIT_5) && (!(currentMask & BIT_5)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_5;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_5)) && (currentMask & BIT_5))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_5);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD3 fault/warning */
    if ((eFields.data & BIT_6) && (!(currentMask & BIT_6)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_6;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_6)) && (currentMask & BIT_6))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_6);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD1 fault/warning */
    if ((eFields.data & BIT_8) && (!(currentMask & BIT_8)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_8;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_8)) && (currentMask & BIT_8))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_8);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD2 fault/warning */
    if ((eFields.data & BIT_9) && (!(currentMask & BIT_9)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_9;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_9)) && (currentMask & BIT_9))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_9);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD3 fault/warning */
    if ((eFields.data & BIT_10) && (!(currentMask & BIT_10)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_10;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_10)) && (currentMask & BIT_10))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_10);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD4 fault/warning */
    if ((eFields.data & BIT_11) && (!(currentMask & BIT_11)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_4;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_11;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_4, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_11)) && (currentMask & BIT_11))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_4;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_11);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_4, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    return 1;
}

static int logEventVrdWarnFault(EventData data, EventFields eFields)
{
    std::vector<uint8_t> eventData(
            ampere::sel::SEL_OEM_DATA_MAX_SIZE, 0xFF);
    char redFishMsgID[MAX_MSG_LEN] = {'\0'};
    char redFishMsg[MAX_MSG_LEN] = {'\0'};
    char comp[MAX_MSG_LEN] = {'\0'};
    u_int16_t currentMask = curEventMask[data.idx];

    eventData[0] = AMPERE_IANA_BYTE_1;
    eventData[1] = AMPERE_IANA_BYTE_2;
    eventData[2] = AMPERE_IANA_BYTE_3;
    eventData[3] = data.eventType;
    eventData[4] = data.eventNum;
    eventData[6] = 0x1 | EVENT_DATA_1 | EVENT_DATA_3;

    snprintf(redFishMsgID, MAX_MSG_LEN,
             "OpenBMC.0.1.%s.Warning", data.redFishMsgID);
    /* SoC VRD fault/warning */
    if ((eFields.data & BIT_0) && (!(currentMask & BIT_0)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (SOC_COMPONENT << 4) | data.socket;
        eventData[8] = 0;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_0;
        ampere::sel::addSelOem("OEM RAS error:", eventData);
        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at SoC_VRD of Socket %d",
                 data.eventName, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_0)) && (currentMask & BIT_0))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (SOC_COMPONENT << 4) | data.socket;
        eventData[8] = 0;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_0);
        ampere::sel::addSelOem("OEM RAS error:", eventData);
        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at SoC_VRD of Socket %d",
                 data.eventName, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD1 fault/warning */
    if ((eFields.data & BIT_1) && (!(currentMask & BIT_1)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_1;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_1)) && (currentMask & BIT_1))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_1);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD2 fault/warning */
    if ((eFields.data & BIT_2) && (!(currentMask & BIT_2)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_2;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_2)) && (currentMask & BIT_2))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_2);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* Core VRD3 fault/warning */
    if ((eFields.data & BIT_3) && (!(currentMask & BIT_3)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_3;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_3)) && (currentMask & BIT_3))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (CORE_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_3);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at CORE_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD1 fault/warning */
    if ((eFields.data & BIT_4) && (!(currentMask & BIT_4)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_4;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_4)) && (currentMask & BIT_4))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_1;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_4);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_1, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD2 fault/warning */
    if ((eFields.data & BIT_5) && (!(currentMask & BIT_5)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_5;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_5)) && (currentMask & BIT_5))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_2;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_5);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_2, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    /* DIMM VRD3 fault/warning */
    if ((eFields.data & BIT_6) && (!(currentMask & BIT_6)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_6;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_6)) && (currentMask & BIT_6))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_3;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_6);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_3, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                    "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                    NULL);
    }

    /* DIMM VRD4 fault/warning */
    if ((eFields.data & BIT_7) && (!(currentMask & BIT_7)))
    {
        eventData[5] = (DIR_ASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_4;
        curEventMask[data.idx] = curEventMask[data.idx] | BIT_7;
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Asserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_4, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }
    else if ((!(eFields.data & BIT_7)) && (currentMask & BIT_7))
    {
        eventData[5] = (DIR_DEASSERTED << 7) | data.eventReadType;
        eventData[7] = (DIMM_COMPONENT << 4) | data.socket;
        eventData[8] = VRD_4;
        curEventMask[data.idx] = curEventMask[data.idx] & (0xffff - BIT_7);
        ampere::sel::addSelOem("OEM RAS error:", eventData);

        snprintf(redFishMsg, MAX_MSG_LEN, "Deasserted.");
        snprintf(comp, MAX_MSG_LEN, "Event %s at DIMM_VRD%d of Socket %d",
                 data.eventName, VRD_4, data.socket);
        sd_journal_send("REDFISH_MESSAGE_ID=%s", redFishMsgID,
                        "REDFISH_MESSAGE_ARGS=%s,%s", comp, redFishMsg,
                        NULL);
    }

    return 1;
}

static int parseAndLogEvents(EventData data, std::string eventLine)
{
    EventFields eventFields;
    std::vector<std::string> result;

    eventLine.erase(std::remove(eventLine.begin(), eventLine.end(), '\n'),
        eventLine.end());
    boost::split(result, eventLine, boost::is_any_of(" "));
    if (result.size() < 2)
        return 0;
    eventFields.type = ampere::utils::parseHexStrToUInt8(result[0]);
    eventFields.data = ampere::utils::parseHexStrToUInt16(result[1]);

    switch (eventFields.type)
    {
        case event_vrd_warn_fault:
            logEventVrdWarnFault(data, eventFields);
            break;
        case event_vrd_hot:
            logEventVrdHot(data, eventFields);
            break;
        case event_dimm_hot:
            logEventDIMMHot(data, eventFields);
            break;
        case event_dimm_2x_refresh:
            logEventDIMM2xRefresh(data, eventFields);
            break;
        default:
            break;
    }

    return 1;
}

static int logEvents(EventData data, const char *fileName)
{
    FILE *fp;
    char* line = NULL;

    /* Read system file */
    fp = fopen(fileName, "r");
    if (!fp)
    {
        return 0;
    }

    size_t len = 0;
    while ((getline(&line, &len, fp)) != -1)
    {
        parseAndLogEvents(data, line);
    }

    fclose(fp);
    if (line)
    {
        free(line);
    }

    return 1;
}

static void getErrorsAndEvents()
{
    std::string filePath;
    u_int8_t index = 0;

    for(index = 0; index < NUMBER_OF_ERRORS; index ++)
    {
        ErrorData data1 = errorTypeTable[index];
        filePath = ampere::utils::getAbsolutePath(
                    data1.socket, data1.label);
        if (filePath != "")
        {
            logErrors(data1, filePath.c_str());
        }
    }

    for(index = 0; index < NUMBER_OF_EVENTS; index ++)
    {
        EventData data2 = eventTypeTable[index];
        filePath = ampere::utils::getAbsolutePath(
                    data2.socket, data2.label);
        if (filePath != "")
        {
            logEvents(data2, filePath.c_str());
        }
    }
}

static void handleHostStateMatch(std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    rasTimer = std::make_unique<phosphor::Timer>(getErrorsAndEvents);

    auto startEventMatcherCallback = [](sdbusplus::message::message& msg) {
        boost::container::flat_map<std::string, std::variant<std::string>>
            propertiesChanged;
        std::string interfaceName;

        msg.read(interfaceName, propertiesChanged);
        if (propertiesChanged.empty())
        {
            return;
        }

        std::string event = propertiesChanged.begin()->first;
        auto variant =
            std::get_if<std::string>(&propertiesChanged.begin()->second);

        if (event.empty() || variant == nullptr)
        {
            return;
        }

        if (event == "CurrentHostState")
        {
            if (*variant ==
                     "xyz.openbmc_project.State.Host.HostState.Running")
            {
                log<level::INFO>("Host is turned on ");
		getErrorsAndEvents();
                rasTimer->start(std::chrono::microseconds(1200000), true);
            }
            else
            {
                log<level::INFO>("Host is turned off ");
                rasTimer->stop();
                auto p = fs::path(RASUEFlagPath);
                if(fs::exists(p))
                {
                    if(std::system(rmFlagCmd) != 0)
                        log<level::INFO>("remove flag RAS UE failed");
                }
            }
        }
    };

    hostStateMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0namespace='xyz.openbmc_project.State.Host'",
        std::move(startEventMatcherCallback));
}

} /* namespace ras */
} /* namespace ampere */

int main()
{
    int ret;
    log<level::INFO>("Starting xyz.openbmc_project.AmpRas.service");

    boost::asio::io_context io;

    /*
     * Add timer to keep sd_event is warm, if this timer is removed,
     * ras timer does not is called.
     */
    phosphor::Timer t2([]() { ; });
    t2.start(std::chrono::microseconds(500000), true);

    auto conn = std::make_shared<sdbusplus::asio::connection>(io);

    ampere::sel::initSelUtil(conn);
    ret = ampere::utils::initHwmonRootPath();
    if (!ret)
    {
        log<level::ERR>("Failed to get Root Path of SMPro Hwmon\n");
        return 1;
    }

    sdbusplus::asio::sd_event_wrapper sdEvents(io);

    ampere::ras::handleHostStateMatch(conn);

    io.run();

    return 0;
}
