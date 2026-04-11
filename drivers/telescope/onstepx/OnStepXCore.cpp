/*
    OnStep X INDI Driver

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "OnStepXCore.h"

#include <defaultdevice.h>
#include <indilogger.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Logging helpers — route through INDI::Logger via the parent device pointer.
// Guards prevent any call when m_dev is null (tests without a device).
// ---------------------------------------------------------------------------
#define OSX_CORE_LOGF(priority, fmt, ...)                                                               \
    do {                                                                                                \
        if (m_dev)                                                                                      \
            INDI::Logger::getInstance().print(m_dev->getDeviceName(), priority, __FILE__, __LINE__,    \
                                              fmt, ##__VA_ARGS__);                                     \
    } while (0)

#define OSX_CORE_LOG_INFO(msg)          OSX_CORE_LOGF(INDI::Logger::DBG_SESSION, "%s", msg)
#define OSX_CORE_LOGF_INFO(fmt, ...)    OSX_CORE_LOGF(INDI::Logger::DBG_SESSION, fmt, ##__VA_ARGS__)
#define OSX_CORE_LOGF_WARN(fmt, ...)    OSX_CORE_LOGF(INDI::Logger::DBG_WARNING, fmt, ##__VA_ARGS__)
#define OSX_CORE_LOGF_ERROR(fmt, ...)   OSX_CORE_LOGF(INDI::Logger::DBG_ERROR,   fmt, ##__VA_ARGS__)
#define OSX_CORE_LOGF_DEBUG(fmt, ...)   OSX_CORE_LOGF(INDI::Logger::DBG_DEBUG,   fmt, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Helper: return true when s can be parsed as a floating-point number.
// Used to distinguish valid sensor readings from error strings.
// ---------------------------------------------------------------------------
static bool isNumeric(const char *s)
{
    if (!s || *s == '\0')
        return false;
    char *end = nullptr;
    strtod(s, &end);
    return end != s && *end == '\0';
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void OnStepXCore::setDevice(INDI::DefaultDevice *dev)
{
    m_dev = dev;
    m_comm.setDevice(dev);
}

void OnStepXCore::setFd(int fd)
{
    m_comm.setFd(fd);
}

const Capabilities &OnStepXCore::caps() const
{
    return m_cap;
}

OnStepXComm &OnStepXCore::comm()
{
    return m_comm;
}

// ---------------------------------------------------------------------------
// Phase 1 — probeController()
// Runs for both binaries (mount and aux).
// Returns true only if firmware identifies itself as OnStepX.
// ---------------------------------------------------------------------------
bool OnStepXCore::probeController()
{
    char reply[256];

    // :GVP# — product identity (both OnStep and OnStepX return "On-Step")
    if (!m_comm.sendCommand(":GVP#", reply))
    {
        OSX_CORE_LOGF_ERROR("probeController: no response to :GVP#");
        return false;
    }
    if (strcmp(reply, "On-Step") != 0)
    {
        OSX_CORE_LOGF_WARN("probeController: unrecognised product string '%s'", reply);
        return false;
    }

    // :GVN# — firmware version string, format "Major.MinorPatch" e.g. "10.24c"
    // OnStepX firmware started at major version 10; anything lower is classic OnStep.
    if (!m_comm.sendCommand(":GVN#", reply))
    {
        OSX_CORE_LOGF_ERROR("probeController: no response to :GVN#");
        return false;
    }
    snprintf(m_cap.firmwareVersion, sizeof(m_cap.firmwareVersion), "%s", reply);

    char *dotPos  = nullptr;
    long  major   = strtol(reply, &dotPos, 10);
    m_cap.isOnStepX = (dotPos != reply && major >= 10);
    if (!m_cap.isOnStepX)
    {
        OSX_CORE_LOGF_WARN("OnStep firmware v%s detected; OnStepX (v10+) required.", reply);
        return false;
    }

    // Remaining firmware strings (best-effort — do not abort on failure)
    if (m_comm.sendCommand(":GVD#", reply, 2000, true))
        snprintf(m_cap.firmwareDate,    sizeof(m_cap.firmwareDate),    "%s", reply);
    if (m_comm.sendCommand(":GVT#", reply, 2000, true))
        snprintf(m_cap.firmwareTime,    sizeof(m_cap.firmwareTime),    "%s", reply);
    if (m_comm.sendCommand(":GVC#", reply, 2000, true))
        snprintf(m_cap.configName,      sizeof(m_cap.configName),      "%s", reply);

    OSX_CORE_LOGF_INFO("OnStepX firmware: %s  date: %s %s  config: %s",
                       m_cap.firmwareVersion, m_cap.firmwareDate,
                       m_cap.firmwareTime,    m_cap.configName);

    // :GX98# — rotator: 'R'=rotator present, 'D'=derotator (AltAz field-de-rotation)
    if (m_comm.sendCommand(":GX98#", reply, 2000, true))
    {
        m_cap.hasRotator   = (reply[0] == 'R' || reply[0] == 'D');
        m_cap.hasDerotator = (reply[0] == 'D');
    }
    OSX_CORE_LOGF_DEBUG("hasRotator=%d  hasDerotator=%d", m_cap.hasRotator, m_cap.hasDerotator);

    // :FA1#..:FA6# — focuser presence (single-char '1'/'0', no '#' terminator)
    m_cap.numFocusers = 0;
    for (int n = 1; n <= 6; n++)
    {
        char cmd[8];
        snprintf(cmd, sizeof(cmd), ":FA%d#", n);
        char c = '0';
        if (m_comm.sendCommandSingleChar(cmd, c, 2000, true) && c == '1')
            m_cap.numFocusers = n;
        else
            break;  // focusers are numbered sequentially; stop at first gap
    }
    OSX_CORE_LOGF_DEBUG("numFocusers=%d", m_cap.numFocusers);

    // :GXY0# — 8-char bitfield, bit n → aux feature slot n active
    if (m_comm.sendCommand(":GXY0#", reply, 2000, true) && strlen(reply) >= 8)
    {
        m_cap.featureMask = 0;
        for (int i = 0; i < 8; i++)
            if (reply[i] == '1')
                m_cap.featureMask |= static_cast<uint8_t>(1 << i);
    }
    OSX_CORE_LOGF_DEBUG("featureMask=0x%02X", m_cap.featureMask);

    // :GUY0# — 8-char bitfield, bit n → USB port slot n active
    if (m_comm.sendCommand(":GUY0#", reply, 2000, true) && strlen(reply) >= 8)
    {
        m_cap.portMask = 0;
        for (int i = 0; i < 8; i++)
            if (reply[i] == '1')
                m_cap.portMask |= static_cast<uint8_t>(1 << i);
    }
    OSX_CORE_LOGF_DEBUG("portMask=0x%02X", m_cap.portMask);

    // :GX9A# — weather: pass if reply is a valid float (sensor present)
    if (m_comm.sendCommand(":GX9A#", reply, 2000, true))
    {
        bool validStart = (reply[0] >= '0' && reply[0] <= '9') ||
                          reply[0] == '-' || reply[0] == '+' || reply[0] == '.';
        char *end = nullptr;
        double val = validStart ? strtod(reply, &end) : 0.0;
        m_cap.hasWeatherRead = validStart && end != reply && val != 0.0;
    }

    // :SX9A,15.0# — weather write: single-char '1' if writable
    if (m_cap.hasWeatherRead)
    {
        char c = '0';
        if (m_comm.sendCommandSingleChar(":SX9A,15.0#", c, 2000, true))
            m_cap.hasWeatherWrite = (c == '1');
    }
    OSX_CORE_LOGF_DEBUG("hasWeatherRead=%d  hasWeatherWrite=%d",
                        m_cap.hasWeatherRead, m_cap.hasWeatherWrite);

    // :Gv# — site elevation readable
    if (m_comm.sendCommand(":Gv#", reply, 2000, true))
        m_cap.hasElevation = isNumeric(reply);

    // :GX9F# — MCU temperature sensor
    if (m_comm.sendCommand(":GX9F#", reply, 2000, true))
        m_cap.hasMcuTemp = isNumeric(reply);

    OSX_CORE_LOGF_DEBUG("hasElevation=%d  hasMcuTemp=%d",
                        m_cap.hasElevation, m_cap.hasMcuTemp);

    OSX_CORE_LOGF_INFO("Probe complete: rotator=%d  focusers=%d  features=0x%02X  "
                       "weather=%d  elevation=%d  mcuTemp=%d",
                       m_cap.hasRotator, m_cap.numFocusers, m_cap.featureMask,
                       m_cap.hasWeatherRead, m_cap.hasElevation, m_cap.hasMcuTemp);
    return true;
}

// ---------------------------------------------------------------------------
// Phase 2 — probeMount()
// Runs only for the mount binary (OnStepXMount::Handshake).
// Returns true only if a mount is present.
// ---------------------------------------------------------------------------
bool OnStepXCore::probeMount()
{
    char reply[256];

    // :GW# — mount type and goto capability (must be non-empty for a mount to be present)
    if (!m_comm.sendCommand(":GW#", reply) || reply[0] == '\0')
    {
        OSX_CORE_LOG_INFO("No mount detected. "
                          "For mount-less OnStepX use indi_onstepx_aux instead.");
        m_cap.hasMount = false;
        return false;
    }

    m_cap.hasMount = true;
    switch (reply[0])
    {
        case 'G':
            m_cap.mountType = MountType::GEM;
            break;
        case 'P':
            m_cap.mountType = MountType::FORK;
            break;
        case 'A':
            m_cap.mountType = MountType::ALTAZM;
            break;
        case 'L':
            m_cap.mountType = MountType::ALTALT;
            break;
        default:
            m_cap.mountType = MountType::UNKNOWN;
            break;
    }
    m_cap.hasGoto = (strlen(reply) >= 3 && reply[2] != 'N');

    OSX_CORE_LOGF_INFO("Mount type: %c  hasGoto=%d", reply[0], m_cap.hasGoto);

    // :Gu# — binary status: 9 bytes all >= 0x80 means supported
    {
        uint8_t bin[9];
        if (m_comm.sendCommandReadN(":Gu#", bin, 9, 2000, true))
        {
            bool allHigh = true;
            for (int i = 0; i < 9; i++)
                if (bin[i] < 0x80)
                {
                    allHigh = false;
                    break;
                }
            m_cap.hasBinaryStatus = allHigh;
        }
    }
    OSX_CORE_LOGF_DEBUG("hasBinaryStatus=%d", m_cap.hasBinaryStatus);

    // $QZ?# — PEC: supported if reply is a known PEC-state character
    if (m_comm.sendCommand("$QZ?#", reply, 2000, true))
    {
        m_cap.hasPec = (reply[0] == '!' || reply[0] == 'I' ||
                        reply[0] == 'p' || reply[0] == 'P' ||
                        reply[0] == 'r' || reply[0] == 'R');
    }
    OSX_CORE_LOGF_DEBUG("hasPec=%d", m_cap.hasPec);

    // :h?# — home sense: first field '-1' means no home sensor
    if (m_comm.sendCommand(":h?#", reply, 2000, true))
        m_cap.hasHomeSense = !(strncmp(reply, "-1,", 3) == 0 || strcmp(reply, "-1") == 0);
    OSX_CORE_LOGF_DEBUG("hasHomeSense=%d", m_cap.hasHomeSense);

    // :GU# — ASCII status string: 'S' present means PPS sync supported
    if (m_comm.sendCommand(":GU#", reply, 2000, true))
        m_cap.hasPPS = (strchr(reply, 'S') != nullptr);
    OSX_CORE_LOGF_DEBUG("hasPPS=%d", m_cap.hasPPS);

    // :SU0.0# — DUT1 correction: single-char '1' if writable
    {
        char c = '0';
        if (m_comm.sendCommandSingleChar(":SU0.0#", c, 2000, true))
            m_cap.hasDUT1 = (c == '1');
    }
    OSX_CORE_LOGF_DEBUG("hasDUT1=%d", m_cap.hasDUT1);

    // :Gm# — pier side: supported if reply is E, W, or N
    if (m_comm.sendCommand(":Gm#", reply, 2000, true))
        m_cap.hasPierSide = (reply[0] == 'E' || reply[0] == 'W' || reply[0] == 'N');
    OSX_CORE_LOGF_DEBUG("hasPierSide=%d", m_cap.hasPierSide);

    OSX_CORE_LOGF_INFO("Mount probe complete: pec=%d  homeSense=%d  pps=%d  "
                       "dut1=%d  pierSide=%d  binaryStatus=%d",
                       m_cap.hasPec, m_cap.hasHomeSense, m_cap.hasPPS,
                       m_cap.hasDUT1, m_cap.hasPierSide, m_cap.hasBinaryStatus);
    return true;
}
