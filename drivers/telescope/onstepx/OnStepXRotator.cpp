/*
    OnStep X INDI Driver — Rotator helper (shared by both binaries)

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

#include "OnStepXRotator.h"
#include "OnStepXComm.h"

#include <defaultdevice.h>
#include <indicom.h>   // f_scansexa, getSexComponents
#include <indilogger.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define ROTATOR_TAB "Rotator"

// Logging helpers — safe to call even if m_dev is null (e.g. during tests)
#define OSX_ROT_LOGF(priority, fmt, ...) \
    do { \
        if (m_dev) \
            INDI::Logger::getInstance().print(m_dev->getDeviceName(), priority, __FILE__, __LINE__, \
                                              fmt, ##__VA_ARGS__); \
    } while (0)
#define OSX_ROT_LOGF_ERROR(fmt, ...) OSX_ROT_LOGF(INDI::Logger::DBG_ERROR, fmt, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXRotator::initProperties(bool hasDerotator)
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    m_derotateSP[0].fill("DEROTATE_OFF", "Off", ISS_ON);
    m_derotateSP[1].fill("DEROTATE_ON",  "On",  ISS_OFF);
    m_derotateSP.fill(dev, "OSX_ROT_DEROTATE",
                      "De-rotator", ROTATOR_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    m_parallacticSP[0].fill("PARALLACTIC_OFF", "Off", ISS_ON);
    m_parallacticSP[1].fill("PARALLACTIC_ON",  "On",  ISS_OFF);
    m_parallacticSP.fill(dev, "OSX_ROT_PARALLACTIC",
                         "Parallactic Track", ROTATOR_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    INDI_UNUSED(hasDerotator);  // stored in cap; queried at updateProperties time
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXRotator::updateProperties(bool connected, bool hasDerotator)
{
    if (connected)
    {
        if (hasDerotator)
        {
            m_dev->defineProperty(m_derotateSP);
            m_dev->defineProperty(m_parallacticSP);
        }
    }
    else
    {
        m_dev->deleteProperty(m_derotateSP);
        m_dev->deleteProperty(m_parallacticSP);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch
// ---------------------------------------------------------------------------
bool OnStepXRotator::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    // De-rotator enable/disable
    if (m_derotateSP.isNameMatch(name))
    {
        m_derotateSP.update(states, names, n);
        const char *cmd = (m_derotateSP[1].getState() == ISS_ON) ? ":r+#" : ":r-#";
        m_comm->sendCommandBlind(cmd);
        m_derotateSP.setState(IPS_OK);
        m_derotateSP.apply();
        return true;
    }

    // Parallactic tracking mode
    if (m_parallacticSP.isNameMatch(name))
    {
        m_parallacticSP.update(states, names, n);
        const char *cmd = (m_parallacticSP[1].getState() == ISS_ON) ? ":SX98,1#" : ":SX98,0#";
        char reply[8];
        if (m_comm->sendCommand(cmd, reply) && reply[0] == '1')
            m_parallacticSP.setState(IPS_OK);
        else
            m_parallacticSP.setState(IPS_ALERT);
        m_parallacticSP.apply();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// handleNumber — nothing custom yet; placeholder for future OSX_ROT_RATE
// ---------------------------------------------------------------------------
bool OnStepXRotator::handleNumber(const char * /*name*/, double * /*values*/,
                                   char * /*names*/[], int /*n*/)
{
    return false;
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXRotator::saveConfig(FILE *fp)
{
    m_derotateSP.save(fp);
    m_parallacticSP.save(fp);
}

// ---------------------------------------------------------------------------
// RotatorInterface virtual implementations
// ---------------------------------------------------------------------------
IPState OnStepXRotator::moveToAngle(double angle)
{
    char cmd[32], reply[8];
    if (!formatAngle(angle, cmd, sizeof(cmd)))
        return IPS_ALERT;

    if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1')
    {
        OSX_ROT_LOGF_ERROR("MoveRotator: command '%s' failed", cmd);
        return IPS_ALERT;
    }

    return IPS_BUSY;
}

bool OnStepXRotator::abortRotator()
{
    return m_comm->sendCommandBlind(":rQ#");
}

IPState OnStepXRotator::homeRotator()
{
    m_comm->sendCommandBlind(":rC#");
    return IPS_BUSY;
}

bool OnStepXRotator::setBacklash(int32_t steps)
{
    char cmd[24], reply[8];
    snprintf(cmd, sizeof(cmd), ":rb%d#", steps);
    if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1')
        return false;
    return true;
}

// ---------------------------------------------------------------------------
// pollStatus — returns angle and motion state for device class to apply
// ---------------------------------------------------------------------------
OnStepXRotator::PollResult OnStepXRotator::pollStatus()
{
    PollResult result;
    char reply[32];

    // Read angle
    if (m_comm->sendCommand(":rG#", reply))
    {
        double angle = 0;
        if (parseAngle(reply, angle))
        {
            result.angle      = angle;
            result.angleValid = true;
        }
    }

    // Read status
    if (m_comm->sendCommand(":rT#", reply))
    {
        result.moving      = (reply[0] == 'M');
        result.statusValid = true;
    }

    return result;
}

// ---------------------------------------------------------------------------
// readInitial — called once on connect; returns angle and backlash
// ---------------------------------------------------------------------------
OnStepXRotator::InitialState OnStepXRotator::readInitial()
{
    InitialState state;
    char reply[32];

    // Current angle
    if (m_comm->sendCommand(":rG#", reply))
    {
        double angle = 0;
        if (parseAngle(reply, angle))
        {
            state.angle      = angle;
            state.angleValid = true;
        }
    }

    // Backlash
    if (m_comm->sendCommand(":rb#", reply))
    {
        char *end;
        long bl = std::strtol(reply, &end, 10);
        if (end != reply)
        {
            state.backlash      = static_cast<int32_t>(bl);
            state.backlashValid = true;
        }
    }

    return state;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

// Parse sexagesimal "DDD:MM:SS.S" angle into decimal degrees
bool OnStepXRotator::parseAngle(const char *reply, double &angleDeg)
{
    double val = 0;
    if (f_scansexa(reply, &val) < 0)
        return false;
    angleDeg = val;
    return true;
}

// Format decimal degrees into ":rS±DDD:MM:SS#"
bool OnStepXRotator::formatAngle(double angleDeg, char *buf, int bufLen)
{
    // Clamp to 0-360
    while (angleDeg < 0)    angleDeg += 360.0;
    while (angleDeg >= 360) angleDeg -= 360.0;

    int d, m, s;
    getSexComponents(angleDeg, &d, &m, &s);
    int written = snprintf(buf, bufLen, ":rS%03d:%02d:%02d#", d, m, s);
    return written > 0 && written < bufLen;
}
