/*
    OnStep X INDI Driver — Pulse guiding and guide rate helper (mount binary only)

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

#include "OnStepXGuide.h"
#include "OnStepXComm.h"

#include <cstdio>
#include <cstdlib>

// "Motion Control" — the standard INDI tab shared with GuiderInterface properties
#define MOTION_TAB "Motion Control"

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXGuide::initProperties()
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    m_guideRateNP[0].fill("GUIDE_RATE_RA",  "RA (x sidereal)",  "%.2f", 0.0, 1.0, 0.01, 0.5);
    m_guideRateNP[1].fill("GUIDE_RATE_DEC", "Dec (x sidereal)", "%.2f", 0.0, 1.0, 0.01, 0.5);
    m_guideRateNP.fill(dev, "OSX_GUIDE_RATE", "Guide Rate",
                       MOTION_TAB, IP_RW, 60, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXGuide::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_guideRateNP);
        readGuideRate();
    }
    else
    {
        m_dev->deleteProperty(m_guideRateNP);
    }
}

// ---------------------------------------------------------------------------
// handleNumber — OSX_GUIDE_RATE write
// ---------------------------------------------------------------------------
bool OnStepXGuide::handleNumber(const char *name, double values[], char *names[], int n)
{
    if (!m_guideRateNP.isNameMatch(name))
        return false;

    m_guideRateNP.update(values, names, n);
    double rate = m_guideRateNP[0].getValue();  // RA element drives the firmware index

    // Map 0.0-1.0 to integer index 0-9
    int idx = static_cast<int>(rate * 9.0 + 0.5);
    if (idx < 0) idx = 0;
    if (idx > 9) idx = 9;

    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":R%d#", idx);
    m_comm->sendCommandBlind(cmd);

    // Mirror the same value to Dec until a separate Dec-axis command is confirmed
    m_guideRateNP[1].setValue(rate);
    m_guideRateNP.setState(IPS_OK);
    m_guideRateNP.apply();
    return true;
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXGuide::saveConfig(FILE *fp)
{
    m_guideRateNP.save(fp);
}

// ---------------------------------------------------------------------------
// Guide pulse methods
// ---------------------------------------------------------------------------
IPState OnStepXGuide::guideNorth(uint32_t ms)
{
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":MGn%u#", ms);
    m_comm->sendCommandBlind(cmd);
    m_guideEndNS = Clock::now() + std::chrono::milliseconds(ms);
    m_guidingNS  = true;
    return IPS_BUSY;
}

IPState OnStepXGuide::guideSouth(uint32_t ms)
{
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":MGs%u#", ms);
    m_comm->sendCommandBlind(cmd);
    m_guideEndNS = Clock::now() + std::chrono::milliseconds(ms);
    m_guidingNS  = true;
    return IPS_BUSY;
}

IPState OnStepXGuide::guideEast(uint32_t ms)
{
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":MGe%u#", ms);
    m_comm->sendCommandBlind(cmd);
    m_guideEndWE = Clock::now() + std::chrono::milliseconds(ms);
    m_guidingWE  = true;
    return IPS_BUSY;
}

IPState OnStepXGuide::guideWest(uint32_t ms)
{
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":MGw%u#", ms);
    m_comm->sendCommandBlind(cmd);
    m_guideEndWE = Clock::now() + std::chrono::milliseconds(ms);
    m_guidingWE  = true;
    return IPS_BUSY;
}

// ---------------------------------------------------------------------------
// checkComplete — call every ReadScopeStatus poll
// ---------------------------------------------------------------------------
void OnStepXGuide::checkComplete()
{
    auto now = Clock::now();

    if (m_guidingNS && now >= m_guideEndNS)
    {
        m_guidingNS = false;
        m_gi->GuideComplete(INDI_EQ_AXIS::AXIS_DE);
    }

    if (m_guidingWE && now >= m_guideEndWE)
    {
        m_guidingWE = false;
        m_gi->GuideComplete(INDI_EQ_AXIS::AXIS_RA);
    }
}

// ---------------------------------------------------------------------------
// readGuideRate — query :GX90# and update OSX_GUIDE_RATE
// ---------------------------------------------------------------------------
void OnStepXGuide::readGuideRate()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    if (!m_comm->sendCommand(":GX90#", reply))
        return;

    char *end;
    double rate = std::strtod(reply, &end);
    if (end == reply || rate <= 0.0)
        return;

    m_guideRateNP[0].setValue(rate);
    m_guideRateNP[1].setValue(rate);
    m_guideRateNP.setState(IPS_OK);
    m_guideRateNP.apply();
}
