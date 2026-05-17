/*
    OnStep X INDI Driver — Alignment helper

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include "OnStepXAlignment.h"
#include "OnStepXComm.h"

#include <defaultdevice.h>
#include <indilogger.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#define ALIGN_TAB "Alignment"

// Control switch indices
enum { CTRL_START = 0, CTRL_ACCEPT = 1 };

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXAlignment::initProperties()
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    // Star count selector
    m_starsSP[0].fill("STARS_1", "1 Star",  ISS_ON);
    m_starsSP[1].fill("STARS_2", "2 Stars", ISS_OFF);
    m_starsSP[2].fill("STARS_3", "3 Stars", ISS_OFF);
    m_starsSP[3].fill("STARS_4", "4 Stars", ISS_OFF);
    m_starsSP[4].fill("STARS_5", "5 Stars", ISS_OFF);
    m_starsSP[5].fill("STARS_6", "6 Stars", ISS_OFF);
    m_starsSP[6].fill("STARS_7", "7 Stars", ISS_OFF);
    m_starsSP[7].fill("STARS_8", "8 Stars", ISS_OFF);
    m_starsSP[8].fill("STARS_9", "9 Stars", ISS_OFF);
    m_starsSP.fill(dev, "OSX_ALIGN_STARS", "Star Count", ALIGN_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Start / Accept Star (momentary)
    m_controlSP[CTRL_START].fill("START",  "Start",       ISS_OFF);
    m_controlSP[CTRL_ACCEPT].fill("ACCEPT","Accept Star",  ISS_OFF);
    m_controlSP.fill(dev, "OSX_ALIGN_CONTROL", "Alignment Control", ALIGN_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Write to EEPROM (momentary)
    m_writeSP[0].fill("WRITE", "Write to EEPROM", ISS_OFF);
    m_writeSP.fill(dev, "OSX_ALIGN_WRITE", "Save Alignment", ALIGN_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Status display
    m_statusTP[0].fill("STATUS",       "Status",       "Not started");
    m_statusTP[1].fill("MAX_STARS",    "Max Stars",    "");
    m_statusTP[2].fill("CURRENT_STAR", "Current Star", "");
    m_statusTP[3].fill("TARGET_STARS", "Target Stars", "");
    m_statusTP.fill(dev, "OSX_ALIGN_STATUS", "Alignment Status", ALIGN_TAB, IP_RO, 60, IPS_IDLE);

    // Polar error display
    m_errorTP[0].fill("POLAR_ALT", "Polar Error Alt (\")", "");
    m_errorTP[1].fill("POLAR_AZ",  "Polar Error Az (\")",  "");
    m_errorTP.fill(dev, "OSX_ALIGN_ERROR", "Polar Error", ALIGN_TAB, IP_RO, 60, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXAlignment::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_starsSP);
        m_dev->defineProperty(m_controlSP);
        m_dev->defineProperty(m_writeSP);
        m_dev->defineProperty(m_statusTP);
        m_dev->defineProperty(m_errorTP);
        updateStatus();
        updatePolarError();
    }
    else
    {
        m_dev->deleteProperty(m_starsSP);
        m_dev->deleteProperty(m_controlSP);
        m_dev->deleteProperty(m_writeSP);
        m_dev->deleteProperty(m_statusTP);
        m_dev->deleteProperty(m_errorTP);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch
// ---------------------------------------------------------------------------
bool OnStepXAlignment::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    // Star count selector
    if (m_starsSP.isNameMatch(name))
    {
        m_starsSP.update(states, names, n);
        m_starsSP.setState(IPS_OK);
        m_starsSP.apply();
        return true;
    }

    // Start / Accept
    if (m_controlSP.isNameMatch(name))
    {
        m_controlSP.update(states, names, n);
        int idx = m_controlSP.findOnSwitchIndex();
        bool ok = false;

        if (idx == CTRL_START)
        {
            int stars = m_starsSP.findOnSwitchIndex() + 1;  // 1-based
            ok = startAlignment(stars);
        }
        else if (idx == CTRL_ACCEPT)
        {
            ok = acceptStar();
        }

        m_controlSP.reset();
        m_controlSP.setState(ok ? IPS_OK : IPS_ALERT);
        m_controlSP.apply();
        if (ok)
            updateStatus();
        return true;
    }

    // Write to EEPROM
    if (m_writeSP.isNameMatch(name))
    {
        m_writeSP.update(states, names, n);
        bool ok = writeAlignment();
        m_writeSP.reset();
        m_writeSP.setState(ok ? IPS_OK : IPS_ALERT);
        m_writeSP.apply();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// saveConfig — nothing persistent; firmware holds alignment state
// ---------------------------------------------------------------------------
void OnStepXAlignment::saveConfig(FILE * /*fp*/)
{
}

// ---------------------------------------------------------------------------
// updateStatus — :A?# -> status text fields
// ---------------------------------------------------------------------------
bool OnStepXAlignment::updateStatus()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE] {};
    if (!m_comm->sendCommand(":A?#", reply))
        return false;

    // Reply is 3 chars "mno": max, current, target star counts.
    // Character ':' represents index 9 (ASCII value after '9').
    int maxStars    = (reply[0] == ':') ? 9 : (reply[0] - '0');
    int currentStar = (reply[1] == ':') ? 9 : (reply[1] - '0');
    int targetStars = (reply[2] == ':') ? 9 : (reply[2] - '0');

    // Describe status
    const char *statusStr = "Ready";
    if (targetStars == 0)
        statusStr = "Not started";
    else if (currentStar < targetStars)
        statusStr = "In progress";
    else
        statusStr = "Complete";

    char maxBuf[8], curBuf[8], tgtBuf[8];
    snprintf(maxBuf, sizeof(maxBuf), "%d", maxStars);
    snprintf(curBuf, sizeof(curBuf), "%d", currentStar);
    snprintf(tgtBuf, sizeof(tgtBuf), "%d", targetStars);

    m_statusTP[0].setText(statusStr);
    m_statusTP[1].setText(maxBuf);
    m_statusTP[2].setText(curBuf);
    m_statusTP[3].setText(tgtBuf);
    m_statusTP.setState(IPS_OK);
    m_statusTP.apply();

    return true;
}

// ---------------------------------------------------------------------------
// updatePolarError — :GX02# and :GX03#
// ---------------------------------------------------------------------------
void OnStepXAlignment::updatePolarError()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];

    if (m_comm->sendCommand(":GX02#", reply))
    {
        m_errorTP[0].setText(reply);
    }

    if (m_comm->sendCommand(":GX03#", reply))
    {
        m_errorTP[1].setText(reply);
    }

    m_errorTP.setState(IPS_OK);
    m_errorTP.apply();
}

// ---------------------------------------------------------------------------
// startAlignment — :A[n]# reply '0' on success
// ---------------------------------------------------------------------------
bool OnStepXAlignment::startAlignment(int stars)
{
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":A%d#", stars);
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    bool ok = m_comm->sendCommand(cmd, reply) && reply[0] == '0';
    if (!ok)
        LOGF_ERROR("Start alignment with %d stars failed", stars);
    return ok;
}

// ---------------------------------------------------------------------------
// acceptStar — :A+# reply '0' on success
// ---------------------------------------------------------------------------
bool OnStepXAlignment::acceptStar()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    bool ok = m_comm->sendCommand(":A+#", reply) && reply[0] == '0';
    if (!ok)
        LOG_ERROR("Accept star command rejected by firmware");
    return ok;
}

// ---------------------------------------------------------------------------
// writeAlignment — :AW# reply '1' on success
// ---------------------------------------------------------------------------
bool OnStepXAlignment::writeAlignment()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    bool ok = m_comm->sendCommand(":AW#", reply) && reply[0] == '1';
    if (ok)
        LOG_INFO("Alignment written to EEPROM");
    else
        LOG_ERROR("Write alignment to EEPROM failed");
    return ok;
}
