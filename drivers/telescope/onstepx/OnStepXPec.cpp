/*
    OnStep X INDI Driver — PEC helper

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include "OnStepXPec.h"
#include "OnStepXComm.h"

#include <defaultdevice.h>
#include <indilogger.h>

#include <cstdlib>
#include <cstring>

#define PEC_TAB "PEC"

// Logging helpers
#define OSX_PEC_LOGF(priority, fmt, ...) \
    do { \
        if (m_dev) \
            INDI::Logger::getInstance().print(m_dev->getDeviceName(), priority, __FILE__, __LINE__, \
                                              fmt, ##__VA_ARGS__); \
    } while (0)
#define OSX_PEC_LOG_INFO(msg)           OSX_PEC_LOGF(INDI::Logger::DBG_SESSION, "%s", msg)
#define OSX_PEC_LOGF_WARN(fmt, ...)     OSX_PEC_LOGF(INDI::Logger::DBG_WARNING, fmt, ##__VA_ARGS__)
#define OSX_PEC_LOGF_ERROR(fmt, ...)    OSX_PEC_LOGF(INDI::Logger::DBG_ERROR,   fmt, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Control switch indices
// ---------------------------------------------------------------------------
enum { CTRL_PLAY = 0, CTRL_STOP = 1, CTRL_READY_REC = 2, CTRL_CLEAR = 3, CTRL_SAVE = 4 };

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXPec::initProperties()
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    // State lights
    m_stateSP[0].fill("IGNORED",      "Ignored");
    m_stateSP[1].fill("READY_PLAY",   "Ready to Play");
    m_stateSP[2].fill("PLAYING",      "Playing");
    m_stateSP[3].fill("READY_RECORD", "Ready to Record");
    m_stateSP[4].fill("RECORDING",    "Recording");
    m_stateSP.fill(dev, "OSX_PEC_STATE", "PEC State", PEC_TAB, IPS_IDLE);

    // Index detection lights
    m_indexSP[0].fill("INDEX_NO",  "Not Detected");
    m_indexSP[1].fill("INDEX_YES", "Detected");
    m_indexSP.fill(dev, "OSX_PEC_INDEX", "Worm Index", PEC_TAB, IPS_IDLE);

    // Control switches (momentary — all off after action)
    m_controlSP[CTRL_PLAY].fill("PLAY",       "Play",            ISS_OFF);
    m_controlSP[CTRL_STOP].fill("STOP",       "Stop",            ISS_OFF);
    m_controlSP[CTRL_READY_REC].fill("READY_RECORD", "Ready Record", ISS_OFF);
    m_controlSP[CTRL_CLEAR].fill("CLEAR",     "Clear Buffer",    ISS_OFF);
    m_controlSP[CTRL_SAVE].fill("SAVE",       "Save to EEPROM",  ISS_OFF);
    m_controlSP.fill(dev, "OSX_PEC_CONTROL", "PEC Control", PEC_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Worm steps
    m_wormNP[0].fill("WORM_STEPS", "Steps/Revolution", "%.0f", 0, 999999, 1, 0);
    m_wormNP.fill(dev, "OSX_PEC_WORM_STEPS", "Worm Period", PEC_TAB, IP_RO, 60, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXPec::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_stateSP);
        m_dev->defineProperty(m_indexSP);
        m_dev->defineProperty(m_controlSP);
        m_dev->defineProperty(m_wormNP);
        readWormSteps();
    }
    else
    {
        m_dev->deleteProperty(m_stateSP);
        m_dev->deleteProperty(m_indexSP);
        m_dev->deleteProperty(m_controlSP);
        m_dev->deleteProperty(m_wormNP);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch
// ---------------------------------------------------------------------------
bool OnStepXPec::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    if (!m_controlSP.isNameMatch(name))
        return false;

    m_controlSP.update(states, names, n);
    int idx = m_controlSP.findOnSwitchIndex();

    const char *cmd = nullptr;
    switch (idx)
    {
        case CTRL_PLAY:      cmd = ":$QZ+#"; break;
        case CTRL_STOP:      cmd = ":$QZ-#"; break;
        case CTRL_READY_REC: cmd = ":$QZ/#"; break;
        case CTRL_CLEAR:     cmd = ":$QZZ#"; break;
        case CTRL_SAVE:      cmd = ":$QZ!#"; break;
        default:
            m_controlSP.setState(IPS_ALERT);
            m_controlSP.apply();
            return true;
    }

    char reply[8];
    bool ok = m_comm->sendCommand(cmd, reply) && reply[0] == '1';
    m_controlSP.reset();   // momentary — all off after sending
    m_controlSP.setState(ok ? IPS_OK : IPS_ALERT);
    m_controlSP.apply();

    if (!ok)
        OSX_PEC_LOGF_ERROR("PEC command '%s' rejected by firmware", cmd);

    return true;
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXPec::saveConfig(FILE * /*fp*/)
{
    // Nothing persistent for PEC; state is always read from firmware on connect
}

// ---------------------------------------------------------------------------
// pollStatus — :$QZ?# -> state + index
// ---------------------------------------------------------------------------
void OnStepXPec::pollStatus()
{
    char reply[16];
    if (!m_comm->sendCommand(":$QZ?#", reply))
        return;

    char state = reply[0];
    bool indexDetected = (reply[1] == '.');

    // Clear all state lights to IDLE first
    for (int i = 0; i < 5; i++)
        m_stateSP[i].setState(IPS_IDLE);

    switch (state)
    {
        case 'I':
            m_stateSP[0].setState(IPS_OK);
            m_stateSP.setState(IPS_IDLE);
            break;
        case 'p':
            m_stateSP[1].setState(IPS_BUSY);
            m_stateSP.setState(IPS_BUSY);
            break;
        case 'P':
            m_stateSP[2].setState(IPS_BUSY);
            m_stateSP.setState(IPS_BUSY);
            break;
        case 'r':
            m_stateSP[3].setState(IPS_BUSY);
            m_stateSP.setState(IPS_BUSY);
            break;
        case 'R':
            m_stateSP[4].setState(IPS_BUSY);
            m_stateSP.setState(IPS_BUSY);
            break;
        default:
            m_stateSP.setState(IPS_ALERT);
            break;
    }
    m_stateSP.apply();

    // Clear index lights
    m_indexSP[0].setState(IPS_IDLE);
    m_indexSP[1].setState(IPS_IDLE);
    if (indexDetected)
    {
        m_indexSP[1].setState(IPS_OK);
        m_indexSP.setState(IPS_OK);
    }
    else
    {
        m_indexSP[0].setState(IPS_IDLE);
        m_indexSP.setState(IPS_IDLE);
    }
    m_indexSP.apply();
}

// ---------------------------------------------------------------------------
// readWormSteps — :VW# -> worm period steps
// ---------------------------------------------------------------------------
void OnStepXPec::readWormSteps()
{
    char reply[32];
    if (!m_comm->sendCommand(":VW#", reply))
        return;

    char *end;
    long val = std::strtol(reply, &end, 10);
    if (end == reply || val <= 0)
        return;

    m_wormNP[0].setValue(static_cast<double>(val));
    m_wormNP.setState(IPS_OK);
    m_wormNP.apply();
}
