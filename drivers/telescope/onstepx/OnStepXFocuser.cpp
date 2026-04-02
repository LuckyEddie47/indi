/*
    OnStep X INDI Driver — Focuser device

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

#include "OnStepXFocuser.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>

#define FOCUS_TAB "Focuser"

OnStepXFocuser::OnStepXFocuser(int slot) : m_slot(slot)
{
    snprintf(m_name, sizeof(m_name), "OnStep X Focuser %d", slot);

    // No Serial/TCP connection — parent device activates us
    setSupportedConnections(CONNECTION_NONE);

    FI::SetCapability(FOCUSER_CAN_ABS_MOVE  |
                      FOCUSER_CAN_REL_MOVE  |
                      FOCUSER_CAN_ABORT     |
                      FOCUSER_CAN_SYNC      |
                      FOCUSER_HAS_BACKLASH);
}

// ---------------------------------------------------------------------------
// DefaultDevice interface
// ---------------------------------------------------------------------------
const char *OnStepXFocuser::getDefaultName()
{
    return m_name;
}

bool OnStepXFocuser::initProperties()
{
    // This call chain: Focuser::initProperties -> DefaultDevice::initProperties
    // -> registers Connection switch, then FI::initProperties registers focuser
    // properties.  Since CONNECTION_NONE is set, no Serial/TCP UI is added.
    INDI::Focuser::initProperties();

    // Override max position to a sensible default (100 000 microns = 100 mm)
    FocusMaxPosNP[0].setValue(100000);

    // Custom temperature property
    m_temperatureNP[0].fill("FOCUS_TEMPERATURE", "Temperature (C)", "%.1f", -50, 100, 0, 0);
    m_temperatureNP.fill(getDeviceName(), "FOCUS_TEMPERATURE_NP",
                         "Temperature", FOCUS_TAB, IP_RO, 60, IPS_IDLE);

    return true;
}

bool OnStepXFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(m_temperatureNP);

        // Read and apply initial values from firmware
        uint32_t maxPos = 0;
        if (cmdGetMax(maxPos))
        {
            FocusMaxPosNP[0].setValue(maxPos);
            FocusMaxPosNP.apply();
        }

        uint32_t curPos = 0;
        if (cmdGetPos(curPos))
        {
            FocusAbsPosNP[0].setValue(curPos);
            FocusAbsPosNP.setState(IPS_OK);
            FocusAbsPosNP.apply();
        }

        double tempC = 0;
        if (cmdGetTemperature(tempC))
        {
            m_temperatureNP[0].setValue(tempC);
            m_temperatureNP.setState(IPS_OK);
            m_temperatureNP.apply();
        }

        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(m_temperatureNP);
    }

    return true;
}

bool OnStepXFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);
    return true;
}

// ---------------------------------------------------------------------------
// Connection — no port; parent calls setConnected(true) for us
// ---------------------------------------------------------------------------
bool OnStepXFocuser::Connect()
{
    if (!m_comm)
    {
        LOG_ERROR("Focuser: parent device not connected. Connect the mount/controller first.");
        return false;
    }
    return true;
}

bool OnStepXFocuser::Disconnect()
{
    return true;
}

// ---------------------------------------------------------------------------
// TimerHit — check for move completion
// ---------------------------------------------------------------------------
void OnStepXFocuser::TimerHit()
{
    if (!isConnected())
        return;

    if (m_moving)
    {
        bool moving = false;
        if (cmdGetStatus(moving))
        {
            if (!moving)
            {
                m_moving = false;
                uint32_t pos = 0;
                if (cmdGetPos(pos))
                {
                    FocusAbsPosNP[0].setValue(pos);
                    FocusAbsPosNP.setState(IPS_OK);
                    FocusAbsPosNP.apply();
                }
                else
                {
                    FocusAbsPosNP.setState(IPS_OK);
                    FocusAbsPosNP.apply();
                }
            }
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

// ---------------------------------------------------------------------------
// pollStatus — called by parent every ~5 s to update position and temperature
// ---------------------------------------------------------------------------
void OnStepXFocuser::pollStatus()
{
    if (!isConnected() || !m_comm)
        return;

    uint32_t pos = 0;
    if (!m_moving && cmdGetPos(pos))
    {
        FocusAbsPosNP[0].setValue(pos);
        FocusAbsPosNP.setState(IPS_OK);
        FocusAbsPosNP.apply();
    }

    double tempC = 0;
    if (cmdGetTemperature(tempC))
    {
        m_temperatureNP[0].setValue(tempC);
        m_temperatureNP.setState(IPS_OK);
        m_temperatureNP.apply();
    }
}

// ---------------------------------------------------------------------------
// FocuserInterface virtuals
// ---------------------------------------------------------------------------
IPState OnStepXFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    if (!cmdGoto(targetTicks))
    {
        LOG_ERROR("MoveAbsFocuser: command failed");
        return IPS_ALERT;
    }
    m_targetPos = targetTicks;
    m_moving    = true;
    FocusAbsPosNP.setState(IPS_BUSY);
    FocusAbsPosNP.apply();
    return IPS_BUSY;
}

IPState OnStepXFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t delta = (dir == FOCUS_INWARD) ? (int32_t)ticks : -(int32_t)ticks;
    if (!cmdMoveRel(delta))
    {
        LOG_ERROR("MoveRelFocuser: command failed");
        return IPS_ALERT;
    }
    m_moving = true;
    FocusAbsPosNP.setState(IPS_BUSY);
    FocusAbsPosNP.apply();
    return IPS_BUSY;
}

bool OnStepXFocuser::AbortFocuser()
{
    bool ok = cmdAbort();
    m_moving = false;
    FocusAbsPosNP.setState(IPS_IDLE);
    FocusAbsPosNP.apply();
    return ok;
}

bool OnStepXFocuser::SyncFocuser(uint32_t ticks)
{
    // :FN[n]# with current position effectively syncs (no dedicated sync command)
    // We use a goto to the same position; firmware will accept it.
    // If firmware has a dedicated sync command (:FK# etc.) it can be swapped in.
    return cmdGoto(ticks);
}

bool OnStepXFocuser::SetFocuserBacklash(int32_t steps)
{
    return cmdSetBacklash(steps);
}

bool OnStepXFocuser::SetFocuserBacklashEnabled(bool enabled)
{
    // OnStepX doesn't have a separate backlash-enable toggle; treat non-zero
    // backlash as enabled.  Send 0 when disabled.
    if (!enabled)
        return cmdSetBacklash(0);
    return true;
}

bool OnStepXFocuser::SetFocuserSpeed(int speed)
{
    return cmdSetSpeed(speed);
}

// ---------------------------------------------------------------------------
// Protocol helpers
// ---------------------------------------------------------------------------
bool OnStepXFocuser::cmdGetPos(uint32_t &pos)
{
    char reply[64];
    if (!m_comm->sendCommandFocuser(m_slot, ":FG#", reply))
        return false;
    char *end;
    long val = std::strtol(reply, &end, 10);
    if (end == reply || val < 0)
        return false;
    pos = static_cast<uint32_t>(val);
    return true;
}

bool OnStepXFocuser::cmdGetMax(uint32_t &maxPos)
{
    char reply[64];
    if (!m_comm->sendCommandFocuser(m_slot, ":FM#", reply))
        return false;
    char *end;
    long val = std::strtol(reply, &end, 10);
    if (end == reply || val <= 0)
        return false;
    maxPos = static_cast<uint32_t>(val);
    return true;
}

bool OnStepXFocuser::cmdGetStatus(bool &moving)
{
    char reply[8];
    if (!m_comm->sendCommandFocuser(m_slot, ":FT#", reply))
        return false;
    // 'M' = moving; anything else (including 'S', '0', 'H') = stopped
    moving = (reply[0] == 'M');
    return true;
}

bool OnStepXFocuser::cmdGetTemperature(double &tempC)
{
    char reply[32];
    if (!m_comm->sendCommandFocuser(m_slot, ":Ft#", reply))
        return false;
    char *end;
    double val = std::strtod(reply, &end);
    if (end == reply)
        return false;
    // OnStepX returns 999 when no temperature sensor is attached
    if (val > 990.0)
        return false;
    tempC = val;
    return true;
}

bool OnStepXFocuser::cmdGoto(uint32_t microns)
{
    char cmd[24], reply[8];
    snprintf(cmd, sizeof(cmd), ":FN%u#", microns);
    if (!m_comm->sendCommandFocuser(m_slot, cmd, reply))
        return false;
    return reply[0] == '1';
}

bool OnStepXFocuser::cmdMoveRel(int32_t microns)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":Fm%d#", microns);
    return m_comm->sendCommandBlindFocuser(m_slot, cmd);
}

bool OnStepXFocuser::cmdAbort()
{
    return m_comm->sendCommandBlindFocuser(m_slot, ":FQ#");
}

bool OnStepXFocuser::cmdSetBacklash(int32_t steps)
{
    char cmd[24], reply[8];
    snprintf(cmd, sizeof(cmd), ":FB%d#", steps);
    if (!m_comm->sendCommandFocuser(m_slot, cmd, reply))
        return false;
    return reply[0] == '1';
}

bool OnStepXFocuser::cmdSetSpeed(int speed)
{
    char cmd[16], reply[8];
    snprintf(cmd, sizeof(cmd), ":FP%d#", speed);
    if (!m_comm->sendCommandFocuser(m_slot, cmd, reply))
        return false;
    return reply[0] == '1';
}
