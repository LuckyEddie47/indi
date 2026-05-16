/*
    OnStep X INDI Driver — Limits and Home helper (mount binary only)

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

#include "OnStepXLimits.h"
#include "OnStepXComm.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#define LIMITS_TAB "Limits"
#define HOME_TAB   "Home"

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXLimits::initProperties()
{  
    // --- Home action switch ---
    m_homeActionSP[0].fill("HOME_FIND", "Find Home",    ISS_OFF);
    m_homeActionSP[1].fill("HOME_SET",  "Set Home Here",ISS_OFF);
    m_homeActionSP.fill(m_dev ? m_dev->getDeviceName() : "",
                        "HOME_ACTION", "Home", HOME_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // --- Auto-home on boot switch ---
    m_autoBootSP[0].fill("AUTO_HOME_ON",  "Enabled",  ISS_OFF);
    m_autoBootSP[1].fill("AUTO_HOME_OFF", "Disabled", ISS_ON);
    m_autoBootSP.fill(m_dev ? m_dev->getDeviceName() : "",
                      "AUTO_HOME", "Auto Home on Boot", HOME_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // --- Home axis offsets ---
    m_homeOffsetNP[0].fill("HOME_OFFSET_AX1", "Axis 1 (arcsec)", "%.1f", -3600, 3600, 1, 0);
    m_homeOffsetNP[1].fill("HOME_OFFSET_AX2", "Axis 2 (arcsec)", "%.1f", -3600, 3600, 1, 0);
    m_homeOffsetNP.fill(m_dev ? m_dev->getDeviceName() : "",
                        "HOME_OFFSETS", "Home Offsets", HOME_TAB, IP_RW, 0, IPS_IDLE);

    // --- Horizon limits ---
    m_horizonLimitNP[0].fill("HORIZON_MIN", "Minimum Alt (deg)", "%.1f", -30, 30, 0.5, -10);
    m_horizonLimitNP[1].fill("HORIZON_MAX", "Maximum Alt (deg)", "%.1f",  60, 90, 0.5,  89);
    m_horizonLimitNP.fill(m_dev ? m_dev->getDeviceName() : "",
                          "HORIZON_LIMITS", "Horizon Limits", LIMITS_TAB, IP_RW, 0, IPS_IDLE);

    // --- Meridian limits ---
    m_meridianLimitNP[0].fill("MERIDIAN_EAST", "E of Meridian (min)", "%.0f", -600, 600, 1, 0);
    m_meridianLimitNP[1].fill("MERIDIAN_WEST", "W of Meridian (min)", "%.0f", -600, 600, 1, 0);
    m_meridianLimitNP.fill(m_dev ? m_dev->getDeviceName() : "",
                           "MERIDIAN_LIMITS", "Meridian Limits", LIMITS_TAB, IP_RW, 0, IPS_IDLE);

    // --- Mount backlash ---
    m_backlashNP[0].fill("MOUNT_BACKLASH_AXIS1", "Axis1 RA/Az (arcsec)",  "%.0f", 0, 3600, 1, 0);
    m_backlashNP[1].fill("MOUNT_BACKLASH_AXIS2", "Axis2 Dec/Alt (arcsec)","%.0f", 0, 3600, 1, 0);
    m_backlashNP.fill(m_dev ? m_dev->getDeviceName() : "",
                      "OSX_MOUNT_BACKLASH", "Mount Backlash", LIMITS_TAB, IP_RW, 0, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXLimits::updateProperties(bool connected, bool hasHomeSense)
{
    if (!m_dev) return;

    if (connected)
    {
        m_dev->defineProperty(m_homeActionSP);
        m_dev->defineProperty(m_autoBootSP);
        if (hasHomeSense)
            m_dev->defineProperty(m_homeOffsetNP);
        m_dev->defineProperty(m_horizonLimitNP);
        m_dev->defineProperty(m_meridianLimitNP);
        m_dev->defineProperty(m_backlashNP);

        // Populate limits from device
        readLimits();
    }
    else
    {
        m_dev->deleteProperty(m_homeActionSP);
        m_dev->deleteProperty(m_autoBootSP);
        m_dev->deleteProperty(m_homeOffsetNP);
        m_dev->deleteProperty(m_horizonLimitNP);
        m_dev->deleteProperty(m_meridianLimitNP);
        m_dev->deleteProperty(m_backlashNP);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch
// ---------------------------------------------------------------------------
bool OnStepXLimits::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    if (m_homeActionSP.isNameMatch(name))
    {
        m_homeActionSP.update(states, names, n);
        m_homeActionSP.setState(IPS_BUSY);

        bool ok = false;
        if (m_homeActionSP[0].getState() == ISS_ON)      // Find Home
            ok = homeFind();
        else if (m_homeActionSP[1].getState() == ISS_ON) // Set Home Here
            ok = homeSet();

        m_homeActionSP.setState(ok ? IPS_OK : IPS_ALERT);
        // Reset buttons — one-shot actions
        m_homeActionSP[0].setState(ISS_OFF);
        m_homeActionSP[1].setState(ISS_OFF);
        m_homeActionSP.apply();
        return true;
    }

    if (m_autoBootSP.isNameMatch(name))
    {
        m_autoBootSP.update(states, names, n);
        bool enable = (m_autoBootSP[0].getState() == ISS_ON);
        bool ok = setAutoHome(enable);
        m_autoBootSP.setState(ok ? IPS_OK : IPS_ALERT);
        m_autoBootSP.apply();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// handleNumber
// ---------------------------------------------------------------------------
bool OnStepXLimits::handleNumber(const char *name, double values[], char *names[], int n)
{
    if (m_homeOffsetNP.isNameMatch(name))
    {
        m_homeOffsetNP.update(values, names, n);
        bool ok = writeHomeOffsets(m_homeOffsetNP[0].getValue(),
                                   m_homeOffsetNP[1].getValue());
        m_homeOffsetNP.setState(ok ? IPS_OK : IPS_ALERT);
        m_homeOffsetNP.apply();
        return true;
    }

    if (m_horizonLimitNP.isNameMatch(name))
    {
        m_horizonLimitNP.update(values, names, n);
        char cmd[32];
        bool ok = true;
        snprintf(cmd, sizeof(cmd), ":Sh%d#", static_cast<int>(m_horizonLimitNP[0].getValue()));
        char reply[64];
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        snprintf(cmd, sizeof(cmd), ":So%d#", static_cast<int>(m_horizonLimitNP[1].getValue()));
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        m_horizonLimitNP.setState(ok ? IPS_OK : IPS_ALERT);
        m_horizonLimitNP.apply();
        return true;
    }

    if (m_meridianLimitNP.isNameMatch(name))
    {
        m_meridianLimitNP.update(values, names, n);
        char cmd[32];
        bool ok = true;
        char reply[64];
        snprintf(cmd, sizeof(cmd), ":SXE9,%d#",
                 static_cast<int>(m_meridianLimitNP[0].getValue()));
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        snprintf(cmd, sizeof(cmd), ":SXEA,%d#",
                 static_cast<int>(m_meridianLimitNP[1].getValue()));
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        m_meridianLimitNP.setState(ok ? IPS_OK : IPS_ALERT);
        m_meridianLimitNP.apply();
        return true;
    }

    if (m_backlashNP.isNameMatch(name))
    {
        m_backlashNP.update(values, names, n);
        char cmd[32];
        bool ok = true;
        char reply[64];
        snprintf(cmd, sizeof(cmd), ":$BR%d#", static_cast<int>(m_backlashNP[0].getValue()));
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        snprintf(cmd, sizeof(cmd), ":$BD%d#", static_cast<int>(m_backlashNP[1].getValue()));
        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;
        m_backlashNP.setState(ok ? IPS_OK : IPS_ALERT);
        m_backlashNP.apply();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXLimits::saveConfig(FILE *fp)
{
    m_autoBootSP.save(fp);
    m_homeOffsetNP.save(fp);
    m_horizonLimitNP.save(fp);
    m_meridianLimitNP.save(fp);
    m_backlashNP.save(fp);
}

// ---------------------------------------------------------------------------
// readLimits
// ---------------------------------------------------------------------------
bool OnStepXLimits::readLimits()
{
    char reply[64];
    bool ok = true;

    // Horizon minimum (:Gh# returns integer degrees)
    if (m_comm->sendCommand(":Gh#", reply))
    {
        double v = std::atof(reply);
        m_horizonLimitNP[0].setValue(v);
    }
    else ok = false;

    // Horizon maximum (:Go# returns integer degrees)
    if (m_comm->sendCommand(":Go#", reply))
    {
        double v = std::atof(reply);
        m_horizonLimitNP[1].setValue(v);
    }
    else ok = false;

    m_horizonLimitNP.setState(ok ? IPS_OK : IPS_ALERT);
    m_horizonLimitNP.apply();

    // Meridian East (:GXE9# returns minutes-past-meridian)
    bool ok2 = true;
    if (m_comm->sendCommand(":GXE9#", reply))
    {
        double v = std::atof(reply);
        m_meridianLimitNP[0].setValue(v);
    }
    else ok2 = false;

    // Meridian West (:GXEA# returns minutes-past-meridian)
    if (m_comm->sendCommand(":GXEA#", reply))
    {
        double v = std::atof(reply);
        m_meridianLimitNP[1].setValue(v);
    }
    else ok2 = false;

    m_meridianLimitNP.setState(ok2 ? IPS_OK : IPS_ALERT);
    m_meridianLimitNP.apply();

    // Mount backlash (:%BR# -> Axis1, :%BD# -> Axis2)
    bool ok3 = true;
    if (m_comm->sendCommand(":%BR#", reply))
        m_backlashNP[0].setValue(std::atof(reply));
    else
        ok3 = false;

    if (m_comm->sendCommand(":%BD#", reply))
        m_backlashNP[1].setValue(std::atof(reply));
    else
        ok3 = false;

    m_backlashNP.setState(ok3 ? IPS_OK : IPS_ALERT);
    m_backlashNP.apply();

    return ok && ok2 && ok3;
}

// ---------------------------------------------------------------------------
// homeFind
// ---------------------------------------------------------------------------
bool OnStepXLimits::homeFind()
{
    // :hC# returns '1' on acceptance; firmware begins homing asynchronously.
    char reply[64];
    if (!m_comm->sendCommand(":hC#", reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Home Find (:hC#) failed");
        return false;
    }
    if (m_dev)
        LOG_DEBUG("Homing started -- watch status for 'h' flag");
    return true;
}

// ---------------------------------------------------------------------------
// homeSet
// ---------------------------------------------------------------------------
bool OnStepXLimits::homeSet()
{
    char reply[64];
    if (!m_comm->sendCommand(":hF#", reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Home Set (:hF#) failed");
        return false;
    }
    if (m_dev)
        LOG_DEBUG("Home position set to current position");
    return true;
}

// ---------------------------------------------------------------------------
// setAutoHome
// ---------------------------------------------------------------------------
bool OnStepXLimits::setAutoHome(bool enabled)
{
    const char *cmd = enabled ? ":hA1#" : ":hA0#";
    char reply[64];
    if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Auto home command failed");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// writeHomeOffsets
// ---------------------------------------------------------------------------
bool OnStepXLimits::writeHomeOffsets(double axis1, double axis2)
{
    char cmd[32];
    char reply[64];
    bool ok = true;

    snprintf(cmd, sizeof(cmd), ":hC1,%d#", static_cast<int>(axis1));
    if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;

    snprintf(cmd, sizeof(cmd), ":hC2,%d#", static_cast<int>(axis2));
    if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1') ok = false;

    if (!ok && m_dev)
        LOG_ERROR("Failed to write home offsets");
    return ok;
}
