/*
    OnStep X INDI Driver — Weather sensor helper (both binaries)

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

#include "OnStepXWeather.h"
#include "OnStepXComm.h"

#include <defaultdevice.h>

#include <cstdio>
#include <cstdlib>

#define WEATHER_TAB "Weather"

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXWeather::initProperties()
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    // --- OSX_WEATHER_SET ---
    m_weatherSetNP[0].fill("WEATHER_TEMP_OFFSET",     "Temp Offset (C)",      "%.2f", -10.0, 10.0, 0.1, 0.0);
    m_weatherSetNP[1].fill("WEATHER_PRESSURE_OFFSET", "Pressure Offset (hPa)","%.2f", -50.0, 50.0, 0.5, 0.0);
    m_weatherSetNP[2].fill("WEATHER_HUMIDITY_OFFSET", "Humidity Offset (%)",  "%.2f", -20.0, 20.0, 0.5, 0.0);
    m_weatherSetNP.fill(dev, "OSX_WEATHER_SET", "Weather Calibration",
                        WEATHER_TAB, IP_RW, 60, IPS_IDLE);

    // --- OSX_DUT1 ---
    m_dut1NP[0].fill("DUT1", "DUT1 (UT1-UTC, s)", "%.3f", -1.0, 1.0, 0.001, 0.0);
    m_dut1NP.fill(dev, "OSX_DUT1", "DUT1 Correction",
                  WEATHER_TAB, IP_RW, 60, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXWeather::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_weatherSetNP);
        m_dev->defineProperty(m_dut1NP);
    }
    else
    {
        m_dev->deleteProperty(m_weatherSetNP);
        m_dev->deleteProperty(m_dut1NP);
    }
}

// ---------------------------------------------------------------------------
// handleNumber
// ---------------------------------------------------------------------------
bool OnStepXWeather::handleNumber(const char *name, double values[], char *names[], int n)
{
    if (m_weatherSetNP.isNameMatch(name))
    {
        m_weatherSetNP.update(values, names, n);

        // Send each offset to firmware: :SX9A,[v]# / :SX9B,[v]# / :SX9C,[v]#
        const char *cmds[3] = { ":SX9A,%.2f#", ":SX9B,%.2f#", ":SX9C,%.2f#" };
        bool allOk = true;
        for (int i = 0; i < 3; i++)
        {
            char cmd[OnStepXComm::CMD_MAX_LEN];
            snprintf(cmd, sizeof(cmd), cmds[i], m_weatherSetNP[i].getValue());
            char reply[OnStepXComm::REPLY_BUF_SIZE];
            if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1')
                allOk = false;
        }
        m_weatherSetNP.setState(allOk ? IPS_OK : IPS_ALERT);
        m_weatherSetNP.apply();
        return true;
    }

    if (m_dut1NP.isNameMatch(name))
    {
        m_dut1NP.update(values, names, n);
        double dut1 = m_dut1NP[0].getValue();

        char cmd[OnStepXComm::CMD_MAX_LEN];
        snprintf(cmd, sizeof(cmd), ":SU%+.3f#", dut1);
        char reply[OnStepXComm::REPLY_BUF_SIZE];
        if (m_comm->sendCommand(cmd, reply) && reply[0] == '1')
            m_dut1NP.setState(IPS_OK);
        else
            m_dut1NP.setState(IPS_ALERT);

        m_dut1NP.apply();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXWeather::saveConfig(FILE *fp)
{
    m_weatherSetNP.save(fp);
    m_dut1NP.save(fp);
}

// ---------------------------------------------------------------------------
// tryRead
// ---------------------------------------------------------------------------
WeatherReading OnStepXWeather::tryRead(const char *cmd)
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    if (!m_comm->sendCommand(cmd, reply))
        return {};

    // OnStepX returns "CE_0", "ERR", or similar for absent/unsupported sensors.
    // Accept only strings that begin with a digit, '-', '+', or '.'.
    if (reply[0] == '\0')
        return {};
    if ((reply[0] < '0' || reply[0] > '9') &&
        reply[0] != '-' && reply[0] != '+' && reply[0] != '.')
        return {};

    char *end;
    double val = std::strtod(reply, &end);
    if (end == reply)
        return {};

    return { true, val };
}

// ---------------------------------------------------------------------------
// readSensors
// ---------------------------------------------------------------------------
SensorData OnStepXWeather::readSensors(bool hasMcuTemp)
{
    SensorData data;

    data.temp     = tryRead(":GX9A#");
    data.pressure = tryRead(":GX9B#");
    data.humidity = tryRead(":GX9C#");
    data.dewpoint = tryRead(":GX9E#");

    // MCU temperature is optional — probe result determines whether to query.
    if (hasMcuTemp)
        data.mcuTemp = tryRead(":GX9F#");

    return data;
}
