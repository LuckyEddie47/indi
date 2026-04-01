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

#include <cstdlib>

// ---------------------------------------------------------------------------
// tryRead
// ---------------------------------------------------------------------------
WeatherReading OnStepXWeather::tryRead(const char *cmd)
{
    char reply[64];
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
