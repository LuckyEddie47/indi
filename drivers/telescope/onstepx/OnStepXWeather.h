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

#pragma once

#include <indiapi.h>   // IPState

class OnStepXComm;

// Encapsulates the weather sensor poll sequence shared by both binaries.
// Plain C++ — no INDI base class.  Holds borrowed pointer (non-owning).
//
// WeatherInterface::setParameterValue() is protected, so this helper
// cannot call it directly.  Instead, readSensors() returns a SensorData
// struct; the caller (which IS a WeatherInterface subclass) applies the
// values via setParameterValue().
//
// Sensor map:
//   :GX9A# -> WEATHER_TEMPERATURE   (ambient, °C)
//   :GX9B# -> WEATHER_PRESSURE      (hPa)
//   :GX9C# -> WEATHER_HUMIDITY      (%)
//   :GX9E# -> WEATHER_DEWPOINT      (°C)
//   :GX9F# -> OSX_MCU_TEMP          (MCU temperature, °C — optional)

struct WeatherReading
{
    bool   ok    { false };
    double value { 0.0 };
};

struct SensorData
{
    WeatherReading temp;
    WeatherReading pressure;
    WeatherReading humidity;
    WeatherReading dewpoint;
    WeatherReading mcuTemp;

    bool anyOk() const
    {
        return temp.ok || pressure.ok || humidity.ok || dewpoint.ok || mcuTemp.ok;
    }
};

class OnStepXWeather
{
    public:
        void setComm(OnStepXComm *comm) { m_comm = comm; }

        // Query all weather sensors and return parsed values.
        // Caller must apply results via WeatherInterface::setParameterValue().
        SensorData readSensors(bool hasMcuTemp);

    private:
        OnStepXComm *m_comm { nullptr };

        // Try reading one sensor command; returns a WeatherReading with
        // ok=true on success.  Returns ok=false if the reply is absent or
        // non-numeric (OnStepX returns "CE_0" or similar for missing sensors).
        WeatherReading tryRead(const char *cmd);
};
