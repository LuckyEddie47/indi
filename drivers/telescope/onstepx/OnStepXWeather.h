/*
    OnStep X INDI Driver — Weather sensor helper (shared by both binaries)

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

    Plain C++ helper -- no INDI base class.  Holds borrowed pointer (non-owning).
    WeatherInterface::setParameterValue() is protected; readSensors() returns a
    SensorData struct so the caller (a WeatherInterface subclass) applies the
    values directly.  Missing or non-numeric replies are returned with ok=false.

    Protocol (OnStepX v10.24c):
      :GX9A# — ambient temperature (deg C, float)
      :GX9B# — barometric pressure (hPa, float)
      :GX9C# — relative humidity (%, float)
      :GX9E# — dew point (deg C, float)
      :GX9F# — MCU temperature (deg C, float; optional, requires hasMcuTemp)

    INDI Properties (Weather tab, via WeatherInterface):
      WEATHER_TEMPERATURE  IP_RO
      WEATHER_PRESSURE     IP_RO
      WEATHER_HUMIDITY     IP_RO
      WEATHER_DEWPOINT     IP_RO
      OSX_MCU_TEMP         IP_RO  (hasMcuTemp only)
*/

#pragma once

#include <indiapi.h>   // IPState

class OnStepXComm;

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
