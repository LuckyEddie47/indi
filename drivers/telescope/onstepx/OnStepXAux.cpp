/*
    OnStep X INDI Driver

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

#include "OnStepXAux.h"

#define WEATHER_TAB "Weather"   // tab name for WeatherInterface properties

OnStepXAux::OnStepXAux() : INDI::WeatherInterface(this)
{
    setVersion(0, 1);
    m_core.setDevice(this);
}

const char *OnStepXAux::getDefaultName()
{
    return "OnStep X Controller";
}

bool OnStepXAux::initProperties()
{
    INDI::DefaultDevice::initProperties();

    WI::initProperties(WEATHER_TAB, WEATHER_TAB);
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)",    -40,  80, 15);
    addParameter("WEATHER_PRESSURE",    "Pressure (hPa)",     800, 1100, 15);
    addParameter("WEATHER_HUMIDITY",    "Humidity (%)",         0,  100, 15);
    addParameter("WEATHER_DEWPOINT",    "Dew Point (C)",      -40,   40, 15);
    addParameter("OSX_MCU_TEMP",        "MCU Temp (C)",       -20,   80, 15);

    m_serialConnection = new Connection::Serial(this);
    m_serialConnection->registerHandshake([&]()
    {
        return Handshake();
    });
    m_serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
    registerConnection(m_serialConnection);

    m_tcpConnection = new Connection::TCP(this);
    m_tcpConnection->setDefaultHost("192.168.4.1");
    m_tcpConnection->setDefaultPort(3131);
    m_tcpConnection->registerHandshake([&]()
    {
        return Handshake();
    });
    registerConnection(m_tcpConnection);

    addAuxControls();

    return true;
}

bool OnStepXAux::updateProperties()
{
    INDI::DefaultDevice::updateProperties();
    WI::updateProperties();
    return true;
}

bool OnStepXAux::Handshake()
{
    if (getActiveConnection() == m_serialConnection)
        m_core.setFd(m_serialConnection->getPortFD());
    else if (getActiveConnection() == m_tcpConnection)
        m_core.setFd(m_tcpConnection->getPortFD());

    if (!m_core.probeController())
    {
        LOG_ERROR("Not an OnStepX controller. Aborting connection.");
        return false;
    }

    m_weather.setComm(&m_core.comm());
    return true;
}

bool OnStepXAux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (WI::processSwitch(dev, name, states, names, n))
        return true;
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXAux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (WI::processNumber(dev, name, values, names, n))
        return true;
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool OnStepXAux::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool OnStepXAux::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    WI::saveConfigItems(fp);
    return true;
}

void OnStepXAux::TimerHit()
{
    if (!isConnected())
        return;

    WI::checkWeatherUpdate();
    SetTimer(getCurrentPollingPeriod());
}

// ---------------------------------------------------------------------------
// updateWeather — WeatherInterface callback
// ---------------------------------------------------------------------------
IPState OnStepXAux::updateWeather()
{
    SensorData data = m_weather.readSensors(m_core.caps().hasMcuTemp);
    if (data.temp.ok)      setParameterValue("WEATHER_TEMPERATURE", data.temp.value);
    if (data.pressure.ok)  setParameterValue("WEATHER_PRESSURE",    data.pressure.value);
    if (data.humidity.ok)  setParameterValue("WEATHER_HUMIDITY",    data.humidity.value);
    if (data.dewpoint.ok)  setParameterValue("WEATHER_DEWPOINT",    data.dewpoint.value);
    if (data.mcuTemp.ok)   setParameterValue("OSX_MCU_TEMP",        data.mcuTemp.value);
    return data.anyOk() ? IPS_OK : IPS_IDLE;
}
