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

OnStepXAux::OnStepXAux() : INDI::RotatorInterface(this),
    INDI::WeatherInterface(this)
{
    setVersion(0, 1);
    m_core.setDevice(this);
    m_auxFeatures.setDevice(this);
    m_rotator.setDevice(this);
    m_weather.setDevice(this);
    m_usbPorts.setDevice(this);
}

const char *OnStepXAux::getDefaultName()
{
    return "OnStep X Controller";
}

bool OnStepXAux::initProperties()
{
    INDI::DefaultDevice::initProperties();

    RI::initProperties("Rotator");
    m_rotator.initProperties(false);  // hasDerotator known only after Handshake

    m_weather.initProperties();

    // --- OSX_FIRMWARE ---
    m_firmwareTP[0].fill("FIRMWARE_VERSION", "Version", "");
    m_firmwareTP[1].fill("FIRMWARE_DATE",    "Date",    "");
    m_firmwareTP[2].fill("FIRMWARE_TIME",    "Time",    "");
    m_firmwareTP[3].fill("FIRMWARE_CONFIG",  "Config",  "");
    m_firmwareTP.fill(getDeviceName(), "OSX_FIRMWARE", "Firmware Info",
                      "OnStepX", IP_RO, 60, IPS_IDLE);

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
    RI::updateProperties();
    WI::updateProperties();

    if (isConnected())
    {
        defineProperty(m_firmwareTP);
        m_firmwareTP[0].setText(m_core.caps().firmwareVersion);
        m_firmwareTP[1].setText(m_core.caps().firmwareDate);
        m_firmwareTP[2].setText(m_core.caps().firmwareTime);
        m_firmwareTP[3].setText(m_core.caps().configName);
        m_firmwareTP.setState(IPS_OK);
        m_firmwareTP.apply();

        createFocusers();

        if (m_core.caps().hasRotator)
        {
            setDriverInterface(getDriverInterface() | ROTATOR_INTERFACE);
            m_rotator.updateProperties(true, m_core.caps().hasDerotator);
            auto init = m_rotator.readInitial();
            if (init.angleValid)
            {
                GotoRotatorNP[0].setValue(init.angle);
                GotoRotatorNP.setState(IPS_OK);
                GotoRotatorNP.apply();
            }
            if (init.backlashValid)
            {
                RotatorBacklashNP[0].setValue(static_cast<double>(init.backlash));
                RotatorBacklashNP.setState(IPS_OK);
                RotatorBacklashNP.apply();
            }
        }

        if (m_core.caps().featureMask)
            m_auxFeatures.discoverAndDefine(m_core.caps().featureMask);

        if(m_core.caps().portMask)
            m_usbPorts.discoverAndDefine(m_core.caps().portMask);

    }
    else
    {
        deleteProperty(m_firmwareTP);
        if (m_core.caps().hasRotator)
            m_rotator.updateProperties(false, false);
        m_auxFeatures.deleteAll();
        m_weather.updateProperties(false);
        m_usbPorts.deleteAll();
    }

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

    m_auxFeatures.setComm(&m_core.comm());
    m_rotator.setComm(&m_core.comm());
    m_weather.setComm(&m_core.comm());
    m_weather.updateProperties(true);
    return true;
}

bool OnStepXAux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (RI::processSwitch(dev, name, states, names, n))
        return true;
    if (WI::processSwitch(dev, name, states, names, n))
        return true;
    if (isConnected() && m_core.caps().hasRotator && m_rotator.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_core.caps().featureMask && m_auxFeatures.handleSwitch(name, states, names, n))
        return true;
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXAux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (RI::processNumber(dev, name, values, names, n))
        return true;
    if (WI::processNumber(dev, name, values, names, n))
        return true;
    if (isConnected() && m_weather.handleNumber(name, values, names, n))
        return true;
    if (isConnected() && m_core.caps().featureMask && m_auxFeatures.handleNumber(name, values, names, n))
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
    RI::saveConfigItems(fp);
    WI::saveConfigItems(fp);
    m_auxFeatures.saveConfig(fp);
    m_rotator.saveConfig(fp);
    m_weather.saveConfig(fp);
    return true;
}

void OnStepXAux::TimerHit()
{
    if (!isConnected())
        return;

    m_pollCount++;
    if (m_pollCount % 10 == 0) WI::checkWeatherUpdate();
    if (m_pollCount % 5 == 0) pollFocusers();
    if (m_pollCount % 5 == 0) pollFeatures();
    if (m_pollCount % 5 == 0) pollUsbPorts();
    if (m_pollCount % 5 == 0) updateRotatorState();

    SetTimer(getCurrentPollingPeriod());
}

// ---------------------------------------------------------------------------
// createFocusers — same pattern as OnStepXMount::createFocusers
// ---------------------------------------------------------------------------
void OnStepXAux::createFocusers()
{
    int nf = m_core.caps().numFocusers;
    for (int i = 0; i < nf && i < (int)m_focusers.size(); i++)
    {
        if (m_focusers[i])
            continue;
        m_focusers[i] = std::make_unique<OnStepXFocuser>(i + 1);
        m_focusers[i]->setComm(&m_core.comm());
        m_focusers[i]->ISGetProperties(nullptr);
        m_focusers[i]->setConnected(true, IPS_OK);
        m_focusers[i]->updateProperties();
    }
}

void OnStepXAux::pollFocusers()
{
    for (auto &f : m_focusers)
        if (f) f->pollStatus();
}

void OnStepXAux::pollFeatures()
{
    if (m_core.caps().featureMask)
        m_auxFeatures.pollStatus();
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

// ---------------------------------------------------------------------------
// updateRotatorState — poll angle and status (~10 s throttle via TimerHit)
// ---------------------------------------------------------------------------
void OnStepXAux::updateRotatorState()
{
    if (!m_core.caps().hasRotator)
        return;

    auto r = m_rotator.pollStatus();

    if (r.angleValid)
        GotoRotatorNP[0].setValue(r.angle);

    if (r.statusValid)
        GotoRotatorNP.setState(r.moving ? IPS_BUSY : IPS_OK);

    if (r.angleValid || r.statusValid)
        GotoRotatorNP.apply();
}

// ---------------------------------------------------------------------------
// RotatorInterface overrides — delegate to m_rotator helper
// ---------------------------------------------------------------------------
IPState OnStepXAux::MoveRotator(double angle)
{
    return m_rotator.moveToAngle(angle);
}

bool OnStepXAux::AbortRotator()
{
    return m_rotator.abortRotator();
}

IPState OnStepXAux::HomeRotator()
{
    return m_rotator.homeRotator();
}

bool OnStepXAux::SetRotatorBacklash(int32_t steps)
{
    return m_rotator.setBacklash(steps);
}

// ---------------------------------------------------------------------------
// pollUsbPorts — poll USB port values (~5 poll throttle)
// ---------------------------------------------------------------------------
void OnStepXAux::pollUsbPorts()
{
    if (m_core.caps().portMask)
        m_usbPorts.pollStatus();
}
