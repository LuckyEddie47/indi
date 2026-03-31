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

OnStepXAux::OnStepXAux()
{
    setVersion(0, 1);
}

const char *OnStepXAux::getDefaultName()
{
    return "OnStep X Controller";
}

bool OnStepXAux::initProperties()
{
    INDI::DefaultDevice::initProperties();

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
    return true;
}

bool OnStepXAux::Handshake()
{
    if (getActiveConnection() == m_serialConnection)
        m_core.setFd(m_serialConnection->getPortFD());
    else if (getActiveConnection() == m_tcpConnection)
        m_core.setFd(m_tcpConnection->getPortFD());

    return false;
}

bool OnStepXAux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXAux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool OnStepXAux::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool OnStepXAux::saveConfigItems(FILE *fp)
{
    return INDI::DefaultDevice::saveConfigItems(fp);
}

void OnStepXAux::TimerHit()
{
    if (!isConnected())
        return;

    SetTimer(getCurrentPollingPeriod());
}
