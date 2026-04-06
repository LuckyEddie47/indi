/*
    OnStep X INDI Driver — Auxiliary controller device class (indi_onstepx_aux)

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

    Top-level device class for OnStepX installations without a mount.
    Composes OnStepXCore (probing), OnStepXAuxFeatures, OnStepXFocuser
    (up to 6 slots), and OnStepXRotator.  Mixes in RotatorInterface and
    WeatherInterface.  All protocol commands are implemented in the
    individual helper classes.
*/

#pragma once

#include "OnStepXAuxFeatures.h"
#include "OnStepXCore.h"
#include "OnStepXFocuser.h"
#include "OnStepXRotator.h"
#include "OnStepXWeather.h"
#include "OnStepXUsbPlugin.h"
#include <defaultdevice.h>
#include <indipropertytext.h>
#include <indirotatorinterface.h>
#include <indiweatherinterface.h>
#include "connectionplugins/connectionserial.h"
#include "connectionplugins/connectiontcp.h"

#include <array>
#include <memory>

class OnStepXAux : public INDI::DefaultDevice,
    public INDI::RotatorInterface,
    public INDI::WeatherInterface
{
    public:
        OnStepXAux();

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;

    protected:
        virtual bool Handshake();
        virtual void TimerHit() override;
        virtual IPState updateWeather() override;

        // RotatorInterface
        virtual IPState MoveRotator(double angle) override;
        virtual bool    AbortRotator() override;
        virtual IPState HomeRotator() override;
        virtual bool    SetRotatorBacklash(int32_t steps) override;

    private:
        void createFocusers();
        void pollFocusers();
        void updateRotatorState();
        void pollFeatures();
        void updateUsbStates();

        OnStepXCore        m_core;
        OnStepXAuxFeatures m_auxFeatures;
        OnStepXRotator     m_rotator;
        OnStepXWeather     m_weather;
        OnStepXUsbPlugin   m_usbPorts;
        INDI::PropertyText m_firmwareTP { 4 };
        Connection::Serial *m_serialConnection { nullptr };
        Connection::TCP    *m_tcpConnection    { nullptr };

        int  m_pollCount { 0 };

        // Focuser child devices — created after Handshake, slot 1..numFocusers
        std::array<std::unique_ptr<OnStepXFocuser>, 6> m_focusers;
};
