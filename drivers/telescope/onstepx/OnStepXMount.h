/*
    OnStep X INDI Driver — Mount device class

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

#include "OnStepXCore.h"
#include "OnStepXFocuser.h"
#include "OnStepXLimits.h"
#include "OnStepXRotator.h"
#include "OnStepXSite.h"
#include "OnStepXStatus.h"
#include "OnStepXTracking.h"
#include "OnStepXWeather.h"

#include <array>
#include <memory>

#include <inditelescope.h>
#include <alignment/AlignmentSubsystemForDrivers.h>
#include <indiguiderinterface.h>
#include <indirotatorinterface.h>
#include <indiweatherinterface.h>

#include <chrono>

class OnStepXMount : public INDI::Telescope,
                     public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers,
                     public INDI::GuiderInterface,
                     public INDI::RotatorInterface,
                     public INDI::WeatherInterface
{
    public:
        OnStepXMount();

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;

    protected:
        virtual bool Handshake() override;
        virtual bool ReadScopeStatus() override;
        virtual bool Goto(double ra, double dec) override;
        virtual bool Sync(double ra, double dec) override;
        virtual bool Abort() override;
        virtual bool Park() override;
        virtual bool UnPark() override;
        virtual bool SetCurrentPark() override;
        virtual bool SetDefaultPark() override;
        virtual bool SetTrackEnabled(bool enabled) override;
        virtual bool SetTrackMode(uint8_t mode) override;
        virtual bool SetTrackRate(double raRate, double deRate) override;
        virtual bool SetSlewRate(int index) override;
        virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
        virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
        virtual bool updateLocation(double latitude, double longitude, double elevation) override;
        virtual bool updateTime(ln_date *utc, double utc_offset) override;
        virtual IPState ExecuteHomeAction(TelescopeHomeAction action) override;

        // GuiderInterface
        virtual IPState GuideNorth(uint32_t ms) override;
        virtual IPState GuideSouth(uint32_t ms) override;
        virtual IPState GuideEast(uint32_t ms) override;
        virtual IPState GuideWest(uint32_t ms) override;

        // RotatorInterface
        virtual IPState MoveRotator(double angle) override;
        virtual bool    AbortRotator() override;
        virtual IPState HomeRotator() override;
        virtual bool    SetRotatorBacklash(int32_t steps) override;

    protected:
        // WeatherInterface — called by WI::checkWeatherUpdate() every polling cycle
        virtual IPState updateWeather() override;

    private:
        // Status refresh — sends :Gu# (preferred) or :GU# (fallback)
        bool refreshMountStatus();

        // ReadScopeStatus delegates
        bool updateCoordinates();
        void updateTrackingState(const MountStatus &s);
        void updateSlewState(const MountStatus &s);
        void updateStatusText(const MountStatus &s);

        void createFocusers();

        // Throttled subsystem updaters
        void updateFocuserStates();
        void updateRotatorState();
        void updateWeatherState();
        void updateFeatureStates()  {}
        void readGuideRate();
        void checkGuideComplete();
        void updateTrackingProperties();

        bool isEquatorial() const;

        OnStepXCore     m_core;
        OnStepXLimits   m_limits;
        OnStepXRotator  m_rotator;
        OnStepXSite     m_site;
        OnStepXTracking m_tracking;
        OnStepXWeather  m_weather;

        // Focuser child devices — created after Handshake, slot 1..numFocusers
        std::array<std::unique_ptr<OnStepXFocuser>, 6> m_focusers;
        MountStatus    m_status;

        // Poll throttle — counts ReadScopeStatus calls
        int  m_pollCount { 0 };

        // Guide pulse completion tracking — steady_clock end-times
        using Clock     = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        bool      m_guidingNS { false };
        bool      m_guidingWE { false };
        TimePoint m_guideEndNS;
        TimePoint m_guideEndWE;

        // Guide rate (read from :GX90#, displayed read-only)
        INDI::PropertyNumber m_guideRateNP {1};
};
