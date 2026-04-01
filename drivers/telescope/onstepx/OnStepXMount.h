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
#include "OnStepXStatus.h"

#include <inditelescope.h>
#include <alignment/AlignmentSubsystemForDrivers.h>

class OnStepXMount : public INDI::Telescope,
                     public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
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
        virtual bool SetSlewRate(int index) override;
        virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
        virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
        virtual bool updateLocation(double latitude, double longitude, double elevation) override;
        virtual bool updateTime(ln_date *utc, double utc_offset) override;

    private:
        // Status refresh — sends :Gu# (preferred) or :GU# (fallback)
        bool refreshMountStatus();

        // ReadScopeStatus delegates
        bool updateCoordinates();
        void updateTrackingState(const MountStatus &s);
        void updateSlewState(const MountStatus &s);
        void updateStatusText(const MountStatus &s);

        // Stubs for throttled subsystems (implemented in later stages)
        void updateFocuserStates()  {}
        void updateRotatorState()   {}
        void updateWeatherState()   {}
        void updateFeatureStates()  {}

        bool isEquatorial() const;

        OnStepXCore  m_core;
        MountStatus  m_status;

        // Poll throttle — counts ReadScopeStatus calls
        int  m_pollCount { 0 };
};
