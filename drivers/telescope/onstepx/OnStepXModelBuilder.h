/*
    OnStep X INDI Driver — PC-side pointing-model builder

    This subsystem owns the driver-side model-building session.  It is kept
    separate from OnStepXAlignment because that class exposes the firmware's
    native N-star alignment mechanism; this class captures Ekos Sync
    observations for a PC-side fit and, in later stages, will fit and replace
    the firmware coefficient model.
*/

#pragma once

#include "OnStepXModelMath.h"
#include "OnStepXStatus.h"

#include <defaultdevice.h>
#include <cstdio>
#include <vector>

class OnStepXComm;

class OnStepXModelBuilder
{
    public:
        using Observation = OnStepXModelMath::Observation;
        using ModelCoefficients = OnStepXModelMath::ModelCoefficients;

        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)            { m_comm = comm; }

        void initProperties();
        void updateProperties(bool connected);
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);
        void saveConfig(FILE *fp);

        // Called from the mount polling path.  A confirmed transition to
        // tracking OFF aborts an active build session and discards it.
        void updateTrackingState(bool tracking);
        void updateMountType(MountStatus::MountType mountType) { m_mountType = mountType; }

        // Called from OnStepXMount::Sync().  When a build is active, the
        // normal firmware Sync is suppressed and the observation is stored.
        // Returns true when the Sync was consumed by the builder.
        bool captureSync(double ra, double dec, MountStatus::PierSide pierSide,
                         MountStatus::MountType mountType);

        bool isBuilding() const { return m_building; }
        std::size_t observationCount() const { return m_observations.size(); }

    private:
        INDI::DefaultDevice *m_dev  { nullptr };
        OnStepXComm         *m_comm { nullptr };

        INDI::PropertySwitch m_buildSP     { 1 };
        INDI::PropertySwitch m_controlSP   { 2 };

        bool m_building  { false };
        bool m_tracking  { false };
        MountStatus::MountType m_mountType { MountStatus::MountType::UNKNOWN };
        double m_latitudeRad { 0.0 };
        std::vector<Observation> m_observations;

        enum { CONTROL_CALCULATE = 0, CONTROL_ABORT = 1 };

        bool startBuild();
        bool abortBuild(const char *reason);
        bool calculateModel();
        bool readCurrentMount(double &mountRAHours, double &mountDecDeg,
                              double &lstHours);
        bool readLatitude(double &latitudeDeg);

        static double wrapHours(double hours);
        const char *getDeviceName() const
        {
            return m_dev ? m_dev->getDeviceName() : "Unknown";
        }
};
