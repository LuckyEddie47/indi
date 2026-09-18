/*
    OnStep X INDI Driver — PC-side pointing-model builder

    This subsystem owns the driver-side model-building session.  It is kept
    separate from OnStepXAlignment because that class exposes the firmware's
    native N-star alignment mechanism; this class captures Ekos Sync
    observations for a PC-side fit and, in later stages, will fit and replace
    the firmware coefficient model.
*/

#pragma once

#include "OnStepXStatus.h"

#include <defaultdevice.h>
#include <cstdio>
#include <vector>

class OnStepXComm;

class OnStepXModelBuilder
{
    public:
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
        struct Observation
        {
            // Native OnStepX model coordinates.  GEM/FORK: HA/Dec;
            // ALTAZM: Az/Alt; ALTALT: AA1/AA2.
            double actualAxis1 { 0.0 };
            double actualAxis2 { 0.0 };
            double mountAxis1  { 0.0 };
            double mountAxis2  { 0.0 };

            // Retain the equatorial source coordinates for diagnostics and
            // for later fitter diagnostics.
            double actualRAHours { 0.0 };
            double actualDecDeg  { 0.0 };
            double mountRAHours  { 0.0 };
            double mountDecDeg   { 0.0 };
            double lstHours      { 0.0 };
            MountStatus::PierSide pierSide { MountStatus::PierSide::NONE };
            MountStatus::MountType mountType { MountStatus::MountType::UNKNOWN };
        };

        struct ModelCoefficients
        {
            // Angular coefficients are radians except hcp/dcp, which are
            // stored as degrees by the OnStepX protocol/model.
            double ax1Cor { 0.0 };
            double ax2Cor { 0.0 };
            double altCor { 0.0 };
            double azmCor { 0.0 };
            double doCor  { 0.0 };
            double pdCor  { 0.0 };
            double dfCor  { 0.0 };
            double tfCor  { 0.0 };
            double hcpDeg { 0.0 };
            double hca    { 0.0 };
            double dcpDeg { 0.0 };
            double dca    { 0.0 };
        };

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
        static double wrapRadians(double radians);
        static void equatorialToNative(double ha, double dec, double latitude,
                                       MountStatus::MountType mountType,
                                       double &axis1, double &axis2);
        static void mountToObservedPlace(double mountAxis1, double mountAxis2,
                                         MountStatus::PierSide pierSide,
                                         MountStatus::MountType mountType,
                                         double latitude,
                                         const ModelCoefficients &model,
                                         double &observedAxis1, double &observedAxis2);

        const char *getDeviceName() const
        {
            return m_dev ? m_dev->getDeviceName() : "Unknown";
        }
};
