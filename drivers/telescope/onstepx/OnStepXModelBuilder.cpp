/*
    OnStep X INDI Driver — PC-side pointing-model builder

    Stage 2 implementation:
      - Build Model mode control
      - Calculate Model / Abort Model controls
      - arbitrary-size observation store
      - Sync interception
      - exact OnStepX equatorial -> native-axis conversions for GEM/FORK,
        ALTAZM and ALTALT
      - exact GeoAlign::mountToObservedPlace() forward-model equations
      - tracking safety invariant

    Numerical fitting, coefficient quantisation, model replacement and
    rollback are deliberately added in later stages.
*/

#include "OnStepXModelBuilder.h"
#include "OnStepXComm.h"

#include <indilogger.h>

#include <cmath>
#include <cstdio>

#define MODEL_TAB "Model"

enum { BUILD_ON = 0 };

void OnStepXModelBuilder::initProperties()
{
    const char *dev = getDeviceName();

    m_buildSP[BUILD_ON].fill("BUILD", "Build Model", ISS_OFF);
    m_buildSP.fill(dev, "OSX_MODEL_BUILD", "Build Model", MODEL_TAB,
                   IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    m_controlSP[CONTROL_CALCULATE].fill("CALCULATE", "Calculate Model", ISS_OFF);
    m_controlSP[CONTROL_ABORT].fill("ABORT", "Abort Model", ISS_OFF);
    m_controlSP.fill(dev, "OSX_MODEL_CONTROL", "Model Control", MODEL_TAB,
                     IP_RW, ISR_ATMOST1, 60, IPS_IDLE);
}

void OnStepXModelBuilder::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_buildSP);
        m_dev->defineProperty(m_controlSP);
    }
    else
    {
        // A disconnect cannot leave a live build session.  The active
        // firmware model is not modified by this stage, so discarding the
        // PC-side observations is sufficient.
        m_building = false;
        m_observations.clear();
        m_buildSP[BUILD_ON].setState(ISS_OFF);
        m_dev->deleteProperty(m_buildSP);
        m_dev->deleteProperty(m_controlSP);
    }
}

bool OnStepXModelBuilder::handleSwitch(const char *name, ISState *states,
                                       char *names[], int n)
{
    if (m_buildSP.isNameMatch(name))
    {
        m_buildSP.update(states, names, n);
        const bool requested = (m_buildSP[BUILD_ON].getState() == ISS_ON);

        bool ok = true;
        if (requested)
            ok = startBuild();
        else if (m_building)
            ok = abortBuild("Build Model disabled");

        m_buildSP[BUILD_ON].setState(ok && m_building ? ISS_ON : ISS_OFF);
        m_buildSP.setState(ok ? IPS_OK : IPS_ALERT);
        m_buildSP.apply();
        return true;
    }

    if (m_controlSP.isNameMatch(name))
    {
        m_controlSP.update(states, names, n);
        const int idx = m_controlSP.findOnSwitchIndex();
        bool ok = false;

        if (idx == CONTROL_CALCULATE)
            ok = calculateModel();
        else if (idx == CONTROL_ABORT)
            ok = abortBuild("User requested Abort Model");

        m_controlSP.reset();
        m_controlSP.setState(ok ? IPS_OK : IPS_ALERT);
        m_controlSP.apply();
        return true;
    }

    return false;
}

void OnStepXModelBuilder::saveConfig(FILE * /*fp*/)
{
    // Build sessions are deliberately not persistent.
}

void OnStepXModelBuilder::updateTrackingState(bool tracking)
{
    const bool wasTracking = m_tracking;
    m_tracking = tracking;

    if (m_building && wasTracking && !tracking)
        abortBuild("Tracking was turned off while Build Model was active");
}

bool OnStepXModelBuilder::startBuild()
{
    if (!m_comm)
    {
        LOG_ERROR("Build Model: communication is not available");
        return false;
    }

    if (!m_tracking)
    {
        LOG_ERROR("Build Model requires tracking to be ON");
        return false;
    }

    if (m_mountType == MountStatus::MountType::UNKNOWN)
    {
        LOG_ERROR("Build Model: mount type is not known");
        return false;
    }

    double latitudeDeg = 0.0;
    if (!readLatitude(latitudeDeg))
    {
        LOG_ERROR("Build Model: failed to read site latitude");
        return false;
    }

    m_latitudeRad = latitudeDeg * 3.14159265358979323846 / 180.0;
    m_observations.clear();
    m_building = true;
    LOGF_INFO("Build Model started at site latitude %.8f deg; waiting for Ekos Mount Model Sync observations",
              latitudeDeg);
    return true;
}

bool OnStepXModelBuilder::abortBuild(const char *reason)
{
    if (!m_building)
    {
        m_observations.clear();
        return true;
    }

    const std::size_t count = m_observations.size();
    m_observations.clear();
    m_building = false;
    m_buildSP[BUILD_ON].setState(ISS_OFF);
    m_buildSP.setState(IPS_OK);
    m_buildSP.apply();

    LOGF_INFO("Build Model aborted (%s); discarded %zu observations", reason, count);
    return true;
}

bool OnStepXModelBuilder::calculateModel()
{
    if (!m_building)
    {
        LOG_ERROR("Calculate Model requested but Build Model is not active");
        return false;
    }

    // The numerical fitter and controlled model-replacement transaction are
    // intentionally not part of Stage 1.
    LOGF_ERROR("Calculate Model is not yet implemented; %zu observations retained",
               m_observations.size());
    return false;
}

bool OnStepXModelBuilder::readCurrentMount(double &mountRAHours,
                                           double &mountDecDeg,
                                           double &lstHours)
{
    if (!m_comm)
        return false;

    char reply[OnStepXComm::REPLY_BUF_SIZE] {};

    if (!m_comm->sendCommand(":GRH#", reply) || f_scansexa(reply, &mountRAHours) < 0)
    {
        LOG_ERROR("Build Model: failed to read mount RA (:GRH#)");
        return false;
    }

    if (!m_comm->sendCommand(":GDH#", reply) || f_scansexa(reply, &mountDecDeg) < 0)
    {
        LOG_ERROR("Build Model: failed to read mount Dec (:GDH#)");
        return false;
    }

    if (!m_comm->sendCommand(":GSH#", reply) || f_scansexa(reply, &lstHours) < 0)
    {
        LOG_ERROR("Build Model: failed to read sidereal time (:GSH#)");
        return false;
    }

    return true;
}

bool OnStepXModelBuilder::readLatitude(double &latitudeDeg)
{
    if (!m_comm)
        return false;

    char reply[OnStepXComm::REPLY_BUF_SIZE] {};
    if (!m_comm->sendCommand(":GtH#", reply) || f_scansexa(reply, &latitudeDeg) < 0)
    {
        LOG_ERROR("Build Model: failed to read site latitude (:GtH#)");
        return false;
    }

    return true;
}

double OnStepXModelBuilder::wrapHours(double hours)
{
    while (hours >= 12.0)
        hours -= 24.0;
    while (hours < -12.0)
        hours += 24.0;
    return hours;
}

bool OnStepXModelBuilder::captureSync(double ra, double dec,
                                      MountStatus::PierSide pierSide,
                                      MountStatus::MountType mountType)
{
    if (!m_building)
        return false;

    double mountRA = 0.0;
    double mountDec = 0.0;
    double lst = 0.0;
    if (!readCurrentMount(mountRA, mountDec, lst))
    {
        LOG_ERROR("Build Model: Sync observation rejected because mount coordinates could not be read");
        return true; // Consume the Sync; never forward it while building.
    }

    if (mountType == MountStatus::MountType::UNKNOWN || mountType != m_mountType)
    {
        LOG_ERROR("Build Model: Sync observation rejected because mount type is unavailable or changed");
        return true;
    }

    const double latitude = m_latitudeRad;
    constexpr double PI = 3.14159265358979323846;
    const double actualHA = wrapHours(lst - ra) * PI / 12.0;
    const double mountHA  = wrapHours(lst - mountRA) * PI / 12.0;
    const double actualDec = dec * PI / 180.0;
    const double mountDecRad = mountDec * PI / 180.0;

    Observation obs;
    obs.actualRAHours = ra;
    obs.actualDecDeg  = dec;
    obs.mountRAHours  = mountRA;
    obs.mountDecDeg   = mountDec;
    obs.lstHours      = lst;
    OnStepXModelMath::MountType mathMountType;
    switch (mountType)
    {
        case MountStatus::MountType::GEM:
            mathMountType = OnStepXModelMath::MountType::GEM;
            break;
        case MountStatus::MountType::FORK:
            mathMountType = OnStepXModelMath::MountType::FORK;
            break;
        case MountStatus::MountType::ALTAZM:
            mathMountType = OnStepXModelMath::MountType::ALTAZM;
            break;
        case MountStatus::MountType::ALTALT:
            mathMountType = OnStepXModelMath::MountType::ALTALT;
            break;
        default:
            LOG_ERROR("Build Model: unsupported mount type");
            return true;
    }

    obs.pierSide = (pierSide == MountStatus::PierSide::WEST)
                       ? OnStepXModelMath::PierSide::WEST
                       : OnStepXModelMath::PierSide::EAST;
    obs.mountType = mathMountType;

    OnStepXModelMath::equatorialToNative(actualHA, actualDec, latitude,
                                         mathMountType,
                                         obs.actualAxis1, obs.actualAxis2);
    OnStepXModelMath::equatorialToNative(mountHA, mountDecRad, latitude,
                                         mathMountType,
                                         obs.mountAxis1, obs.mountAxis2);

    m_observations.push_back(obs);

    LOGF_INFO("Build Model: captured observation %zu: actual axes %.6f %.6f, "
              "mount axes %.6f %.6f, RA %.6fh Dec %.6f, pier side %d",
              m_observations.size(), obs.actualAxis1, obs.actualAxis2,
              obs.mountAxis1, obs.mountAxis2, obs.actualRAHours,
              obs.actualDecDeg, static_cast<int>(obs.pierSide));
    return true;
}
