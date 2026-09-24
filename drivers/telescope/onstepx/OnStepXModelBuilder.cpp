/*
    OnStep X INDI Driver — PC-side pointing-model builder

    Stage 7 implementation:
      - Build Model mode control
      - Calculate Model / Abort Model controls
      - arbitrary-size observation store
      - Sync interception
      - exact OnStepX equatorial -> native-axis conversions for GEM/FORK,
        ALTAZM and ALTALT
      - exact GeoAlign::mountToObservedPlace() forward-model equations
      - tracking safety invariant
      - numerical fitting and protocol quantisation
      - readback-verified firmware coefficient replacement with rollback
      - model activation and EEPROM persistence
*/

#include "OnStepXModelBuilder.h"
#include "OnStepXComm.h"
#include "OnStepXModelFitter.h"
#include "OnStepXModelProtocol.h"

#include <indilogger.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define MODEL_TAB "Model"

enum { BUILD_ON = 0 };

namespace
{
constexpr double ARCSEC_PER_RAD =
    206264.80624709636;

struct ResidualStatistics
{
    double rmsArcsec { 0.0 };
    double maxAbsArcsec { 0.0 };
    bool finite { true };
};

ResidualStatistics evaluateResiduals(
    const std::vector<OnStepXModelMath::Observation> &observations,
    const OnStepXModelMath::ModelCoefficients &model,
    double latitude)
{
    ResidualStatistics statistics;

    double sumSquared = 0.0;
    std::size_t count = 0;

    for (const auto &observation : observations)
    {
        double r1 = 0.0;
        double r2 = 0.0;

        OnStepXModelMath::residual(
            observation,
            model,
            latitude,
            r1,
            r2);

        r1 *= ARCSEC_PER_RAD;
        r2 *= ARCSEC_PER_RAD;

        if (!std::isfinite(r1) ||
            !std::isfinite(r2))
        {
            statistics.finite = false;
            return statistics;
        }

        sumSquared += r1 * r1;
        sumSquared += r2 * r2;

        statistics.maxAbsArcsec =
            std::max(
                statistics.maxAbsArcsec,
                std::abs(r1));

        statistics.maxAbsArcsec =
            std::max(
                statistics.maxAbsArcsec,
                std::abs(r2));

        count += 2;
    }

    if (count == 0)
    {
        statistics.finite = false;
        return statistics;
    }

    statistics.rmsArcsec =
        std::sqrt(
            sumSquared /
            static_cast<double>(count));

    if (!std::isfinite(statistics.rmsArcsec) ||
        !std::isfinite(statistics.maxAbsArcsec))
    {
        statistics.finite = false;
    }

    return statistics;
}
}

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

    m_latitudeRad =
        latitudeDeg * 3.14159265358979323846 / 180.0;

    m_observations.clear();
    m_pendingModel = ModelCoefficients {};
    m_pendingProtocol = OnStepXModelProtocol::Values {};
    m_hasPendingModel = false;

    m_building = true;
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

    m_pendingModel = ModelCoefficients {};
    m_pendingProtocol = OnStepXModelProtocol::Values {};
    m_hasPendingModel = false;

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
        LOG_ERROR(
            "Calculate Model requested but Build Model is not active");

        return false;
    }

    const std::size_t observationCount =
        m_observations.size();

    LOGF_INFO(
        "Calculate Model: fitting %zu observations",
        observationCount);

    const auto fit =
        OnStepXModelFitter::fit(
            m_observations,
            m_latitudeRad);

    if (!fit.success())
    {
        LOGF_ERROR(
            "Calculate Model: fitter failed "
            "(status %d, rank %zu, iterations %zu); "
            "%zu observations retained",
            static_cast<int>(fit.status),
            fit.rank,
            fit.iterations,
            observationCount);

        return false;
    }

    LOGF_INFO(
        "Calculate Model: fit succeeded "
        "(rank %zu, iterations %zu, RMS %.9f arcsec, "
        "max %.9f arcsec)",
        fit.rank,
        fit.iterations,
        fit.rmsArcsec,
        fit.maxAbsResidualArcsec);

    /*
     * Convert the fitted floating-point model to the exact integer
     * representation accepted by :SX0.
     */
    const auto protocol =
        OnStepXModelProtocol::quantize(
            fit.model);

    /*
     * Reconstruct the model exactly as it will exist in firmware after
     * those integer values have been written.
     */
    const auto quantizedModel =
        OnStepXModelProtocol::dequantize(
            protocol);

    /*
     * Validate the actual post-quantisation model against the captured
     * observations using the same residual geometry as the fitter.
     */
    const auto quantizedResiduals =
        evaluateResiduals(
            m_observations,
            quantizedModel,
            m_latitudeRad);

    if (!quantizedResiduals.finite)
    {
        LOGF_ERROR(
            "Calculate Model: quantised model produced "
            "non-finite residuals; %zu observations retained",
            observationCount);

        return false;
    }

    LOGF_INFO(
        "Calculate Model: quantised model validation "
        "RMS %.9f arcsec, max %.9f arcsec",
        quantizedResiduals.rmsArcsec,
        quantizedResiduals.maxAbsArcsec);

    /*
     * Keep the exact protocol values as well as the reconstructed
     * floating-point model. The next stage will use these values for
     * the controlled :SX0 transaction.
     */
    m_pendingProtocol = protocol;
    m_pendingModel = quantizedModel;
    m_hasPendingModel = true;

    LOG_INFO(
        "Calculate Model: quantised model is ready for "
        "firmware upload; Build Model remains active");

    if (!replaceFirmwareModel())
    {
        LOGF_ERROR(
            "Calculate Model: firmware replacement failed; "
            "%zu observations retained",
            observationCount);
        return false;
    }

    m_observations.clear();
    m_pendingModel = ModelCoefficients {};
    m_pendingProtocol = OnStepXModelProtocol::Values {};
    m_hasPendingModel = false;
    m_building = false;
    m_buildSP[BUILD_ON].setState(ISS_OFF);
    m_buildSP.setState(IPS_OK);
    m_buildSP.apply();

    LOG_INFO(
        "Calculate Model: firmware model activated and persisted; "
        "Build Model completed successfully");

    return true;
}

bool OnStepXModelBuilder::readFirmwareModel(
    OnStepXModelProtocol::Values &values)
{
    if (!m_comm)
        return false;

    const auto indices =
        OnStepXModelProtocol::coefficientIndices();

    char reply[OnStepXComm::REPLY_BUF_SIZE] {};

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        char index = indices[i];
        if (i == 6)
            index = OnStepXModelProtocol::dfCoefficientIndex(
                static_cast<OnStepXModelMath::MountType>(m_mountType));

        char cmd[OnStepXComm::CMD_MAX_LEN];
        snprintf(cmd, sizeof(cmd), ":GX0%c#", index);

        if (!m_comm->sendCommand(cmd, reply))
        {
            LOGF_ERROR(
                "Build Model: failed to read firmware coefficient %c",
                index);
            return false;
        }

        errno = 0;
        char *end = nullptr;
        const long long value = std::strtoll(reply, &end, 10);

        if (errno == ERANGE || end == reply || *end != '\0')
        {
            LOGF_ERROR(
                "Build Model: invalid firmware coefficient reply for %c: '%s'",
                index, reply);
            return false;
        }

        switch (i)
        {
            case 0: values.ax1Cor = value; break;
            case 1: values.ax2Cor = value; break;
            case 2: values.altCor = value; break;
            case 3: values.azmCor = value; break;
            case 4: values.doCor  = value; break;
            case 5: values.pdCor  = value; break;
            case 6: values.dfCor  = value; break;
            case 7: values.tfCor  = value; break;
            case 8: values.hcp    = value; break;
            case 9: values.hca    = value; break;
            case 10: values.dcp   = value; break;
            case 11: values.dca   = value; break;
        }
    }

    return true;
}

bool OnStepXModelBuilder::writeFirmwareModel(
    const OnStepXModelProtocol::Values &values)
{
    if (!m_comm)
        return false;

    const auto indices =
        OnStepXModelProtocol::coefficientIndices();

    char reply[OnStepXComm::REPLY_BUF_SIZE] {};

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        char index = indices[i];
        if (i == 6)
            index = OnStepXModelProtocol::dfCoefficientIndex(
                static_cast<OnStepXModelMath::MountType>(m_mountType));

        std::int64_t value = 0;
        switch (i)
        {
            case 0: value = values.ax1Cor; break;
            case 1: value = values.ax2Cor; break;
            case 2: value = values.altCor; break;
            case 3: value = values.azmCor; break;
            case 4: value = values.doCor;  break;
            case 5: value = values.pdCor;  break;
            case 6: value = values.dfCor;  break;
            case 7: value = values.tfCor;  break;
            case 8: value = values.hcp;    break;
            case 9: value = values.hca;    break;
            case 10: value = values.dcp;   break;
            case 11: value = values.dca;   break;
        }

        char cmd[OnStepXComm::CMD_MAX_LEN];
        snprintf(cmd, sizeof(cmd), ":SX0%c,%lld#",
                 index, static_cast<long long>(value));

        if (!m_comm->sendCommand(cmd, reply) || reply[0] != '1')
        {
            LOGF_ERROR(
                "Build Model: firmware coefficient write failed for %c",
                index);
            return false;
        }
    }

    return true;
}

bool OnStepXModelBuilder::replaceFirmwareModel()
{
    if (!m_hasPendingModel || !m_comm)
    {
        LOG_ERROR("Build Model: no pending model or communication unavailable");
        return false;
    }

    OnStepXModelProtocol::Values original;
    if (!readFirmwareModel(original))
    {
        LOG_ERROR(
            "Build Model: unable to establish firmware-model rollback state; "
            "no coefficients were changed");
        return false;
    }

    if (!writeFirmwareModel(m_pendingProtocol))
    {
        const bool rollbackOk = writeFirmwareModel(original);
        LOGF_ERROR(
            "Build Model: coefficient upload failed; rollback %s",
            rollbackOk ? "succeeded" : "FAILED");
        return false;
    }

    OnStepXModelProtocol::Values readback;
    if (!readFirmwareModel(readback) ||
        readback.ax1Cor != m_pendingProtocol.ax1Cor ||
        readback.ax2Cor != m_pendingProtocol.ax2Cor ||
        readback.altCor != m_pendingProtocol.altCor ||
        readback.azmCor != m_pendingProtocol.azmCor ||
        readback.doCor  != m_pendingProtocol.doCor  ||
        readback.pdCor  != m_pendingProtocol.pdCor  ||
        readback.dfCor  != m_pendingProtocol.dfCor  ||
        readback.tfCor  != m_pendingProtocol.tfCor  ||
        readback.hcp    != m_pendingProtocol.hcp    ||
        readback.hca    != m_pendingProtocol.hca    ||
        readback.dcp    != m_pendingProtocol.dcp    ||
        readback.dca    != m_pendingProtocol.dca)
    {
        const bool rollbackOk = writeFirmwareModel(original);
        LOGF_ERROR(
            "Build Model: firmware coefficient readback mismatch; rollback %s",
            rollbackOk ? "succeeded" : "FAILED");
        return false;
    }

    char reply[OnStepXComm::REPLY_BUF_SIZE] {};
    if (!m_comm->sendCommand(":SX09,2#", reply) || reply[0] != '1')
    {
        LOG_ERROR("Build Model: firmware model activation failed");
        return false;
    }

    if (!m_comm->sendCommand(":AW#", reply) || reply[0] != '1')
    {
        LOG_ERROR("Build Model: firmware model persistence failed");
        return false;
    }

    return true;
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
