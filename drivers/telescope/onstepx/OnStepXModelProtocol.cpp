#include "OnStepXModelProtocol.h"

#include <cmath>

namespace
{
constexpr double ARCSEC_PER_RAD =
    206264.80624709636;

constexpr double DEG_PER_RAD =
    180.0 / 3.1415926535897932384626433832795;

constexpr double RAD_PER_ARCSEC =
    1.0 / ARCSEC_PER_RAD;

constexpr double RAD_PER_DEG =
    3.1415926535897932384626433832795 / 180.0;

std::int64_t roundArcsec(double radians)
{
    return static_cast<std::int64_t>(
        std::lround(radians * ARCSEC_PER_RAD));
}

std::int64_t roundDegrees(double radians)
{
    return static_cast<std::int64_t>(
        std::lround(radians * DEG_PER_RAD));
}
}

OnStepXModelProtocol::Values
OnStepXModelProtocol::quantize(
    const OnStepXModelMath::ModelCoefficients &model)
{
    Values values;

    values.ax1Cor = roundArcsec(model.ax1Cor);
    values.ax2Cor = roundArcsec(model.ax2Cor);
    values.altCor = roundArcsec(model.altCor);
    values.azmCor = roundArcsec(model.azmCor);
    values.doCor  = roundArcsec(model.doCor);
    values.pdCor  = roundArcsec(model.pdCor);
    values.dfCor  = roundArcsec(model.dfCor);
    values.tfCor  = roundArcsec(model.tfCor);

    values.hcp = roundDegrees(model.hcp);
    values.hca = roundArcsec(model.hca);

    values.dcp = roundDegrees(model.dcp);
    values.dca = roundArcsec(model.dca);

    return values;
}

OnStepXModelMath::ModelCoefficients
OnStepXModelProtocol::dequantize(
    const Values &values)
{
    OnStepXModelMath::ModelCoefficients model;

    model.ax1Cor =
        static_cast<double>(values.ax1Cor) *
        RAD_PER_ARCSEC;

    model.ax2Cor =
        static_cast<double>(values.ax2Cor) *
        RAD_PER_ARCSEC;

    model.altCor =
        static_cast<double>(values.altCor) *
        RAD_PER_ARCSEC;

    model.azmCor =
        static_cast<double>(values.azmCor) *
        RAD_PER_ARCSEC;

    model.doCor =
        static_cast<double>(values.doCor) *
        RAD_PER_ARCSEC;

    model.pdCor =
        static_cast<double>(values.pdCor) *
        RAD_PER_ARCSEC;

    model.dfCor =
        static_cast<double>(values.dfCor) *
        RAD_PER_ARCSEC;

    model.tfCor =
        static_cast<double>(values.tfCor) *
        RAD_PER_ARCSEC;

    model.hcp =
        static_cast<double>(values.hcp) *
        RAD_PER_DEG;

    model.hca =
        static_cast<double>(values.hca) *
        RAD_PER_ARCSEC;

    model.dcp =
        static_cast<double>(values.dcp) *
        RAD_PER_DEG;

    model.dca =
        static_cast<double>(values.dca) *
        RAD_PER_ARCSEC;

    return model;
}

char OnStepXModelProtocol::dfCoefficientIndex(
    OnStepXModelMath::MountType mountType)
{
    if (mountType ==
            OnStepXModelMath::MountType::FORK ||
        mountType ==
            OnStepXModelMath::MountType::ALTAZM)
    {
        return '6';
    }

    return '7';
}