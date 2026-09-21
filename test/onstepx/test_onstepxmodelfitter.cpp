#include "OnStepXModelFitter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace
{
constexpr double PI = 3.1415926535897932384626433832795;
constexpr double DEG = PI / 180.0;
constexpr double ARCSEC = DEG / 3600.0;

using Math = OnStepXModelMath;
using MountType = Math::MountType;
using PierSide = Math::PierSide;

Math::ModelCoefficients testModel()
{
    Math::ModelCoefficients model;

    model.ax1Cor =  2.0 * ARCSEC;
    model.ax2Cor = -3.0 * ARCSEC;
    model.altCor =  4.0 * ARCSEC;
    model.azmCor = -5.0 * ARCSEC;
    model.doCor  =  6.0 * ARCSEC;
    model.pdCor  = -4.0 * ARCSEC;
    model.dfCor  =  7.0 * ARCSEC;
    model.tfCor  = -3.0 * ARCSEC;

    model.hcp =  0.6;
    model.hca =  8.0 * ARCSEC;

    model.dcp = -0.8;
    model.dca =  6.0 * ARCSEC;

    return model;
}

Math::Observation syntheticObservation(
    double mountAxis1,
    double mountAxis2,
    MountType mountType,
    PierSide pierSide,
    const Math::ModelCoefficients &model,
    double latitude)
{
    Math::Observation observation;

    observation.mountAxis1 = mountAxis1;
    observation.mountAxis2 = mountAxis2;

    observation.mountType = mountType;
    observation.pierSide = pierSide;

    double observedAxis1 = 0.0;
    double observedAxis2 = 0.0;

    Math::predictObserved(
        observation,
        model,
        latitude,
        observedAxis1,
        observedAxis2);

    observation.actualAxis1 = observedAxis1;
    observation.actualAxis2 = observedAxis2;

    return observation;
}

std::vector<Math::Observation> makeSyntheticObservations(
    MountType mountType,
    const Math::ModelCoefficients &model,
    double latitude)
{
    std::vector<Math::Observation> observations;

    /*
     * The points deliberately cover both axes over a substantial
     * range.  GEM/FORK also use both pier sides.
     */
    constexpr std::size_t COUNT = 90;

    observations.reserve(COUNT);

    for (std::size_t i = 0; i < COUNT; ++i)
    {
        const double x =
            static_cast<double>(i);

        const double axis1 =
            -2.4 +
            0.055 * x +
            0.17 * std::sin(0.31 * x);

        const double axis2 =
            -0.95 +
            0.021 * x +
            0.23 * std::cos(0.27 * x);

        const PierSide side =
            (mountType == MountType::GEM ||
             mountType == MountType::FORK)
                ? ((i & 1)
                       ? PierSide::WEST
                       : PierSide::EAST)
                : PierSide::EAST;

        observations.push_back(
            syntheticObservation(
                axis1,
                axis2,
                mountType,
                side,
                model,
                latitude));
    }

    return observations;
}

void expectRecoveredModel(
    MountType mountType)
{
    const double latitude =
        0.72;

    const auto trueModel =
        testModel();

    const auto observations =
        makeSyntheticObservations(
            mountType,
            trueModel,
            latitude);

    const auto result =
        OnStepXModelFitter::fit(
            observations,
            latitude);

    ASSERT_TRUE(result.success())
        << "fit status = "
        << static_cast<int>(result.status);

    EXPECT_EQ(result.rank, 12u);

    EXPECT_LT(result.rmsArcsec, 1.0e-6);
    EXPECT_LT(result.maxAbsResidualArcsec, 1.0e-5);

    EXPECT_NEAR(
        result.model.ax1Cor,
        trueModel.ax1Cor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.ax2Cor,
        trueModel.ax2Cor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.altCor,
        trueModel.altCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.azmCor,
        trueModel.azmCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.doCor,
        trueModel.doCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.pdCor,
        trueModel.pdCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.dfCor,
        trueModel.dfCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.tfCor,
        trueModel.tfCor,
        1.0e-12);

    EXPECT_NEAR(
        result.model.hca,
        trueModel.hca,
        1.0e-12);

    EXPECT_NEAR(
        result.model.dca,
        trueModel.dca,
        1.0e-12);

    /*
     * Phase is periodic.  Compare the reconstructed cosine
     * components instead of relying on a particular wrapped
     * phase representation.
     */
    const double trueHc =
        trueModel.hca *
        std::cos(trueModel.hcp);

    const double trueHs =
        -trueModel.hca *
        std::sin(trueModel.hcp);

    const double fittedHc =
        result.model.hca *
        std::cos(result.model.hcp);

    const double fittedHs =
        -result.model.hca *
        std::sin(result.model.hcp);

    const double trueDc =
        trueModel.dca *
        std::cos(trueModel.dcp);

    const double trueDs =
        -trueModel.dca *
        std::sin(trueModel.dcp);

    const double fittedDc =
        result.model.dca *
        std::cos(result.model.dcp);

    const double fittedDs =
        -result.model.dca *
        std::sin(result.model.dcp);

    EXPECT_NEAR(fittedHc, trueHc, 1.0e-12);
    EXPECT_NEAR(fittedHs, trueHs, 1.0e-12);

    EXPECT_NEAR(fittedDc, trueDc, 1.0e-12);
    EXPECT_NEAR(fittedDs, trueDs, 1.0e-12);
}

TEST(OnStepXModelFitter, RecoversGemModel)
{
    expectRecoveredModel(MountType::GEM);
}

TEST(OnStepXModelFitter, RecoversForkModel)
{
    expectRecoveredModel(MountType::FORK);
}

TEST(OnStepXModelFitter, RecoversAltAzmModel)
{
    expectRecoveredModel(MountType::ALTAZM);
}

TEST(OnStepXModelFitter, RecoversAltAltModel)
{
    expectRecoveredModel(MountType::ALTALT);
}

TEST(OnStepXModelFitter, RejectsInsufficientObservations)
{
    const double latitude = 0.72;

    std::vector<Math::Observation> observations;

    for (std::size_t i = 0; i < 11; ++i)
    {
        Math::Observation observation;

        observation.mountAxis1 =
            0.1 * static_cast<double>(i);

        observation.mountAxis2 =
            0.2 * static_cast<double>(i);

        observation.actualAxis1 =
            observation.mountAxis1;

        observation.actualAxis2 =
            observation.mountAxis2;

        observation.mountType =
            MountType::GEM;

        observation.pierSide =
            PierSide::EAST;

        observations.push_back(observation);
    }

    const auto result =
        OnStepXModelFitter::fit(
            observations,
            latitude);

    EXPECT_FALSE(result.success());

    EXPECT_EQ(
        result.status,
        OnStepXModelFitter::FitResult::Status::
            INSUFFICIENT_OBSERVATIONS);
}

TEST(OnStepXModelFitter, RejectsRankDeficientGeometry)
{
    const double latitude =
        0.72;

    const auto model =
        testModel();

    std::vector<Math::Observation> observations;

    /*
     * Every observation is identical.  This cannot provide
     * independent information for all twelve fitted parameters.
     */
    for (std::size_t i = 0; i < 30; ++i)
    {
        observations.push_back(
            syntheticObservation(
                0.8,
                0.3,
                MountType::GEM,
                PierSide::EAST,
                model,
                latitude));
    }

    const auto result =
        OnStepXModelFitter::fit(
            observations,
            latitude);

    EXPECT_FALSE(result.success());

    EXPECT_LT(result.rank, 12u);
}

TEST(OnStepXModelFitter, RecoversZeroModel)
{
    const double latitude =
        0.72;

    Math::ModelCoefficients zeroModel;

    const auto observations =
        makeSyntheticObservations(
            MountType::GEM,
            zeroModel,
            latitude);

    const auto result =
        OnStepXModelFitter::fit(
            observations,
            latitude);

    ASSERT_TRUE(result.success())
        << "fit status = "
        << static_cast<int>(result.status);

    EXPECT_EQ(result.rank, 12u);

    EXPECT_LT(result.rmsArcsec, 1.0e-8);
    EXPECT_LT(result.maxAbsResidualArcsec, 1.0e-7);

    EXPECT_NEAR(result.model.ax1Cor, 0.0, 1.0e-10);
    EXPECT_NEAR(result.model.ax2Cor, 0.0, 1.0e-10);
    EXPECT_NEAR(result.model.altCor, 0.0, 1.0e-10);
    EXPECT_NEAR(result.model.azmCor, 0.0, 1.0e-10);
    EXPECT_NEAR(result.model.doCor,  0.0, 1.0e-10);
    EXPECT_NEAR(result.model.pdCor,  0.0, 1.0e-10);
    EXPECT_NEAR(result.model.dfCor,  0.0, 1.0e-10);
    EXPECT_NEAR(result.model.tfCor,  0.0, 1.0e-10);
    EXPECT_NEAR(result.model.hca,   0.0, 1.0e-10);
    EXPECT_NEAR(result.model.dca,   0.0, 1.0e-10);
}

TEST(OnStepXModelFitter, ReportsResidualStatistics)
{
    const double latitude =
        0.72;

    const auto model =
        testModel();

    const auto observations =
        makeSyntheticObservations(
            MountType::GEM,
            model,
            latitude);

    const auto result =
        OnStepXModelFitter::fit(
            observations,
            latitude);

    ASSERT_TRUE(result.success());

    EXPECT_TRUE(
        std::isfinite(result.rmsArcsec));

    EXPECT_TRUE(
        std::isfinite(result.maxAbsResidualArcsec));

    EXPECT_GE(
        result.rmsArcsec,
        0.0);

    EXPECT_GE(
        result.maxAbsResidualArcsec,
        result.rmsArcsec);
}
}