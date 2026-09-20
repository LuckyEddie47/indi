/*
    OnStep X INDI Driver — pure model mathematics tests

    These tests deliberately include only OnStepXModelMath.h. They must not
    construct OnStepXModelBuilder, OnStepXComm, DefaultDevice, or any other
    INDI/hardware object.
*/

#include "OnStepXModelMath.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
constexpr double PI = 3.14159265358979323846;

double deg(double value)
{
    return value * PI / 180.0;
}

double hours(double value)
{
    return value * PI / 12.0;
}

OnStepXModelMath::Observation baseObservation(
    OnStepXModelMath::MountType mountType,
    OnStepXModelMath::PierSide pierSide = OnStepXModelMath::PierSide::EAST)
{
    OnStepXModelMath::Observation observation;
    observation.mountType = mountType;
    observation.pierSide = pierSide;
    return observation;
}

} // namespace

TEST(OnStepXModelMath, WrapRadians)
{
    EXPECT_DOUBLE_EQ(OnStepXModelMath::wrapRadians(0.0), 0.0);
    EXPECT_DOUBLE_EQ(OnStepXModelMath::wrapRadians(PI), PI);
    EXPECT_DOUBLE_EQ(OnStepXModelMath::wrapRadians(-PI), PI);

    EXPECT_NEAR(OnStepXModelMath::wrapRadians(3.0 * PI), PI, 1e-15);
    EXPECT_NEAR(OnStepXModelMath::wrapRadians(-3.0 * PI), PI, 1e-15);
    EXPECT_NEAR(OnStepXModelMath::wrapRadians(deg(270.0)), deg(-90.0), 1e-15);
    EXPECT_NEAR(OnStepXModelMath::wrapRadians(deg(-270.0)), deg(90.0), 1e-15);
}

TEST(OnStepXModelMath, EquatorialToNativeGemUsesHourAngleAndDeclination)
{
    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::equatorialToNative(
        hours(3.0), deg(-20.0), deg(52.0),
        OnStepXModelMath::MountType::GEM, axis1, axis2);

    EXPECT_NEAR(axis1, hours(3.0), 1e-15);
    EXPECT_NEAR(axis2, deg(-20.0), 1e-15);
}

TEST(OnStepXModelMath, EquatorialToNativeForkUsesHourAngleAndDeclination)
{
    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::equatorialToNative(
        hours(-5.0), deg(35.0), deg(52.0),
        OnStepXModelMath::MountType::FORK, axis1, axis2);

    EXPECT_NEAR(axis1, hours(-5.0), 1e-15);
    EXPECT_NEAR(axis2, deg(35.0), 1e-15);
}

TEST(OnStepXModelMath, EquatorialToNativeGemWrapsHourAngle)
{
    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::equatorialToNative(
        hours(13.0), deg(10.0), deg(52.0),
        OnStepXModelMath::MountType::GEM, axis1, axis2);

    EXPECT_NEAR(axis1, hours(-11.0), 1e-15);
    EXPECT_NEAR(axis2, deg(10.0), 1e-15);
}

TEST(OnStepXModelMath, EquatorialToNativeAltAz)
{
    double axis1 = 0.0;
    double axis2 = 0.0;

    // At latitude 0, dec=0 and HA=+6h, the exact OnStepX equToHor
    // sequence gives altitude 0 and azimuth +90 degrees.
    OnStepXModelMath::equatorialToNative(
        hours(6.0), 0.0, 0.0,
        OnStepXModelMath::MountType::ALTAZM, axis1, axis2);

    EXPECT_NEAR(axis1, deg(-90.0), 1e-14);
    EXPECT_NEAR(axis2, 0.0, 1e-14);
}

TEST(OnStepXModelMath, EquatorialToNativeAltAlt)
{
    double axis1 = 0.0;
    double axis2 = 0.0;

    // Same non-singular horizontal point, followed by the exact OnStepX
    // horToAa transformation.
    OnStepXModelMath::equatorialToNative(
        hours(6.0), 0.0, 0.0,
        OnStepXModelMath::MountType::ALTALT, axis1, axis2);

    EXPECT_NEAR(axis1, deg(90.0), 1e-14);
    EXPECT_NEAR(axis2, 0.0, 1e-14);
}

TEST(OnStepXModelMath, ForwardModelWithZeroCoefficientsReturnsMountAxes)
{
    const OnStepXModelMath::ModelCoefficients model;
    const double latitude = deg(52.0);

    for (const auto mountType : {
             OnStepXModelMath::MountType::GEM,
             OnStepXModelMath::MountType::FORK,
             OnStepXModelMath::MountType::ALTAZM,
             OnStepXModelMath::MountType::ALTALT })
    {
        for (const auto pierSide : {
                 OnStepXModelMath::PierSide::EAST,
                 OnStepXModelMath::PierSide::WEST })
        {
            auto observation = baseObservation(mountType, pierSide);
            observation.mountAxis1 = deg(23.0);
            observation.mountAxis2 = deg(31.0);

            double axis1 = 0.0;
            double axis2 = 0.0;

            OnStepXModelMath::predictObserved(
                observation, model, latitude, axis1, axis2);

            EXPECT_NEAR(axis1, observation.mountAxis1, 1e-14);
            EXPECT_NEAR(axis2, observation.mountAxis2, 1e-14);
        }
    }
}

TEST(OnStepXModelMath, ForwardModelAppliesAxisCorrections)
{
    OnStepXModelMath::ModelCoefficients model;
    model.ax1Cor = deg(1.25);
    model.ax2Cor = deg(-0.75);

    auto observation = baseObservation(
        OnStepXModelMath::MountType::GEM,
        OnStepXModelMath::PierSide::EAST);
    observation.mountAxis1 = deg(20.0);
    observation.mountAxis2 = deg(30.0);

    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::predictObserved(
        observation, model, deg(52.0), axis1, axis2);

    EXPECT_NEAR(axis1, deg(21.25), 1e-14);
    EXPECT_NEAR(axis2, deg(30.75), 1e-14);
}

TEST(OnStepXModelMath, Axis2CorrectionChangesSignOnWestPier)
{
    OnStepXModelMath::ModelCoefficients model;
    model.ax2Cor = deg(1.0);

    auto east = baseObservation(
        OnStepXModelMath::MountType::GEM,
        OnStepXModelMath::PierSide::EAST);
    east.mountAxis1 = deg(20.0);
    east.mountAxis2 = deg(30.0);

    auto west = baseObservation(
        OnStepXModelMath::MountType::GEM,
        OnStepXModelMath::PierSide::WEST);
    west.mountAxis1 = east.mountAxis1;
    west.mountAxis2 = east.mountAxis2;

    double eastAxis1 = 0.0;
    double eastAxis2 = 0.0;
    double westAxis1 = 0.0;
    double westAxis2 = 0.0;

    OnStepXModelMath::predictObserved(east, model, deg(52.0), eastAxis1, eastAxis2);
    OnStepXModelMath::predictObserved(west, model, deg(52.0), westAxis1, westAxis2);

    EXPECT_NEAR(eastAxis2, deg(29.0), 1e-14);
    EXPECT_NEAR(westAxis2, deg(31.0), 1e-14);
}

TEST(OnStepXModelMath, AltitudeAzimuthModelUsesItsMountSpecificDfTerm)
{
    OnStepXModelMath::ModelCoefficients model;
    model.dfCor = deg(1.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::ALTAZM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::predictObserved(
        observation, model, deg(52.0), axis1, axis2);

    EXPECT_NEAR(axis1, observation.mountAxis1, 1e-14);
    EXPECT_NEAR(axis2,
                observation.mountAxis2 + model.dfCor * std::cos(deg(30.0)),
                1e-14);
}

TEST(OnStepXModelMath, GemModelUsesLatitudeDependentDfTerm)
{
    OnStepXModelMath::ModelCoefficients model;
    model.dfCor = deg(1.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    const double latitude = deg(52.0);
    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::predictObserved(
        observation, model, latitude, axis1, axis2);

    const double expected =
        observation.mountAxis2 -
        model.dfCor *
        (std::cos(latitude) * std::cos(observation.mountAxis1) +
         std::sin(latitude) * std::tan(observation.mountAxis2));

    EXPECT_NEAR(axis1, observation.mountAxis1, 1e-14);
    EXPECT_NEAR(axis2, expected, 1e-14);
}

TEST(OnStepXModelMath, PureDoTermMatchesForwardEquation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.doCor = deg(1.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(observation, model, deg(52.0), axis1, axis2);

    const double expected =
        observation.mountAxis1 + model.doCor / std::cos(observation.mountAxis2);

    EXPECT_NEAR(axis1, expected, 1e-14);
    EXPECT_NEAR(axis2, observation.mountAxis2, 1e-14);
}

TEST(OnStepXModelMath, PurePdTermMatchesForwardEquation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.pdCor = deg(1.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(observation, model, deg(52.0), axis1, axis2);

    const double expected =
        observation.mountAxis1 - model.pdCor * std::tan(observation.mountAxis2);

    EXPECT_NEAR(axis1, expected, 1e-14);
    EXPECT_NEAR(axis2, observation.mountAxis2, 1e-14);
}

TEST(OnStepXModelMath, PureAltAzmTermsMatchForwardEquation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.altCor = deg(0.8);
    model.azmCor = deg(-0.6);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(observation, model, deg(52.0), axis1, axis2);

    const double a1 =
        -model.azmCor * std::cos(observation.mountAxis1) *
            std::tan(observation.mountAxis2) +
        model.altCor * std::sin(observation.mountAxis1) *
            std::tan(observation.mountAxis2);

    const double a2 =
        model.azmCor * std::sin(observation.mountAxis1) +
        model.altCor * std::cos(observation.mountAxis1);

    EXPECT_NEAR(axis1, observation.mountAxis1 + a1, 1e-14);
    EXPECT_NEAR(axis2, observation.mountAxis2 + a2, 1e-14);
}

TEST(OnStepXModelMath, PureCosineTermsMatchForwardEquation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.hcp = deg(3.0);
    model.hca = deg(1.0);
    model.dcp = deg(-4.0);
    model.dca = deg(0.5);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(observation, model, deg(52.0), axis1, axis2);

    const double expectedAxis1 =
        observation.mountAxis1 +
        std::cos(observation.mountAxis1 + model.hcp) * model.hca;
    const double expectedAxis2 =
        observation.mountAxis2 +
        std::cos(observation.mountAxis2 + model.dcp) * model.dca;

    EXPECT_NEAR(axis1, expectedAxis1, 1e-14);
    EXPECT_NEAR(axis2, expectedAxis2, 1e-14);
}

TEST(OnStepXModelMath, TfTermMatchesForwardEquation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.tfCor = deg(1.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(20.0);

    const double latitude = deg(52.0);
    double axis1 = 0.0;
    double axis2 = 0.0;

    OnStepXModelMath::predictObserved(
        observation, model, latitude, axis1, axis2);

    const double tfH =
        model.tfCor *
        (std::cos(latitude) * std::sin(observation.mountAxis1) /
         std::cos(observation.mountAxis2));

    const double tfD =
        model.tfCor *
        (std::cos(latitude) * std::cos(observation.mountAxis1) *
             std::sin(observation.mountAxis2) -
         std::sin(latitude) * std::cos(observation.mountAxis2));

    EXPECT_NEAR(axis1, observation.mountAxis1 + tfH, 1e-14);
    EXPECT_NEAR(axis2, observation.mountAxis2 + tfD, 1e-14);
}

TEST(OnStepXModelMath, ResidualIsZeroForModelGeneratedObservation)
{
    OnStepXModelMath::ModelCoefficients model;
    model.ax1Cor = deg(0.2);
    model.ax2Cor = deg(-0.3);
    model.altCor = deg(0.1);
    model.azmCor = deg(-0.15);
    model.doCor = deg(0.05);
    model.pdCor = deg(-0.07);
    model.dfCor = deg(0.08);
    model.tfCor = deg(-0.04);
    model.hcp = deg(2.0);
    model.hca = deg(0.03);
    model.dcp = deg(-3.0);
    model.dca = deg(0.02);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(35.0);
    observation.mountAxis2 = deg(25.0);

    double actualAxis1 = 0.0;
    double actualAxis2 = 0.0;
    OnStepXModelMath::predictObserved(
        observation, model, deg(52.0), actualAxis1, actualAxis2);

    observation.actualAxis1 = actualAxis1;
    observation.actualAxis2 = actualAxis2;

    double r1 = 0.0;
    double r2 = 0.0;
    OnStepXModelMath::residual(
        observation, model, deg(52.0), r1, r2);

    EXPECT_NEAR(r1, 0.0, 1e-14);
    EXPECT_NEAR(r2, 0.0, 1e-14);
}

TEST(OnStepXModelMath, ResidualWrapsAxis1AcrossPi)
{
    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(179.0);
    observation.mountAxis2 = deg(20.0);
    observation.actualAxis1 = deg(-179.0);
    observation.actualAxis2 = observation.mountAxis2;

    const OnStepXModelMath::ModelCoefficients model;
    double r1 = 0.0;
    double r2 = 0.0;

    OnStepXModelMath::residual(
        observation, model, deg(52.0), r1, r2);

    EXPECT_NEAR(r1, deg(2.0) * std::cos(deg(20.0)), 1e-14);
    EXPECT_NEAR(r2, 0.0, 1e-14);
}

TEST(OnStepXModelMath, ResidualAxis1IsWeightedByCosActualAxis2)
{
    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = 0.0;
    observation.mountAxis2 = deg(60.0);
    observation.actualAxis1 = deg(1.0);
    observation.actualAxis2 = deg(60.0);

    const OnStepXModelMath::ModelCoefficients model;
    double r1 = 0.0;
    double r2 = 0.0;

    OnStepXModelMath::residual(
        observation, model, deg(52.0), r1, r2);

    EXPECT_NEAR(r1, deg(1.0) * 0.5, 1e-14);
    EXPECT_NEAR(r2, 0.0, 1e-14);
}

TEST(OnStepXModelMath, ForwardModelPreservesAxisBounds)
{
    OnStepXModelMath::ModelCoefficients model;
    model.ax1Cor = deg(10.0);
    model.ax2Cor = deg(20.0);
    model.altCor = deg(10.0);
    model.azmCor = deg(10.0);
    model.doCor = deg(5.0);
    model.pdCor = deg(5.0);
    model.dfCor = deg(5.0);
    model.tfCor = deg(5.0);
    model.hca = deg(2.0);
    model.dca = deg(2.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(170.0);
    observation.mountAxis2 = deg(80.0);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(
        observation, model, deg(52.0), axis1, axis2);

    EXPECT_LE(axis2, deg(90.0));
    EXPECT_GE(axis2, deg(-90.0));
    EXPECT_LE(axis1, deg(180.0));
    EXPECT_GE(axis1, deg(-180.0));
}

TEST(OnStepXModelMath, PoleGuardSkipsSingularModelTerms)
{
    OnStepXModelMath::ModelCoefficients model;
    model.doCor = deg(10.0);
    model.pdCor = deg(10.0);
    model.dfCor = deg(10.0);
    model.tfCor = deg(10.0);
    model.altCor = deg(10.0);
    model.azmCor = deg(10.0);
    model.hca = deg(10.0);
    model.dca = deg(10.0);

    auto observation = baseObservation(OnStepXModelMath::MountType::GEM);
    observation.mountAxis1 = deg(30.0);
    observation.mountAxis2 = deg(89.99);

    double axis1 = 0.0;
    double axis2 = 0.0;
    OnStepXModelMath::predictObserved(
        observation, model, deg(52.0), axis1, axis2);

    // The current implementation intentionally skips the model terms inside
    // the pole guard, while retaining the axis corrections and bounds.
    EXPECT_NEAR(axis1, observation.mountAxis1, 1e-14);
    EXPECT_NEAR(axis2, observation.mountAxis2, 1e-14);
}
