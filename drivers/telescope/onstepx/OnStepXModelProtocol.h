#pragma once

#include "OnStepXModelMath.h"

#include <array>
#include <cstdint>

class OnStepXModelProtocol
{
    public:
        struct Values
        {
            std::int64_t ax1Cor { 0 };
            std::int64_t ax2Cor { 0 };
            std::int64_t altCor { 0 };
            std::int64_t azmCor { 0 };
            std::int64_t doCor  { 0 };
            std::int64_t pdCor  { 0 };
            std::int64_t dfCor  { 0 };
            std::int64_t tfCor  { 0 };

            std::int64_t hcp { 0 };
            std::int64_t hca { 0 };
            std::int64_t dcp { 0 };
            std::int64_t dca { 0 };
        };

        /*
         * Convert the internal floating-point model to the integer
         * representation used by :SX0.
         *
         * Angular corrections:
         *   arcseconds -> integer arcseconds
         *
         * Phase coefficients:
         *   radians -> integer degrees
         *
         * Amplitude coefficients:
         *   radians -> integer arcseconds
         */
        static Values quantize(
            const OnStepXModelMath::ModelCoefficients &model);

        /*
         * Convert the integer protocol representation back into the
         * floating-point representation used by OnStepXModelMath.
         *
         * This represents the model that actually exists in firmware
         * after :SX0 has been applied, rather than the original
         * unquantized fit.
         */
        static OnStepXModelMath::ModelCoefficients dequantize(
            const Values &values);

        /*
         * Return the coefficient index used by :SX0/:GX0 for dfCor.
         *
         * Firmware uses:
         *   6 for FORK and ALTAZM
         *   7 otherwise
         */
        static char dfCoefficientIndex(
            OnStepXModelMath::MountType mountType);

        /*
         * Return the twelve protocol indices in the order used by
         * Values and by the firmware.
         */
        static constexpr std::array<char, 12> coefficientIndices()
        {
            return {
                '0', '1', '2', '3',
                '4', '5', '6', '8',
                'a', 'b', 'c', 'd'
            };
        };
};