/*
    OnStep X INDI Driver — PC-side pointing-model fitter

    Pure numerical fitting component. No INDI, OnStepXComm, logging,
    or device dependencies.
*/

#pragma once

#include "OnStepXModelMath.h"

#include <cstddef>
#include <vector>

class OnStepXModelFitter
{
    public:
        struct FitResult
        {
            enum class Status
            {
                SUCCESS,
                INSUFFICIENT_OBSERVATIONS,
                SINGULAR,
                NON_FINITE,
                NO_CONVERGENCE
            };

            Status status { Status::NO_CONVERGENCE };

            OnStepXModelMath::ModelCoefficients model {};

            double rmsArcsec { 0.0 };
            double maxAbsResidualArcsec { 0.0 };

            std::size_t iterations { 0 };
            std::size_t rank { 0 };

            bool success() const
            {
                return status == Status::SUCCESS;
            }
        };

        /*
         * Fit the complete 12-parameter OnStepX pointing model.
         *
         * The input observations must already contain the actual and
         * commanded/native mount coordinates required by OnStepXModelMath.
         *
         * The latitude is radians.
         */
        static FitResult fit(
            const std::vector<OnStepXModelMath::Observation> &observations,
            double latitude);
};
