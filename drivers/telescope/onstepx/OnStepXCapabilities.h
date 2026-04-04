/*
    OnStep X INDI Driver — Capability flags (shared by both binaries)

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

    Populated in two phases during Handshake (see OnStepXCore):
      Phase 1 -- probeController(), both binaries: isOnStepX, hasRotator,
        hasDerotator, hasWeather*, hasElevation, hasMcuTemp, numFocusers,
        featureMask, firmwareVersion/Date/Time, configName.
      Phase 2 -- probeMount(), mount binary only: hasMount, hasGoto,
        hasBinaryStatus, hasPec, hasHomeSense, hasPPS, hasDUT1, hasPierSide,
        mountType.
*/

#pragma once

#include <cstdint>

enum class MountType { GEM, FORK, ALTAZM, ALTALT, UNKNOWN };

struct Capabilities
{
    // -----------------------------------------------------------------------
    // Phase 1 — probeController() — populated for both binaries
    // -----------------------------------------------------------------------
    bool    isOnStepX       = false;
    bool    hasRotator      = false;
    bool    hasDerotator    = false;    // AltAz field-derotation
    bool    hasWeatherRead  = false;
    bool    hasWeatherWrite = false;
    bool    hasElevation    = false;
    bool    hasMcuTemp      = false;
    int     numFocusers     = 0;        // 0-6
    uint8_t featureMask     = 0;        // bits 0-7, one per aux feature slot
    uint8_t portMask        = 0;        // bits 0-7, one per USB port

    char firmwareVersion[16] {};
    char firmwareDate[16]    {};
    char firmwareTime[16]    {};
    char configName[48]      {};

    // -----------------------------------------------------------------------
    // Phase 2 — probeMount() — populated for mount binary only
    // -----------------------------------------------------------------------
    bool      hasMount        = false;
    bool      hasGoto         = false;
    bool      hasBinaryStatus = false;  // :Gu# binary status supported
    bool      hasPec          = false;
    bool      hasHomeSense    = false;
    bool      hasPPS          = false;  // PPS sync
    bool      hasDUT1         = false;
    bool      hasPierSide     = false;
    MountType mountType       = MountType::UNKNOWN;
};
