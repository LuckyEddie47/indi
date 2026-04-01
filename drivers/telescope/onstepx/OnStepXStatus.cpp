/*
    OnStep X INDI Driver — Mount status parsers

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
*/

#include "OnStepXStatus.h"

#include <cstring>

// ---------------------------------------------------------------------------
// parseGU — ASCII :GU# reply
//
// Reply format (ref §5.3 of the master plan):
//   A variable-length string of flag characters followed by exactly 3 digits:
//     str[len-3]  = pulse guide rate select (0-9)
//     str[len-2]  = guide rate select (0-9)
//     str[len-1]  = error code (0-9)
//
// The flag characters (anywhere in str[0..len-4]) are:
//   n   not tracking          (absence → tracking)
//   N   no goto               (absence → goto active)
//   p   park: unparked
//   I   park: parking
//   P   park: parked
//   F   park: failed
//   H   at home position
//   h   homing in progress
//   B   auto-home at boot
//   S   PPS synced
//   G   pulse guide active
//   g   guide active
//   r   refraction comp (dual if 's' absent, single if 's' present)
//   t   OnTrack comp    (dual if 's' absent, single if 's' present)
//   s   single-axis modifier for r/t
//   (   lunar rate
//   O   solar rate
//   k   King rate
//   w   waiting at home
//   u   pause-at-home enabled
//   z   buzzer enabled
//   a   auto meridian flip
//   R   PEC data recorded
//   /   PEC: ignore
//   ,   PEC: ready to play
//   ~   PEC: playing
//   ;   PEC: ready to record
//   ^   PEC: recording
//   E   mount type GEM
//   K   mount type FORK
//   A   mount type ALTAZM
//   L   mount type ALTALT
//   o   pier side none
//   T   pier side east
//   W   pier side west
// ---------------------------------------------------------------------------
bool OnStepXStatus::parseGU(const char *reply, MountStatus &out)
{
    if (!reply || reply[0] == '\0')
        return false;

    int len = static_cast<int>(strlen(reply));
    // Need at least 3 trailing digits.
    if (len < 3)
        return false;

    out = MountStatus{};    // reset to defaults

    // Parse the 3 trailing digits.
    out.pulseGuideRateSelect = reply[len - 3] - '0';
    out.guideRateSelect      = reply[len - 2] - '0';
    out.errorCode            = reply[len - 1] - '0';

    // Clamp to valid range in case firmware sends unexpected data.
    if (out.pulseGuideRateSelect < 0 || out.pulseGuideRateSelect > 9) out.pulseGuideRateSelect = 0;
    if (out.guideRateSelect      < 0 || out.guideRateSelect      > 9) out.guideRateSelect      = 0;
    if (out.errorCode            < 0 || out.errorCode            > 9) out.errorCode            = 0;

    // Scan flag region: str[0..len-4] (inclusive).
    // Default: tracking=true, gotoActive=false (set by absence/presence of flags).
    bool notTracking = false;
    bool noGoto      = false;
    bool hasSingle   = false;    // 's' present — modifier for r/t
    bool hasR        = false;    // 'r' refraction comp
    bool hasT        = false;    // 't' OnTrack comp

    int flagLen = len - 3;       // number of flag characters

    for (int i = 0; i < flagLen; i++)
    {
        char c = reply[i];
        switch (c)
        {
            // Tracking / goto
            case 'n': notTracking           = true;                               break;
            case 'N': noGoto                = true;                               break;

            // Park states — last one encountered wins if multiple present.
            case 'p': out.parkState         = MountStatus::ParkState::UNPARKED;  break;
            case 'I': out.parkState         = MountStatus::ParkState::PARKING;   break;
            case 'P': out.parkState         = MountStatus::ParkState::PARKED;    break;
            case 'F': out.parkState         = MountStatus::ParkState::FAILED;    break;

            // Home
            case 'H': out.atHome            = true;                               break;
            case 'h': out.homing            = true;                               break;
            case 'B': out.autoHomeAtBoot    = true;                               break;

            // Misc
            case 'S': out.ppsSynced         = true;                               break;
            case 'G': out.pulseGuideActive  = true;                               break;
            case 'g': out.guideActive       = true;                               break;
            case 'w': out.waitingAtHome     = true;                               break;
            case 'u': out.pauseAtHomeEnabled = true;                              break;
            case 'z': out.buzzerEnabled     = true;                               break;
            case 'a': out.autoMeridianFlip  = true;                               break;

            // Tracking compensation modifiers (resolved after scan)
            case 'r': hasR                  = true;                               break;
            case 't': hasT                  = true;                               break;
            case 's': hasSingle             = true;                               break;

            // Track rates
            case '(': out.trackRate         = MountStatus::TrackRate::LUNAR;     break;
            case 'O': out.trackRate         = MountStatus::TrackRate::SOLAR;     break;
            case 'k': out.trackRate         = MountStatus::TrackRate::KING;      break;

            // PEC
            case 'R': out.pecRecorded       = true;                               break;
            case '/': out.pecState          = MountStatus::PecState::IGNORE;     break;
            case ',': out.pecState          = MountStatus::PecState::READY_PLAY; break;
            case '~': out.pecState          = MountStatus::PecState::PLAYING;    break;
            case ';': out.pecState          = MountStatus::PecState::READY_RECORD; break;
            case '^': out.pecState          = MountStatus::PecState::RECORDING;  break;

            // Mount type
            case 'E': out.mountType         = MountStatus::MountType::GEM;       break;
            case 'K': out.mountType         = MountStatus::MountType::FORK;      break;
            case 'A': out.mountType         = MountStatus::MountType::ALTAZM;    break;
            case 'L': out.mountType         = MountStatus::MountType::ALTALT;    break;

            // Pier side
            case 'o': out.pierSide          = MountStatus::PierSide::NONE;       break;
            case 'T': out.pierSide          = MountStatus::PierSide::EAST;       break;
            case 'W': out.pierSide          = MountStatus::PierSide::WEST;       break;

            default: break;
        }
    }

    // Resolve boolean flags
    out.tracking   = !notTracking;
    out.gotoActive = !noGoto;

    // Resolve tracking compensation (r/t + optional s modifier)
    if (hasR)
        out.trackComp = hasSingle ? MountStatus::TrackComp::REFRACTION_SINGLE
                                  : MountStatus::TrackComp::REFRACTION_DUAL;
    else if (hasT)
        out.trackComp = hasSingle ? MountStatus::TrackComp::ONTRACK_SINGLE
                                  : MountStatus::TrackComp::ONTRACK_DUAL;

    return true;
}

// ---------------------------------------------------------------------------
// parseGu — binary :Gu# reply (9 bytes, all >= 0x80)
//
// The firmware ORs 0x80 into every byte as a validity marker. The remaining
// 7 bits in each byte encode status as follows:
//
//   Byte 0 — tracking / goto / park / home
//     bit 0 : not tracking (1 = not tracking)
//     bit 1 : no goto in progress (1 = no goto)
//     bits 2-3 : park state (0=unparked, 1=parking, 2=parked, 3=failed)
//     bit 4 : at home
//     bit 5 : homing in progress
//     bit 6 : auto-home at boot
//
//   Byte 1 — guide / misc
//     bit 0 : pulse guide active
//     bit 1 : guide active
//     bit 2 : PPS synced
//     bit 3 : waiting at home
//     bit 4 : pause-at-home enabled
//     bit 5 : buzzer enabled
//     bit 6 : auto meridian flip
//
//   Byte 2 — track rate / compensation
//     bits 0-2 : track rate (0=sidereal, 1=lunar, 2=solar, 3=king, 4=custom)
//     bits 3-4 : track comp (0=none, 1=refraction_dual, 2=refraction_single,
//                            3=ontrack_dual, 4=ontrack_single)
//
//   Byte 3 — pier side / mount type
//     bits 0-1 : pier side (0=none, 1=east, 2=west)
//     bits 2-3 : mount type (0=GEM, 1=FORK, 2=ALTAZM, 3=ALTALT)
//
//   Byte 4 — PEC
//     bit 0 : PEC data recorded
//     bits 1-3 : PEC state (0=ignore, 1=ready_play, 2=playing,
//                           3=ready_record, 4=recording)
//
//   Byte 5 : pulse guide rate select (0x80 | rate, rate 0-9)
//   Byte 6 : guide rate select       (0x80 | rate, rate 0-9)
//   Byte 7 : reserved
//   Byte 8 : error code              (0x80 | code, code 0-9)
//
// Note: this bit layout must be validated against OnStepX firmware source
// when hardware testing becomes available. The format validation (9 bytes,
// all >= 0x80) is definitive; the field assignments are provisional.
// ---------------------------------------------------------------------------
bool OnStepXStatus::parseGu(const uint8_t *bytes, int len, MountStatus &out)
{
    if (len != 9)
        return false;

    for (int i = 0; i < 9; i++)
        if (bytes[i] < 0x80)
            return false;

    out = MountStatus{};    // reset to defaults

    // Byte 0 — tracking / goto / park / home
    uint8_t b0 = bytes[0] & 0x7F;
    out.tracking        = !(b0 & 0x01);
    out.gotoActive      =  (b0 & 0x02) == 0;   // bit 1 set → no goto
    {
        uint8_t parkBits = (b0 >> 2) & 0x03;
        switch (parkBits)
        {
            case 1:  out.parkState = MountStatus::ParkState::PARKING; break;
            case 2:  out.parkState = MountStatus::ParkState::PARKED;  break;
            case 3:  out.parkState = MountStatus::ParkState::FAILED;  break;
            default: out.parkState = MountStatus::ParkState::UNPARKED; break;
        }
    }
    out.atHome          = (b0 & 0x10) != 0;
    out.homing          = (b0 & 0x20) != 0;
    out.autoHomeAtBoot  = (b0 & 0x40) != 0;

    // Byte 1 — guide / misc
    uint8_t b1 = bytes[1] & 0x7F;
    out.pulseGuideActive  = (b1 & 0x01) != 0;
    out.guideActive       = (b1 & 0x02) != 0;
    out.ppsSynced         = (b1 & 0x04) != 0;
    out.waitingAtHome     = (b1 & 0x08) != 0;
    out.pauseAtHomeEnabled = (b1 & 0x10) != 0;
    out.buzzerEnabled     = (b1 & 0x20) != 0;
    out.autoMeridianFlip  = (b1 & 0x40) != 0;

    // Byte 2 — track rate / compensation
    uint8_t b2 = bytes[2] & 0x7F;
    switch (b2 & 0x07)
    {
        case 1:  out.trackRate = MountStatus::TrackRate::LUNAR;  break;
        case 2:  out.trackRate = MountStatus::TrackRate::SOLAR;  break;
        case 3:  out.trackRate = MountStatus::TrackRate::KING;   break;
        case 4:  out.trackRate = MountStatus::TrackRate::CUSTOM; break;
        default: out.trackRate = MountStatus::TrackRate::SIDEREAL; break;
    }
    switch ((b2 >> 3) & 0x07)
    {
        case 1: out.trackComp = MountStatus::TrackComp::REFRACTION_DUAL;    break;
        case 2: out.trackComp = MountStatus::TrackComp::REFRACTION_SINGLE;  break;
        case 3: out.trackComp = MountStatus::TrackComp::ONTRACK_DUAL;       break;
        case 4: out.trackComp = MountStatus::TrackComp::ONTRACK_SINGLE;     break;
        default: out.trackComp = MountStatus::TrackComp::NONE;              break;
    }

    // Byte 3 — pier side / mount type
    uint8_t b3 = bytes[3] & 0x7F;
    switch (b3 & 0x03)
    {
        case 1:  out.pierSide = MountStatus::PierSide::EAST; break;
        case 2:  out.pierSide = MountStatus::PierSide::WEST; break;
        default: out.pierSide = MountStatus::PierSide::NONE; break;
    }
    switch ((b3 >> 2) & 0x03)
    {
        case 1:  out.mountType = MountStatus::MountType::FORK;   break;
        case 2:  out.mountType = MountStatus::MountType::ALTAZM; break;
        case 3:  out.mountType = MountStatus::MountType::ALTALT; break;
        default: out.mountType = MountStatus::MountType::GEM;    break;
    }

    // Byte 4 — PEC
    uint8_t b4 = bytes[4] & 0x7F;
    out.pecRecorded = (b4 & 0x01) != 0;
    switch ((b4 >> 1) & 0x07)
    {
        case 1:  out.pecState = MountStatus::PecState::READY_PLAY;    break;
        case 2:  out.pecState = MountStatus::PecState::PLAYING;       break;
        case 3:  out.pecState = MountStatus::PecState::READY_RECORD;  break;
        case 4:  out.pecState = MountStatus::PecState::RECORDING;     break;
        default: out.pecState = MountStatus::PecState::IGNORE;        break;
    }

    // Bytes 5-6: rates; byte 8: error code
    out.pulseGuideRateSelect = bytes[5] & 0x7F;
    out.guideRateSelect      = bytes[6] & 0x7F;
    out.errorCode            = bytes[8] & 0x7F;

    return true;
}
