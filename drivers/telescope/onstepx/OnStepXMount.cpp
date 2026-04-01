/*
    OnStep X INDI Driver — Mount device class

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

#include "OnStepXMount.h"

#include <indicom.h>
#include <libastro.h>
#include <libnova/julian_day.h>

#include <cmath>
#include <cstdio>
#include <cstring>

// Bring alignment types into scope without polluting the global namespace.
using INDI::AlignmentSubsystem::AlignmentDatabaseEntry;
using INDI::AlignmentSubsystem::TelescopeDirectionVector;

// ---------------------------------------------------------------------------
// Track mode indices — must match addTrackMode() call order in initProperties()
// ---------------------------------------------------------------------------
static constexpr int TRACK_SIDEREAL = 0;
static constexpr int TRACK_LUNAR    = 1;
static constexpr int TRACK_SOLAR    = 2;
static constexpr int TRACK_KING     = 3;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
OnStepXMount::OnStepXMount()
{
    setVersion(0, 1);
    m_core.setDevice(this);

    // Provisional capability set — refined in Handshake() once probed.
    SetTelescopeCapability(
        TELESCOPE_CAN_GOTO       |
        TELESCOPE_CAN_SYNC       |
        TELESCOPE_CAN_PARK       |
        TELESCOPE_CAN_ABORT      |
        TELESCOPE_HAS_TIME       |
        TELESCOPE_HAS_LOCATION   |
        TELESCOPE_HAS_TRACK_MODE |
        TELESCOPE_CAN_CONTROL_TRACK,
        4);   // 4 slew rates
}

// ---------------------------------------------------------------------------
// Identification
// ---------------------------------------------------------------------------
const char *OnStepXMount::getDefaultName()
{
    return "OnStep X";
}

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
bool OnStepXMount::initProperties()
{
    INDI::Telescope::initProperties();
    SetParkDataType(PARK_RA_DEC);

    // Track modes — order must match TRACK_* constants above
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_LUNAR",    "Lunar");
    AddTrackMode("TRACK_SOLAR",    "Solar");
    AddTrackMode("TRACK_KING",     "King");

    // Slew rate labels are set by the base class (inditelescope.cpp) when
    // SetTelescopeCapability(caps, 4) is called — no override needed here.

    // Alignment subsystem — initialised after Handshake for AltAz only,
    // but the AlignmentSubsystemForDrivers members are harmless to construct.

    return true;
}

// ---------------------------------------------------------------------------
// updateProperties — register/deregister properties on connect/disconnect
// ---------------------------------------------------------------------------
bool OnStepXMount::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        if (!isEquatorial())
            InitAlignmentProperties(this);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Handshake
// ---------------------------------------------------------------------------
bool OnStepXMount::Handshake()
{
    m_core.setFd(PortFD);

    if (!m_core.probeController())
    {
        LOG_ERROR("Not an OnStepX controller. Aborting connection.");
        return false;
    }

    if (!m_core.probeMount())
    {
        LOG_ERROR("No mount detected. For mount-less OnStepX use indi_onstepx_aux instead.");
        return false;
    }

    const Capabilities &cap = m_core.caps();

    uint32_t telescopeCaps =
        TELESCOPE_CAN_GOTO | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_PARK |
        TELESCOPE_CAN_ABORT | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION |
        TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK;

    if (cap.hasPierSide)
        telescopeCaps |= TELESCOPE_HAS_PIER_SIDE;

    SetTelescopeCapability(telescopeCaps, 4);

    return true;
}

// ---------------------------------------------------------------------------
// ReadScopeStatus — ≤80 lines, delegates to named updaters
// ---------------------------------------------------------------------------
bool OnStepXMount::ReadScopeStatus()
{
    if (!refreshMountStatus())
        return false;

    if (!updateCoordinates())
        return false;

    updateTrackingState(m_status);
    updateSlewState(m_status);

    if (m_core.caps().hasPierSide)
    {
        if (m_status.pierSide == MountStatus::PierSide::EAST)
            setPierSide(PIER_EAST);
        else if (m_status.pierSide == MountStatus::PierSide::WEST)
            setPierSide(PIER_WEST);
    }

    m_pollCount++;
    if (m_pollCount % 5  == 0) updateFocuserStates();
    if (m_pollCount % 10 == 0) updateRotatorState();
    if (m_pollCount % 30 == 0) updateWeatherState();
    if (m_pollCount % 5  == 0) updateFeatureStates();
    updateStatusText(m_status);

    return true;
}

// ---------------------------------------------------------------------------
// refreshMountStatus — :Gu# preferred; falls back to :GU#
// ---------------------------------------------------------------------------
bool OnStepXMount::refreshMountStatus()
{
    if (m_core.caps().hasBinaryStatus)
    {
        uint8_t bin[9];
        if (m_core.comm().sendCommandReadN(":Gu#", bin, 9))
        {
            if (OnStepXStatus::parseGu(bin, 9, m_status))
                return true;
            LOG_WARN("Binary :Gu# parse failed -- falling back to :GU#");
        }
    }

    char reply[256];
    if (!m_core.comm().sendCommand(":GU#", reply))
    {
        LOG_ERROR("Failed to read mount status (:GU#)");
        return false;
    }

    if (!OnStepXStatus::parseGU(reply, m_status))
    {
        LOGF_ERROR("Failed to parse :GU# reply: '%s'", reply);
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// updateCoordinates — equatorial or AltAz path
// ---------------------------------------------------------------------------
bool OnStepXMount::updateCoordinates()
{
    char reply[256];

    if (isEquatorial())
    {
        // OnStepX handles its own pointing model — use firmware coords directly.
        double ra = 0, dec = 0;

        if (!m_core.comm().sendCommand(":GRH#", reply))
        {
            LOG_ERROR("Failed to read RA (:GRH#)");
            return false;
        }
        if (f_scansexa(reply, &ra) < 0)
        {
            LOGF_ERROR("Failed to parse RA: '%s'", reply);
            return false;
        }

        if (!m_core.comm().sendCommand(":GDH#", reply))
        {
            LOG_ERROR("Failed to read Dec (:GDH#)");
            return false;
        }
        if (f_scansexa(reply, &dec) < 0)
        {
            LOGF_ERROR("Failed to parse Dec: '%s'", reply);
            return false;
        }

        NewRaDec(ra, dec);
    }
    else
    {
        // AltAz: read Az/Alt and convert to RA/Dec via alignment subsystem.
        double az = 0, alt = 0;

        if (!m_core.comm().sendCommand(":GZH#", reply))
        {
            LOG_ERROR("Failed to read Az (:GZH#)");
            return false;
        }
        if (f_scansexa(reply, &az) < 0)
        {
            LOGF_ERROR("Failed to parse Az: '%s'", reply);
            return false;
        }

        if (!m_core.comm().sendCommand(":GAH#", reply))
        {
            LOG_ERROR("Failed to read Alt (:GAH#)");
            return false;
        }
        if (f_scansexa(reply, &alt) < 0)
        {
            LOGF_ERROR("Failed to parse Alt: '%s'", reply);
            return false;
        }

        INDI::IHorizontalCoordinates hor { az, alt };
        INDI::IEquatorialCoordinates eq  { 0, 0 };

        // Try alignment subsystem correction first.
        TelescopeDirectionVector tdv =
            TelescopeDirectionVectorFromAltitudeAzimuth(hor);
        double raOut = 0, decOut = 0;
        if (TransformTelescopeToCelestial(tdv, raOut, decOut))
        {
            NewRaDec(raOut, decOut);
        }
        else
        {
            // Fallback: raw geometric conversion.
            INDI::HorizontalToEquatorial(&hor, &m_Location,
                                         ln_get_julian_from_sys(), &eq);
            NewRaDec(eq.rightascension, eq.declination);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// updateTrackingState
// ---------------------------------------------------------------------------
void OnStepXMount::updateTrackingState(const MountStatus &s)
{
    if (s.parkState == MountStatus::ParkState::PARKED)
    {
        TrackState = SCOPE_PARKED;
        return;
    }
    if (s.homing || s.parkState == MountStatus::ParkState::PARKING)
    {
        TrackState = SCOPE_SLEWING;
        return;
    }
    TrackState = s.tracking ? SCOPE_TRACKING : SCOPE_IDLE;
}

// ---------------------------------------------------------------------------
// updateSlewState
// ---------------------------------------------------------------------------
void OnStepXMount::updateSlewState(const MountStatus &s)
{
    if (s.gotoActive)
        TrackState = SCOPE_SLEWING;
}

// ---------------------------------------------------------------------------
// updateStatusText — called every poll cycle (stub; expands in later stages)
// ---------------------------------------------------------------------------
void OnStepXMount::updateStatusText(const MountStatus &s)
{
    INDI_UNUSED(s);
}

// ---------------------------------------------------------------------------
// isEquatorial
// ---------------------------------------------------------------------------
bool OnStepXMount::isEquatorial() const
{
    ::MountType mt = m_core.caps().mountType;
    return (mt == ::MountType::GEM || mt == ::MountType::FORK);
}

// ---------------------------------------------------------------------------
// Goto
// ---------------------------------------------------------------------------
bool OnStepXMount::Goto(double ra, double dec)
{
    // Format RA as HH:MM:SS.S and Dec as ±DD:MM:SS
    char raStr[32], decStr[32];
    fs_sexa(raStr,  ra,  2, 360000);   // RA:  2-digit hour field,  0.1-arcsec precision
    fs_sexa(decStr, dec, 3, 360000);   // Dec: 3-digit degree field (±90)

    char cmd[64];

    snprintf(cmd, sizeof(cmd), ":Sr%s#", raStr);
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOG_ERROR("Goto: failed to set RA");
        return false;
    }

    snprintf(cmd, sizeof(cmd), ":Sd%s#", decStr);
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOG_ERROR("Goto: failed to set Dec");
        return false;
    }

    char reply[256];
    if (!m_core.comm().sendCommand(":MS#", reply))
    {
        LOG_ERROR("Goto: :MS# failed");
        return false;
    }

    if (reply[0] != '0')
    {
        LOGF_ERROR("Goto rejected by firmware (reply: '%s')", reply);
        return false;
    }

    TrackState = SCOPE_SLEWING;
    LOGF_INFO("Slewing to RA: %s  Dec: %s", raStr, decStr);
    return true;
}

// ---------------------------------------------------------------------------
// Sync
// ---------------------------------------------------------------------------
bool OnStepXMount::Sync(double ra, double dec)
{
    char raStr[32], decStr[32];
    fs_sexa(raStr,  ra,  2, 360000);
    fs_sexa(decStr, dec, 3, 360000);

    char cmd[64];

    snprintf(cmd, sizeof(cmd), ":Sr%s#", raStr);
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOG_ERROR("Sync: failed to set RA");
        return false;
    }

    snprintf(cmd, sizeof(cmd), ":Sd%s#", decStr);
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOG_ERROR("Sync: failed to set Dec");
        return false;
    }

    char reply[256];
    if (!m_core.comm().sendCommand(":CS#", reply))
    {
        LOG_ERROR("Sync: :CS# failed");
        return false;
    }

    // For AltAz mounts, add an alignment point to the subsystem.
    if (!isEquatorial())
    {
        char azReply[256], altReply[256];
        double az = 0, alt = 0;
        if (m_core.comm().sendCommand(":GZH#", azReply)  && f_scansexa(azReply,  &az)  == 0 &&
            m_core.comm().sendCommand(":GAH#", altReply) && f_scansexa(altReply, &alt) == 0)
        {
            INDI::IHorizontalCoordinates hor { az, alt };
            AlignmentDatabaseEntry entry;
            entry.ObservationJulianDate = ln_get_julian_from_sys();
            entry.RightAscension        = ra;
            entry.Declination           = dec;
            entry.TelescopeDirection    = TelescopeDirectionVectorFromAltitudeAzimuth(hor);
            entry.PrivateDataSize       = 0;
            GetAlignmentDatabase().push_back(entry);
            UpdateSize();
            Initialise(this);
        }
    }

    LOGF_INFO("Sync to RA: %s  Dec: %s", raStr, decStr);
    return true;
}

// ---------------------------------------------------------------------------
// Abort
// ---------------------------------------------------------------------------
bool OnStepXMount::Abort()
{
    if (!m_core.comm().sendCommandBlind(":Q#"))
    {
        LOG_ERROR("Abort failed");
        return false;
    }
    TrackState = SCOPE_TRACKING;
    return true;
}

// ---------------------------------------------------------------------------
// Park / UnPark / SetCurrentPark / SetDefaultPark
// ---------------------------------------------------------------------------
bool OnStepXMount::Park()
{
    if (!m_core.comm().sendCommandBlind(":hP#"))
    {
        LOG_ERROR("Park command failed");
        return false;
    }
    TrackState = SCOPE_PARKING;
    return true;
}

bool OnStepXMount::UnPark()
{
    if (!m_core.comm().sendCommandBlind(":hR#"))
    {
        LOG_ERROR("UnPark command failed");
        return false;
    }
    SetParked(false);
    TrackState = SCOPE_TRACKING;
    return true;
}

bool OnStepXMount::SetCurrentPark()
{
    if (!m_core.comm().sendCommandBlind(":hQ#"))
    {
        LOG_ERROR("SetCurrentPark command failed");
        return false;
    }
    return true;
}

bool OnStepXMount::SetDefaultPark()
{
    // OnStepX has no "default park" concept distinct from the current park position.
    return SetCurrentPark();
}

// ---------------------------------------------------------------------------
// Tracking
// ---------------------------------------------------------------------------
bool OnStepXMount::SetTrackEnabled(bool enabled)
{
    const char *cmd = enabled ? ":Te#" : ":Td#";
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOGF_ERROR("SetTrackEnabled(%d) failed", enabled);
        return false;
    }
    return true;
}

bool OnStepXMount::SetTrackMode(uint8_t mode)
{
    const char *cmd = nullptr;
    switch (mode)
    {
        case TRACK_SIDEREAL: cmd = ":T+#"; break;
        case TRACK_LUNAR:    cmd = ":TL#"; break;
        case TRACK_SOLAR:    cmd = ":TS#"; break;
        case TRACK_KING:     cmd = ":TK#"; break;
        default:
            LOGF_ERROR("SetTrackMode: unknown mode %d", mode);
            return false;
    }
    if (!m_core.comm().sendCommandBlind(cmd))
    {
        LOGF_ERROR("SetTrackMode(%d) command failed", mode);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Slew rate
// ---------------------------------------------------------------------------
bool OnStepXMount::SetSlewRate(int index)
{
    const char *cmds[] = { ":RG#", ":RC#", ":RM#", ":RS#" };
    if (index < 0 || index > 3)
    {
        LOGF_ERROR("SetSlewRate: invalid index %d", index);
        return false;
    }
    if (!m_core.comm().sendCommandBlind(cmds[index]))
    {
        LOGF_ERROR("SetSlewRate(%d) command failed", index);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Manual motion (arrow buttons in Ekos)
// ---------------------------------------------------------------------------
bool OnStepXMount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (command == MOTION_START)
    {
        const char *cmd = (dir == DIRECTION_NORTH) ? ":Mn#" : ":Ms#";
        if (!m_core.comm().sendCommandBlind(cmd))
        {
            LOG_ERROR("MoveNS start failed");
            return false;
        }
    }
    else
    {
        if (!m_core.comm().sendCommandBlind(":Q#"))
        {
            LOG_ERROR("MoveNS stop failed");
            return false;
        }
    }
    return true;
}

bool OnStepXMount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (command == MOTION_START)
    {
        const char *cmd = (dir == DIRECTION_WEST) ? ":Mw#" : ":Me#";
        if (!m_core.comm().sendCommandBlind(cmd))
        {
            LOG_ERROR("MoveWE start failed");
            return false;
        }
    }
    else
    {
        if (!m_core.comm().sendCommandBlind(":Q#"))
        {
            LOG_ERROR("MoveWE stop failed");
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Location / Time — stubs; implemented fully in Stage 5
// ---------------------------------------------------------------------------
bool OnStepXMount::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(latitude);
    INDI_UNUSED(longitude);
    INDI_UNUSED(elevation);
    return true;   // return true so INDI doesn't report a failure at startup
}

bool OnStepXMount::updateTime(ln_date *utc, double utc_offset)
{
    INDI_UNUSED(utc);
    INDI_UNUSED(utc_offset);
    return true;
}

// ---------------------------------------------------------------------------
// ISNew* — forward unhandled events to base class
// ---------------------------------------------------------------------------
bool OnStepXMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (isConnected() && !isEquatorial())
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (isConnected() && !isEquatorial())
        ProcessAlignmentNumberProperties(this, name, values, names, n);
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool OnStepXMount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (isConnected() && !isEquatorial())
        ProcessAlignmentTextProperties(this, name, texts, names, n);
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool OnStepXMount::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);
    if (!isEquatorial())
        SaveAlignmentConfigProperties(fp);
    return true;
}
