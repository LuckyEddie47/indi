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
#include <cstring>

#define WEATHER_TAB "Weather"   // tab name for WeatherInterface properties

// Bring alignment types into scope without polluting the global namespace.
using INDI::AlignmentSubsystem::AlignmentDatabaseEntry;
using INDI::AlignmentSubsystem::TelescopeDirectionVector;

// ---------------------------------------------------------------------------
// Track mode indices — must match AddTrackMode() call order in initProperties()
// ---------------------------------------------------------------------------
enum { TRACK_SIDEREAL = 0, TRACK_LUNAR = 1, TRACK_SOLAR = 2, TRACK_KING = 3 };

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
OnStepXMount::OnStepXMount() : INDI::GuiderInterface(this),
                               INDI::RotatorInterface(this),
                               INDI::WeatherInterface(this)
{
    setVersion(0, 1);
    m_alignment.setDevice(this);
    m_core.setDevice(this);
    m_auxFeatures.setDevice(this);
    m_guide.setDevice(this);
    m_guide.setGuiderInterface(this);
    m_info.setDevice(this);
    m_limits.setDevice(this);
    m_pec.setDevice(this);
    m_rotator.setDevice(this);
    m_site.setDevice(this);
    m_tracking.setDevice(this);
    m_weather.setDevice(this);
    // comm pointers set in updateProperties after connect

    // Provisional capability set — refined in Handshake() once probed.
    SetTelescopeCapability(
        TELESCOPE_CAN_GOTO        |
        TELESCOPE_CAN_SYNC        |
        TELESCOPE_CAN_PARK        |
        TELESCOPE_CAN_ABORT       |
        TELESCOPE_HAS_TIME        |
        TELESCOPE_HAS_LOCATION    |
        TELESCOPE_HAS_TRACK_MODE  |
        TELESCOPE_CAN_CONTROL_TRACK |
        TELESCOPE_HAS_TRACK_RATE  |
        TELESCOPE_CAN_HOME_FIND   |
        TELESCOPE_CAN_HOME_SET,
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

    // Limits and home
    m_limits.initProperties();

    // Advanced tracking properties
    m_tracking.initProperties();

    // PEC
    m_pec.initProperties();

    // Alignment
    m_alignment.initProperties();

    // Rotator interface — standard ABS_ROTATOR_ANGLE, ROTATOR_ABORT_MOTION, etc.
    RI::initProperties("Rotator");

    // Custom rotator properties (de-rotator, parallactic — only shown for AltAz/derotator)
    m_rotator.initProperties(false);  // hasDerotator known only after Handshake

    // Guider interface — standard TELESCOPE_TIMED_GUIDE_NS/WE properties
    GI::initProperties(MOTION_TAB);

    // Guide rate and pulse helper
    m_guide.initProperties();

    // Info (firmware display, status text, reticle)
    m_info.initProperties();

    // Weather calibration and DUT1 (owned by weather helper, shown on Weather tab)
    m_weather.initProperties();

    // Weather interface — tab name, parameter group name
    WI::initProperties(WEATHER_TAB, WEATHER_TAB);
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)",    -40,  80, 15);
    addParameter("WEATHER_PRESSURE",    "Pressure (hPa)",     800, 1100, 15);
    addParameter("WEATHER_HUMIDITY",    "Humidity (%)",         0,  100, 15);
    addParameter("WEATHER_DEWPOINT",    "Dew Point (C)",      -40,   40, 15);
    addParameter("OSX_MCU_TEMP",        "MCU Temp (C)",       -20,   80, 15);

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
    GI::updateProperties();
    RI::updateProperties();
    WI::updateProperties();

    if (isConnected())
    {
        if (!isEquatorial())
            InitAlignmentProperties(this);

        m_alignment.setComm(&m_core.comm());
        m_auxFeatures.setComm(&m_core.comm());
        m_guide.setComm(&m_core.comm());
        m_info.setComm(&m_core.comm());
        m_pec.setComm(&m_core.comm());
        m_site.setComm(&m_core.comm());
        m_limits.setComm(&m_core.comm());
        m_rotator.setComm(&m_core.comm());
        m_tracking.setComm(&m_core.comm());
        m_weather.setComm(&m_core.comm());
        m_weather.updateProperties(true);
        m_limits.updateProperties(true, m_core.caps().hasHomeSense);
        m_tracking.updateProperties(true);
        m_guide.updateProperties(true);
        m_info.updateProperties(true, m_core.caps());
        createFocusers();

        if (m_core.caps().hasRotator)
        {
            setDriverInterface(getDriverInterface() | ROTATOR_INTERFACE);
            m_rotator.updateProperties(true, m_core.caps().hasDerotator);
            auto init = m_rotator.readInitial();
            if (init.angleValid)
            {
                GotoRotatorNP[0].setValue(init.angle);
                GotoRotatorNP.setState(IPS_OK);
                GotoRotatorNP.apply();
            }
            if (init.backlashValid)
            {
                RotatorBacklashNP[0].setValue(static_cast<double>(init.backlash));
                RotatorBacklashNP.setState(IPS_OK);
                RotatorBacklashNP.apply();
            }
        }

        if (m_core.caps().featureMask)
            m_auxFeatures.discoverAndDefine(m_core.caps().featureMask);

        if (m_core.caps().hasPec)
            m_pec.updateProperties(true);

        m_alignment.updateProperties(true);
    }
    else
    {
        m_alignment.updateProperties(false);
        m_limits.updateProperties(false, false);
        m_tracking.updateProperties(false);
        m_guide.updateProperties(false);
        m_info.updateProperties(false, {});
        m_weather.updateProperties(false);
        if (m_core.caps().hasRotator)
            m_rotator.updateProperties(false, false);
        m_auxFeatures.deleteAll();
        if (m_core.caps().hasPec)
            m_pec.updateProperties(false);
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

    // Home find/set are always available on OnStepX; pier side is probed.
    uint32_t telescopeCaps =
        TELESCOPE_CAN_GOTO       | TELESCOPE_CAN_SYNC       | TELESCOPE_CAN_PARK  |
        TELESCOPE_CAN_ABORT      | TELESCOPE_HAS_TIME       | TELESCOPE_HAS_LOCATION |
        TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK |
        TELESCOPE_HAS_TRACK_RATE |
        TELESCOPE_CAN_HOME_FIND  | TELESCOPE_CAN_HOME_SET;

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

    m_guide.checkComplete();

    m_pollCount++;
    if (m_pollCount % 5  == 0) updateFocuserStates();
    if (m_pollCount % 10 == 0) updateRotatorState();
    if (m_pollCount % 10 == 0) updatePecStatus();
    if (m_pollCount % 30 == 0) updateAlignmentStatus();
    if (m_pollCount % 30 == 0) updateWeatherState();
    if (m_pollCount % 5  == 0) updateFeatureStates();
    m_info.updateStatus(m_status);

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
        if (TrackState == SCOPE_PARKING)
            SetParked(true);     // Notify INDI park state machine — first transition only
        TrackState = SCOPE_PARKED;
        return;
    }

    if (s.parkState == MountStatus::ParkState::FAILED)
    {
        if (TrackState == SCOPE_PARKING)
        {
            LOG_ERROR("Park failed");
            TrackState = SCOPE_IDLE;
        }
        return;
    }

    if (s.homing || s.parkState == MountStatus::ParkState::PARKING)
    {
        TrackState = SCOPE_SLEWING;
        return;
    }

    TrackState = s.tracking ? SCOPE_TRACKING : SCOPE_IDLE;

    // Sync advanced tracking properties from status
    m_tracking.syncStatus(s);
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
    char reply[64];

    // :Sr# and :Sd# reply '1' on acceptance, '0' on format error.
    // Must use sendCommand (not blind) so firmware rejection is detected.
    snprintf(cmd, sizeof(cmd), ":Sr%s#", raStr);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("Goto: firmware rejected RA '%s'", raStr);
        return false;
    }

    snprintf(cmd, sizeof(cmd), ":Sd%s#", decStr);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("Goto: firmware rejected Dec '%s'", decStr);
        return false;
    }

    char msReply[256];
    if (!m_core.comm().sendCommand(":MS#", msReply))
    {
        LOG_ERROR("Goto: :MS# failed");
        return false;
    }

    if (msReply[0] != '0')
    {
        LOGF_ERROR("Goto rejected by firmware (reply: '%s')", msReply);
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
    char reply[256];

    snprintf(cmd, sizeof(cmd), ":Sr%s#", raStr);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("Sync: firmware rejected RA '%s'", raStr);
        return false;
    }

    snprintf(cmd, sizeof(cmd), ":Sd%s#", decStr);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("Sync: firmware rejected Dec '%s'", decStr);
        return false;
    }

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
    // Do not force a specific TrackState here — the next ReadScopeStatus
    // poll will set it correctly from the firmware status.  Forcing
    // SCOPE_TRACKING was wrong when the mount was idle (not tracking)
    // before the abort.
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
// SetTrackRate — called by INDI when the user selects "Custom" track mode
// and edits TELESCOPE_TRACK_RATE.  raRate / deRate are arcsec/s offsets from
// sidereal.  OnStepX accepts them as:
//   :RA[f]#   RA custom rate (arcsec/s)
//   :RE[f]#   DE custom rate (arcsec/s)
// Both commands return '1' on success.
// ---------------------------------------------------------------------------
bool OnStepXMount::SetTrackRate(double raRate, double deRate)
{
    char cmd[48], reply[4];

    snprintf(cmd, sizeof(cmd), ":RA%f#", raRate);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("SetTrackRate: RA command failed (raRate=%.4f)", raRate);
        return false;
    }

    snprintf(cmd, sizeof(cmd), ":RE%f#", deRate);
    if (!m_core.comm().sendCommand(cmd, reply) || reply[0] != '1')
    {
        LOGF_ERROR("SetTrackRate: DE command failed (deRate=%.4f)", deRate);
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
// Location
// ---------------------------------------------------------------------------
bool OnStepXMount::updateLocation(double latitude, double longitude, double elevation)
{
    return m_site.writeLocation(latitude, longitude, elevation);
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
bool OnStepXMount::updateTime(ln_date *utc, double utc_offset)
{
    return m_site.writeTime(utc, utc_offset);
}

// ---------------------------------------------------------------------------
// updateWeatherState — called from ReadScopeStatus throttle (every ~30 polls)
// ---------------------------------------------------------------------------
void OnStepXMount::updateWeatherState()
{
    WI::checkWeatherUpdate();
}

// ---------------------------------------------------------------------------
// updateWeather — WeatherInterface callback; queries sensors from firmware
// ---------------------------------------------------------------------------
IPState OnStepXMount::updateWeather()
{
    SensorData data = m_weather.readSensors(m_core.caps().hasMcuTemp);
    if (data.temp.ok)      setParameterValue("WEATHER_TEMPERATURE", data.temp.value);
    if (data.pressure.ok)  setParameterValue("WEATHER_PRESSURE",    data.pressure.value);
    if (data.humidity.ok)  setParameterValue("WEATHER_HUMIDITY",    data.humidity.value);
    if (data.dewpoint.ok)  setParameterValue("WEATHER_DEWPOINT",    data.dewpoint.value);
    if (data.mcuTemp.ok)   setParameterValue("OSX_MCU_TEMP",        data.mcuTemp.value);
    return data.anyOk() ? IPS_OK : IPS_IDLE;
}

// ---------------------------------------------------------------------------
// Guide pulse methods — delegate to OnStepXGuide
// ---------------------------------------------------------------------------
IPState OnStepXMount::GuideNorth(uint32_t ms) { return m_guide.guideNorth(ms); }
IPState OnStepXMount::GuideSouth(uint32_t ms) { return m_guide.guideSouth(ms); }
IPState OnStepXMount::GuideEast(uint32_t ms)  { return m_guide.guideEast(ms);  }
IPState OnStepXMount::GuideWest(uint32_t ms)  { return m_guide.guideWest(ms);  }

// ---------------------------------------------------------------------------
// createFocusers — called once from updateProperties on first connect.
// Instantiates OnStepXFocuser objects for each detected slot (1..numFocusers),
// hands them the shared comm object, and announces them to the INDI bus.
// ---------------------------------------------------------------------------
void OnStepXMount::createFocusers()
{
    int nf = m_core.caps().numFocusers;
    for (int i = 0; i < nf && i < (int)m_focusers.size(); i++)
    {
        if (m_focusers[i])
            continue;  // already created (shouldn't happen, but guard anyway)

        m_focusers[i] = std::make_unique<OnStepXFocuser>(i + 1);
        m_focusers[i]->setComm(&m_core.comm());
        // Announce the device: registers it in the global device list so that
        // clients (Ekos) see it as a separate focuser device in the same process.
        m_focusers[i]->ISGetProperties(nullptr);
        // Mark it as connected (no own port — parent's connection is shared)
        m_focusers[i]->setConnected(true, IPS_OK);
        m_focusers[i]->updateProperties();
    }
}

// ---------------------------------------------------------------------------
// updatePecStatus — poll :$QZ?# state (~10 poll throttle)
// ---------------------------------------------------------------------------
void OnStepXMount::updatePecStatus()
{
    if (m_core.caps().hasPec)
        m_pec.pollStatus();
}

// ---------------------------------------------------------------------------
// updateAlignmentStatus — refresh :A?# status (~30 poll throttle)
// ---------------------------------------------------------------------------
void OnStepXMount::updateAlignmentStatus()
{
    m_alignment.updateStatus();
}

// ---------------------------------------------------------------------------
// updateFeatureStates — poll aux feature slot values (~5 poll throttle)
// ---------------------------------------------------------------------------
void OnStepXMount::updateFeatureStates()
{
    if (m_core.caps().featureMask)
        m_auxFeatures.pollStatus();
}

// ---------------------------------------------------------------------------
// updateRotatorState — poll rotator angle and motion status (~10 s throttle)
// ---------------------------------------------------------------------------
void OnStepXMount::updateRotatorState()
{
    if (!m_core.caps().hasRotator)
        return;

    auto r = m_rotator.pollStatus();

    if (r.angleValid)
        GotoRotatorNP[0].setValue(r.angle);

    if (r.statusValid)
        GotoRotatorNP.setState(r.moving ? IPS_BUSY : IPS_OK);

    if (r.angleValid || r.statusValid)
        GotoRotatorNP.apply();
}

// ---------------------------------------------------------------------------
// RotatorInterface overrides — delegate to m_rotator helper
// ---------------------------------------------------------------------------
IPState OnStepXMount::MoveRotator(double angle)
{
    return m_rotator.moveToAngle(angle);
}

bool OnStepXMount::AbortRotator()
{
    return m_rotator.abortRotator();
}

IPState OnStepXMount::HomeRotator()
{
    return m_rotator.homeRotator();
}

bool OnStepXMount::SetRotatorBacklash(int32_t steps)
{
    return m_rotator.setBacklash(steps);
}

// ---------------------------------------------------------------------------
// updateFocuserStates — poll each active focuser (position + temperature)
// ---------------------------------------------------------------------------
void OnStepXMount::updateFocuserStates()
{
    for (auto &f : m_focusers)
    {
        if (f)
            f->pollStatus();
    }
}

// ---------------------------------------------------------------------------
// ISNew* — forward unhandled events to base class
// ---------------------------------------------------------------------------
bool OnStepXMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (RI::processSwitch(dev, name, states, names, n))
        return true;
    if (WI::processSwitch(dev, name, states, names, n))
        return true;
    if (isConnected() && m_info.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_limits.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_tracking.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_core.caps().hasPec && m_pec.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_core.caps().hasRotator && m_rotator.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_core.caps().featureMask && m_auxFeatures.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && m_alignment.handleSwitch(name, states, names, n))
        return true;
    if (isConnected() && !isEquatorial())
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (RI::processNumber(dev, name, values, names, n))
        return true;
    if (GI::processNumber(dev, name, values, names, n))
        return true;
    if (WI::processNumber(dev, name, values, names, n))
        return true;
    if (isConnected() && m_guide.handleNumber(name, values, names, n))
        return true;
    if (isConnected() && m_tracking.handleNumber(name, values, names, n))
        return true;
    if (isConnected() && m_weather.handleNumber(name, values, names, n))
        return true;
    if (isConnected() && m_limits.handleNumber(name, values, names, n))
        return true;
    if (isConnected() && m_core.caps().featureMask && m_auxFeatures.handleNumber(name, values, names, n))
        return true;
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
    RI::saveConfigItems(fp);
    WI::saveConfigItems(fp);
    m_alignment.saveConfig(fp);
    m_auxFeatures.saveConfig(fp);
    m_limits.saveConfig(fp);
    m_guide.saveConfig(fp);
    m_info.saveConfig(fp);
    m_weather.saveConfig(fp);
    m_pec.saveConfig(fp);
    m_rotator.saveConfig(fp);
    m_tracking.saveConfig(fp);
    if (!isEquatorial())
        SaveAlignmentConfigProperties(fp);
    return true;
}

// ---------------------------------------------------------------------------
// ExecuteHomeAction
// ---------------------------------------------------------------------------
IPState OnStepXMount::ExecuteHomeAction(TelescopeHomeAction action)
{
    switch (action)
    {
        case HOME_FIND:
            return m_limits.homeFind() ? IPS_BUSY : IPS_ALERT;
        case HOME_SET:
            return m_limits.homeSet() ? IPS_OK : IPS_ALERT;
        default:
            return IPS_ALERT;
    }
}
