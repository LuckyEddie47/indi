/*
    OnStep X INDI Driver — Site/time helper (mount binary only)

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

#include "OnStepXSite.h"
#include "OnStepXComm.h"

#include <indilogger.h>
#include <indicom.h>                    // f_scansexa, getSexComponentsIID
#include <libnova/utility.h>            // ln_date_to_zonedate
#include <libnova/julian_day.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// writeLocation
// ---------------------------------------------------------------------------
bool OnStepXSite::writeLocation(double latitude, double longitude, double elevation)
{
    char reply[64];

    // --- Latitude  (+DD:MM:SS, signed) ---
    int latD, latM;
    double latS;
    getSexComponentsIID(std::fabs(latitude), &latD, &latM, &latS);
    char latCmd[32];
    snprintf(latCmd, sizeof(latCmd), ":St%c%02d:%02d:%04.1f#",
             latitude >= 0 ? '+' : '-', latD, latM, latS);

    if (!m_comm->sendCommand(latCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_ERROR, __FILE__, __LINE__,
                "Failed to set site latitude");
        return false;
    }

    // --- Longitude  (DDD:MM:SS, West-positive as OnStepX expects) ---
    // INDI sends 0-360 East; OnStepX :Sg# expects West-positive (old LX200 convention).
    double osx_lon = 360.0 - longitude;
    if (osx_lon >= 360.0) osx_lon -= 360.0;
    if (osx_lon <    0.0) osx_lon += 360.0;

    int lonD, lonM;
    double lonS;
    getSexComponentsIID(osx_lon, &lonD, &lonM, &lonS);
    char lonCmd[32];
    snprintf(lonCmd, sizeof(lonCmd), ":Sg%03d:%02d:%04.1f#", lonD, lonM, lonS);

    if (!m_comm->sendCommand(lonCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_ERROR, __FILE__, __LINE__,
                "Failed to set site longitude");
        return false;
    }

    // --- Elevation (integer metres) ---
    char elvCmd[32];
    snprintf(elvCmd, sizeof(elvCmd), ":Sv%d#", static_cast<int>(std::round(elevation)));
    if (!m_comm->sendCommand(elvCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_WARNING, __FILE__, __LINE__,
                "Failed to set site elevation (non-critical)");
        // Elevation write failure is non-critical -- continue.
    }

    if (m_dev)
        INDI::Logger::getInstance().print(m_dev->getDeviceName(),
            INDI::Logger::DBG_SESSION, __FILE__, __LINE__,
            "Site location sent to controller");

    return true;
}

// ---------------------------------------------------------------------------
// readLocation
// ---------------------------------------------------------------------------
bool OnStepXSite::readLocation(double &latitude, double &longitude, double &elevation)
{
    char reply[64];

    // Latitude -- :GtH# returns "+DD:MM:SS.S"
    if (!m_comm->sendCommand(":GtH#", reply))
        return false;
    double lat = 0;
    if (f_scansexa(reply, &lat) < 0)
        return false;
    latitude = lat;

    // Longitude -- :GgH# returns West-positive "DDD:MM:SS.S"; convert to East-positive.
    if (!m_comm->sendCommand(":GgH#", reply))
        return false;
    double osx_lon = 0;
    if (f_scansexa(reply, &osx_lon) < 0)
        return false;
    longitude = 360.0 - osx_lon;
    if (longitude >= 360.0) longitude -= 360.0;
    if (longitude <    0.0) longitude += 360.0;

    // Elevation -- :Gv# returns integer string (metres)
    if (!m_comm->sendCommand(":Gv#", reply))
    {
        elevation = 0;  // non-critical -- use zero if unavailable
    }
    else
    {
        elevation = std::atof(reply);
    }

    return true;
}

// ---------------------------------------------------------------------------
// writeTime
// ---------------------------------------------------------------------------
bool OnStepXSite::writeTime(const ln_date *utc, double utc_offset)
{
    char reply[64];

    // 1. Set UTC offset (timezone).
    //    OnStepX accepts ":SG[sHH]#" or ":SG[sHH:MM]#" where MM is 00, 30, or 45.
    //    Snap the fractional-hour part to the nearest valid minute value.
    int    tzH       = static_cast<int>(utc_offset);          // integer hours (signed)
    double fracHour  = utc_offset - tzH;                       // e.g. 0.5 for +5:30
    // Handle negative offsets: e.g. -5.5 → tzH=-5, fracHour=-0.5 → abs=0.5
    if (fracHour < 0) fracHour = -fracHour;
    int fracMin = static_cast<int>(std::round(fracHour * 60.0)); // 0, 30, or 45 typical
    // Snap to nearest allowed value (0, 30, 45) per firmware spec
    if      (fracMin < 15) fracMin = 0;
    else if (fracMin < 37) fracMin = 30;
    else if (fracMin < 53) fracMin = 45;
    else                 { fracMin = 0; tzH += (utc_offset >= 0 ? 1 : -1); }

    char tzCmd[32];
    if (fracMin == 0)
        snprintf(tzCmd, sizeof(tzCmd), ":SG%+03d#", tzH);
    else
        snprintf(tzCmd, sizeof(tzCmd), ":SG%+03d:%02d#", tzH, fracMin);

    if (!m_comm->sendCommand(tzCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_ERROR, __FILE__, __LINE__,
                "Failed to set UTC offset (:SG#)");
        return false;
    }

    // 2. Compute local time and date by applying the UTC offset.
    //    Use the exact (unsnapped) utc_offset for the local time calculation.
    ln_zonedate lzd;
    ln_date mutable_utc = *utc;
    ln_date_to_zonedate(&mutable_utc, &lzd, static_cast<int>(utc_offset * 3600.0));

    // 3. Send local time  ":SLHH:MM:SS#"
    int secs = static_cast<int>(lzd.seconds);
    char timeCmd[32];
    snprintf(timeCmd, sizeof(timeCmd), ":SL%02d:%02d:%02d#", lzd.hours, lzd.minutes, secs);
    if (!m_comm->sendCommand(timeCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_ERROR, __FILE__, __LINE__,
                "Failed to set local time (:SL#)");
        return false;
    }

    // 4. Send local date  ":SCMM/DD/YY#"
    //    Reply may be "1Updating        #" -- only first char matters.
    char dateCmd[32];
    snprintf(dateCmd, sizeof(dateCmd), ":SC%02d/%02d/%02d#",
             lzd.months, lzd.days, lzd.years % 100);
    if (!m_comm->sendCommand(dateCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            INDI::Logger::getInstance().print(m_dev->getDeviceName(),
                INDI::Logger::DBG_ERROR, __FILE__, __LINE__,
                "Failed to set local date (:SC#)");
        return false;
    }

    if (m_dev)
        INDI::Logger::getInstance().print(m_dev->getDeviceName(),
            INDI::Logger::DBG_SESSION, __FILE__, __LINE__,
            "Date/time sent to controller");

    return true;
}

