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

#include <indicom.h>                    // f_scansexa, getSexComponentsIID
#include <libnova/utility.h>            // ln_date_to_zonedate
#include <libnova/julian_day.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define SITES_TAB "Sites"

// ---------------------------------------------------------------------------
// initProperties
// ---------------------------------------------------------------------------
void OnStepXSite::initProperties()
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";

    // --- OSX_SITE_SELECT ---
    m_siteSelectSP[0].fill("SITE_1", "Site 1", ISS_ON);
    m_siteSelectSP[1].fill("SITE_2", "Site 2", ISS_OFF);
    m_siteSelectSP[2].fill("SITE_3", "Site 3", ISS_OFF);
    m_siteSelectSP[3].fill("SITE_4", "Site 4", ISS_OFF);
    m_siteSelectSP.fill(dev, "OSX_SITE_SELECT", "Site Profile",
                        SITES_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // --- OSX_SITE_NAME ---
    m_siteNameTP[0].fill("SITE_NAME", "Name", "");
    m_siteNameTP.fill(dev, "OSX_SITE_NAME", "Site Name",
                      SITES_TAB, IP_RW, 60, IPS_IDLE);
}

// ---------------------------------------------------------------------------
// updateProperties
// ---------------------------------------------------------------------------
void OnStepXSite::updateProperties(bool connected)
{
    if (connected)
    {
        m_dev->defineProperty(m_siteSelectSP);
        m_dev->defineProperty(m_siteNameTP);
        readSiteNames();
    }
    else
    {
        m_dev->deleteProperty(m_siteSelectSP);
        m_dev->deleteProperty(m_siteNameTP);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch — OSX_SITE_SELECT
// ---------------------------------------------------------------------------
bool OnStepXSite::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    if (!m_siteSelectSP.isNameMatch(name))
        return false;

    m_siteSelectSP.update(states, names, n);

    // Determine which site was selected (1-based)
    int site = 1;
    for (int i = 0; i < 4; i++)
        if (m_siteSelectSP[i].getState() == ISS_ON) { site = i + 1; break; }

    double lat = 0, lon = 0, elev = 0;
    if (selectSite(site, lat, lon, elev))
    {
        m_lastLat         = lat;
        m_lastLon         = lon;
        m_lastElev        = elev;
        m_locationUpdated = true;
        m_siteSelectSP.setState(IPS_OK);
        readSiteNames();
    }
    else
    {
        m_siteSelectSP.setState(IPS_ALERT);
    }
    m_siteSelectSP.apply();
    return true;
}

// ---------------------------------------------------------------------------
// handleText — OSX_SITE_NAME
// ---------------------------------------------------------------------------
bool OnStepXSite::handleText(const char *name, char *texts[], char *names[], int n)
{
    if (!m_siteNameTP.isNameMatch(name))
        return false;

    m_siteNameTP.update(texts, names, n);
    const char *newName = m_siteNameTP[0].getText();

    // Command depends on active site slot
    const char *cmds[4] = { ":SM", ":SN", ":SO", ":SP" };
    int idx = m_activeSite - 1;
    if (idx < 0 || idx > 3) idx = 0;

    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), "%s%s#", cmds[idx], newName);
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    if (m_comm->sendCommand(cmd, reply) && reply[0] == '1')
    {
        m_siteNameTP.setState(IPS_OK);
        // Refresh switch labels too
        readSiteNames();
    }
    else
    {
        m_siteNameTP.setState(IPS_ALERT);
    }
    m_siteNameTP.apply();
    return true;
}

// ---------------------------------------------------------------------------
// saveConfig — site names are stored in firmware; nothing to persist here
// ---------------------------------------------------------------------------
void OnStepXSite::saveConfig(FILE *fp)
{
    (void)fp;
}

// ---------------------------------------------------------------------------
// readSiteNames — query :GM#/:GN#/:GO#/:GP#; update labels and name field
// ---------------------------------------------------------------------------
void OnStepXSite::readSiteNames()
{
    const char *cmds[4]  = { ":GM#", ":GN#", ":GO#", ":GP#" };
    char reply[OnStepXComm::REPLY_BUF_SIZE];
    char names[4][64]    = { "Site 1", "Site 2", "Site 3", "Site 4" };

    for (int i = 0; i < 4; i++)
    {
        if (m_comm->sendCommand(cmds[i], reply) && reply[0] != '\0')
            snprintf(names[i], sizeof(names[i]), "%s", reply);
        m_siteSelectSP[i].setLabel(names[i]);
    }

    m_siteSelectSP.setState(IPS_OK);
    m_siteSelectSP.apply();

    // Populate name field with active site's name
    int idx = m_activeSite - 1;
    if (idx < 0 || idx > 3) idx = 0;
    m_siteNameTP[0].setText(names[idx]);
    m_siteNameTP.setState(IPS_OK);
    m_siteNameTP.apply();
}

// ---------------------------------------------------------------------------
// selectSite — send :W[n]#, then re-read location
// ---------------------------------------------------------------------------
bool OnStepXSite::selectSite(int n, double &lat, double &lon, double &elev)
{
    // :W[n]# is blind (no reply)
    char cmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(cmd, sizeof(cmd), ":W%d#", n);
    m_comm->sendCommandBlind(cmd);

    m_activeSite = n;

    // Re-read location so caller can update GEOGRAPHIC_COORD
    return readLocation(lat, lon, elev);
}

// ---------------------------------------------------------------------------
// writeLocation
// ---------------------------------------------------------------------------
bool OnStepXSite::writeLocation(double latitude, double longitude, double elevation)
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];

    // --- Latitude  (+DD:MM:SS, signed) ---
    int latD, latM;
    double latS;
    getSexComponentsIID(std::fabs(latitude), &latD, &latM, &latS);
    normaliseDMS(&latD, &latM, &latS);
    char latCmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(latCmd, sizeof(latCmd), ":St%+.02d:%02d:%.02f#",
         (int)latD, latM, latS);

    if (!m_comm->sendCommand(latCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Failed to set site latitude");
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
    normaliseDMS(&lonD, &lonM, &lonS);
    if (lonD >= 360) lonD -= 360;
    char lonCmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(lonCmd, sizeof(lonCmd), ":Sg%.03d:%02d:%.02f#", lonD, lonM, lonS);

    if (!m_comm->sendCommand(lonCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Failed to set site longitude");
        return false;
    }

    // --- Elevation (integer metres) ---
    char elvCmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(elvCmd, sizeof(elvCmd), ":Sv%d#", static_cast<int>(std::round(elevation)));
    if (!m_comm->sendCommand(elvCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_WARN("Failed to set site elevation (non-critical)");
        // Elevation write failure is non-critical -- continue.
    }

    if (m_dev)
        LOG_DEBUG("Site location sent to controller");

    return true;
}

// ---------------------------------------------------------------------------
// readLocation
// ---------------------------------------------------------------------------
bool OnStepXSite::readLocation(double &latitude, double &longitude, double &elevation)
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];

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
    char reply[OnStepXComm::REPLY_BUF_SIZE];

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

    char tzCmd[OnStepXComm::CMD_MAX_LEN];
    if (fracMin == 0)
        snprintf(tzCmd, sizeof(tzCmd), ":SG%+03d#", tzH);
    else
        snprintf(tzCmd, sizeof(tzCmd), ":SG%+03d:%02d#", tzH, fracMin);

    if (!m_comm->sendCommand(tzCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Failed to set UTC offset (:SG#)");
        return false;
    }

    // 2. Compute local time and date by applying the UTC offset.
    //    Use the exact (unsnapped) utc_offset for the local time calculation.
    ln_zonedate lzd;
    ln_date mutable_utc = *utc;
    ln_date_to_zonedate(&mutable_utc, &lzd, static_cast<int>(utc_offset * 3600.0));

    // 3. Send local time  ":SLHH:MM:SS#"
    int secs = static_cast<int>(lzd.seconds);
    char timeCmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(timeCmd, sizeof(timeCmd), ":SL%02d:%02d:%02d#", lzd.hours, lzd.minutes, secs);
    if (!m_comm->sendCommand(timeCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Failed to set local time (:SL#)");
        return false;
    }

    // 4. Send local date  ":SCMM/DD/YY#"
    //    Reply may be "1Updating        #" -- only first char matters.
    char dateCmd[OnStepXComm::CMD_MAX_LEN];
    snprintf(dateCmd, sizeof(dateCmd), ":SC%02d/%02d/%02d#",
             lzd.months, lzd.days, lzd.years % 100);
    if (!m_comm->sendCommand(dateCmd, reply) || reply[0] != '1')
    {
        if (m_dev)
            LOG_ERROR("Failed to set local date (:SC#)");
        return false;
    }

    if (m_dev)
        LOG_DEBUG("Date/time sent to controller");

    return true;
}

void OnStepXSite::normaliseDMS(int *d, int *m, double *s)
{
    // Round to 2dp first, matching the precision of the format string
    *s = round(*s * 100.0) / 100.0;

    if (*s >= 60.0) {
        *s -= 60.0;
        (*m)++;
    }
    if (*m >= 60) {
        *m -= 60;
        (*d)++;
    }
}