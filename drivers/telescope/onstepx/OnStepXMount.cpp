/*
    OnStep X INDI Driver

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

OnStepXMount::OnStepXMount()
{
    setVersion(0, 1);
    m_core.setDevice(this);
    SetTelescopeCapability(
        TELESCOPE_CAN_GOTO       |
        TELESCOPE_CAN_SYNC       |
        TELESCOPE_CAN_PARK       |
        TELESCOPE_CAN_ABORT      |
        TELESCOPE_HAS_TIME       |
        TELESCOPE_HAS_LOCATION   |
        TELESCOPE_HAS_TRACK_MODE |
        TELESCOPE_CAN_CONTROL_TRACK,
        4);
}

const char *OnStepXMount::getDefaultName()
{
    return "OnStep X";
}

bool OnStepXMount::initProperties()
{
    INDI::Telescope::initProperties();
    SetParkDataType(PARK_RA_DEC);
    return true;
}

bool OnStepXMount::updateProperties()
{
    INDI::Telescope::updateProperties();
    return true;
}

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

bool OnStepXMount::ReadScopeStatus()
{
    return false;
}

bool OnStepXMount::Goto(double ra, double dec)
{
    INDI_UNUSED(ra);
    INDI_UNUSED(dec);
    return false;
}

bool OnStepXMount::Sync(double ra, double dec)
{
    INDI_UNUSED(ra);
    INDI_UNUSED(dec);
    return false;
}

bool OnStepXMount::Abort()
{
    return false;
}

bool OnStepXMount::Park()
{
    return false;
}

bool OnStepXMount::UnPark()
{
    return false;
}

bool OnStepXMount::SetCurrentPark()
{
    return false;
}

bool OnStepXMount::SetDefaultPark()
{
    return false;
}

bool OnStepXMount::SetTrackEnabled(bool enabled)
{
    INDI_UNUSED(enabled);
    return false;
}

bool OnStepXMount::SetTrackMode(uint8_t mode)
{
    INDI_UNUSED(mode);
    return false;
}

bool OnStepXMount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    return false;
}

bool OnStepXMount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    return false;
}

bool OnStepXMount::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(latitude);
    INDI_UNUSED(longitude);
    INDI_UNUSED(elevation);
    return false;
}

bool OnStepXMount::updateTime(ln_date *utc, double utc_offset)
{
    INDI_UNUSED(utc);
    INDI_UNUSED(utc_offset);
    return false;
}

bool OnStepXMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool OnStepXMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool OnStepXMount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool OnStepXMount::saveConfigItems(FILE *fp)
{
    return INDI::Telescope::saveConfigItems(fp);
}
