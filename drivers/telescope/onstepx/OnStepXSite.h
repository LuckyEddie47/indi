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

#pragma once

#include <libnova/ln_types.h>

class OnStepXComm;
namespace INDI { class DefaultDevice; }

// Handles site location and time synchronisation for the mount binary.
// Plain C++ -- no INDI base class.  Holds borrowed pointers (non-owning).
class OnStepXSite
{
    public:
        void setComm(OnStepXComm *comm)          { m_comm = comm; }
        void setDevice(INDI::DefaultDevice *dev)  { m_dev  = dev; }

        // Write latitude / longitude / elevation to the controller.
        //   latitude  : decimal degrees, -90..+90  (North positive)
        //   longitude : decimal degrees, 0..360    (East positive, INDI convention)
        //   elevation : metres above sea level
        bool writeLocation(double latitude, double longitude, double elevation);

        // Read back location from the controller.
        //   latitude  : decimal degrees, -90..+90
        //   longitude : decimal degrees, 0..360  (East positive, converted from OnStepX West-positive)
        bool readLocation(double &latitude, double &longitude, double &elevation);

        // Push UTC date/time + timezone offset to the controller.
        //   utc        : broken-down UTC date/time (libnova)
        //   utc_offset : hours east of UTC (e.g. +10 for AEST, -5 for EST)
        bool writeTime(const ln_date *utc, double utc_offset);

    private:
        OnStepXComm        *m_comm { nullptr };
        INDI::DefaultDevice *m_dev { nullptr };
};
