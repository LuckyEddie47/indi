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

    Plain C++ helper -- no INDI base class.  Non-owning borrowed pointers.
    Implements updateLocation() and updateTime() for OnStepXMount.

    Protocol (OnStepX v10.24c):
      :St[±DD:MM:SS.S]# — set latitude, North positive (reply '1')
      :Sg[DDD:MM:SS.S]# — set longitude, West positive (INDI convention is East; inverted here)
      :Sv[n]#           — set elevation in metres (reply '1')
      :Gt#              — get latitude (sexagesimal, North positive)
      :Gg#              — get longitude (sexagesimal, West positive)
      :Gev#             — get elevation (integer, metres)
      :SG[±HH.H]#       — set UTC offset, hours East of UTC (reply '1')
      :SL[HH:MM:SS]#    — set local time (reply '1')
      :SC[MM/DD/YY]#    — set local date (reply '1')
*/

#pragma once

#include <libnova/ln_types.h>

class OnStepXComm;
namespace INDI { class DefaultDevice; }

// Plain C++ -- no INDI base class.  Non-owning borrowed pointers.
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
