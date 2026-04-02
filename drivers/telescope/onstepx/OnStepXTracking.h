/*
    OnStep X INDI Driver — Tracking control helper

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

#include "OnStepXStatus.h"

#include <defaultdevice.h>

class OnStepXComm;

// Encapsulates the advanced tracking properties for the mount binary:
//
//   OSX_TRACK_COMP       — tracking compensation mode (Full/Refraction/Off)
//                          Write: :To# / :Tr# / :Tn#
//                          Read:  MountStatus.trackComp from :GU#
//
//   OSX_TRACK_COMP_AXES  — compensation axis count (Single/Dual)
//                          Write: :T1# / :T2#
//                          Read:  MountStatus.trackComp from :GU#
//
//   OSX_FREQ_ADJ         — momentary frequency nudge (−/+/Reset)
//                          Write: :T-# / :T+# / :TR# (all blind)
//
//   OSX_AUTO_FLIP        — meridian auto-flip enable (Off/On)
//                          Write: :SX95,0# / :SX95,1#  (reply '1' = ok)
//                          Read:  :GX95#  → '0' / '1'
//
//   OSX_PREFERRED_PIER   — preferred pier side for new gotos (West/East/Best)
//                          Write: :SX96,W# / :SX96,E# / :SX96,B# (reply '1')
//                          Read:  :GX96#  → 'W' / 'E' / 'B'
//
// Plain C++, no INDI base class.  Non-owning borrowed pointers.
class OnStepXTracking
{
    public:
        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)          { m_comm = comm; }

        void initProperties();
        void updateProperties(bool connected);

        // Returns true if the event was consumed.
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);

        void saveConfig(FILE *fp);

        // Sync displayed state from a freshly polled MountStatus (trackComp field).
        void syncStatus(const MountStatus &s);

        // Query :GX95# and :GX96# to initialise autoFlip and preferredPier
        // after connecting (called once from updateProperties when connected).
        void readSettings();

    private:
        INDI::DefaultDevice *m_dev  { nullptr };
        OnStepXComm         *m_comm { nullptr };

        // Tracking compensation mode: Full / Refraction / Off
        INDI::PropertySwitch m_trackCompSP    {3};
        // Compensation axis count: Single / Dual
        INDI::PropertySwitch m_trackAxisSP    {2};
        // Momentary frequency nudge buttons: − / + / Reset
        INDI::PropertySwitch m_freqAdjSP      {3};
        // Meridian auto-flip: Off / On
        INDI::PropertySwitch m_autoFlipSP     {2};
        // Preferred pier side for new gotos: West / East / Best
        INDI::PropertySwitch m_preferredPierSP{3};
};
