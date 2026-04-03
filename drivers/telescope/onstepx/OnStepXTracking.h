/*
    OnStep X INDI Driver — Tracking control helper (mount binary only)

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
    Advanced tracking properties beyond the standard SetTrackMode/SetTrackRate.

    Protocol (OnStepX v10.24c):
      :To#         — tracking compensation: full (both axes)
      :Tr#         — tracking compensation: refraction only
      :Tn#         — tracking compensation: off
      :T1#         — compensation axes: single
      :T2#         — compensation axes: dual
      :T-#         — frequency nudge: decrease (blind)
      :T+#         — frequency nudge: increase (blind)
      :TR#         — frequency nudge: reset (blind)
      :SX95,0/1#   — meridian auto-flip: 0=off, 1=on (reply '1')
      :GX95#       — read auto-flip setting ('0' / '1')
      :SX96,W/E/B# — preferred pier side (reply '1')
      :GX96#       — read preferred pier side ('W' / 'E' / 'B')
      trackComp state also derived from :GU# status string each poll.

    INDI Properties (Tracking tab):
      OSX_TRACK_COMP       IP_RW  ISR_1OFMANY  3 switches: Full / Refraction / Off
      OSX_TRACK_COMP_AXES  IP_RW  ISR_1OFMANY  2 switches: Single / Dual
      OSX_FREQ_ADJ         IP_RW  ISR_ATMOST1  3 switches: Down / Up / Reset (momentary)
      OSX_AUTO_FLIP        IP_RW  ISR_1OFMANY  2 switches: Off / On
      OSX_PREFERRED_PIER   IP_RW  ISR_1OFMANY  3 switches: West / East / Best
      OSX_SLEW_RATE_MAX    IP_RW  Number[1]    Max slew rate in deg/sec (:GX4C# / :Rs[d.d]#)
      OSX_TRACK_FREQ       IP_RO  Number[1]    Current tracking frequency Hz (:GT#)
*/

#pragma once

#include "OnStepXStatus.h"

#include <defaultdevice.h>

class OnStepXComm;

class OnStepXTracking
{
    public:
        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)          { m_comm = comm; }

        void initProperties();
        void updateProperties(bool connected);

        // Returns true if the event was consumed.
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);
        bool handleNumber(const char *name, double values[], char *names[], int n);

        // Property accessors (used by tests and by OnStepXMount for state sync)
        INDI::PropertyNumber &slewRateMaxNP() { return m_slewRateMaxNP; }
        INDI::PropertyNumber &trackFreqNP()   { return m_trackFreqNP; }

        void saveConfig(FILE *fp);

        // Sync displayed state from a freshly polled MountStatus (trackComp field).
        // Also polls :GT# to update OSX_TRACK_FREQ.
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
        // Max slew rate (deg/sec)
        INDI::PropertyNumber m_slewRateMaxNP   {1};
        // Current tracking frequency (Hz, IP_RO)
        INDI::PropertyNumber m_trackFreqNP     {1};

        int m_syncCount { 0 };  // poll counter for throttling :GT# query
};
