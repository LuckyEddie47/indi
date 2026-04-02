/*
    OnStep X INDI Driver — Rotator helper (shared by both binaries)

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

#include <defaultdevice.h>

#include <cstdint>
#include <cstdio>

class OnStepXComm;

// Encapsulates rotator wiring for the device classes.
//
// This helper is NOT a RotatorInterface subclass.  Instead it provides
// concrete implementations of the RotatorInterface virtuals; the device
// class (which IS a RotatorInterface subclass) simply delegates each
// virtual to its m_rotator member.
//
// GotoRotatorNP, RotatorBacklashNP, etc. are protected in RotatorInterface,
// so the helper cannot access them directly.  Instead, pollStatus() and
// readInitial() return plain structs; the device class applies the values
// to its own RI properties.
//
// Protocol (OnStepX v10.24c):
//   :rG#     — get angle (sexagesimal DDD:MM:SS.S, reply '#'-terminated)
//   :rT#     — status ('M'=moving, 'S'=stopped)
//   :rS±DDD:MM:SS# — goto angle (reply '1')
//   :rC#     — goto home (no reply)
//   :rQ#     — abort (no reply, does NOT abort de-rotator)
//   :rb[n]#  — set backlash n steps (reply '1')
//   :rb#     — get backlash (reply: integer '#'-terminated)
//   :r+#     — enable de-rotator (AltAz only; no reply)
//   :r-#     — disable de-rotator (AltAz only; no reply)
//   :GX98#   — probe: 'D'=derotator capable, 'R'=rotator only, '0'=absent
//   :SX98,0/1# — de-rotate parallactic: 0=no, 1=yes (reply '1')
//
// Capabilities set: CAN_ABORT | CAN_HOME | HAS_BACKLASH
//
// De-rotator properties are created only when hasDerotator is true
// (driven by cap.hasDerotator from probeController).
class OnStepXRotator
{
    public:
        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)          { m_comm = comm; }

        // Called from device class initProperties (always)
        void initProperties(bool hasDerotator);

        // Called from device class updateProperties
        void updateProperties(bool connected, bool hasDerotator);

        // Called from device class ISNewSwitch / ISNewNumber
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);
        bool handleNumber(const char *name, double *values, char *names[], int n);

        void saveConfig(FILE *fp);

        // RotatorInterface virtual implementations — device class delegates here
        IPState moveToAngle(double angle);
        bool    abortRotator();
        IPState homeRotator();
        bool    setBacklash(int32_t steps);

        // Data returned by pollStatus() — device class applies to its RI properties
        struct PollResult
        {
            double  angle       { 0 };
            bool    angleValid  { false };
            bool    moving      { false };
            bool    statusValid { false };
        };

        // Data returned by readInitial() — device class applies on connect
        struct InitialState
        {
            double  angle          { 0 };
            bool    angleValid     { false };
            int32_t backlash       { 0 };
            bool    backlashValid  { false };
        };

        // Periodic status poll — call from device's poll throttle (~10 s)
        PollResult    pollStatus();

        // Read current angle and backlash once on connect
        InitialState  readInitial();

    private:
        INDI::DefaultDevice *m_dev  { nullptr };
        OnStepXComm         *m_comm { nullptr };

        // De-rotator: enable/disable (AltAz only)
        INDI::PropertySwitch m_derotateSP       {2};
        // Parallactic tracking mode for de-rotator
        INDI::PropertySwitch m_parallacticSP    {2};

        bool parseAngle(const char *reply, double &angleDeg);
        bool formatAngle(double angleDeg, char *buf, int bufLen);
};
