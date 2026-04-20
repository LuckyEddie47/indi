/*
    OnStep X INDI Driver — Focuser device (shared by both binaries)

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

    One INDI::Focuser child device per physical focuser slot (slots 1-6).
    The parent device (OnStepXMount or OnStepXAux) creates instances after
    Handshake, calls setComm() to wire in the shared comm object, then
    calls ISGetProperties(nullptr) to announce the device to INDI clients.
    1 tick = 1 micron throughout this class.

    Protocol (OnStepX v10.24c):
      All commands are prefixed with :FA[slot]# to select the focuser slot.
      :FG#    — get position (integer, microns)
      :FM#    — get max position (integer, microns)
      :FT#    — status: 'M'=moving, 'S'=stopped
      :Ft#    — temperature (float, deg C); 999 = no sensor
      :FN[n]# — goto absolute position n microns (reply '1')
      :Fm[n]# — move relative n microns, signed (no reply)
      :FQ#    — abort (no reply)
      :FB[n]# — set backlash n steps (reply '1')
      :FP[n]# — set speed 1-4 (reply '1')

    INDI Properties (standard FocuserInterface):
      ABS_FOCUS_POSITION    IP_RW  1 number: position (microns)
      REL_FOCUS_POSITION    IP_RW  1 number: relative move (microns)
      FOCUS_ABORT_MOTION    IP_RW  1 switch
      FOCUS_BACKLASH_STEPS  IP_RW  1 number: backlash (steps)
      OSX_FOCUS_TEMPERATURE IP_RO  1 number: temperature (deg C)
*/

#pragma once

#include "OnStepXComm.h"

#include <indifocuser.h>

class OnStepXFocuser : public INDI::Focuser
{
    public:
        explicit OnStepXFocuser(int slot);

        // Called by parent after Handshake: wire in the shared comm object.
        void setComm(OnStepXComm *comm)
        {
            m_comm = comm;
        }

        // Returns the 1-based focuser slot number.
        // Required by OnStepXFocuserHotPlugHandler for duplicate detection.
        int slot() const
        {
            return m_slot;
        }

        // Periodic status update — call from parent's poll throttle (every ~5 s).
        void pollStatus();

        // DefaultDevice
        const char *getDefaultName() override;
        bool initProperties() override;
        bool updateProperties() override;
        bool saveConfigItems(FILE *fp) override;

        // Connection — no port to open; parent activates us.
        bool Connect() override;
        bool Disconnect() override;

        // Timer — checks for in-progress move completion
        void TimerHit() override;

        // FocuserInterface
        IPState MoveAbsFocuser(uint32_t targetTicks) override;
        IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
        bool    AbortFocuser() override;
        bool    SyncFocuser(uint32_t ticks) override;
        bool    SetFocuserBacklash(int32_t steps) override;
        bool    SetFocuserBacklashEnabled(bool enabled) override;
        bool    SetFocuserSpeed(int speed) override;

    private:
        int           m_slot;
        OnStepXComm  *m_comm  { nullptr };
        char          m_name[32] {};  // "OnStep X Focuser N"

        bool          m_moving { false };
        uint32_t      m_targetPos { 0 };

        // Custom: temperature readout (not in FocuserInterface)
        INDI::PropertyNumber m_temperatureNP {1};

        // Protocol helpers (all use sendCommandFocuser / sendCommandBlindFocuser)
        bool cmdGetPos(uint32_t &pos);
        bool cmdGetMax(uint32_t &maxPos);
        bool cmdGetStatus(bool &moving);
        bool cmdGetTemperature(double &tempC);
        bool cmdGoto(uint32_t microns);
        bool cmdMoveRel(int32_t microns);
        bool cmdAbort();
        bool cmdSetBacklash(int32_t steps);
        bool cmdSetSpeed(int speed);
};
