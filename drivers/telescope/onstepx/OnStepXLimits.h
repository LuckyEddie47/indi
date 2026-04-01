/*
    OnStep X INDI Driver — Limits and Home helper (mount binary only)

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

#include <indipropertyswitch.h>
#include <indipropertynumber.h>

class OnStepXComm;
namespace INDI { class DefaultDevice; }

// Manages horizon limits, meridian limits, home control, and home offsets.
// Plain C++ -- no INDI base class.  Holds borrowed pointers (non-owning).
//
// The owning OnStepXMount calls init/update/handle* from the standard
// INDI entry points.
class OnStepXLimits
{
    public:
        void setComm(OnStepXComm *comm)          { m_comm = comm; }
        void setDevice(INDI::DefaultDevice *dev)  { m_dev  = dev; }

        // Called from OnStepXMount::initProperties().
        void initProperties();

        // Called from OnStepXMount::updateProperties() after connect/disconnect.
        void updateProperties(bool connected, bool hasHomeSense);

        // Called from OnStepXMount::ISNewSwitch().
        // Returns true if the event was consumed.
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);

        // Called from OnStepXMount::ISNewNumber().
        // Returns true if the event was consumed.
        bool handleNumber(const char *name, double values[], char *names[], int n);

        // Called from OnStepXMount::saveConfigItems().
        void saveConfig(FILE *fp);

        // Read horizon and meridian limits from the controller into properties.
        bool readLimits();

        // Execute a home action: FIND -> :hC#, SET -> :hF#
        // GO is not a separate command on OnStepX -- homing is always FIND.
        bool homeFind();
        bool homeSet();

        // Send auto-boot setting: enabled -> :hA1# / disabled -> :hA0#
        bool setAutoHome(bool enabled);

        // Write home axis offsets (axis1, axis2) via :hC1,[n]# and :hC2,[n]#
        bool writeHomeOffsets(double axis1, double axis2);

        // Property references (used by OnStepXMount for state updates)
        INDI::PropertySwitch &homeActionSP()  { return m_homeActionSP; }
        INDI::PropertySwitch &autoBootSP()    { return m_autoBootSP; }
        INDI::PropertyNumber &homeOffsetNP()  { return m_homeOffsetNP; }
        INDI::PropertyNumber &horizonLimitNP(){ return m_horizonLimitNP; }
        INDI::PropertyNumber &meridianLimitNP(){ return m_meridianLimitNP; }

    private:
        OnStepXComm        *m_comm { nullptr };
        INDI::DefaultDevice *m_dev { nullptr };

        INDI::PropertySwitch m_homeActionSP  { 2 };  // Find Home / Set Home
        INDI::PropertySwitch m_autoBootSP    { 2 };  // Auto-home on boot: On/Off
        INDI::PropertyNumber m_homeOffsetNP  { 2 };  // Axis1, Axis2 home offsets
        INDI::PropertyNumber m_horizonLimitNP  { 2 };// Min alt, Max alt (degrees)
        INDI::PropertyNumber m_meridianLimitNP { 2 };// East, West meridian (minutes)
};
