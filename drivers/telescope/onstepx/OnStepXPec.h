/*
    OnStep X INDI Driver — PEC helper (mount binary only)

    Protocol (OnStepX v10.24c):
      :$QZ?#   — get PEC status: one char from {I,p,P,r,R}, optional '.' suffix = index detected
                 I=Ignore (not supported), p=ready-to-play, P=Playing, r=ready-to-record, R=Recording
      :$QZ+#   — enable PEC playback (reply '1')
      :$QZ-#   — disable PEC playback (reply '1')
      :$QZ/#   — ready to record PEC (reply '1')
      :$QZZ#   — clear PEC buffer (reply '1')
      :$QZ!#   — write PEC to EEPROM (reply '1')
      :VW#     — worm period (worm gear steps per revolution, integer, '#'-terminated)

    INDI Properties (all on "PEC" tab):
      OSX_PEC_STATE  IP_RO  5 lights: Ignored / Ready-to-Play / Playing / Ready-to-Record / Recording
      OSX_PEC_INDEX  IP_RO  2 lights: Not Detected / Detected
      OSX_PEC_CONTROL  IP_RW  ISR_ATMOST1: Play / Stop / Ready-Record / Clear / Save-EEPROM
      OSX_PEC_WORM_STEPS  IP_RO  1 number: worm steps

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#pragma once

#include <defaultdevice.h>
#include <cstdio>

class OnStepXComm;

class OnStepXPec
{
    public:
        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)          { m_comm = comm; }

        // Called from device class initProperties
        void initProperties();

        // Called from device class updateProperties
        void updateProperties(bool connected);

        // Called from ISNewSwitch
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);

        void saveConfig(FILE *fp);

        // Periodic status poll — call from ReadScopeStatus throttle (every ~10 polls)
        void pollStatus();

        // Read worm steps once on connect
        void readWormSteps();

    private:
        INDI::DefaultDevice *m_dev  { nullptr };
        OnStepXComm         *m_comm { nullptr };

        // PEC state lights: Ignored / Ready-to-Play / Playing / Ready-to-Record / Recording
        INDI::PropertyLight  m_stateSP  {5};
        // Index detection lights
        INDI::PropertyLight  m_indexSP  {2};
        // Control switches: Play / Stop / Ready-Record / Clear / Save-EEPROM
        INDI::PropertySwitch m_controlSP {5};
        // Worm period (read-only)
        INDI::PropertyNumber m_wormNP    {1};
};
