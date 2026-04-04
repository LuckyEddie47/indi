/*
    OnStep X INDI Driver — USB Plugin helper (shared by both binaries)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    Handles dynamic discovery and control of OnStepX aux feature slots 1-8.
    Slot count and types are probed after connect.  INDI::OutputInterface was
    not used because it requires the output count at initProperties() time,
    before the firmware has been queried.

    Protocol (USB Plugin):
      :GUY0#        — 8-char mask ('1'=configured, '0'=not), positions 0-7 = slots 1-8
      :GUY[n]#      — port_name: "name"
      :GUX[n]#      — current value (0=OFF/1=ON integer, '#'-terminated)
      :SUX[n],V[v]# — set value (SWITCH 0=OFF/1=ON)
      All :SUX# commands reply '1' on success.

    INDI Properties:
      Port   -> "USB" tab      (OSX_USB_[label])
*/

#pragma once

#include <defaultdevice.h>
#include <array>
#include <cstdint>
#include <cstdio>

class OnStepXComm;

class OnStepXUsbPlugin
{
    public:
        void setDevice(INDI::DefaultDevice *dev)
        {
            m_dev = dev;
        }
        void setComm(OnStepXComm *comm)
        {
            m_comm = comm;
        }

        // Called from updateProperties(true): probe each slot and defineProperty
        void discoverAndDefine(uint8_t portsMask);

        // Called from updateProperties(false): deleteProperty for all active slots
        void deleteAll();

        // Called from ISNewSwitch
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);

        // Periodic poll — call every ~5 polls from TimerHit / ReadScopeStatus
        void pollStatus();

        void saveConfig(FILE *fp);

    protected:

        static void makePropBase(const char *label, int slotIdx, char *out, int outLen);

    private:

        struct Port
        {
            int         index    = 0;             // 1-8
            char        label[32] = {};           // human name from :GUY[n]#
            char        propBase[48] = {};        // sanitized INDI name root
            bool        active   = false;

            // SWITCH: single On/Off switch
            INDI::PropertySwitch switchSP   {2};
        };

        std::array<Port, 8> m_ports;

        INDI::DefaultDevice *m_dev  = nullptr;
        OnStepXComm         *m_comm = nullptr;

        bool probePort(int idx, Port &port);
        void definePort(Port &port);
        void deletePort(Port &port);

        bool sendWriteInt(int portIdx, char field, int value);
};
