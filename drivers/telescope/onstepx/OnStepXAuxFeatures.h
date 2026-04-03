/*
    OnStep X INDI Driver — Auxiliary feature helper (shared by both binaries)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    Handles dynamic discovery and control of OnStepX aux feature slots 1-8.
    Slot count and types are probed after connect.  INDI::OutputInterface was
    not used because it requires the output count at initProperties() time,
    before the firmware has been queried.

    Protocol (OnStepX v10.24c):
      :GXY0#        — 8-char mask ('1'=configured, '0'=not), positions 0-7 = slots 1-8
      :GXY[n]#      — slot info: "name,T" where T = 0(SWITCH) 1(ANALOG) 2(DEW) 3(IVO)
      :GXX[n]#      — current value (integer, '#'-terminated)
      :SXX[n],V[v]# — set value (SWITCH 0/1, ANALOG 0-255, DEW power %)
      :SXX[n],E[v]# — DEW enable (1) / disable (0)
      :SXX[n],Z[v]# — DEW zero point (deg C x 10, integer)
      :SXX[n],S[v]# — DEW span (deg C x 10, integer)
      :SXX[n],D[v]# — INTERVALOMETER exposure duration (ms)
      :SXX[n],C[v]# — INTERVALOMETER count (0=unlimited)
      All :SXX# commands reply '1' on success.

    INDI Properties:
      SWITCH / ANALOG slots   -> "Outputs" tab      (OSX_OUT_[label])
      DEW_HEATER slots        -> "Dew Heaters" tab  (OSX_DEW_[label])
      INTERVALOMETER slots    -> "Intervalometer" tab (OSX_IVO_[label])
*/

#pragma once

#include <defaultdevice.h>
#include <array>
#include <cstdint>
#include <cstdio>

class OnStepXComm;

class OnStepXAuxFeatures
{
    public:
        void setDevice(INDI::DefaultDevice *dev) { m_dev = dev; }
        void setComm(OnStepXComm *comm)          { m_comm = comm; }

        // Called from updateProperties(true): probe each slot and defineProperty
        void discoverAndDefine(uint8_t featureMask);

        // Called from updateProperties(false): deleteProperty for all active slots
        void deleteAll();

        // Called from ISNewSwitch / ISNewNumber
        bool handleSwitch(const char *name, ISState *states, char *names[], int n);
        bool handleNumber(const char *name, double *values, char *names[], int n);

        // Periodic poll — call every ~5 polls from TimerHit / ReadScopeStatus
        void pollStatus();

        void saveConfig(FILE *fp);

    protected:
        enum class FeatureType { SWITCH, ANALOG, DEW_HEATER, INTERVALOMETER, UNKNOWN };

        static void makePropBase(const char *label, int slotIdx, char *out, int outLen);
        static FeatureType parseType(char t);

    private:

        struct Slot
        {
            int         index    = 0;             // 1-8
            char        label[32] = {};           // human name from :GXY[n]#
            char        propBase[48] = {};        // sanitized INDI name root
            FeatureType type     = FeatureType::UNKNOWN;
            bool        active   = false;

            // SWITCH: single On/Off switch
            INDI::PropertySwitch switchSP   {2};

            // ANALOG: 0-255 value
            INDI::PropertyNumber analogNP   {1};

            // DEW_HEATER: enable switch + value/zero/span numbers
            INDI::PropertySwitch dewEnSP    {2};
            INDI::PropertyNumber dewNP      {3};  // [0]=power [1]=zero [2]=span

            // INTERVALOMETER: start/stop switch + params
            INDI::PropertySwitch ivoEnSP    {2};
            INDI::PropertyNumber ivoNP      {4};  // [0]=interval [1]=duration [2]=delay [3]=count
        };

        std::array<Slot, 8> m_slots;

        INDI::DefaultDevice *m_dev  = nullptr;
        OnStepXComm         *m_comm = nullptr;

        bool probeSlot(int idx, Slot &slot);
        void defineSlot(Slot &slot);
        void deleteSlot(Slot &slot);

        bool sendWriteInt(int slotIdx, char field, int value);
};
