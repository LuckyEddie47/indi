/*
    OnStep X INDI Driver — Alignment helper (mount binary only)

    Implements OnStepX geometric alignment (N-star).

    Protocol (OnStepX v10.24c):
      :A?#     — get status: 3 chars "mno" (m=max stars, n=current, o=target)
                 n=':' means current star index 9
      :A[n]#   — start alignment with n stars (1-9), reply '0' on success
      :A+#     — accept current star, reply '0' on success
      :AW#     — write alignment to EEPROM, reply '1' on success
      :GX02#   — polar error altitude correction (arcsec, float)
      :GX03#   — polar error azimuth correction (arcsec, float)

    INDI Properties (all on "Alignment" tab):
      OSX_ALIGN_STARS    IP_RW  ISR_1OFMANY  9 switches: "1 Star" .. "9 Stars"
      OSX_ALIGN_CONTROL  IP_RW  ISR_ATMOST1  2 switches: "Start" / "Accept Star"
      OSX_ALIGN_WRITE    IP_RW  ISR_ATMOST1  1 switch:   "Write to EEPROM"
      OSX_ALIGN_STATUS   IP_RO  4 text fields:
                                  Status / Max Stars / Current Star / Target Stars
      OSX_ALIGN_ERROR    IP_RO  2 text fields: Polar Error Alt / Polar Error Az

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#pragma once

#include <defaultdevice.h>
#include <cstdio>

class OnStepXComm;

class OnStepXAlignment
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

        // Refresh alignment status from :A?# and apply to OSX_ALIGN_STATUS.
        // Returns false if firmware didn't respond.
        bool updateStatus();

        // Read polar error corrections from :GX02# and :GX03#
        void updatePolarError();

    private:
        INDI::DefaultDevice *m_dev  { nullptr };
        OnStepXComm         *m_comm { nullptr };

        // Star count selector (1-9)
        INDI::PropertySwitch m_starsSP   {9};
        // Start / Accept Star
        INDI::PropertySwitch m_controlSP {2};
        // Write to EEPROM
        INDI::PropertySwitch m_writeSP   {1};
        // Status text
        INDI::PropertyText   m_statusTP  {4};
        // Polar error text
        INDI::PropertyText   m_errorTP   {2};

        bool startAlignment(int stars);
        bool acceptStar();
        bool writeAlignment();
};
