/*
    OnStep X INDI Driver — Pulse guiding and guide rate helper (mount binary only)

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

    Plain C++ helper -- no INDI base class.  Extracted from OnStepXMount in
    Stage 15 to keep OnStepXMount.cpp within a manageable size.

    Protocol (OnStepX v10.24c):
      :MGn{ms}#  — guide North   (blind send, no reply)
      :MGs{ms}#  — guide South   (blind send, no reply)
      :MGe{ms}#  — guide East    (blind send, no reply)
      :MGw{ms}#  — guide West    (blind send, no reply)
      :GX90#     — read guide rate (x sidereal, float)
      :Rn#       — set guide rate index 0-9 (blind send)

    INDI Properties (Motion Control tab):
      OSX_GUIDE_RATE  IP_RW  Number[2]  RA (x sid) / Dec (x sid), range 0.0-1.0
                              Write maps 0.0-1.0 to index 0-9 via :Rn#.
                              RA and Dec share a single firmware rate until a
                              separate Dec-axis command is confirmed by hardware.
*/

#pragma once

#include <indipropertynumber.h>
#include <indiguiderinterface.h>

#include <chrono>

class OnStepXComm;
namespace INDI { class DefaultDevice; }

class OnStepXGuide
{
    public:
        void setDevice(INDI::DefaultDevice *dev)           { m_dev = dev; }
        void setComm(OnStepXComm *comm)                    { m_comm = comm; }
        void setGuiderInterface(INDI::GuiderInterface *gi) { m_gi = gi; }

        void initProperties();
        void updateProperties(bool connected);
        bool handleNumber(const char *name, double values[], char *names[], int n);
        void saveConfig(FILE *fp);

        // GuiderInterface delegations — blind-send the pulse and start the timer
        IPState guideNorth(uint32_t ms);
        IPState guideSouth(uint32_t ms);
        IPState guideEast(uint32_t ms);
        IPState guideWest(uint32_t ms);

        // Call each ReadScopeStatus poll; fires GuideComplete when timer expires
        void checkComplete();

        // Query :GX90# and update OSX_GUIDE_RATE
        void readGuideRate();

    private:
        INDI::DefaultDevice   *m_dev  { nullptr };
        OnStepXComm           *m_comm { nullptr };
        INDI::GuiderInterface *m_gi   { nullptr };

        INDI::PropertyNumber m_guideRateNP { 2 };

        using Clock     = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;

        bool      m_guidingNS { false };
        bool      m_guidingWE { false };
        TimePoint m_guideEndNS;
        TimePoint m_guideEndWE;
};
