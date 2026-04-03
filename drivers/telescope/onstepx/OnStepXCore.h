/*
    OnStep X INDI Driver — Handshake and capability probing (shared by both binaries)

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

    Owns the shared OnStepXComm instance and the Capabilities struct.
    probeController() runs for both binaries; probeMount() is mount-only.

    Probe commands (OnStepX v10.24c):
      :GVP#  — product name; must return "OnStepX" (Phase 1)
      :GVN#  — firmware version string (Phase 1)
      :GVD#  — firmware date string (Phase 1)
      :GVT#  — firmware time string (Phase 1)
      :GXY0# — 8-char aux-feature mask + capability flags (Phase 1)
      :GX9A# — weather sensor probe: non-"0" reply = sensor present (Phase 1)
      :Gu#   — binary status probe: any reply confirms hasBinaryStatus (Phase 2)
      :GU#   — ASCII status string; used to derive mount type, PEC, pier side (Phase 2)
*/

#pragma once

#include "OnStepXComm.h"
#include "OnStepXCapabilities.h"

namespace INDI { class DefaultDevice; }

class OnStepXCore
{
    public:
        OnStepXCore() = default;

        // Called once at startup by the owning INDI device.
        void setDevice(INDI::DefaultDevice *dev);

        // Called from Handshake() after the connection plugin provides a valid fd.
        void setFd(int fd);

        bool probeController();   // Phase 1 — both binaries
        bool probeMount();        // Phase 2 — mount binary only

        OnStepXComm          &comm();
        const Capabilities   &caps() const;

    private:
        OnStepXComm          m_comm;
        Capabilities         m_cap;
        INDI::DefaultDevice *m_dev { nullptr };
};
