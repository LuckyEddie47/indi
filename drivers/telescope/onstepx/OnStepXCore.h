/*
    OnStep X INDI Driver

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

#include "OnStepXComm.h"

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

        OnStepXComm &comm();

    private:
        OnStepXComm m_comm;
};
