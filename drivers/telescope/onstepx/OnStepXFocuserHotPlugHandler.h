/*
    OnStep X INDI Driver — Focuser HotPlug Handler

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

    Implements HotPlugCapableDevice so that OnStepXFocuser child devices
    are registered with the INDI server via HotPlugManager.

    discoverConnectedDeviceIdentifiers() returns the slot numbers ("1".."N")
    that probeController() found populated.  createDevice() constructs the
    OnStepXFocuser, wires in the shared OnStepXComm, and returns it as a
    shared_ptr<DefaultDevice>.

    Because HotPlugManager::start() contains an early-return guard
    "if (hotPlugTimer.isActive()) return", another driver loaded in the
    same process (e.g. ASI CCD) may have already started the singleton
    timer before our driver connects.  In that case start() is a no-op
    and our handler would never receive its first poll tick in time.

    performInitialScan() replicates one checkHotPlugEvents() pass
    synchronously so devices are always created immediately on connect,
    regardless of whether the manager timer was already running.
    registerHandler() + start() are still called so that subsequent
    hotplug events continue to be handled by the manager.
*/

#pragma once

#include "OnStepXCore.h"
#include "OnStepXFocuser.h"

#include <hotplugcapabledevice.h>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

class OnStepXFocuserHotPlugHandler : public INDI::HotPlugCapableDevice
{
    public:
        // core must remain valid for the lifetime of this handler.
        explicit OnStepXFocuserHotPlugHandler(OnStepXCore *core);
        ~OnStepXFocuserHotPlugHandler() override;

        // HotPlugCapableDevice interface
        std::vector<std::string>
            discoverConnectedDeviceIdentifiers() override;

        std::shared_ptr<INDI::DefaultDevice>
            createDevice(const std::string &identifier) override;

        void destroyDevice(std::shared_ptr<INDI::DefaultDevice> device) override;

        const std::map<std::string, std::shared_ptr<INDI::DefaultDevice>> &
            getManagedDevices() const override;

        // Perform one discovery+creation pass synchronously.
        // Called directly from OnStepXMount::updateProperties() because
        // HotPlugManager::start() is a no-op if the singleton timer is
        // already running (e.g. started by another driver such as ASI CCD).
        // When no other driver has called start(), this still creates devices
        // immediately rather than waiting for the first timer tick.
        void performInitialScan();

    private:
        OnStepXCore *m_core { nullptr };

        std::deque<std::shared_ptr<OnStepXFocuser>>              m_focusers;
        mutable std::map<std::string,
                         std::shared_ptr<INDI::DefaultDevice>>   m_view;
};
