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
*/

#include "OnStepXFocuserHotPlugHandler.h"

#include <indilogger.h>
#include <hotplugmanager.h>

#include <stdexcept>

// ---------------------------------------------------------------------------
OnStepXFocuserHotPlugHandler::OnStepXFocuserHotPlugHandler(OnStepXCore *core)
    : m_core(core)
{
}

OnStepXFocuserHotPlugHandler::~OnStepXFocuserHotPlugHandler()
{
    for (const auto &f : m_focusers)
        f->deleteProperty(nullptr);
    m_focusers.clear();
    m_view.clear();
}

// ---------------------------------------------------------------------------
// discoverConnectedDeviceIdentifiers
//
// Returns the slot numbers that probeController() already established.
// No hardware communication needed here — the probe already confirmed
// which slots are populated.
// ---------------------------------------------------------------------------
std::vector<std::string>
OnStepXFocuserHotPlugHandler::discoverConnectedDeviceIdentifiers()
{
    std::vector<std::string> ids;
    if (!m_core)
        return ids;

    int n = m_core->caps().numFocusers;
    for (int i = 1; i <= n; i++)
        ids.push_back(std::to_string(i));

    LOGF_DEBUG("OnStepXFocuserHotPlugHandler: discovered %d focuser slot(s)", n);
    return ids;
}

// ---------------------------------------------------------------------------
// createDevice
//
// Constructs the focuser for the given slot identifier, wires in the
// shared comm, and returns it as a shared_ptr<DefaultDevice>.
// ---------------------------------------------------------------------------
std::shared_ptr<INDI::DefaultDevice>
OnStepXFocuserHotPlugHandler::createDevice(const std::string &identifier)
{
    int slot;
    try
    {
        slot = std::stoi(identifier);
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("OnStepXFocuserHotPlugHandler: invalid slot identifier '%s': %s",
                   identifier.c_str(), e.what());
        return nullptr;
    }

    // Guard against duplicates
    for (const auto &f : m_focusers)
    {
        if (f->slot() == slot)
        {
            LOGF_DEBUG("OnStepXFocuserHotPlugHandler: slot %d already created", slot);
            return f;
        }
    }

    auto dev = std::make_shared<OnStepXFocuser>(slot);
    dev->setComm(&m_core->comm());
    m_focusers.push_back(dev);

    LOGF_INFO("OnStepXFocuserHotPlugHandler: created focuser slot %d", slot);
    return dev;
}

// ---------------------------------------------------------------------------
// destroyDevice
// ---------------------------------------------------------------------------
void OnStepXFocuserHotPlugHandler::destroyDevice(
    std::shared_ptr<INDI::DefaultDevice> device)
{
    auto f = std::dynamic_pointer_cast<OnStepXFocuser>(device);
    if (!f)
    {
        LOG_ERROR("OnStepXFocuserHotPlugHandler::destroyDevice: not an OnStepXFocuser");
        return;
    }

    f->setConnected(false, IPS_OK);  // Drive updateProperties() disconnect branch before forcibly unpublishing
    f->deleteProperty(nullptr);

    auto it = std::remove_if(m_focusers.begin(), m_focusers.end(),
                             [&](const std::shared_ptr<OnStepXFocuser> &d)
    {
        return d == f;
    });
    if (it != m_focusers.end())
        m_focusers.erase(it, m_focusers.end());
}

// ---------------------------------------------------------------------------
// getManagedDevices
// ---------------------------------------------------------------------------
const std::map<std::string, std::shared_ptr<INDI::DefaultDevice>> &
        OnStepXFocuserHotPlugHandler::getManagedDevices() const
{
    m_view.clear();
    for (const auto &f : m_focusers)
        m_view[std::to_string(f->slot())] = f;
    return m_view;
}

// ---------------------------------------------------------------------------
// performInitialScan
//
// Replicates one HotPlugManager::checkHotPlugEvents() pass synchronously.
// Used because HotPlugManager::start() is guarded by
// "if (hotPlugTimer.isActive()) return" — if another driver (e.g. ASI CCD)
// has already called start(), our subsequent start() call is silently
// ignored and our handler is never polled on the first tick.
//
// This is called unconditionally so devices are always created immediately
// on connect regardless of timer state.  registerHandler() + start() are
// still called so that subsequent hotplug events are handled by the manager.
// ---------------------------------------------------------------------------
void OnStepXFocuserHotPlugHandler::performInitialScan()
{
    LOG_DEBUG("OnStepXFocuserHotPlugHandler::performInitialScan called");

    std::vector<std::string> ids = discoverConnectedDeviceIdentifiers();
    const auto &managed = getManagedDevices();

    for (const std::string &id : ids)
    {
        if (managed.find(id) == managed.end())
        {
            LOGF_DEBUG("OnStepXFocuserHotPlugHandler: creating device for slot %s", id.c_str());
            auto dev = createDevice(id);
            if (dev)
            {
                LOGF_DEBUG("OnStepXFocuserHotPlugHandler: calling ISGetProperties for slot %s", id.c_str());
                dev->ISGetProperties(nullptr);
                dev->setConnected(true, IPS_OK);  // Must follow ISGetProperties() so clients receive the base property list before connected-branch properties
            }
            else
            {
                LOGF_ERROR("OnStepXFocuserHotPlugHandler: createDevice returned null for slot %s", id.c_str());
            }
        }
    }
}
