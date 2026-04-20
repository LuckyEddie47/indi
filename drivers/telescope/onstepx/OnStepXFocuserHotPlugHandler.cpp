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
// Called by HotPlugManager to find out which devices exist.
// We return the slot numbers that probeController() already found.
// Slot numbers are 1-based strings: "1", "2", ... "N".
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

    return ids;
}

// ---------------------------------------------------------------------------
// createDevice
//
// Called by HotPlugManager for each identifier returned by discover().
// Constructs the focuser, wires in the shared comm, and returns it.
// HotPlugManager registers the returned device with the INDI server.
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

    f->deleteProperty(nullptr);

    auto it = std::remove_if(m_focusers.begin(), m_focusers.end(),
                             [&](const std::shared_ptr<OnStepXFocuser> &d)
                             { return d == f; });
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
