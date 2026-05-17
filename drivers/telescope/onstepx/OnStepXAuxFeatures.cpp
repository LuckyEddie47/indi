/*
    OnStep X INDI Driver — Auxiliary feature helper

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include "OnStepXAuxFeatures.h"
#include "OnStepXComm.h"

#include <defaultdevice.h>
#include <indilogger.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define AUX_TAB_OUTPUTS      "Outputs"
#define AUX_TAB_DEW          "Dew Heaters"
#define AUX_TAB_IVO          "Intervalometer"

// ---------------------------------------------------------------------------
// discoverAndDefine — probe each active slot and create INDI properties
// ---------------------------------------------------------------------------
void OnStepXAuxFeatures::discoverAndDefine(uint8_t featureMask)
{
    for (int i = 0; i < 8; i++)
    {
        if (!(featureMask & (1u << i)))
            continue;

        Slot &slot = m_slots[i];
        slot = Slot{};           // reset from previous connection
        slot.index = i + 1;

        if (!probeSlot(i + 1, slot))
        {
            LOGF_WARN("Could not probe aux slot %d, skipping", i + 1);
            continue;
        }

        slot.active = true;
        defineSlot(slot);
        LOGF_DEBUG("Aux slot %d: '%s' type=%d", i + 1, slot.label, (int)slot.type);
    }
}

// ---------------------------------------------------------------------------
// deleteAll — remove all slot properties on disconnect
// ---------------------------------------------------------------------------
void OnStepXAuxFeatures::deleteAll()
{
    for (auto &slot : m_slots)
    {
        if (slot.active)
            deleteSlot(slot);
    }
}

// ---------------------------------------------------------------------------
// handleSwitch
// ---------------------------------------------------------------------------
bool OnStepXAuxFeatures::handleSwitch(const char *name, ISState *states, char *names[], int n)
{
    for (auto &slot : m_slots)
    {
        if (!slot.active)
            continue;

        if (slot.type == FeatureType::SWITCH && slot.switchSP.isNameMatch(name))
        {
            slot.switchSP.update(states, names, n);
            int val = (slot.switchSP[1].getState() == ISS_ON) ? 1 : 0;
            if (sendWriteInt(slot.index, 'V', val))
                slot.switchSP.setState(IPS_OK);
            else
                slot.switchSP.setState(IPS_ALERT);
            slot.switchSP.apply();
            return true;
        }

        if (slot.type == FeatureType::DEW_HEATER && slot.dewEnSP.isNameMatch(name))
        {
            slot.dewEnSP.update(states, names, n);
            int val = (slot.dewEnSP[1].getState() == ISS_ON) ? 1 : 0;
            if (sendWriteInt(slot.index, 'E', val))
                slot.dewEnSP.setState(IPS_OK);
            else
                slot.dewEnSP.setState(IPS_ALERT);
            slot.dewEnSP.apply();
            return true;
        }

        if (slot.type == FeatureType::INTERVALOMETER && slot.ivoEnSP.isNameMatch(name))
        {
            slot.ivoEnSP.update(states, names, n);
            int val = (slot.ivoEnSP[1].getState() == ISS_ON) ? 1 : 0;
            if (sendWriteInt(slot.index, 'V', val))
                slot.ivoEnSP.setState(IPS_OK);
            else
                slot.ivoEnSP.setState(IPS_ALERT);
            slot.ivoEnSP.apply();
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// handleNumber
// ---------------------------------------------------------------------------
bool OnStepXAuxFeatures::handleNumber(const char *name, double *values, char *names[], int n)
{
    for (auto &slot : m_slots)
    {
        if (!slot.active)
            continue;

        if (slot.type == FeatureType::ANALOG && slot.analogNP.isNameMatch(name))
        {
            slot.analogNP.update(values, names, n);
            int val = static_cast<int>(slot.analogNP[0].getValue());
            if (sendWriteInt(slot.index, 'V', val))
                slot.analogNP.setState(IPS_OK);
            else
                slot.analogNP.setState(IPS_ALERT);
            slot.analogNP.apply();
            return true;
        }

        if (slot.type == FeatureType::DEW_HEATER && slot.dewNP.isNameMatch(name))
        {
            slot.dewNP.update(values, names, n);
            // [0]=power [1]=zero [2]=span
            bool ok = sendWriteInt(slot.index, 'V', (int)slot.dewNP[0].getValue()) &&
                      sendWriteInt(slot.index, 'Z', (int)(slot.dewNP[1].getValue() * 10)) &&
                      sendWriteInt(slot.index, 'S', (int)(slot.dewNP[2].getValue() * 10));
            slot.dewNP.setState(ok ? IPS_OK : IPS_ALERT);
            slot.dewNP.apply();
            return true;
        }

        if (slot.type == FeatureType::INTERVALOMETER && slot.ivoNP.isNameMatch(name))
        {
            slot.ivoNP.update(values, names, n);
            // [0]=interval(s) [1]=duration(ms) [2]=delay(ms) [3]=count
            bool ok = sendWriteInt(slot.index, 'D', (int)slot.ivoNP[1].getValue()) &&
                      sendWriteInt(slot.index, 'C', (int)slot.ivoNP[3].getValue());
            slot.ivoNP.setState(ok ? IPS_OK : IPS_ALERT);
            slot.ivoNP.apply();
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// pollStatus — refresh current values from :GXX[n]#
// ---------------------------------------------------------------------------
void OnStepXAuxFeatures::pollStatus()
{
    char reply[OnStepXComm::REPLY_BUF_SIZE];

    for (auto &slot : m_slots)
    {
        if (!slot.active)
            continue;

        char cmd[OnStepXComm::CMD_MAX_LEN];
        snprintf(cmd, sizeof(cmd), ":GXX%d#", slot.index);
        if (!m_comm->sendCommand(cmd, reply))
            continue;

        char *end;
        long val = std::strtol(reply, &end, 10);
        if (end == reply)
            continue;

        switch (slot.type)
        {
            case FeatureType::SWITCH:
                slot.switchSP[0].setState(val == 0 ? ISS_ON : ISS_OFF);
                slot.switchSP[1].setState(val != 0 ? ISS_ON : ISS_OFF);
                slot.switchSP.setState(IPS_OK);
                slot.switchSP.apply();
                break;

            case FeatureType::ANALOG:
                slot.analogNP[0].setValue(static_cast<double>(val));
                slot.analogNP.setState(IPS_OK);
                slot.analogNP.apply();
                break;

            case FeatureType::DEW_HEATER:
                // :GXX[n]# returns current power value
                slot.dewNP[0].setValue(static_cast<double>(val));
                slot.dewNP.setState(IPS_OK);
                slot.dewNP.apply();
                break;

            case FeatureType::INTERVALOMETER:
                // :GXX[n]# returns running state (0=stopped, 1=running)
                slot.ivoEnSP[0].setState(val == 0 ? ISS_ON : ISS_OFF);
                slot.ivoEnSP[1].setState(val != 0 ? ISS_ON : ISS_OFF);
                slot.ivoEnSP.setState(IPS_OK);
                slot.ivoEnSP.apply();
                break;

            default:
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// saveConfig
// ---------------------------------------------------------------------------
void OnStepXAuxFeatures::saveConfig(FILE *fp)
{
    for (auto &slot : m_slots)
    {
        if (!slot.active)
            continue;
        switch (slot.type)
        {
            case FeatureType::SWITCH:  slot.switchSP.save(fp); break;
            case FeatureType::ANALOG:  slot.analogNP.save(fp); break;
            case FeatureType::DEW_HEATER:
                slot.dewEnSP.save(fp);
                slot.dewNP.save(fp);
                break;
            case FeatureType::INTERVALOMETER:
                slot.ivoEnSP.save(fp);
                slot.ivoNP.save(fp);
                break;
            default: break;
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool OnStepXAuxFeatures::probeSlot(int idx, Slot &slot)
{
    char cmd[OnStepXComm::CMD_MAX_LEN], reply[OnStepXComm::REPLY_BUF_SIZE];
    snprintf(cmd, sizeof(cmd), ":GXY%d#", idx);
    if (!m_comm->sendCommand(cmd, reply))
        return false;

    // Expected reply format: "name,T" where T is a type digit
    // e.g. "DEW_A,2"
    char *comma = strrchr(reply, ',');
    char typeChar = '1';   // default: SWITCH

    if (comma && comma[1] != '\0')
    {
        typeChar = comma[1];
        *comma = '\0';  // terminate name at comma
    }

    snprintf(slot.label, sizeof(slot.label), "%s", reply);
    makePropBase(reply, idx, slot.propBase, sizeof(slot.propBase));
    slot.type = parseType(typeChar);

    return true;
}

void OnStepXAuxFeatures::defineSlot(Slot &slot)
{
    const char *dev = m_dev ? m_dev->getDeviceName() : "";
    char nameBuf[64];

    switch (slot.type)
    {
        case FeatureType::SWITCH:
        {
            snprintf(nameBuf, sizeof(nameBuf), "%s_SW", slot.propBase);
            slot.switchSP[0].fill("OFF", "Off", ISS_ON);
            slot.switchSP[1].fill("ON",  "On",  ISS_OFF);
            slot.switchSP.fill(dev, nameBuf, slot.label,
                               AUX_TAB_OUTPUTS, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
            m_dev->defineProperty(slot.switchSP);
            break;
        }

        case FeatureType::ANALOG:
        {
            snprintf(nameBuf, sizeof(nameBuf), "%s_AN", slot.propBase);
            slot.analogNP[0].fill("VALUE", "Value (0-255)", "%.0f", 0, 255, 1, 0);
            slot.analogNP.fill(dev, nameBuf, slot.label,
                               AUX_TAB_OUTPUTS, IP_RW, 60, IPS_IDLE);
            m_dev->defineProperty(slot.analogNP);
            break;
        }

        case FeatureType::DEW_HEATER:
        {
            // Enable switch
            snprintf(nameBuf, sizeof(nameBuf), "%s_EN", slot.propBase);
            char enableLabel[64];
            snprintf(enableLabel, sizeof(enableLabel), "%s Enable", slot.label);
            slot.dewEnSP[0].fill("DISABLE", "Disable", ISS_ON);
            slot.dewEnSP[1].fill("ENABLE",  "Enable",  ISS_OFF);
            slot.dewEnSP.fill(dev, nameBuf, enableLabel,
                              AUX_TAB_DEW, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
            m_dev->defineProperty(slot.dewEnSP);

            // Numbers: power, zero point, span
            snprintf(nameBuf, sizeof(nameBuf), "%s_NP", slot.propBase);
            slot.dewNP[0].fill("POWER", "Power (%)",        "%.0f",  0, 100, 1, 0);
            slot.dewNP[1].fill("ZERO",  "Zero Point (C)",   "%.1f", -20, 40, 0.5, 0);
            slot.dewNP[2].fill("SPAN",  "Span (C)",         "%.1f",   0, 20, 0.5, 5);
            slot.dewNP.fill(dev, nameBuf, slot.label,
                            AUX_TAB_DEW, IP_RW, 60, IPS_IDLE);
            m_dev->defineProperty(slot.dewNP);
            break;
        }

        case FeatureType::INTERVALOMETER:
        {
            // Start/stop switch
            snprintf(nameBuf, sizeof(nameBuf), "%s_EN", slot.propBase);
            char enableLabel[64];
            snprintf(enableLabel, sizeof(enableLabel), "%s Control", slot.label);
            slot.ivoEnSP[0].fill("STOP",  "Stop",  ISS_ON);
            slot.ivoEnSP[1].fill("START", "Start", ISS_OFF);
            slot.ivoEnSP.fill(dev, nameBuf, enableLabel,
                              AUX_TAB_IVO, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
            m_dev->defineProperty(slot.ivoEnSP);

            // Parameters
            snprintf(nameBuf, sizeof(nameBuf), "%s_NP", slot.propBase);
            slot.ivoNP[0].fill("INTERVAL", "Interval (s)",  "%.1f",   0, 3600, 1, 10);
            slot.ivoNP[1].fill("DURATION", "Duration (ms)", "%.0f",   0, 60000, 100, 1000);
            slot.ivoNP[2].fill("DELAY",    "Delay (ms)",    "%.0f",   0, 60000, 100, 0);
            slot.ivoNP[3].fill("COUNT",    "Count (0=inf)", "%.0f",   0, 9999,  1,   0);
            slot.ivoNP.fill(dev, nameBuf, slot.label,
                            AUX_TAB_IVO, IP_RW, 60, IPS_IDLE);
            m_dev->defineProperty(slot.ivoNP);
            break;
        }

        default:
        {
            // Unknown type — treat as a plain 0-255 analog output
            snprintf(nameBuf, sizeof(nameBuf), "%s_AN", slot.propBase);
            slot.analogNP[0].fill("VALUE", "Value (0-255)", "%.0f", 0, 255, 1, 0);
            slot.analogNP.fill(dev, nameBuf, slot.label,
                               AUX_TAB_OUTPUTS, IP_RW, 60, IPS_IDLE);
            m_dev->defineProperty(slot.analogNP);
            break;
        }
    }
}

void OnStepXAuxFeatures::deleteSlot(Slot &slot)
{
    if (!m_dev)
        return;
    switch (slot.type)
    {
        case FeatureType::SWITCH:
            m_dev->deleteProperty(slot.switchSP);
            break;
        case FeatureType::ANALOG:
        case FeatureType::UNKNOWN:
            m_dev->deleteProperty(slot.analogNP);
            break;
        case FeatureType::DEW_HEATER:
            m_dev->deleteProperty(slot.dewEnSP);
            m_dev->deleteProperty(slot.dewNP);
            break;
        case FeatureType::INTERVALOMETER:
            m_dev->deleteProperty(slot.ivoEnSP);
            m_dev->deleteProperty(slot.ivoNP);
            break;
    }
    slot.active = false;
}

bool OnStepXAuxFeatures::sendWriteInt(int slotIdx, char field, int value)
{
    char cmd[OnStepXComm::CMD_MAX_LEN], reply[OnStepXComm::REPLY_BUF_SIZE];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", slotIdx, field, value);
    if (!m_comm->sendCommand(cmd, reply))
        return false;
    return reply[0] == '1';
}

// Produce a sanitized INDI property name base from the firmware label.
// e.g. "Dew A" -> "OSX_DEW_A" (slot index appended as fallback if label empty)
void OnStepXAuxFeatures::makePropBase(const char *label, int slotIdx, char *out, int outLen)
{
    char tmp[64];
    // Copy, replace non-alphanumeric with '_', uppercase
    int j = 0;
    for (int i = 0; label[i] && j < (int)sizeof(tmp) - 1; i++)
    {
        char c = label[i];
        if (std::isalnum((unsigned char)c))
            tmp[j++] = static_cast<char>(std::toupper((unsigned char)c));
        else if (j > 0 && tmp[j-1] != '_')
            tmp[j++] = '_';
    }
    // Trim trailing underscores
    while (j > 0 && tmp[j-1] == '_')
        j--;
    tmp[j] = '\0';

    if (j == 0)
        snprintf(out, outLen, "OSX_AUX%d", slotIdx);
    else
        snprintf(out, outLen, "OSX_%s", tmp);
}

OnStepXAuxFeatures::FeatureType OnStepXAuxFeatures::parseType(char t)
{
    switch (t)
    {
        case '1': case 'S': case 's': return FeatureType::SWITCH;
        case '2': case 'A': case 'a': return FeatureType::ANALOG;
        case '3': case 'D': case 'd': return FeatureType::DEW_HEATER;
        case '4': case 'I': case 'i': return FeatureType::INTERVALOMETER;
        default:                      return FeatureType::UNKNOWN;
    }
}
