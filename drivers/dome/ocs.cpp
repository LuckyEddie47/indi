/*******************************************************************************
 Structure based on indi_rolloffino - https://github.com/wotalota/indi-rolloffino
 Itself an edited version of the Dome Simulator
 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

 Communication functions taken from lx200_OnStep

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

/*******************************************************************************
Driver for the Observatory Control System (OCS).
An open source system authored by Howard Dutton (also the author of OnStep).
Refer to https://onstep.groups.io/g/onstep-ocs/wiki
Capabilites include: Roll off roof, dome roof, weather monitoring,
themostat control, power monitoring & control, and GPIO.
Hardware communication is via a simple text protocol similar to the LX200.
USB and network connections supported.
*******************************************************************************/

#include "ocs.h"
//#include "connectionplugins/connectioninterface.h"
#include "indicom.h"
#include "termios.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>

// Custom tabs
#define THERMOSTAT_TAB "Thermostat"
#define POWER_TAB "Power"
#define LIGHTS_TAB "Lights"
#define SENSORS_TAB "Sensors"
#define WEATHER_TAB "Weather"

/* Add mutex to communications */
std::mutex ocsCommsLock;

// We declare an auto pointer to OCS.
std::unique_ptr<OCS> ocs(new OCS());

OCS::OCS()
{
    SetDomeCapability(DOME_CAN_ABORT | DOME_HAS_SHUTTER);
}

// Overrides
void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    ocs->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ocs->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ocs->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    ocs->ISNewNumber(dev, name, values, names, n);
}

void ISSnoopDevice(XMLEle *root)
{
    ocs->ISSnoopDevice(root);
}

void OCS::ISGetProperties(const char *dev)
{
    INDI::Dome::ISGetProperties(dev);

}

/********************************************************************************************
** Client has changed the state of a switch, update
*********************************************************************************************/
bool OCS::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}


bool OCS::ISNewNumber(const char *dev,const char *name,double values[],char *names[],int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
        if (!strcmp(Thermostat_setpointsNP.name, name)) {
            if ( THERMOSTAT_SETPOINT_COUNT == n) {
                for (int parameter = THERMOSTAT_HEAT_SETPOINT; parameter < THERMOSTAT_SETPOINT_COUNT; parameter++) {
                    if (parameter == THERMOSTAT_HEAT_SETPOINT) {
                        char thermostat_setpoint_command[CMD_MAX_LEN];
                        sprintf(thermostat_setpoint_command, "%s%.0f%s",
                                OCS_set_thermostat_heat_setpoint_part, values[THERMOSTAT_HEAT_SETPOINT], OCS_command_terminator);
                        if (!sendOCSCommand(thermostat_setpoint_command)) {
                            LOGF_INFO("Set Thermostat heat setpoint to: %.0f deg.C", values[THERMOSTAT_HEAT_SETPOINT]);
                        } else {
                            LOG_WARN("Failed to set Thermostat heat setpoint");
                        }
                    }
                    else if (parameter == THERMOSTAT_COOL_SETPOINT) {
                        char thermostat_setpoint_command[CMD_MAX_LEN];
                        sprintf(thermostat_setpoint_command, "%s%.0f%s",
                                OCS_set_thermostat_cool_setpoint_part, values[THERMOSTAT_COOL_SETPOINT], OCS_command_terminator);
                        if (!sendOCSCommand(thermostat_setpoint_command)) {
                            LOGF_INFO("Set Thermostat cool setpoint to: %.0f deg.C", values[THERMOSTAT_COOL_SETPOINT]);
                        } else {
                            LOG_WARN("Failed to set Thermostat heat setpoint");
                        }
                    }
                }
                IUUpdateNumber(&Thermostat_setpointsNP, values, names, n);
                Thermostat_setpointsNP.s = IPS_OK;
                IDSetNumber(&Thermostat_setpointsNP, nullptr);
                return true;
            }
        }
    }

    return INDI::Dome::ISNewNumber(dev,name,values,names,n);
}

bool OCS::ISSnoopDevice(XMLEle *root)
{
    return INDI::Dome::ISSnoopDevice(root);
}

/**************************************************************************************
** INDI is asking us for our default device name.
** Check that it matches Ekos selection menu and ParkData.xml names
***************************************************************************************/
const char *OCS::getDefaultName()
{
    return (const char *)"OCS";
}
/**************************************************************************************
** INDI request to init properties. Connected Define properties to Ekos
***************************************************************************************/
bool OCS::initProperties()
{
    INDI::Dome::initProperties();

    // Thermostat tab controls
    IUFillTextVector(&Thermostat_StatusTP, Thermostat_StatusT, THERMOSTAT_COUNT, getDeviceName(), "THERMOSTAT_STATUS", "Obsy Status",
                     THERMOSTAT_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Thermostat_StatusT[THERMOSTAT_TEMERATURE], "THERMOSTAT_TEMPERATURE", "Temperature deg.C", "NA");
    IUFillText(&Thermostat_StatusT[THERMOSTAT_HUMIDITY], "THERMOSTAT_HUMIDITY", "Humidity %", "NA");

    IUFillNumberVector(&Thermostat_setpointsNP, Thermostat_setpointN, THERMOSTAT_SETPOINT_COUNT, getDeviceName(), "THERMOSTAT_SETPOINTS", "Setpoints",
                       THERMOSTAT_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Thermostat_setpointN[THERMOSTAT_HEAT_SETPOINT], "THERMOSTAT_HEAT_SETPOINT", "Heat deg.C (0=OFF)", "%.0f", 0, 40, 1, 0);
    IUFillNumber(&Thermostat_setpointN[THERMOSTAT_COOL_SETPOINT], "THERMOSTAT_COOL_SETPOINT", "Cool deg.C (0=OFF)", "%.0f", 0, 40, 1, 0);

    // Sensors tab controls
    IUFillNumberVector(&SenseNP, SenseN, SENSE_COUNT, getDeviceName(), "SENSE_INPUTS", "Inputs",
                     SENSORS_TAB, IP_RO, 60, IPS_OK);
    IUFillNumber(&SenseN[SENSE_1], "SENSE_INPUT_1", "Input 1", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_2], "SENSE_INPUT_2", "Input 2", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_3], "SENSE_INPUT_3", "Input 3", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_4], "SENSE_INPUT_4", "Input 4", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_5], "SENSE_INPUT_5", "Input 5", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_6], "SENSE_INPUT_6", "Input 6", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_7], "SENSE_INPUT_7", "Input 7", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_8], "SENSE_INPUT_8", "Input 8", "%.2f", 0.00, 5.00, 0.01, 0);

    // Power devices tab controls
    IUFillSwitchVector(&Power_Device1SP, Power_Device1S, 1, getDeviceName(), "POWER_DEVICE1", "Device 1",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device1S[0], "POWER_DEVICE1", "DEVICE 1", ISS_OFF);
    IUFillTextVector(&Power_Device_Name1TP, Power_Device_Name1T, 1, getDeviceName(), "POWER_DEVICE_1_NAME", "Device 1",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name1T[0], "DEVICE_1_NAME", "Name", POWER_DEVICE1_NAME);

    IUFillSwitchVector(&Power_Device2SP, Power_Device2S, 1, getDeviceName(), "POWER_DEVICE2", "Device 2",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device2S[0], "POWER_DEVICE2", "DEVICE 2", ISS_OFF);
    IUFillTextVector(&Power_Device_Name2TP, Power_Device_Name2T, 1, getDeviceName(), "POWER_DEVICE_2_NAME", "Device 2",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name2T[0], "DEVICE_2_NAME", "Name", POWER_DEVICE2_NAME);

    IUFillSwitchVector(&Power_Device3SP, Power_Device3S, 1, getDeviceName(), "POWER_DEVICE3", "Device 3",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device3S[0], "POWER_DEVICE3", "DEVICE 3", ISS_OFF);
    IUFillTextVector(&Power_Device_Name3TP, Power_Device_Name3T, 1, getDeviceName(), "POWER_DEVICE_3_NAME", "Device 3",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name3T[0], "DEVICE_3_NAME", "Name", POWER_DEVICE3_NAME);

    IUFillSwitchVector(&Power_Device4SP, Power_Device4S, 1, getDeviceName(), "POWER_DEVICE4", "Device 4",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device4S[0], "POWER_DEVICE4", "DEVICE 4", ISS_OFF);
    IUFillTextVector(&Power_Device_Name4TP, Power_Device_Name4T, 1, getDeviceName(), "POWER_DEVICE_4_NAME", "Device 4",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name4T[0], "DEVICE_4_NAME", "Name", POWER_DEVICE4_NAME);

    IUFillSwitchVector(&Power_Device5SP, Power_Device5S, 1, getDeviceName(), "POWER_DEVICE5", "Device 5",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device5S[0], "POWER_DEVICE5", "DEVICE 5", ISS_OFF);
    IUFillTextVector(&Power_Device_Name5TP, Power_Device_Name5T, 1, getDeviceName(), "POWER_DEVICE_5_NAME", "Device 5",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name5T[0], "DEVICE_5_NAME", "Name", POWER_DEVICE5_NAME);

    IUFillSwitchVector(&Power_Device6SP, Power_Device6S, 1, getDeviceName(), "POWER_DEVICE6", "Device 6",
                       POWER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Power_Device6S[0], "POWER_DEVICE6", "DEVICE 6", ISS_OFF);
    IUFillTextVector(&Power_Device_Name6TP, Power_Device_Name6T, 1, getDeviceName(), "POWER_DEVICE_6_NAME", "Device 6",
               POWER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Power_Device_Name6T[0], "DEVICE_6_NAME", "Name", POWER_DEVICE6_NAME);

    addAuxControls();
    return true;
}

/********************************************************************************************
** INDI request to update the properties because there is a change in CONNECTION status
** This function is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool OCS::updateProperties()
{
    INDI::Dome::updateProperties();
    if (isConnected())
    {
        defineProperty(&Thermostat_StatusTP);
        defineProperty(&Thermostat_setpointsNP);
        defineProperty(&SenseNP);
        if (power_device_relays[0] > 0) {
            defineProperty(&Power_Device1SP);
            defineProperty(&Power_Device_Name1TP);
        }
        if (power_device_relays[1] > 0) {
            defineProperty(&Power_Device2SP);
            defineProperty(&Power_Device_Name2TP);
        }
        if (power_device_relays[2] > 0) {
            defineProperty(&Power_Device3SP);
            defineProperty(&Power_Device_Name3TP);
        }
        if (power_device_relays[3] > 0) {
            defineProperty(&Power_Device4SP);
            defineProperty(&Power_Device_Name4TP);
        }
        if (power_device_relays[4] > 0) {
            defineProperty(&Power_Device5SP);
            defineProperty(&Power_Device_Name5TP);
        }
        if (power_device_relays[5] > 0) {
            defineProperty(&Power_Device6SP);
            defineProperty(&Power_Device_Name6TP);
        }
    }
    else
    {
        deleteProperty(Thermostat_StatusTP.name);
        deleteProperty(Thermostat_setpointsNP.name);
        deleteProperty(SenseNP.name);
        if (power_device_relays[0] > 0) {
            deleteProperty(Power_Device1SP.name);
            deleteProperty(Power_Device_Name1TP.name);
        }
        if (power_device_relays[1] > 0) {
            deleteProperty(Power_Device2SP.name);
            deleteProperty(Power_Device_Name2TP.name);
        }
        if (power_device_relays[2] > 0) {
            deleteProperty(Power_Device3SP.name);
            deleteProperty(Power_Device_Name3TP.name);
        }
        if (power_device_relays[3] > 0) {
            deleteProperty(Power_Device4SP.name);
            deleteProperty(Power_Device_Name4TP.name);
        }
        if (power_device_relays[4] > 0) {
            deleteProperty(Power_Device5SP.name);
            deleteProperty(Power_Device_Name5TP.name);
        }
        if (power_device_relays[5] > 0) {
            deleteProperty(Power_Device6SP.name);
            deleteProperty(Power_Device_Name6TP.name);
        }
    }
    return true;
}

/********************************************************************************************
* Poll properties for updates
********************************************************************************************/
void OCS::TimerHit()
{
    // Get the roof/shutter status
    char roof_status_response[RB_MAX_LEN] = {0};
    int roof_status_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, roof_status_response, OCS_get_roof_status);
    if (roof_status_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        char *split;
        split = strtok(roof_status_response, ",");
        if (strcmp(split, "o") == 0) {
            if (getShutterState() != SHUTTER_MOVING) {
                setShutterState(SHUTTER_MOVING);
            }
            split = strtok(NULL, ",");
            LOGF_DEBUG("Roof/shutter is opening. %s", split);
        } else if (strcmp(split, "c") == 0) {
            if (getShutterState() != SHUTTER_MOVING) {
                setShutterState(SHUTTER_MOVING);
            }
            split = strtok(NULL, ",");
            LOGF_DEBUG("Roof/shutter is closing. %s", split);
        } else if (strcmp(split, "i") == 0) {
            split = strtok(NULL, ",");
            if (strcmp(split, "OPEN") == 0) {
                if (getShutterState() != SHUTTER_OPENED) {
                    setShutterState(SHUTTER_OPENED);
                }
                LOG_DEBUG("Roof/shutter is open");
            } else if (strcmp(split, "CLOSED") == 0) {
                if (getShutterState() != SHUTTER_CLOSED) {
                    setShutterState(SHUTTER_CLOSED);
                }
                LOG_DEBUG("Roof/shutter is closed");
            } else if (strcmp(split, "No Error") == 0) {
                LOG_DEBUG("Roof/shutter is idle");
            }
        }
    }

    // Get the last roof error (if any)
    char roof_error_response[RB_MAX_LEN] = {0};
    int roof_error_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, roof_error_response, OCS_get_roof_last_error);
    if (roof_error_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        if (getShutterState() != SHUTTER_ERROR) {
            setShutterState(SHUTTER_ERROR);
            LOGF_DEBUG("roof_error_error_or_fail = %d", roof_error_error_or_fail);
            LOGF_DEBUG("roof_error_response = %s", roof_error_response);
        }
        if (strcmp(roof_error_response, "RERR_OPEN_SAFETY_INTERLOCK") == 0 &&
                strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Open safety interlock");
        } else if (strcmp(roof_error_response, "RERR_CLOSE_SAFETY_INTERLOCK") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Close safety interlock");
        } else if (strcmp(roof_error_response, "RERR_OPEN_UNKNOWN") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Open unknown");
        } else if (strcmp(roof_error_response, "RERR_OPEN_LIMIT_SW") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Open limit switch");
        } else if (strcmp(roof_error_response, "RERR_OPEN_MAX_TIME") == 0 &&
            strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Open max time exceeded");
        } else if (strcmp(roof_error_response, "RERR_OPEN_MIN_TIME") == 0 &&
            strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Open min time not reached");
        } else if (strcmp(roof_error_response, "RERR_CLOSE_UNKNOWN") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Close unknow");
        } else if (strcmp(roof_error_response, "RERR_CLOSE_LIMIT_SW") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Close limit switch");
        } else if (strcmp(roof_error_response, "RERR_CLOSE_MAX_TIME") == 0 &&
                   strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Close max time exceeded");
        } else if (strcmp(roof_error_response, "RERR_CLOSE_MIN_TIME") == 0 &&
            strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Close min time not reached");
        } else if (strcmp(roof_error_response, "RERR_LIMIT_SW") == 0 &&
                strcmp(roof_error_response, last_shutter_error) != 0) {
            strncpy(last_shutter_error,roof_status_response, RB_MAX_LEN);
            LOG_WARN("Roof/shutter error - Both open & close limit switches active together");
        }
    } else if (roof_error_error_or_fail == 1) {
        LOGF_WARN("Communication error on get Roof/Shutter last error %s, this update aborted, will try again...", OCS_get_roof_last_error);
    }

    // Get the Obsy Thermostat readings
    char thermostat_status_response[RB_MAX_LEN] = {0};
    int thermostat_status_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, thermostat_status_response, OCS_get_thermostat_status);
    if (thermostat_status_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        char *split;
        split = strtok(thermostat_status_response, ",");
        IUSaveText(&Thermostat_StatusT[THERMOSTAT_TEMERATURE], split);
        split = strtok(NULL, ",");
        IUSaveText(&Thermostat_StatusT[THERMOSTAT_HUMIDITY], split);
        IDSetText(&Thermostat_StatusTP, nullptr);
    }
    else {
        LOGF_WARN("Communication error on get Thermostat Status %s, this update aborted, will try again...", OCS_get_thermostat_status);
        LOGF_WARN("thermostat_status_error_or_fail = %d", thermostat_status_error_or_fail);
        LOGF_WARN("thermostat_status_response = %s", thermostat_status_response);
    }

    // Get the Thermstat setpoints
    int thermostat_heat_setpoint;
    char value[RB_MAX_LEN] = {0};
    int hot_setpoint_error_or_fail = getCommandIntResponse(PortFD, &thermostat_heat_setpoint, value, OCS_get_thermostat_heat_setpoint);
    if (hot_setpoint_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        Thermostat_setpointN[THERMOSTAT_HEAT_SETPOINT].value = thermostat_heat_setpoint;
        IDSetNumber(&Thermostat_setpointsNP, nullptr);
    }
    else {
        LOGF_WARN("Communication error on get Thermostat Heat Setpoint %s, this update aborted, will try again...", OCS_get_thermostat_heat_setpoint);
    }

    int thermostat_vent_setpoint;
    int cool_setpoint_error_or_fail = getCommandIntResponse(PortFD, &thermostat_vent_setpoint, value, OCS_get_thermostat_cool_setpoint);
    if (cool_setpoint_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        Thermostat_setpointN[THERMOSTAT_COOL_SETPOINT].value = thermostat_vent_setpoint;
        IDSetNumber(&Thermostat_setpointsNP, nullptr);
    }
    else {
        LOGF_WARN("Communication error on get Thermostat Cool Setpoint %s, this update aborted, will try again...", OCS_get_thermostat_cool_setpoint);
    }

    // Get the Sense Inputs values

    // Power tab
    IDSetText(&Power_Device_Name1TP, nullptr);
    IDSetText(&Power_Device_Name2TP, nullptr);
    IDSetText(&Power_Device_Name3TP, nullptr);
    IDSetText(&Power_Device_Name4TP, nullptr);
    IDSetText(&Power_Device_Name5TP, nullptr);
    IDSetText(&Power_Device_Name6TP, nullptr);

    // Timer loop control
    if (!isConnected())
        return; //  No need to reset timer if we are not connected anymore

    SetTimer(getCurrentPollingPeriod());
}

IPState OCS::ControlShutter(ShutterOperation operation)
{
    if (operation == SHUTTER_OPEN) {
        sendOCSCommandBlind(OCS_roof_open);
    }
    else if (operation == SHUTTER_CLOSE) {
       sendOCSCommandBlind(OCS_roof_close);
    }

    // We have to delay the polling timer to account for the delays built
    // into the functions feeding into the OCS get roof status function
    // that allow for the delays between roof/shutter start/end of travel
    // and the activation of the respective interlock switches
    // Delay from OCS in seconds, need to convert to ms and add 1/2 second
    SetTimer((ROOF_TIME_PRE_MOTION * 1000) + 500);

    return IPS_BUSY;
}

/************************************************************************************
 * Called from Dome, BaseDevice to establish contact with device
 ************************************************************************************/
bool OCS::Handshake()
{
    bool handshake_status = false;

    if (PortFD > 0) {
        Connection::Interface *activeConnection = getActiveConnection();
        if (!activeConnection->name().compare("CONNECTION_TCP")) {
            LOG_INFO("Network based connection, detection timeouts set to 2 seconds");
            OCSTimeoutMicroSeconds = 0;
            OCSTimeoutSeconds = 2;
        }
        else {
            LOG_INFO("Non-Network based connection, detection timeouts set to 0.2 seconds");
            OCSTimeoutMicroSeconds = 200000;
            OCSTimeoutSeconds = 0;
        }

        char handshake_response[RB_MAX_LEN] = {0};
        handshake_status = getCommandSingleCharErrorOrLongResponse(PortFD, handshake_response, OCS_handshake);
        if (strcmp(handshake_response, "OCS") == 0)
        {
            LOG_DEBUG("OCS handshake established");
            handshake_status = true;
            GetCapabilites();
        }
        else {
            LOGF_DEBUG("OCS handshake error, reponse was: %s", handshake_response);
        }
    }
    else {
        LOG_ERROR("OCS can't handshake, device not connected");
    }

    return handshake_status;
}

/**************************************************************************************
** Query connected OCS for capabilities - called from Handshake
***************************************************************************************/

void OCS::GetCapabilites()
{
    // Get dome presence
    char OCS_dome_present_response[RB_MAX_LEN] = {0};
    int OCS_dome_present_error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, OCS_dome_present_response, OCS_get_dome_status);
    if (OCS_dome_present_error_or_fail > 0) {
        SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK | DOME_CAN_ABS_MOVE | DOME_CAN_SYNC |
                          DOME_HAS_BACKLASH | DOME_HAS_SHUTTER);
        LOG_DEBUG("OCS has dome");
    } else {
        LOG_DEBUG("OCS does not have dome");
    }

    // Get roof delays
    char roof_timeout_response[RB_MAX_LEN] = {0};
    int roof_timeout_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, roof_timeout_response, OCS_get_timeouts);
    if (roof_timeout_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        char *split;
        split = strtok(roof_timeout_response, ",");
        ROOF_TIME_PRE_MOTION = atoi(split);
        split = strtok(NULL, ",");
        ROOF_TIME_POST_MOTION = atoi(split);
    }
    else {
        LOGF_WARN("Communication error on get roof delays %s, this update aborted, will try again...", OCS_get_timeouts);
        LOGF_DEBUG("thermostat_status_error_or_fail = %d", roof_timeout_error_or_fail);
        LOGF_DEBUG("thermostat_status_response = %s", roof_timeout_response);
    }

    // Get power relay definitions
    char power_relay_definitions_response[RB_MAX_LEN] = {0};
    int power_relay_definitions_error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, power_relay_definitions_response, OCS_get_power_definitions);
    if (power_relay_definitions_error_or_fail > 1) {
        char *split;
        split = strtok(power_relay_definitions_response, ",");
        for (int deviceNo = 0; deviceNo < POWER_DEVICE_COUNT; deviceNo ++) {
            power_device_relays[deviceNo] = atoi (split);
            split = strtok(NULL, ",");
        }
        int powerDisabled = 0;
        for (int deviceNo = 1; deviceNo <= POWER_DEVICE_COUNT; deviceNo ++) {
            powerDisabled += power_device_relays[deviceNo];
        }
        if (powerDisabled != (-1 * POWER_DEVICE_COUNT)) {
            power_tab_enabled = true;
            for (int deviceNo = 1; deviceNo <= POWER_DEVICE_COUNT; deviceNo ++) {
                if (power_device_relays[(deviceNo - 1)] != -1)
                {
                    char power_relay_name_response[RB_MAX_LEN] = {0};
                    char get_power_device_name_command[CMD_MAX_LEN] = {0};
                    sprintf(get_power_device_name_command, "%s%i%s",
                            OCS_get_power_names_part, deviceNo, OCS_command_terminator);
                    int power_relay_name_error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, power_relay_name_response, get_power_device_name_command);
                    if (power_relay_name_error_or_fail > 0) {
                        if (deviceNo == 1) {
                            strncpy(POWER_DEVICE1_NAME, power_relay_name_response, sizeof(POWER_DEVICE1_NAME));
                            IUSaveText(&Power_Device_Name1T[0], POWER_DEVICE1_NAME);
                            IDSetText(&Power_Device_Name1TP, nullptr);
                        } else if (deviceNo == 2) {
                            strncpy(POWER_DEVICE2_NAME, power_relay_name_response, sizeof(POWER_DEVICE2_NAME));
                            IUSaveText(&Power_Device_Name2T[0], POWER_DEVICE2_NAME);
                            IDSetText(&Power_Device_Name2TP, nullptr);
                        } else if (deviceNo == 3) {
                            strncpy(POWER_DEVICE3_NAME, power_relay_name_response, sizeof(POWER_DEVICE3_NAME));
                            IUSaveText(&Power_Device_Name3T[0], POWER_DEVICE3_NAME);
                            IDSetText(&Power_Device_Name3TP, nullptr);
                        } else if (deviceNo == 4) {
                            strncpy(POWER_DEVICE4_NAME, power_relay_name_response, sizeof(POWER_DEVICE4_NAME));
                            IUSaveText(&Power_Device_Name4T[0], POWER_DEVICE4_NAME);
                            IDSetText(&Power_Device_Name4TP, nullptr);
                        } else if (deviceNo == 5) {
                            strncpy(POWER_DEVICE5_NAME, power_relay_name_response, sizeof(POWER_DEVICE5_NAME));
                            IUSaveText(&Power_Device_Name5T[0], POWER_DEVICE5_NAME);
                            IDSetText(&Power_Device_Name5TP, nullptr);
                        } else if (deviceNo == 61) {
                            strncpy(POWER_DEVICE6_NAME, power_relay_name_response, sizeof(POWER_DEVICE6_NAME));
                            IUSaveText(&Power_Device_Name6T[0], POWER_DEVICE6_NAME);
                            IDSetText(&Power_Device_Name6TP, nullptr);
                        }
                    }
                }
            }
        }
    }


    // Get light relay definitions
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool OCS::Connect()
{
    bool status = INDI::Dome::Connect();
    return status;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool OCS::Disconnect()
{
    bool status = INDI::Dome::Disconnect();
    return status;
}
/********************************************************************
 * OCS command functions, copied from lx200_OnStep
 *******************************************************************/

bool OCS::sendOCSCommandBlind(const char *cmd)
{
    int error_type;
    int nbytes_write = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(PortFD, TCIFLUSH);


    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
    {
        LOGF_ERROR("CHECK CONNECTION: Error sending command %s", cmd);
        return 0; //Fail if we can't write
        //return error_type;
    }

    return 1;
}

bool OCS::sendOCSCommand(const char *cmd)
{
    char response[1] = {0};
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(PortFD, TCIFLUSH);

    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(PortFD, response, 1, OCSTimeoutSeconds, OCSTimeoutMicroSeconds, &nbytes_read);

    tcflush(PortFD, TCIFLUSH);
    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%c>", response[0]);

    if (nbytes_read < 1)
    {
        LOG_WARN("Timeout/Error on response. Check connection.");
        return false;
    }

    return (response[0] == '0'); //OnStep uses 0 for success and non zero for failure, in *most* cases;
}

int OCS::getCommandSingleCharResponse(int fd, char *data, const char *cmd)
{
    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, 1, OCSTimeoutSeconds, OCSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    if (error_type != TTY_OK)
        return error_type;

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) //given this function that should always be true, as should nbytes_read always be 1
    {
        data[nbytes_read] = '\0';
    }
    else
    {
        LOG_DEBUG("got RB_MAX_LEN bytes back (which should never happen), last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);

    return nbytes_read;
}


int OCS::getCommandDoubleResponse(int fd, double *value, char *data, const char *cmd)
{
    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OCSTimeoutSeconds, OCSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) //If within buffer, terminate string with \0 (in case it didn't find the #)
    {
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    }
    else
    {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);

    if (error_type != TTY_OK)
    {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }

    if (sscanf(data, "%lf", value) != 1)
    {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }

    return nbytes_read;
}

int OCS::getCommandIntResponse(int fd, int *value, char *data, const char *cmd)
{
    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OCSTimeoutSeconds, OCSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) //If within buffer, terminate string with \0 (in case it didn't find the #)
    {
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    }
    else
    {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }
    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    if (error_type != TTY_OK)
    {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }
    if (sscanf(data, "%i", value) != 1)
    {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }
    return nbytes_read;
}

int OCS::getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd)
{
    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OCSTimeoutSeconds, OCSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) //If within buffer, terminate string with \0 (in case it didn't find the #)
    {
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    }
    else
    {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);

    if (error_type != TTY_OK)
    {
        LOGF_DEBUG("Error %d", error_type);
        return error_type;
    }
    return nbytes_read;
}

int OCS::flushIO(int fd)
{
    tcflush(fd, TCIOFLUSH);
    int error_type = 0;
    int nbytes_read;
    std::unique_lock<std::mutex> guard(ocsCommsLock);
    tcflush(fd, TCIOFLUSH);
    do
    {
        char discard_data[RB_MAX_LEN] = {0};
        error_type = tty_read_section_expanded(fd, discard_data, '#', 0, 1000, &nbytes_read);
        if (error_type >= 0)
        {
            LOGF_DEBUG("flushIO: Information in buffer: Bytes: %u, string: %s", nbytes_read, discard_data);
        }
        //LOGF_DEBUG("flushIO: error_type = %i", error_type);
    }
    while (error_type > 0);
    return 0;
}
