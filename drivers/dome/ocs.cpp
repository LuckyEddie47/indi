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
#define WEATHER_TAB "Weather"
#define THERMOSTAT_TAB "Thermostat"
#define GPIO_TAB "GPIO"
//const char *GPIO_TAB = "GPIO";

// From indi_rolloffino
#define ROLLOFF_DURATION 15               // Seconds until Roof is fully opened or closed
#define INACTIVE_STATUS  5                // Seconds between updating status lights
#define ROR_D_PRESS      1000             // Milliseconds after issuing command before expecting response
#define MAX_CNTRL_COM_ERR 10             // Maximum consecutive errors communicating with Arduino
// Read only
#define ROOF_OPENED_SWITCH "OPENED"
#define ROOF_CLOSED_SWITCH "CLOSED"
#define ROOF_LOCKED_SWITCH "LOCKED"
#define ROOF_AUX_SWITCH    "AUXSTATE"

// Write only
#define ROOF_OPEN_RELAY     "OPEN"
#define ROOF_CLOSE_RELAY    "CLOSE"
#define ROOF_ABORT_RELAY    "ABORT"
#define ROOF_LOCK_RELAY     "LOCK"
#define ROOF_AUX_RELAY      "AUXSET"

// Arduino controller interface limits
#define MAXINOCMD        15          // Command buffer
#define MAXINOTARGET     15          // Target buffer
#define MAXINOVAL        127         // Value bufffer, sized to contain NAK error strings
#define MAXINOLINE       63          // Sized to contain outgoing command requests
#define MAXINOBUF        255         // Sized for maximum overall input / output
#define MAXINOERR        255         // System call error message buffer
#define MAXINOWAIT       2           // seconds

// Driver version id
#define VERSION_ID      "20211115"
// End from indi_rolloffino

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

//    // Load Sync position
//    defineProperty(&RoofTimeoutNP);
//    loadConfig(true, "ENCODER_TICKS");
}

/********************************************************************************************
** Client has changed the state of a switch, update
*********************************************************************************************/
bool OCS::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
//    bool switchOn = false;
//    // Make sure the call is for our device
//    if(dev != nullptr && strcmp(dev,getDeviceName()) == 0)
//    {
//        // Check if the call for our Lock switch
//        if (strcmp(name, LockSP.name) == 0)
//        {
//            // Find out which state is requested by the client
//            const char *actionName = IUFindOnSwitchName(states, names, n);
//            // If it is the same state as actionName, then we do nothing. i.e.
//            // if actionName is LOCK_ON and our Lock switch is already on, we return
//            int currentLockIndex = IUFindOnSwitchIndex(&LockSP);
//            DEBUGF(INDI::Logger::DBG_SESSION, "Lock state Requested: %s, Current: %s", actionName, LockS[currentLockIndex].name);
//            if (!strcmp(actionName, LockS[currentLockIndex].name))
//            {
//                DEBUGF(INDI::Logger::DBG_SESSION, "Lock switch is already %s", LockS[currentLockIndex].label);
//                LockSP.s = IPS_IDLE;
//                IDSetSwitch(&LockSP, NULL);
//                return true;
//            }
//            // Update the switch state
//            IUUpdateSwitch(&LockSP, states, names, n);
//            currentLockIndex = IUFindOnSwitchIndex(&LockSP);
//            LockSP.s = IPS_OK;
//            IDSetSwitch(&LockSP, nullptr);
//            if (strcmp(LockS[currentLockIndex].name, "LOCK_ENABLE") == 0)
//                switchOn = true;
//            setRoofLock(switchOn);
//            updateRoofStatus();
//        }
//
//        // Check if the call for our Aux switch
//        if (strcmp(name, AuxSP.name) == 0)
//        {
//            // Find out which state is requested by the client
//            const char *actionName = IUFindOnSwitchName(states, names, n);
//            // If it is the same state as actionName, then we do nothing. i.e.
//            // if actionName is AUX_ON and our Aux switch is already on, we return
//            int currentAuxIndex = IUFindOnSwitchIndex(&AuxSP);
//            DEBUGF(INDI::Logger::DBG_SESSION, "Auxiliary state Requested: %s, Current: %s", actionName, AuxS[currentAuxIndex].name);
//            if (!strcmp(actionName, AuxS[currentAuxIndex].name))
//            {
//                DEBUGF(INDI::Logger::DBG_SESSION, "Auxiliary switch is already %s", AuxS[currentAuxIndex].label);
//                AuxSP.s = IPS_IDLE;
//                IDSetSwitch(&AuxSP, NULL);
//                return true;
//            }
//            // Update the switch state
//            IUUpdateSwitch(&AuxSP, states, names, n);
//            currentAuxIndex = IUFindOnSwitchIndex(&AuxSP);
//            AuxSP.s = IPS_OK;
//            IDSetSwitch(&AuxSP, nullptr);
//            if (strcmp(AuxS[currentAuxIndex].name, "AUX_ENABLE") == 0)
//                switchOn = true;
//            setRoofAux(switchOn);
//            updateRoofStatus();
//        }
//    }
    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}


bool OCS::ISNewNumber(const char *dev,const char *name,double values[],char *names[],int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
        if (!strcmp(Thermostat_setpointsNP.name, name)) {
            if ( THERMOSTAT_SETPOINT_COUNT == n) {
                for (int parameter = THERMOSTAT_HEAT_SETPOINT; parameter < THERMOSTAT_SETPOINT_COUNT; parameter++) {
                    if (parameter == THERMOSTAT_HEAT_SETPOINT) {
                        char thermostat_setpoint_command[RB_MAX_LEN];
                        sprintf(thermostat_setpoint_command, "%s%.0f%s",
                                OCS_set_thermostat_heat_setpoint_part, values[THERMOSTAT_HEAT_SETPOINT], OCS_command_terminator);
                        if (!sendOCSCommand(thermostat_setpoint_command)) {
                            LOGF_INFO("Set Thermostat heat setpoint to: %.0f deg.C", values[THERMOSTAT_HEAT_SETPOINT]);
                        } else {
                            LOG_WARN("Failed to set Thermostat heat setpoint");
                        }
                    }
                    else if (parameter == THERMOSTAT_COOL_SETPOINT) {
                        char thermostat_setpoint_command[RB_MAX_LEN];
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


//void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
//               char *names[], int n)
//{
//    INDI_UNUSED(dev);
//    INDI_UNUSED(name);
//    INDI_UNUSED(sizes);
//    INDI_UNUSED(blobsizes);
//    INDI_UNUSED(blobs);
//    INDI_UNUSED(formats);
//    INDI_UNUSED(names);
//    INDI_UNUSED(n);
//}



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


//    IUFillTextVector(&ThermostatStatusTP, ThermostatStatusT, 1, getDeviceName(), "ThermostatStatus", "Thermostat status", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
//    IUFillText(&ThermostatStatusTP, ThermostatStatusT,);

//    IUFillSwitch(&LockS[LOCK_DISABLE], "LOCK_DISABLE", "Off", ISS_ON);
//    IUFillSwitch(&LockS[LOCK_ENABLE], "LOCK_ENABLE", "On", ISS_OFF);
//    IUFillSwitchVector(&LockSP, LockS, 2, getDeviceName(), "LOCK", "Lock", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
//
//    IUFillSwitch(&AuxS[AUX_DISABLE], "AUX_DISABLE", "Off", ISS_ON);
//    IUFillSwitch(&AuxS[AUX_ENABLE], "AUX_ENABLE", "On", ISS_OFF);
//    IUFillSwitchVector(&AuxSP, AuxS, 2, getDeviceName(), "AUX", "Auxiliary", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
//
//    IUFillLight(&RoofStatusL[ROOF_STATUS_OPENED], "ROOF_OPENED", "Opened", IPS_IDLE);
//    IUFillLight(&RoofStatusL[ROOF_STATUS_CLOSED], "ROOF_CLOSED", "Closed", IPS_IDLE);
//    IUFillLight(&RoofStatusL[ROOF_STATUS_MOVING], "ROOF_MOVING", "Moving", IPS_IDLE);
//    IUFillLight(&RoofStatusL[ROOF_STATUS_LOCKED], "ROOF_LOCK", "Roof Lock", IPS_IDLE);
//    IUFillLight(&RoofStatusL[ROOF_STATUS_AUXSTATE], "ROOF_AUXILIARY", "Roof Auxiliary", IPS_IDLE);
//    IUFillLightVector(&RoofStatusLP, RoofStatusL, 5, getDeviceName(), "ROOF STATUS", "Roof Status", THERMOSTAT_TAB, IPS_BUSY);

//    IUFillText(&OnstepStat[0], ":GU# return", "", "");
//    IUFillTextVector(&OnstepStatTP, OnstepStat, 11, getDeviceName(), "OnStep Status", "", THERMOSTAT_TAB, IP_RO, 0, IPS_OK);


//    IUFillNumber(&RoofTimeoutN[0], "ROOF_TIMEOUT", "Timeout in Seconds", "%3.0f", 1, 300, 1, 15);
//    IUFillNumberVector(&RoofTimeoutNP, RoofTimeoutN, 1, getDeviceName(), "ROOF_MOVEMENT", "Roof Movement", OPTIONS_TAB, IP_RW,
//                       60, IPS_IDLE);

//    SetParkDataType(PARK_NONE);
//    defineProperty(&ThermostatStatusTP);;

    // Thermostat tab controls
    IUFillTextVector(&Thermostat_StatusTP, Thermostat_StatusT, THERMOSTAT_COUNT, getDeviceName(), "THERMOSTAT_STATUS", "Obsy Status",
                     THERMOSTAT_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Thermostat_StatusT[THERMOSTAT_TEMERATURE], "THERMOSTAT_TEMPERATURE", "Temperature deg.C", "NA");
    IUFillText(&Thermostat_StatusT[THERMOSTAT_HUMIDITY], "THERMOSTAT_HUMIDITY", "Humidity %", "NA");

    IUFillNumberVector(&Thermostat_setpointsNP, Thermostat_setpointN, THERMOSTAT_SETPOINT_COUNT, getDeviceName(), "THERMOSTAT_SETPOINTS", "Setpoints",
                       THERMOSTAT_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Thermostat_setpointN[THERMOSTAT_HEAT_SETPOINT], "THERMOSTAT_HEAT_SETPOINT", "Heat deg.C (0=OFF)", "%.0f", 0, 40, 1, 0);
    IUFillNumber(&Thermostat_setpointN[THERMOSTAT_COOL_SETPOINT], "THERMOSTAT_COOL_SETPOINT", "Cool deg.C (0=OFF)", "%.0f", 0, 40, 1, 0);

    // GPIO tab controls
    IUFillNumberVector(&SenseNP, SenseN, SENSE_COUNT, getDeviceName(), "SENSE_INPUTS", "Inputs",
                     GPIO_TAB, IP_RO, 60, IPS_OK);
    IUFillNumber(&SenseN[SENSE_1], "SENSE_INPUT_1", "Input 1", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_2], "SENSE_INPUT_2", "Input 2", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_3], "SENSE_INPUT_3", "Input 3", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_4], "SENSE_INPUT_4", "Input 4", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_5], "SENSE_INPUT_5", "Input 5", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_6], "SENSE_INPUT_6", "Input 6", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_7], "SENSE_INPUT_7", "Input 7", "%.2f", 0.00, 5.00, 0.01, 0);
    IUFillNumber(&SenseN[SENSE_8], "SENSE_INPUT_8", "Input 8", "%.2f", 0.00, 5.00, 0.01, 0);

    IUFillSwitchVector(&RelaySP, RelayS, RELAY_COUNT, getDeviceName(), "RELAY_OUTPUTS", "Outputs",
                       GPIO_TAB, IP_RW, ISR_NOFMANY, 60, IPS_OK);
    IUFillSwitch(&RelayS[RELAY_1], "RELAY_OUTPUT_1", "Output 1", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_2], "RELAY_OUTPUT_2", "Output 2", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_3], "RELAY_OUTPUT_3", "Output 3", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_4], "RELAY_OUTPUT_4", "Output 4", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_5], "RELAY_OUTPUT_5", "Output 5", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_6], "RELAY_OUTPUT_6", "Output 6", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_7], "RELAY_OUTPUT_7", "Output 7", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_8], "RELAY_OUTPUT_8", "Output 8", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_9], "RELAY_OUTPUT_9", "Output 9", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_10], "RELAY_OUTPUT_10", "Output 10", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_11], "RELAY_OUTPUT_11", "Output 11", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_12], "RELAY_OUTPUT_12", "Output 12", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_13], "RELAY_OUTPUT_13", "Output 13", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_14], "RELAY_OUTPUT_14", "Output 14", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_15], "RELAY_OUTPUT_15", "Output 15", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_16], "RELAY_OUTPUT_16", "Output 16", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_17], "RELAY_OUTPUT_17", "Output 17", ISS_OFF);
    IUFillSwitch(&RelayS[RELAY_18], "RELAY_OUTPUT_18", "Output 18", ISS_OFF);

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
//        defineProperty(&LockSP);            // Lock Switch,
//        defineProperty(&AuxSP);             // Aux Switch,
//        defineProperty(&RoofStatusLP);      // All the roof status lights
//        defineProperty(&OnstepStatTP);
//        defineProperty(&RoofTimeoutNP);
//        setupConditions();
        defineProperty(&Thermostat_StatusTP);
        defineProperty(&Thermostat_setpointsNP);
        defineProperty(&SenseNP);
        defineProperty(&RelaySP);
    }
    else
    {
//        deleteProperty(RoofStatusLP.name);  // Delete the roof status lights
//        deleteProperty(OnstepStatTP.name);
//        deleteProperty(LockSP.name);        // Delete the Lock Switch buttons
//        deleteProperty(AuxSP.name);         // Delete the Auxiliary Switch buttons
//        deleteProperty(RoofTimeoutNP.name);
        deleteProperty(Thermostat_StatusTP.name);
        deleteProperty(Thermostat_setpointsNP.name);
        deleteProperty(SenseNP.name);
        deleteProperty(RelaySP.name);
    }
    return true;
}

/********************************************************************************************
** Establish conditions on a connect.
*********************************************************************************************/
//bool OCS::setupConditions()
//{
//    updateRoofStatus();
//
//    // If we have Dome parking data
//    if (InitPark())
//    {
//        DEBUG(INDI::Logger::DBG_SESSION, "Dome parking data was obtained");
//    }
//        // If we do not have Dome parking data
//    else
//     {
//         DEBUG(INDI::Logger::DBG_SESSION,"Dome parking data was not obtained");
//     }
//
//    Dome::DomeState curState = getDomeState();
//    switch (curState)
//    {
//        case DOME_UNKNOWN:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNKNOWN");
//            break;
//        case    DOME_ERROR:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_ERROR");
//            break;
//        case DOME_IDLE:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_IDLE ");
//            break;
//        case     DOME_MOVING:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_MOVING");
//            break;
//        case     DOME_SYNCED:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_SYNCED");
//            break;
//        case     DOME_PARKING:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_PARKING");
//            break;
//        case    DOME_UNPARKING:
//            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNPARKING");
//            break;
//        case    DOME_PARKED:
//            if (isParked())
//            {
//                DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_PARKED");
//            }
//            else
//            {
//                DEBUG(INDI::Logger::DBG_SESSION,"Dome state is DOME_PARKED but Dome status is unparked");
//            }
//            break;
//        case    DOME_UNPARKED:
//            if (!isParked())
//            {
//                DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNPARKED");
//            }
//            else
//            {
//                DEBUG(INDI::Logger::DBG_SESSION,"Dome state is DOME_UNPARKED but Dome status is parked");
//            }
//            break;
//
//    }
//
//     // If the roof is clearly fully opened or fully closed, set the Dome::IsParked status to match.
//     // Otherwise if Dome:: park status different from roof status, o/p message (the roof might be or need to be operating manually)
//     // If park status is the same, and Dome:: state does not match, change the Dome:: state.
//     if (isParked())
//     {
//         if (fullyOpenedLimitSwitch == ISS_ON)
//         {
//             SetParked(false);
//         }
//         else if (fullyClosedLimitSwitch == ISS_OFF)
//         {
//                DEBUG(INDI::Logger::DBG_WARNING,"Dome indicates it is parked but roof closed switch not set, manual intervention needed");
//         }
//         else
//         {
//             // When Dome status agrees with roof status (closed), but Dome state differs, set Dome state to parked
//             if (curState != DOME_PARKED)
//             {
//                 DEBUG(INDI::Logger::DBG_SESSION,"Setting Dome state to DOME_PARKED.");
//                 setDomeState(DOME_PARKED);
//             }
//         }
//     }
//     else
//     {
//         if (fullyClosedLimitSwitch == ISS_ON)
//         {
//             SetParked(true);
//         }
//         else if (fullyOpenedLimitSwitch == ISS_OFF)
//         {
//             DEBUG(INDI::Logger::DBG_WARNING,"Dome indicates it is unparked but roof open switch is not set, manual intervention needed");
//         }
//         else
//             // When Dome status agrees with roof status (open), but Dome state differs, set Dome state to unparked
//         {
//             if (curState != DOME_UNPARKED)
//             {
//                 DEBUG(INDI::Logger::DBG_SESSION,"Setting Dome state to DOME_UNPARKED.");
//                 setDomeState(DOME_UNPARKED);
//             }
//         }
//     }
//    return true;
//}



//void OCS::updateRoofStatus()
//{
//    bool auxiliaryState = false;
//    bool lockedState = false;
//    bool openedState = false;
//    bool closedState = false;
//
//    getFullOpenedLimitSwitch(&openedState);
//    getFullClosedLimitSwitch(&closedState);
//    getRoofLockedSwitch(&lockedState);
//    getRoofAuxSwitch(&auxiliaryState);
//
//    if (!openedState && !closedState && !roofOpening && !roofClosing)
//        DEBUG(INDI::Logger::DBG_WARNING,"Roof stationary, neither opened or closed, adjust to match PARK button");
//    if (openedState && closedState)
//        DEBUG(INDI::Logger::DBG_WARNING,"Roof showing it is both opened and closed according to the controller");
//
//    RoofStatusL[ROOF_STATUS_AUXSTATE].s = IPS_IDLE;
//    RoofStatusL[ROOF_STATUS_LOCKED].s = IPS_IDLE;
//    RoofStatusL[ROOF_STATUS_OPENED].s = IPS_IDLE;
//    RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_IDLE;
//    RoofStatusL[ROOF_STATUS_MOVING].s = IPS_IDLE;
//    RoofStatusLP.s = IPS_IDLE;
//
//    if (auxiliaryState)
//    {
//        RoofStatusL[ROOF_STATUS_AUXSTATE].s = IPS_OK;
//    }
//    if (lockedState)
//    {
//        RoofStatusL[ROOF_STATUS_LOCKED].s = IPS_ALERT;         // Red to indicate lock is on
//        if (closedState)
//        {
//            RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_OK;            // Closed and locked roof status is normal
//            RoofStatusLP.s = IPS_OK;                               // Summary roof status
//        }
//        // An actual roof lock would not be expected unless roof was closed.
//        // However the controller might be using it to prevent motion for some other reason.
//        else if (openedState)
//        {
//            RoofStatusL[ROOF_STATUS_OPENED].s = IPS_OK;         // Possible, rely on open/close lights to indicate situation
//            RoofStatusLP.s = IPS_OK;
//        }
//        else if (roofOpening || roofClosing)
//        {
//            RoofStatusLP.s = IPS_ALERT;                            // Summary roof status
//            RoofStatusL[ROOF_STATUS_MOVING].s = IPS_ALERT;         // Should not be moving while locked
//        }
//    }
//    else
//    {
//        if (openedState || closedState)
//        {
//            if (openedState && !closedState)
//            {
//                roofOpening = false;
//                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_OK;
//                RoofStatusLP.s = IPS_OK;
//            }
//            if (closedState && !openedState)
//            {
//                roofClosing = false;
//                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_OK;
//                RoofStatusLP.s = IPS_OK;
//            }
//        }
//        else if (roofOpening || roofClosing)
//        {
//            if (roofOpening)
//            {
//                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_BUSY;
//                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
//            }
//            else if (roofClosing)
//            {
//                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_BUSY;
//                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
//            }
//            RoofStatusLP.s = IPS_BUSY;
//        }
//
//        // Roof is stationary, neither opened or closed
//        else
//        {
//            if (roofTimedOut == EXPIRED_OPEN)
//                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_ALERT;
//            else if (roofTimedOut == EXPIRED_CLOSE)
//                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_ALERT;
//            RoofStatusLP.s = IPS_ALERT;
//        }
//    }
//    IDSetLight(&RoofStatusLP, nullptr);
//}

/*******************************************************************************************
 * Poll properties for updates
 ******************************************************************************************/
//void OCS::TimerHit()
//{
//    if (!isConnected())
//            return;
//
//    SetTimer(getCurrentPollingPeriod());
//}

/********************************************************************************************
* Poll properties for updates
********************************************************************************************/
void OCS::TimerHit()
{
    // Get the roof status
    char roof_status_response[RB_MAX_LEN] = {0};
    int roof_status_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, roof_status_response, OCS_get_roof_status);
    if (roof_status_error_or_fail > 1) { //> 1 as an OnStep error would be 1 char in response
        char *split;
        split = strtok(roof_status_response, ",");
        if (strcmp(split, "o") == 0) {
            setShutterState(SHUTTER_MOVING);
            split = strtok(NULL, ",");
            LOGF_DEBUG("Roof/shutter is opening. %s", split);
        } else if (strcmp(split, "c") == 0) {
            setShutterState(SHUTTER_MOVING);
            split = strtok(NULL, ",");
            LOGF_DEBUG("Roof/shutter is closing. %s", split);
        } else if (strcmp(split, "i") == 0) {
            split = strtok(NULL, ",");
            if (strcmp(split, "OPEN") == 0) {
                setShutterState(SHUTTER_OPENED);
                LOG_DEBUG("Roof/shutter is open");
            } else if (strcmp(split, "CLOSED") == 0) {
                setShutterState(SHUTTER_CLOSED);
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

            LOGF_WARN("roof_error_error_or_fail = %d", roof_error_error_or_fail);
            LOGF_WARN("roof_error_response = %s", roof_error_response);

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


    // Timer loop control
    if (!isConnected())
        return; //  No need to reset timer if we are not connected anymore

    SetTimer(getCurrentPollingPeriod());
}

//float OCS::CalcTimeLeft(timeval start)
//{
//    double timesince;
//    double timeleft;
//    struct timeval now { 0, 0 };
//    gettimeofday(&now, nullptr);
//
//    timesince =
//            (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) - (double)(start.tv_sec * 1000.0 + start.tv_usec / 1000);
//    timesince = timesince / 1000;
//    timeleft  = MotionRequest - timesince;
//    return timeleft;
//}
//
//bool OCS::saveConfigItems(FILE *fp)
//{
//    bool status = INDI::Dome::saveConfigItems(fp);
//    IUSaveConfigNumber(fp, &RoofTimeoutNP);
//    return status;
//}
//
///*
// * Direction: DOME_CW Clockwise = Open; DOME-CCW Counter clockwise = Close
// * Operation: MOTION_START, | MOTION_STOP
// */
//IPState OCS::Move(DomeDirection dir, DomeMotionCommand operation)
//{
//    updateRoofStatus();
//    if (operation == MOTION_START)
//    {
//        if (roofLockedSwitch)
//        {
//            LOG_WARN("Roof is externally locked, no movement possible");
//            return IPS_ALERT;
//        }
//        if (roofOpening)
//        {
//            LOG_WARN("Roof is in process of opening, wait for completion or abort current operation");
//            return IPS_OK;
//        }
//        if (roofClosing)
//        {
//            LOG_WARN("Roof is in process of closing, wait for completion or abort current operation");
//            return IPS_OK;
//        }
//
//        // Open Roof
//        // DOME_CW --> OPEN. If we are asked to "open" while we are fully opened as the
//        // limit switch indicates, then we simply return false.
//        if (dir == DOME_CW)
//        {
//            if (fullyOpenedLimitSwitch == ISS_ON)
//            {
//                LOG_WARN("DOME_CW directive received but roof is already fully opened");
//                SetParked(false);
//                return IPS_ALERT;
//            }
//            // getWeatherState is no longer available
//            //else if (getWeatherState() == IPS_ALERT)
//            //{
//            //    LOG_WARN("Weather conditions are in the danger zone. Cannot open roof");
//            //    return IPS_ALERT;
//            //}
//
//            // Initiate action
//            if (roofOpen())
//            {
//                roofOpening = true;
//                roofClosing = false;
//                LOG_INFO("Roof is opening...");
//            }
//            else
//            {
//                LOG_WARN("Failed to operate controller to open roof");
//                return IPS_ALERT;
//            }
//        }
//
//        // Close Roof
//        else if (dir == DOME_CCW)
//        {
//            if (fullyClosedLimitSwitch == ISS_ON)
//            {
//                SetParked(true);
//                LOG_WARN("DOME_CCW directive received but roof is already fully closed");
//                return IPS_ALERT;
//            }
//            else if (INDI::Dome::isLocked())
//            {
//                DEBUG(INDI::Logger::DBG_WARNING,
//                  "Cannot close dome when mount is locking. See: Telescope parkng policy, in options tab");
//                return IPS_ALERT;
//            }
//            // Initiate action
//            if (roofClose())
//            {
//                roofClosing = true;
//                roofOpening = false;
//                LOG_INFO("Roof is closing...");
//            }
//            else
//            {
//                LOG_WARN("Failed to operate controller to close roof");
//                return IPS_ALERT;
//            }
//        }
//        roofTimedOut = EXPIRED_CLEAR;
//        MotionRequest = (int)RoofTimeoutN[0].value;
//        LOGF_DEBUG("Roof motion timeout setting: %d", (int)MotionRequest);
//        gettimeofday(&MotionStart, nullptr);
//        SetTimer(1000);
//        return IPS_BUSY;
//    }
//     return    IPS_ALERT;
//}
///*
// * Close Roof
// *
// */
//IPState OCS::Park()
//{
//    IPState rc = INDI::Dome::Move(DOME_CCW, MOTION_START);
//
//    if (rc == IPS_BUSY)
//    {
//        LOG_INFO("RollOff ino is parking...");
//        return IPS_BUSY;
//    }
//    else
//        return IPS_ALERT;
//}
//
///*
// * Open Roof
// *
// */
//IPState OCS::UnPark()
//{
//    IPState rc = INDI::Dome::Move(DOME_CW, MOTION_START);
//    if (rc == IPS_BUSY)
//    {
//        LOG_INFO("RollOff ino is unparking...");
//        return IPS_BUSY;
//    }
//    else
//        return IPS_ALERT;
//}
//
///*
// * Abort motion
// */
//bool OCS::Abort()
//{
//    bool lockState;
//    bool openState;
//    bool closeState;
//
//    updateRoofStatus();
//    lockState = (roofLockedSwitch == ISS_ON);
//    openState = (fullyOpenedLimitSwitch == ISS_ON);
//    closeState = (fullyClosedLimitSwitch == ISS_ON);
//
//    if (lockState)
//    {
//        LOG_WARN("Roof is externally locked, no action taken on abort request");
//        return true;
//    }
//
//    if (closeState && DomeMotionSP.s != IPS_BUSY)
//    {
//        LOG_WARN("Roof appears to be closed and stationary, no action taken on abort request");
//        return true;
//    }
//    else if (openState && DomeMotionSP.s != IPS_BUSY)
//    {
//        LOG_WARN("Roof appears to be open and stationary, no action taken on abort request");
//        return true;
//    }
//    else if (DomeMotionSP.s != IPS_BUSY)
//    {
//        LOG_WARN("Roof appears to be partially open and stationary, no action taken on abort request");
//    }
//    else if (DomeMotionSP.s == IPS_BUSY)
//    {
//        if (DomeMotionS[DOME_CW].s == ISS_ON)
//        {
//            LOG_WARN("Abort roof action requested while the roof was opening. Direction correction may be needed on the next move request.");
//        }
//        else if (DomeMotionS[DOME_CCW].s == ISS_ON)
//        {
//            LOG_WARN("Abort roof action requested while the roof was closing. Direction correction may be needed on the next move request.");
//        }
//        roofClosing = false;
//        roofOpening = false;
//        MotionRequest = -1;
//        roofAbort();
//    }
//
//    // If both limit switches are off, then we're neither parked nor unparked.
//    if (fullyOpenedLimitSwitch == ISS_OFF && fullyClosedLimitSwitch == ISS_OFF)
//    {
//        IUResetSwitch(&ParkSP);
//        ParkSP.s = IPS_IDLE;
//        IDSetSwitch(&ParkSP, nullptr);
//    }
//    return true;
//}
//
//bool OCS::getFullOpenedLimitSwitch(bool* switchState)
//{
//    if (isSimulation())
//    {
//        if (simRoofOpen)
//        {
//            fullyOpenedLimitSwitch = ISS_ON;
//            *switchState = true;
//        }
//        else
//        {
//            fullyOpenedLimitSwitch = ISS_OFF;
//            *switchState = false;
//        }
//        return true;
//    }
//
//    if (readRoofSwitch(ROOF_OPENED_SWITCH, switchState))
//    {
//        if (*switchState)
//            fullyOpenedLimitSwitch = ISS_ON;
//        else
//            fullyOpenedLimitSwitch = ISS_OFF;
//        return true;
//    }
//    else
//    {
//        LOG_WARN("Unable to obtain from the controller whether or not the roof is opened");
//        return false;
//    }
//}
//
//bool OCS::getFullClosedLimitSwitch(bool* switchState)
//{
//    if (isSimulation())
//    {
//        if (simRoofClosed)
//        {
//            fullyClosedLimitSwitch = ISS_ON;
//            *switchState = true;
//        }
//        else
//        {
//            fullyClosedLimitSwitch = ISS_OFF;
//            *switchState = false;
//        }
//        return true;
//    }
//
//    if (readRoofSwitch(ROOF_CLOSED_SWITCH, switchState))
//    {
//        if (*switchState)
//            fullyClosedLimitSwitch = ISS_ON;
//        else
//            fullyClosedLimitSwitch = ISS_OFF;
//        return true;
//    }
//    else
//    {
//        LOG_WARN("Unable to obtain from the controller whether or not the roof is closed");
//        return false;
//    }
//}
//
//bool OCS::getRoofLockedSwitch(bool* switchState)
//{
//    // If there is no lock switch, return success with status false
//    if (isSimulation())
//    {
//        roofLockedSwitch = ISS_OFF;
//        *switchState = false;                     // Not locked
//        return true;
//    }
//    if (readRoofSwitch(ROOF_LOCKED_SWITCH, switchState))
//    {
//        if (*switchState)
//            roofLockedSwitch = ISS_ON;
//        else
//            roofLockedSwitch = ISS_OFF;
//        return true;
//    }
//    else
//    {
//        LOG_WARN("Unable to obtain from the controller whether or not the roof is externally locked");
//        return false;
//    }
//}
//
//bool OCS::getRoofAuxSwitch(bool* switchState)
//{
//    // If there is no lock switch, return success with status false
//    if (isSimulation())
//    {
//        if (AuxS[AUX_ENABLE].s == ISS_OFF)
//        {
//            roofAuxiliarySwitch = ISS_OFF;
//            *switchState = false;
//            return true;
//        }
//        else
//        {
//            roofAuxiliarySwitch = ISS_ON;
//            *switchState = true;
//            return true;
//        }
//    }
//    if (readRoofSwitch(ROOF_AUX_SWITCH, switchState))
//    {
//        if (*switchState)
//            roofAuxiliarySwitch = ISS_ON;
//        else
//            roofAuxiliarySwitch = ISS_OFF;
//        return true;
//    }
//    else
//    {
//        LOG_WARN("Unable to obtain from the controller whether or not the obs Aux switch is being used");
//        return false;
//    }
//}
///*
// * -------------------------------------------------------------------------------------------
// *
// */
//bool OCS::roofOpen()
//{
//    if (isSimulation())
//    {
//        return true;
//    }
//    return pushRoofButton(ROOF_OPEN_RELAY, true, false);
//}
//
//bool OCS::roofClose()
//{
//    if (isSimulation())
//    {
//        return true;
//    }
//    return pushRoofButton(ROOF_CLOSE_RELAY, true, false);
//}
//
//bool OCS::roofAbort()
//{
//    if (isSimulation())
//    {
//        return true;
//    }
//    return pushRoofButton(ROOF_ABORT_RELAY, true, false);
//}
//
//bool OCS::setRoofLock(bool switchOn)
//{
//    if (isSimulation())
//    {
//        return false;
//    }
//    return pushRoofButton(ROOF_LOCK_RELAY, switchOn, true);
//}
//
//bool OCS::setRoofAux(bool switchOn)
//{
//    if (isSimulation())
//    {
//        return false;
//    }
//    return pushRoofButton(ROOF_AUX_RELAY, switchOn, true);
//}
//
///*
// * If unable to determine switch state due to errors, return false.
// * If no errors return true. Return in result true if switch and false if switch off.
// */
//bool OCS::readRoofSwitch(const char* roofSwitchId, bool *result)
//{
//    char readBuffer[MAXINOBUF];
//    char writeBuffer[MAXINOLINE];
//    bool status;
//
//    if (!contactEstablished)
//    {
//        LOG_WARN("No contact with the roof controller has been established");
//        return false;
//    }
//    if (roofSwitchId == 0)
//        return false;
//    memset(writeBuffer, 0, sizeof(writeBuffer));
//    strcpy(writeBuffer, "(GET:");
//    strcat(writeBuffer, roofSwitchId);
//    strcat(writeBuffer, ":0)");
//    if (!writeIno(writeBuffer))
//        return false;
//    memset(readBuffer, 0, sizeof(readBuffer));
//    if (!readIno(readBuffer))
//        return false;
//    status = evaluateResponse(readBuffer, result);
//    return status;
//}
//
///*
// * See if the controller is running
// */
//bool OCS::initialContact(void)
//{
//    char readBuffer[MAXINOBUF];
//    bool result = false;
//    contactEstablished = false;
//    if (writeIno("(CON:0:0)"))
//    {
//        memset(readBuffer, 0, sizeof(readBuffer));
//        if (readIno(readBuffer))
//        {
//            contactEstablished = evaluateResponse(readBuffer, &result);
//            return contactEstablished;
//        }
//    }
//    return false;
//}
//
///*
// * Whether roof is moving or stopped in any position along with the nature of the button requested will
// * determine the effect on the roof. This could mean stopping, or starting in a reversed direction.
// */
//bool OCS::pushRoofButton(const char* button, bool switchOn, bool ignoreLock)
//{
//    char readBuffer[MAXINOBUF];
//    char writeBuffer[MAXINOBUF];
//    bool status;
//    bool switchState = false;
//    bool responseState = false;  //true if the value in response to command was "ON"
//
//    if (!contactEstablished)
//    {
//        LOG_WARN("No contact with the roof controller has been established");
//        return false;
//    }
//    status = getRoofLockedSwitch(&switchState);        // In case it has been locked since the driver connected
//    if ((status && !switchState) || ignoreLock)
//    {
//        memset(writeBuffer, 0, sizeof(writeBuffer));
//        strcpy(writeBuffer, "(SET:");
//        strcat(writeBuffer, button);
//        if (switchOn)
//            strcat(writeBuffer, ":ON)");
//        else
//            strcat(writeBuffer, ":OFF)");
//        LOGF_DEBUG("Button pushed: %s", writeBuffer);
//        if (!writeIno(writeBuffer))             // Push identified button & get response
//            return false;
//        msSleep(ROR_D_PRESS);
//        memset(readBuffer, 0, sizeof(readBuffer));
//
//        status = readIno(readBuffer);
//        evaluateResponse(readBuffer, &responseState); // To get a log of what was returned in response to the command
//        return status;                                // Did the read itself successfully connect
//    }
//    else
//    {
//        LOG_WARN("Roof external lock state prevents roof movement");
//        return false;
//    }
//}
//
///*
// * if ACK return true and set result true|false indicating if switch is on
// *
// */
// bool OCS::evaluateResponse(char* buff, bool* result)
//{
//    char inoCmd[MAXINOCMD+1];
//    char inoTarget[MAXINOTARGET+1];
//    char inoVal[MAXINOVAL+1];
//
//    *result = false;
//    strcpy(inoCmd, strtok(buff, "(:"));
//    strcpy(inoTarget, strtok(nullptr,":"));
//    strcpy(inoVal, strtok(nullptr,")"));
//    LOGF_DEBUG("Returned from roof controller: Cmd: %s, Target: %s, Value: %s", inoCmd, inoTarget, inoVal);
//    if ((strcmp(inoCmd,"NAK")) == 0)
//    {
//        LOGF_WARN("Negative response from roof controller error: %s", inoVal);
//        return false;
//    }
//    *result = (strcmp(inoVal, "ON") == 0);
//    return true;
//}
//
//bool OCS::readIno(char* retBuf)
//{
//    bool stop = false;
//    bool start_found = false;
//    int status;
//    int retCount = 0;
//    int totalCount = 0;
//    char* bufPtr = retBuf;
//    char errMsg[MAXINOERR];
//
//    while (!stop)
//    {
//        bufPtr = bufPtr + retCount;
//        status = tty_read(PortFD, bufPtr, 1, MAXINOWAIT, &retCount);
//        if (status != TTY_OK)
//        {
//            tty_error_msg(status, errMsg, MAXINOERR);
//            LOGF_DEBUG("Roof control connection error: %s", errMsg);
//            communicationErrors++;
//            return false;
//        }
//        if (retCount > 0)
//        {
//            communicationErrors = 0;
//            if (*bufPtr == 0X28)             // '('   Start found
//                start_found = true;
//            if (!start_found)
//                retCount = 0;
//            totalCount += retCount;
//            if ((*bufPtr == 0X29) || (totalCount >= MAXINOBUF-2))    // ')'   End found
//            {
//                *(++bufPtr) = 0;
//                stop = true;
//            }
//        }
//    }
//    return true;
//}
//
//bool OCS::writeIno(const char* msg)
//{
//    int retMsgLen = 0;
//    int status;
//    char errMsg[MAXINOERR];
//
//    if (strlen(msg) >= MAXINOLINE)
//    {
//        LOG_ERROR("Roof controller command message too long");
//        return false;
//    }
//    LOGF_DEBUG("Sent to roof controller: %s", msg);
//    tcflush(PortFD, TCIOFLUSH);
//    status = tty_write_string(PortFD, msg, &retMsgLen);
//    if (status != TTY_OK)
//    {
//        tty_error_msg(status, errMsg, MAXINOERR);
//        LOGF_DEBUG("roof control connection error: %s", errMsg);
//        return false;
//    }
//    return true;
//}
//
//void OCS::msSleep (int mSec)
//{
//    struct timespec req = {0,0};
//    req.tv_sec = 0;
//    req.tv_nsec = mSec * 1000000L;
//    nanosleep(&req, (struct timespec *)nullptr);
//}

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
    SetTimer(2000);

    return IPS_BUSY;
}

/************************************************************************************
 * Called from Dome, BaseDevice to establish contact with device
 ************************************************************************************/
bool OCS::Handshake()
{
    bool handshake_status = false;

    if (PortFD > 0)
    {
        Connection::Interface *activeConnection = getActiveConnection();
        if (!activeConnection->name().compare("CONNECTION_TCP"))
        {
            LOG_INFO("Network based connection, detection timeouts set to 2 seconds");
            OCSTimeoutMicroSeconds = 0;
            OCSTimeoutSeconds = 2;
        }
        else
        {
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

            char OCS_dome_present_response[RB_MAX_LEN] = {0};
            int OCS_dome_present = getCommandSingleCharErrorOrLongResponse(PortFD, OCS_dome_present_response, OCS_get_dome_status);
            if (OCS_dome_present > 0) {
                SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK | DOME_CAN_ABS_MOVE | DOME_CAN_SYNC |
                                  DOME_HAS_BACKLASH | DOME_HAS_SHUTTER);
                LOG_DEBUG("OCS has dome");
            } else {
                LOG_DEBUG("OCS does not have dome");
            }
        }
        else
        {
            LOG_DEBUG("OCS handshake error, reponse was:");
            LOG_DEBUG(handshake_response);
        }
    }
    else
    {
        LOG_ERROR("OCS can't handshake, device not connected");
    }

    return handshake_status;
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
