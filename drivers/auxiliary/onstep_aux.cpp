/*
    OnStep Aux
    A driver to support all ancillary features of OnStepX minus telescope control
    Large parts copied from the LX200_OnStep driver

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

#include "onstep_aux.h"
#include "connectionplugins/connectiontcp.h"
#include "connectionplugins/connectionserial.h"
#include "connectionplugins/connectioninterface.h"
#include "indicom.h"

#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <mutex>





/* To do
 * connection not saving in config (not not restored?) - think it's done - test
 * switch names, handler, updates
 * dew names, handler, update
 * intervalometer all
 * outputs all
 */





// Debug only - required for attaching debugger
//  #include <signal.h>
//  #include <unistd.h>
// Debug only end

// Additional tabs
#define ROTATOR_TAB "Rotator"
#define WEATHER_TAB "Weather"
#define SWITCH_TAB "Switches"
#define DEW_TAB "Dew Heaters"
#define INTERVALOMETER_TAB "Intervalometers"
#define OUTPUT_TAB "Ouputs"
#define MANUAL_TAB "Manual"

// Define auto pointer to ourselves
std::unique_ptr<OnStep_Aux> OnStepAux(new OnStep_Aux());

// Mutex for communications
std::mutex osaCommsLock;

OnStep_Aux::OnStep_Aux() : INDI::DefaultDevice(), FI(this),  RI(this), WI(this), PI(this)
{
// Debug only
// Halts the process at this point. Allows remote debugger to attach which is required
// when launching the driver from a client eg. Ekos
//  kill(getpid(), SIGSTOP);
// Debug only end

    setVersion(0, 1);
    SlowTimer.callOnTimeout(std::bind(&OnStep_Aux::SlowTimerHit, this));
}

const char *OnStep_Aux::getDefaultName()
{
    return "OnStep Aux";
}

/******************************************************************
 * Called from connectionInterface to establish contact with device
 *****************************************************************/
bool OnStep_Aux::Handshake()
{
    if (getActiveConnection() == serialConnection) {
        PortFD = serialConnection->getPortFD();
    } else if (getActiveConnection() == tcpConnection) {
        PortFD = tcpConnection->getPortFD();
    }

    if (PortFD < 0) {
        LOG_ERROR("Failed to get valid file descriptor from connection)");
        return false;
    }

    bool handshake_status = false;
    char response[RB_MAX_LEN] = {0};
    handshake_status = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_handshake);
    if (strcmp(response, "On-Step") == 0) {
        LOG_INFO("OnStep Aux handshake established");
        handshake_status = true;
        GetCapabilites();
    } else {
        LOGF_INFO("OnStep Aux handshake error, reponse was: %s", response);
    }

    return handshake_status;
}

/**************************************************************
 * Query connected OCS for capabilities - called from Handshake
 **************************************************************/
void OnStep_Aux::GetCapabilites()
{
    uint16_t capabilities = getDriverInterface();
    // Get firmware version
    char response[RB_MAX_LEN] = {0};
    int error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_firmware);
    if (error_or_fail > 1) {
        IUSaveText(&Status_ItemsT[STATUS_FIRMWARE], response);
        IDSetText(&Status_ItemsTP, nullptr);
        LOGF_DEBUG("OnStepX version: %s", response);
    } else {
        LOG_ERROR("OnStepX version not retrieved");
    }
    if (std::stof(response) < minimum_OS_fw) {
        LOGF_WARN("OnStepX version %s is lower than this driver expects (%1.1f). Behaviour is unknown.", response, minimum_OS_fw);
    }

    // Discover focuser
    memset(response, 0, RB_MAX_LEN);
    int intResponse = 0;
    error_or_fail = getCommandIntResponse(PortFD, &intResponse, response, OS_get_defined_focusers);
    if (error_or_fail > 0 && intResponse > 0) {
        hasFocuser = true;
        FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT);
        LOG_DEBUG("Focuser found, enabling Focuser Tab");
    } else {
        LOG_DEBUG("Focuser not found, disabling Focuser Tab");
        capabilities &= ~FOCUSER_INTERFACE;
    }

    // Discover rotator
    memset(response, 0, RB_MAX_LEN);
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_defined_rotator);
    if (error_or_fail > 1 ) {
        if (response[0] == 'D' || response[0] == 'R') {
            LOG_DEBUG("Rotator found, enabling Rotator Tab");
            hasRotator = true;
            RI::SetCapability(ROTATOR_CAN_ABORT | ROTATOR_CAN_HOME | ROTATOR_HAS_BACKLASH);
        }
        if (response[0] == 'D') {
            defineProperty(&OSRotatorDerotateSP);
        }
    } else {
        LOG_DEBUG("Rotator not found, disabling Rotator Tab");
        capabilities &= ~ ROTATOR_INTERFACE;
    }

    // Discover weather sensors
    for (int measurement = 0; measurement < WEATHER_MEASUREMENTS_COUNT; measurement ++) {
        char command[CMD_MAX_LEN];
        switch(measurement) {
            case WEATHER_TEMPERATURE:
                indi_strlcpy(command, OS_get_temperature, sizeof(command));
                break;
            case WEATHER_PRESSURE:
                indi_strlcpy(command, OS_get_pressure, sizeof(command));
                break;
            case WEATHER_HUMIDITY:
                indi_strlcpy(command, OS_get_humidity, sizeof(command));
                break;
            case WEATHER_DEW_POINT:
                indi_strlcpy(command, OS_get_dew_point, sizeof(command));
                break;
            default:
                break;
        }

        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, command);
        if (error_or_fail > 1 &&
            strcmp(response, "N/A") != 0 &&
            strcmp(response, "nan") != 0 &&
            strcmp(response, "0") != 0) {
            weather_enabled[measurement] = 1;
            hasWeather = true;
        } else {
            weather_enabled[measurement] = 0;
        }
    }

    // Available weather measurements are now defined as = 1, unavailable as = 0
    // so we can sum these to check if any are defined, if not then keep tab disabled
    int weatherDisabled = 0;
    for (int wmeasure = 1; wmeasure < WEATHER_MEASUREMENTS_COUNT; wmeasure ++) {
        weatherDisabled += weather_enabled[wmeasure];
    }
    if (weatherDisabled > 0) {
        weather_tab_enabled = true;
        LOG_DEBUG("Weather sensor(s) found, enabling Weather Tab");
    } else {
        LOG_DEBUG("Weather sensor not found, disabling Weather Tab");
        capabilities &= ~WEATHER_INTERFACE;
    }


    // Split feature discovery to switches / dew / power / outputs each with own tab


    // Discover features
    memset(response, 0, RB_MAX_LEN);
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_defined_features);
    if (error_or_fail > 0) {
        int value = conversion_error;
        try {
            value = std::stoi(response);
        } catch (const std::invalid_argument&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_features, response);
        } catch (const std::out_of_range&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_features, response);
        }
        if (value > 0 ) {
            hasSwitch = true;
            LOG_DEBUG("Auxiliary Feature(s) found, enabling Features Tab");
            std::string features = response;
            for (uint digit = 0; digit < max_features; digit++) {
                features_enabled[digit] = features[digit] - '0';
            }
            // Get feature names and types
            for (int feature = 0; feature < max_features; feature++) {
                memset(response, 0, RB_MAX_LEN);
                char cmd[CMD_MAX_LEN] = {0};
                snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_feature_definiton_part, (feature + 1), OS_command_terminator);
                error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, cmd);


                LOGF_DEBUG("error_or_fail: %d", error_or_fail);


                if (error_or_fail > 0) {
                    char *split;
                    split = strtok(response, ",");
                    if (strcmp(split, "N/A") != 0) {
                        if (charToInt(split) != conversion_error) {
                            features_name[feature] = charToInt(split);
                        }
                    }
                    split = strtok(NULL, ",");
                    if (strcmp(split, "N/A") != 0) {
                        if (charToInt(split) != conversion_error) {
                            features_type[feature] = static_cast<feature_types>(charToInt(split));
                        }
                    }


                    LOGF_DEBUG("Feature%d name:%s, type:%d", feature, features_name[feature].c_str(), features_type[feature]);


                    switch(feature) {
                    case 1:


                        LOGF_DEBUG("In feature1, type:%d, name:%s", features_type[feature], features_name[feature].c_str());


                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch1_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch1_nameTP, nullptr);


                            LOG_DEBUG("In Switch1 set");


                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew1_name[0], features_name[feature].c_str());
                            IDSetText(&Dew1TP, nullptr);


                            LOG_DEBUG("In Dew1 set");
                        }

                        LOG_DEBUG("Leaving Feature1 name set");

                        break;
                    case 2:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch2_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch2_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew2_name[0], features_name[feature].c_str());
                            IDSetText(&Dew2TP, nullptr);
                        }
                        break;
                    case 3:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch3_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch3_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew3_name[0], features_name[feature].c_str());
                            IDSetText(&Dew3TP, nullptr);
                        }
                        break;
                    case 4:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch4_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch4_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew4_name[0], features_name[feature].c_str());
                            IDSetText(&Dew4TP, nullptr);
                        }
                        break;
                    case 5:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch5_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch5_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew5_name[0], features_name[feature].c_str());
                            IDSetText(&Dew5TP, nullptr);
                        }
                        break;
                    case 6:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch6_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch6_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew6_name[0], features_name[feature].c_str());
                            IDSetText(&Dew6TP, nullptr);
                        }
                        break;
                    case 7:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch7_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch7_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew7_name[0], features_name[feature].c_str());
                            IDSetText(&Dew7TP, nullptr);
                        }
                        break;
                    case 8:
                        if (features_type[feature] == (SWITCH | MOMENTARY_SWITCH | COVER_SWITCH)) {
                            IUSaveText(&Switch8_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch8_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew8_name[0], features_name[feature].c_str());
                            IDSetText(&Dew8TP, nullptr);
                        }
                        break;
                    default:
                        break;
                    }
                } else {
                    LOG_WARN("Failed to get feature definition");
                }
            }
        }
    } else {
        LOG_DEBUG("Auxiliary Feature not found, disabling Features Tab");
        capabilities &= ~AUX_INTERFACE;
    }

    //    PI::SetCapability(POWER_HAS_USB_TOGGLE);

    setDriverInterface(capabilities);
    syncDriverInfo();

    // Start polling timer (e.g., every 1000ms)
//    SetTimer(getCurrentPollingPeriod());

    // Start the slow timer for weather updates
    SlowTimer.start(60000);
    // Call the slow property update once as this is startup and we want to populate now
    SlowTimerHit();
}


/**********************************************
 * Called from defaultDevice after construction
 **********************************************/
bool OnStep_Aux::initProperties()
{
    DefaultDevice::initProperties();
    setDriverInterface(FOCUSER_INTERFACE | ROTATOR_INTERFACE | WEATHER_INTERFACE | POWER_INTERFACE | AUX_INTERFACE);

    //FocuserInterface
    //Initial, these will be updated later.


    // MAIN_CONTROL_TAB
    //-----------------
    IUFillSwitch(&ReticS[0], "PLUS", "Light", ISS_OFF);
    IUFillSwitch(&ReticS[1], "MOINS", "Dark", ISS_OFF);
    IUFillSwitchVector(&ReticSP, ReticS, 2, getDeviceName(), "RETICULE_BRIGHTNESS", "Reticule +/-", MAIN_CONTROL_TAB, IP_RW,
                       ISR_ATMOST1, 60, IPS_IDLE);

    IUFillText(&ObjectInfoT[0], "Info", "", "");
    IUFillTextVector(&ObjectInfoTP, ObjectInfoT, 1, getDeviceName(), "Object Info", "", MAIN_CONTROL_TAB,
                     IP_RO, 0, IPS_IDLE);

    //    // ============== FIRMWARE_TAB
    //    IUFillText(&VersionT[0], "Date", "", "");
    //    IUFillText(&VersionT[1], "Time", "", "");
    //    IUFillText(&VersionT[2], "Number", "", "");
    //    IUFillText(&VersionT[3], "Name", "", "");
    //    IUFillTextVector(&VersionTP, VersionT, 4, getDeviceName(), "Firmware Info", "", FIRMWARE_TAB, IP_RO, 0, IPS_IDLE);
    //

    // COMMUNICATION_TAB
    // CONNECTION_TAB
    // OPTIONS_TAB
    // Constructed form Standard Indi aux controls
    //--------------------------------------------
    addAuxControls();

    // FOCUS_TAB
    //----------
    if (hasFocuser) {
        FI::initProperties(FOCUS_TAB);

        FocusRelPosNP[0].min = 0.;
        FocusRelPosNP[0].max = 30000.;
        FocusRelPosNP[0].value = 0;
        FocusRelPosNP[0].step = 10;
        FocusAbsPosNP[0].min = 0.;
        FocusAbsPosNP[0].max = 60000.;
        FocusAbsPosNP[0].value = 0;
        FocusAbsPosNP[0].step = 10;

        IUFillSwitch(&OSFocus1InitializeS[0], "Focus1_0", "Zero", ISS_OFF);
        IUFillSwitch(&OSFocus1InitializeS[1], "Focus1_2", "Mid", ISS_OFF);
        //     IUFillSwitch(&OSFocus1InitializeS[2], "Focus1_3", "max", ISS_OFF);
        IUFillSwitchVector(&OSFocus1InitializeSP, OSFocus1InitializeS, 2, getDeviceName(), "Foc1Rate", "Initialize", FOCUS_TAB,
                           IP_RW, ISR_ATMOST1, 0, IPS_IDLE);
        // Focus T° Compensation
        // Property must be FOCUS_TEMPERATURE to be recognized by Ekos
        IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE", "TFC T°", "%+2.2f", 0, 1, 0.25,
                     25);  //default value is meaningless
        IUFillNumber(&FocusTemperatureN[1], "TFC Δ T°", "TFC Δ T°", "%+2.2f", 0, 1, 0.25, 25);  //default value is meaningless
        IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 2, getDeviceName(), "FOCUS_TEMPERATURE", "Focuser T°",
                           FOCUS_TAB, IP_RO, 0,
                           IPS_IDLE);
        IUFillSwitch(&TFCCompensationS[0], "Off", "Compensation: OFF", ISS_OFF);
        IUFillSwitch(&TFCCompensationS[1], "On", "Compensation: ON", ISS_OFF);
        IUFillSwitchVector(&TFCCompensationSP, TFCCompensationS, 2, getDeviceName(), "Compensation T°", "Temperature Compensation",
                           FOCUS_TAB, IP_RW,
                           ISR_1OFMANY, 0, IPS_IDLE);

        IUFillNumber(&TFCCoefficientN[0], "TFC Coefficient", "TFC Coefficient µm/°C", "%+03.5f", -999.99999, 999.99999, 1, 100);
        IUFillNumberVector(&TFCCoefficientNP, TFCCoefficientN, 1, getDeviceName(), "TFC Coefficient", "", FOCUS_TAB, IP_RW, 0,
                           IPS_IDLE);
        IUFillNumber(&TFCDeadbandN[0], "TFC Deadband", "TFC Deadband µm", "%g", 1, 32767, 1, 5);
        IUFillNumberVector(&TFCDeadbandNP, TFCDeadbandN, 1, getDeviceName(), "TFC Deadband", "", FOCUS_TAB, IP_RW, 0, IPS_IDLE);
        // End Focus T° Compensation
    }

    // ROTATOR_TAB
    //------------
    if (hasRotator) {
        RI::initProperties(ROTATOR_TAB);

        IUFillSwitch(&OSRotatorDerotateS[0], "Derotate_OFF", "OFF", ISS_OFF);
        IUFillSwitch(&OSRotatorDerotateS[1], "Derotate_ON", "ON", ISS_OFF);
        IUFillSwitchVector(&OSRotatorDerotateSP, OSRotatorDerotateS, 2, getDeviceName(), "Derotate_Status", "DEROTATE", ROTATOR_TAB,
                           IP_RW,
                           ISR_ATMOST1, 0, IPS_IDLE);
    }

    // WEATHER_TAB
    //------------
    if (hasWeather) {
        WI::initProperties(WEATHER_TAB, WEATHER_TAB);
        addParameter("WEATHER_TEMPERATURE", "Temperature (C)", -40, 50, 15);
        addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
        addParameter("WEATHER_BAROMETER", "Pressure (hPa)", 0, 1500, 15);
        addParameter("WEATHER_DEWPOINT", "Dew Point (C)", 0, 50, 15); // From OnStep
        setCriticalParameter("WEATHER_TEMPERATURE");
    }

    // SWITCH_TAB
    //----------------------
    IUFillSwitchVector(&Switch1SP, Switch1S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch1", "Device 1",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch1S[ON_SWITCH], "Switch1_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch1S[OFF_SWITCH], "Switch1_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch1_nameTP, Switch1_nameT, 1, getDeviceName(), "Switch_1_NAME", "Device 1",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch1_nameT[0], "DEVICE_1_NAME", "Name", "");

    IUFillSwitchVector(&Switch2SP, Switch2S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch2", "Device 2",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch2S[ON_SWITCH], "Switch2_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch2S[OFF_SWITCH], "Switch2_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch2_nameTP, Switch2_nameT, 1, getDeviceName(), "Switch_2_NAME", "Device 2",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch2_nameT[0], "DEVICE_2_NAME", "Name", "");

    IUFillSwitchVector(&Switch3SP, Switch3S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch3", "Device 3",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch3S[ON_SWITCH], "Switch3_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch3S[OFF_SWITCH], "Switch3_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch3_nameTP, Switch3_nameT, 1, getDeviceName(), "Switch_3_NAME", "Device 3",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch3_nameT[0], "DEVICE_3_NAME", "Name", "");

    IUFillSwitchVector(&Switch4SP, Switch4S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch4", "Device 4",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch4S[ON_SWITCH], "Switch4_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch4S[OFF_SWITCH], "Switch4_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch4_nameTP, Switch4_nameT, 1, getDeviceName(), "Switch_4_NAME", "Device 4",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch4_nameT[0], "DEVICE_4_NAME", "Name", "");

    IUFillSwitchVector(&Switch5SP, Switch5S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch5", "Device 5",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch5S[ON_SWITCH], "Switch5_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch5S[OFF_SWITCH], "Switch5_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch5_nameTP, Switch5_nameT, 1, getDeviceName(), "Switch_5_NAME", "Device 5",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch5_nameT[0], "DEVICE_5_NAME", "Name", "");

    IUFillSwitchVector(&Switch6SP, Switch6S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch6", "Device 6",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch6S[ON_SWITCH], "Switch6_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch6S[OFF_SWITCH], "Switch6_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch6_nameTP, Switch6_nameT, 1, getDeviceName(), "Switch_6_NAME", "Device 6",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch6_nameT[0], "DEVICE_6_NAME", "Name", "");

    IUFillSwitchVector(&Switch7SP, Switch7S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch7", "Device 7",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch7S[ON_SWITCH], "Switch7_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch7S[OFF_SWITCH], "Switch7_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch7_nameTP, Switch7_nameT, 1, getDeviceName(), "Switch_7_NAME", "Device 7",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch7_nameT[0], "DEVICE_7_NAME", "Name", "");

    IUFillSwitchVector(&Switch8SP, Switch8S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch8", "Device 8",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch8S[ON_SWITCH], "Switch8_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch8S[OFF_SWITCH], "Switch8_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Switch8_nameTP, Switch8_nameT, 1, getDeviceName(), "Switch_8_NAME", "Device 8",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch8_nameT[0], "DEVICE_8_NAME", "Name", "");

//    PI::initProperties(POWER_TAB);

    // MANUAL_TAB
    //-----------

    // Debug only
    IUFillTextVector(&Arbitary_CommandTP, Arbitary_CommandT, 1, getDeviceName(), "ARBITARY_COMMAND", "Command",
                     MANUAL_TAB, IP_RW, 60, IPS_IDLE);
    IUFillText(&Arbitary_CommandT[0], "ARBITARY_COMMANDT", "Response:", ":IP#");
    // Debug only end

    // Connection and handshake registration
    if (osaConnection & CONNECTION_SERIAL) {
        serialConnection = new Connection::Serial(this);
        serialConnection->registerHandshake([&]() { return Handshake(); });
        serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
        registerConnection(serialConnection);
    } else if (osaConnection & CONNECTION_TCP) {
        tcpConnection = new Connection::TCP(this);
        tcpConnection->setDefaultHost("192.168.0.1");
        tcpConnection->setDefaultPort(9999);
        tcpConnection->registerHandshake([&]() { return Handshake(); });
        registerConnection(tcpConnection);
    }

    return true;
}

bool OnStep_Aux::updateProperties()
{
    DefaultDevice::updateProperties();

    if (isConnected()) {
        loadConfig(true);

        Connection::Interface *activeConnection = getActiveConnection();
        if (!activeConnection->name().compare("CONNECTION_TCP")) {
            LOG_INFO("Network based connection, detection timeouts set to 2 seconds");
            OSTimeoutMicroSeconds = 0;
            OSTimeoutSeconds = 2;
        } else {
            LOG_INFO("Non-Network based connection, detection timeouts set to 0.1 seconds");
            OSTimeoutMicroSeconds = 100000;
            OSTimeoutSeconds = 0;
        }

        if (hasFocuser) {
            defineProperty(&OSFocus1InitializeSP);
            // Focus T° Compensation
            defineProperty(&FocusTemperatureNP);
            defineProperty(&TFCCompensationSP);
            defineProperty(&TFCCoefficientNP);
            defineProperty(&TFCDeadbandNP);
            FI::updateProperties();
        }

        if (hasRotator) {
            defineProperty(&OSRotatorRateSP);
            defineProperty(&OSRotatorDerotateSP);
        }

        if (hasWeather) {
            WI::updateProperties();
        }

        if (hasSwitch) {
            for (int OSfeature = 0; OSfeature < max_features; OSfeature++) {
                if (features_enabled[OSfeature] == 1) {
                    if (features_type[OSfeature] == SWITCH ||
                        features_type[OSfeature] == MOMENTARY_SWITCH ||
                        features_type[OSfeature] == COVER_SWITCH) {
                        switch (OSfeature) {
                        case 0:
                            defineProperty(&Switch1SP);
                            defineProperty(&Switch1_nameTP);
                            break;
                        case 1:
                            defineProperty(&Switch2SP);
                            defineProperty(&Switch2_nameTP);
                            break;
                        case 2:
                            defineProperty(&Switch3SP);
                            defineProperty(&Switch3_nameTP);
                            break;
                        case 3:
                            defineProperty(&Switch4SP);
                            defineProperty(&Switch4_nameTP);
                            break;
                        case 4:
                            defineProperty(&Switch5SP);
                            defineProperty(&Switch5_nameTP);
                            break;
                        case 5:
                            defineProperty(&Switch6SP);
                            defineProperty(&Switch6_nameTP);
                            break;
                        case 6:
                            defineProperty(&Switch7SP);
                            defineProperty(&Switch7_nameTP);
                            break;
                        case 7:
                            defineProperty(&Switch8SP);
                            defineProperty(&Switch8_nameTP);
                            break;
                        default:
                            break;
                        }
                    } else if (features_type[OSfeature] == DEW_HEATER) {
                        switch (OSfeature) {
                        case 0:
                            defineProperty(&Dew1TP);
                            defineProperty(&Dew1SP);
                            defineProperty(&Dew1NP);
                            break;
                        case 1:
                            defineProperty(&Dew2TP);
                            defineProperty(&Dew2SP);
                            defineProperty(&Dew2NP);
                            break;
                        case 2:
                            defineProperty(&Dew3TP);
                            defineProperty(&Dew3SP);
                            defineProperty(&Dew3NP);
                            break;
                        case 3:
                            defineProperty(&Dew4TP);
                            defineProperty(&Dew4SP);
                            defineProperty(&Dew4NP);
                            break;
                        case 4:
                            defineProperty(&Dew5TP);
                            defineProperty(&Dew5SP);
                            defineProperty(&Dew5NP);
                            break;
                        case 5:
                            defineProperty(&Dew6TP);
                            defineProperty(&Dew6SP);
                            defineProperty(&Dew6NP);
                            break;
                        case 6:
                            defineProperty(&Dew7TP);
                            defineProperty(&Dew7SP);
                            defineProperty(&Dew7NP);
                            break;
                        case 7:
                            defineProperty(&Dew8TP);
                            defineProperty(&Dew8SP);
                            defineProperty(&Dew8NP);
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }

        // Debug only
        defineProperty(&Arbitary_CommandTP);
        // Debug only end

    } else {
        if (hasFocuser) {
            deleteProperty(OSFocus1InitializeSP.name);
            // Focus T° Compensation
            deleteProperty(FocusTemperatureNP.name);
            deleteProperty(TFCCompensationSP.name);
            deleteProperty(TFCCoefficientNP.name);
            deleteProperty(TFCDeadbandNP.name);
        }

        if (hasRotator) {
            deleteProperty(OSRotatorRateSP.name);
            deleteProperty(OSRotatorDerotateSP.name);
        }

        if (hasWeather) {
        }

        if (hasSwitch) {
            for (int OSfeature = 0; OSfeature < max_features; OSfeature++) {
                if (features_enabled[OSfeature] == 1) {
                    if (features_type[OSfeature] == SWITCH ||
                        features_type[OSfeature] == MOMENTARY_SWITCH ||
                        features_type[OSfeature] == COVER_SWITCH) {
                        switch (OSfeature) {
                        case 0:
                            deleteProperty(Switch1SP.name);
                            deleteProperty(Switch1_nameTP.name);
                            break;
                        case 1:
                            deleteProperty(Switch2SP.name);
                            deleteProperty(Switch2_nameTP.name);
                            break;
                        case 2:
                            deleteProperty(Switch3SP.name);
                            deleteProperty(Switch3_nameTP.name);
                            break;
                        case 3:
                            deleteProperty(Switch4SP.name);
                            deleteProperty(Switch4_nameTP.name);
                            break;
                        case 4:
                            deleteProperty(Switch5SP.name);
                            deleteProperty(Switch5_nameTP.name);
                            break;
                        case 5:
                            deleteProperty(Switch6SP.name);
                            deleteProperty(Switch6_nameTP.name);
                            break;
                        case 6:
                            deleteProperty(Switch7SP.name);
                            deleteProperty(Switch7_nameTP.name);
                            break;
                        case 7:
                            deleteProperty(Switch8SP.name);
                            deleteProperty(Switch8_nameTP.name);
                            break;
                        default:
                            break;
                        }
                    } else if (features_type[OSfeature] == DEW_HEATER) {
                        switch (OSfeature) {
                        case 0:
                            deleteProperty(Dew1TP.name);
                            deleteProperty(Dew1SP.name);
                            deleteProperty(Dew1NP.name);
                            break;
                        case 1:
                            deleteProperty(Dew2TP.name);
                            deleteProperty(Dew2SP.name);
                            deleteProperty(Dew2NP.name);
                            break;
                        case 2:
                            deleteProperty(Dew3TP.name);
                            deleteProperty(Dew3SP.name);
                            deleteProperty(Dew3NP.name);
                            break;
                        case 3:
                            deleteProperty(Dew4TP.name);
                            deleteProperty(Dew4SP.name);
                            deleteProperty(Dew4NP.name);
                            break;
                        case 4:
                            deleteProperty(Dew5TP.name);
                            deleteProperty(Dew5SP.name);
                            deleteProperty(Dew5NP.name);
                            break;
                        case 5:
                            deleteProperty(Dew6TP.name);
                            deleteProperty(Dew6SP.name);
                            deleteProperty(Dew6NP.name);
                            break;
                        case 6:
                            deleteProperty(Dew7TP.name);
                            deleteProperty(Dew7SP.name);
                            deleteProperty(Dew7NP.name);
                            break;
                        case 7:
                            deleteProperty(Dew8TP.name);
                            deleteProperty(Dew8SP.name);
                            deleteProperty(Dew8NP.name);
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }

        // Debug only
        deleteProperty(Arbitary_CommandTP.name);
        // Debug only end

        return false;
    }

    return true;
}


bool OnStep_Aux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {

        LOGF_DEBUG("Got an IsNewSwitch for: %s", name);

//        // Output devices
//        //---------------
//        if (strcmp(Output1SP.name, name) == 0) {
//            IUUpdateSwitch(&Output1SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT1_ON") == 0) {
//                    char set_output_1_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_1_on_cmd, "%s1,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output1SP, nullptr);
//                    return sendOsaCommand(set_output_1_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT1_OFF") == 0) {
//                    char set_output_1_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_1_off_cmd, "%s1,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output1SP, nullptr);
//                    return sendOsaCommand(set_output_1_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output1SP, nullptr);
//            return false;
//        } else if (strcmp(Output2SP.name, name) == 0) {
//            IUUpdateSwitch(&Output2SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT2_ON") == 0) {
//                    char set_output_2_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_2_on_cmd, "%s2,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output2SP, nullptr);
//                    return sendOsaCommand(set_output_2_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT2_OFF") == 0) {
//                    char set_output_2_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_2_off_cmd, "%s2,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output2SP, nullptr);
//                    return sendOsaCommand(set_output_2_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output2SP, nullptr);
//            return false;
//        } else if (strcmp(Output3SP.name, name) == 0) {
//            IUUpdateSwitch(&Output3SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT3_ON") == 0) {
//                    char set_output_3_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_3_on_cmd, "%s3,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output3SP, nullptr);
//                    return sendOsaCommand(set_output_3_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT3_OFF") == 0) {
//                    char set_output_3_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_3_off_cmd, "%s3,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output3SP, nullptr);
//                    return sendOsaCommand(set_output_3_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output3SP, nullptr);
//            return false;
//        } else if (strcmp(Output4SP.name, name) == 0) {
//            IUUpdateSwitch(&Output4SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT4_ON") == 0) {
//                    char set_output_4_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_4_on_cmd, "%s4,V1%s", Osa_setFeaturePart,  Osa_command_terminator);
//                    IDSetSwitch(&Output4SP, nullptr);
//                    return sendOsaCommand(set_output_4_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT4_OFF") == 0) {
//                    char set_output_4_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_4_off_cmd, "%s4,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output4SP, nullptr);
//                    return sendOsaCommand(set_output_4_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output4SP, nullptr);
//            return false;
//        } else if (strcmp(Output5SP.name, name) == 0) {
//            IUUpdateSwitch(&Output5SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT5_ON") == 0) {
//                    char set_output_5_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_5_on_cmd, "%s5,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output5SP, nullptr);
//                    return sendOsaCommand(set_output_5_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT5_OFF") == 0) {
//                    char set_output_5_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_5_off_cmd, "%s5,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output5SP, nullptr);
//                    return sendOsaCommand(set_output_5_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output5SP, nullptr);
//            return false;
//        } else if (strcmp(Output6SP.name, name) == 0) {
//            IUUpdateSwitch(&Output6SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT6_ON") == 0) {
//                    char set_output_6_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_6_on_cmd, "%s6,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output6SP, nullptr);
//                    return sendOsaCommand(set_output_6_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT6_OFF") == 0) {
//                    char set_output_6_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_6_off_cmd, "%s6,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output6SP, nullptr);
//                    return sendOsaCommand(set_output_6_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output6SP, nullptr);
//            return false;
//        } else if (strcmp(Output7SP.name, name) == 0) {
//            IUUpdateSwitch(&Output7SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT7_ON") == 0) {
//                    char set_output_7_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_7_on_cmd, "%s7,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output7SP, nullptr);
//                    return sendOsaCommand(set_output_7_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT7_OFF") == 0) {
//                    char set_output_7_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_7_off_cmd, "%s7,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output7SP, nullptr);
//                    return sendOsaCommand(set_output_7_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output7SP, nullptr);
//            return false;
//        } else if (strcmp(Output8SP.name, name) == 0) {
//            IUUpdateSwitch(&Output8SP, states, names, n);
//            for (int i = 0; i < n; i++) {
//                if (strcmp(names[i], "OUTPUT8_ON") == 0) {
//                    char set_output_8_on_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_8_on_cmd, "%s8,V1%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output8SP, nullptr);
//                    return sendOsaCommand(set_output_8_on_cmd);
//                } else if (strcmp(names[i], "OUTPUT8_OFF") == 0) {
//                    char set_output_8_off_cmd[CMD_MAX_LEN];
//                    sprintf(set_output_8_off_cmd, "%s8,V0%s", Osa_setFeaturePart, Osa_command_terminator);
//                    IDSetSwitch(&Output8SP, nullptr);
//                    return sendOsaCommand(set_output_8_off_cmd);
//                }
//            }
//            IDSetSwitch(&Output8SP, nullptr);
//            return false;
//        }

        return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
    } else {
        return false;
    }
}

bool OnStep_Aux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    // Focus T° Compensation
    if (!strcmp(name, TFCCoefficientNP.name)) {
        // :FC[sn.n]# Set focuser temperature compensation coefficient in µ/°C
        if (abs(values[0]) < 1000) {    //Range is -999.999 .. + 999.999
            char cmd[CMD_MAX_LEN] = {0};
//            snprintf(cmd, 15, ":FC%+3.5f#", values[0]);
            snprintf(cmd, 15, "%s%+3.5f%s", OS_get_focuser_temp_comp_coef, values[0], OS_command_terminator);
            sendOSCommandBlind(cmd);
            TFCCoefficientNP.s = IPS_OK;
            IDSetNumber(&TFCCoefficientNP, "TFC Coefficient set to %+3.5f", values[0]);
        } else {
            TFCCoefficientNP.s = IPS_ALERT;
            IDSetNumber(&TFCCoefficientNP, "Setting TFC Coefficient Failed");
        }
        return true;
    }

    if (!strcmp(name, TFCDeadbandNP.name))
    {
        // :FD[n]#    Set focuser temperature compensation deadband amount (in steps or microns)
        if ((values[0] >= 1) && (values[0] <= 32768)) {  //Range is 1 .. 32767
            char cmd[CMD_MAX_LEN] = {0};
//            snprintf(cmd, 15, ":FD%d#", (int)values[0]);
            snprintf(cmd, 15, "%s%d%s", OS_get_focuser_deadband, (int)values[0], OS_command_terminator);
            sendOSCommandBlind(cmd);
            TFCDeadbandNP.s = IPS_OK;
            IDSetNumber(&TFCDeadbandNP, "TFC Deadbandset to %d", (int)values[0]);
        } else {
            TFCDeadbandNP.s = IPS_ALERT;
            IDSetNumber(&TFCDeadbandNP, "Setting TFC Deadband Failed");
        }
        return true;
    }
    // end Focus T° Compensation

    if (strstr(name, "WEATHER_")) {
        return WI::processNumber(dev, name, values, names, n);
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}


/*****************************************
 * Client has changed a text field, update
 *****************************************/
bool OnStep_Aux::ISNewText(const char *dev,const char *name,char *texts[],char *names[],int n)
{
    // Debug only - Manual tab, Arbitary command
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
        if (!strcmp(Arbitary_CommandTP.name, name)) {
            if (1 == n) {
                char response[RB_MAX_LEN] = {0};
                int error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, response, texts[0]);
                if (error_or_fail > 0) {
                    if (strcmp(response, "") == 0) {
                        indi_strlcpy(response, "No response", sizeof(response));
                    }
                } else {
                    char error_code[RB_MAX_LEN] = {0};
                    if (error_or_fail == TTY_TIME_OUT) {
                        indi_strlcpy(response, "No response", sizeof(response));
                    } else {
                        sprintf(error_code, "Error: %d", error_or_fail);
                        indi_strlcpy(response, error_code, sizeof(response));
                    }
                }
                // Replace the user entered string with the OCS response
                indi_strlcpy(texts[0], response, RB_MAX_LEN);
                IUUpdateText(&Arbitary_CommandTP, texts, names, n);
                IDSetText(&Arbitary_CommandTP, nullptr);
                return true;
            }
        }
        return false;
    // Debug only end

    } else {
        return false;
    }
}

//void OnStep_Aux::TimerHit()
//{
//    if (!isConnected())
//    {
//        return;
//    }
//
//    // Get temperatures etc.
////    readSettings();
//    timerIndex = SetTimer(getCurrentPollingPeriod());
//}

/*******************
 * Focuser functions
 ******************/

IPState OnStep_Aux::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    double output;
    char cmd[32];
    output = duration;
    if (dir == FOCUS_INWARD) output = 0 - output;
    snprintf(cmd, sizeof(cmd), "%s%5f%s",OS_move_focuser_rel_part, output, OS_command_terminator);
    sendOSCommandBlind(cmd);
    return IPS_BUSY; // Normal case, should be set to normal by update.
}

IPState OnStep_Aux::MoveAbsFocuser (uint32_t targetTicks)
{
    if (FocusAbsPosNP[0].getMax() >= int(targetTicks) && FocusAbsPosNP[0].getMin() <= int(targetTicks)) {
        char cmd[32];
        char response[RB_MAX_LEN] = {0};
        snprintf(cmd, sizeof(cmd), "%s%06d%s", OS_move_focuser_abs_part, int(targetTicks), OS_command_terminator);
        int error_or_fail = getCommandSingleCharResponse(PortFD, response, cmd);

        if (error_or_fail > 1) {
            return IPS_BUSY; // Normal case, should be set to normal by update.
        } else {
            return IPS_ALERT;
        }
    } else {
        LOG_INFO("Unable to move focuser, out of range");
        return IPS_ALERT;
    }
}

IPState OnStep_Aux::MoveRelFocuser (FocusDirection dir, uint32_t ticks)
{
    int output;
    char cmd[32];
    output = ticks;
    if (dir == FOCUS_INWARD) output = 0 - ticks;
    snprintf(cmd, sizeof(cmd), "%s%04d%s", OS_move_focuser_rel_part, output, OS_command_terminator);
    sendOSCommandBlind(cmd);
    return IPS_BUSY; // Normal case, should be set to normal by update.
}

bool OnStep_Aux::AbortFocuser ()
{
    char cmd[CMD_MAX_LEN] = {0};
    strncpy(cmd, OS_stop_focuser, sizeof(cmd));
    return sendOSCommandBlind(cmd);
}

int OnStep_Aux::OSUpdateFocuser()
{
    char value[RB_MAX_LEN] = {0};
    int value_int;
    int error_or_fail = getCommandIntResponse(PortFD, &value_int, value, OS_get_focuser_position);
    if (error_or_fail > 1) {
        FocusAbsPosNP[0].setValue( value_int);
        //         double current = FocusAbsPosNP[0].getValue();
        FocusAbsPosNP.apply();
        LOGF_DEBUG("Current focuser: %d, %f", value_int, FocusAbsPosNP[0].getValue());
    }
    char valueStatus[RB_MAX_LEN] = {0};
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, valueStatus, OS_get_focuser_status);
    if (error_or_fail > 0 ) {
        if (valueStatus[0] == 'S') {
            FocusRelPosNP.setState(IPS_OK);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_OK);
            FocusAbsPosNP.apply();
        } else if (valueStatus[0] == 'M') {
            FocusRelPosNP.setState(IPS_BUSY);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_BUSY);
            FocusAbsPosNP.apply();
        } else {
            LOG_WARN("Communication :FT# error, check connection.");
            //INVALID REPLY
            FocusRelPosNP.setState(IPS_ALERT);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_ALERT);
            FocusAbsPosNP.apply();
        }
    } else {
        //INVALID REPLY
        LOG_WARN("Communication :FT# error, check connection.");
        FocusRelPosNP.setState(IPS_ALERT);
        FocusRelPosNP.apply();
        FocusAbsPosNP.setState(IPS_ALERT);
        FocusAbsPosNP.apply();
    }
    char focus_max[RB_MAX_LEN] = {0};
    int focus_max_int;
    int fm_error = getCommandIntResponse(PortFD, &focus_max_int, focus_max, OS_get_focuser_max);
    if (fm_error > 0) {
        FocusAbsPosNP[0].setMax(focus_max_int);
        FocusAbsPosNP.updateMinMax();
        FocusAbsPosNP.apply();
        LOGF_DEBUG("focus_max: %s, %i, fm_nbchar: %i", focus_max, focus_max_int, fm_error);
    } else {
        LOG_WARN("Communication :FM# error, check connection.");
        LOGF_WARN("focus_max: %s, %u, fm_error: %i", focus_max, focus_max[0], fm_error);
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_min[RB_MAX_LEN] = {0};
    int focus_min_int ;
    int fi_error = getCommandIntResponse(PortFD, &focus_min_int, focus_min, OS_get_focuser_min);
    if (fi_error > 0) {
        FocusAbsPosNP[0].setMin( focus_min_int);
        FocusAbsPosNP.updateMinMax();
        FocusAbsPosNP.apply();
        LOGF_DEBUG("focus_min: %s, %i fi_nbchar: %i", focus_min, focus_min_int, fi_error);
    } else {
        LOG_WARN("Communication :FI# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_T[RB_MAX_LEN] = {0};
    double focus_T_double ;
    int ft_error = getCommandDoubleResponse(PortFD, &focus_T_double, focus_T, OS_get_focuser_temperature);
    if (ft_error > 0) {
        FocusTemperatureN[0].value = atof(focus_T);
        IDSetNumber(&FocusTemperatureNP, nullptr);
        LOGF_DEBUG("focus T°: %s, focus_T_double %i ft_nbcar: %i", focus_T, focus_T_double, ft_error);
    } else {
        LOG_WARN("Communication :Ft# error, check connection.");
        LOGF_DEBUG("focus T°: %s, focus_T_double %i ft_nbcar: %i", focus_T, focus_T_double, ft_error);
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_TD[RB_MAX_LEN] = {0};
    int focus_TD_int ;
    int fe_error = getCommandIntResponse(PortFD, &focus_TD_int, focus_TD, OS_get_focuser_diff_temperature);
    if (fe_error > 0) {
        FocusTemperatureN[1].value =  atof(focus_TD);
        IDSetNumber(&FocusTemperatureNP, nullptr);
        LOGF_DEBUG("focus Differential T°: %s, %i fi_nbchar: %i", focus_TD, focus_TD_int, fe_error);
    } else {
        LOG_WARN("Communication :Fe# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_Coeficient[RB_MAX_LEN] = {0};
    int focus_Coefficient_int ;
    int fC_error = getCommandIntResponse(PortFD, &focus_Coefficient_int, focus_Coeficient, OS_get_focuser_temp_comp_coef);
    if (fC_error > 0) {
        TFCCoefficientN[0].value =  atof(focus_Coeficient);
        IDSetNumber(&TFCCoefficientNP, nullptr);
        LOGF_DEBUG("TFC Coefficient: %s, %i fC_nbchar: %i", focus_Coeficient, focus_Coefficient_int, fC_error);
    } else {
        LOG_WARN("Communication :FC# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_Deadband[RB_MAX_LEN] = {0};
    int focus_Deadband_int ;
    int fD_error = getCommandIntResponse(PortFD, &focus_Deadband_int, focus_Deadband, OS_get_focuser_deadband);
    if (fD_error > 0) {
        TFCDeadbandN[0].value =  focus_Deadband_int;
        IDSetNumber(&TFCDeadbandNP, nullptr);
        LOGF_DEBUG("TFC Deadband: %s, %i fD_nbchar: %i", focus_Deadband, focus_Deadband_int, fD_error);
    } else {
        LOG_WARN("Communication :FD# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char response[RB_MAX_LEN];
    int res = getCommandSingleCharResponse(PortFD, response, OS_get_focuser_temp_comp_en);
    if (res > 0) {
        if (strcmp(response, "0")) {
            TFCCompensationSP.s = IPS_OK;
            TFCCompensationS[0].s = ISS_OFF;
            TFCCompensationS[1].s = ISS_ON;
        } else if (strcmp(response, "1")) {
            TFCCompensationSP.s = IPS_OK;
            TFCCompensationS[0].s = ISS_ON;
            TFCCompensationS[1].s = ISS_OFF;
        }
        IDSetSwitch(&TFCCompensationSP, nullptr);
        LOGF_DEBUG("TFC Enable: fc_nbchar:%d Fc_response: %s", res, response);
    } else {
        //LOGF_DEBUG("TFC Enable1: fc_error:%i Fc_response: %s", res, response);
        LOG_WARN("Communication :Fc# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    FI::updateProperties();
    LOGF_DEBUG("After update properties: FocusAbsPosN min: %f max: %f", FocusAbsPosNP[0].getMin(), FocusAbsPosNP[0].getMax());

    return 0;
}

/********************
 * Rotator functions
 *******************/

int OnStep_Aux::OSUpdateRotator()
{
    char response[RB_MAX_LEN];
    double double_value;
    if(hasRotator) {
        int error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_rotator_angle);
        if (error_or_fail == 1 && response[0] == '0') { //1 char return, response 0 = no Rotator
            LOG_INFO("Detected Response that Rotator is not present, disabling further checks");
            hasRotator = false;
            return 0; //Return 0, as this is not a communication error
        }
        if (error_or_fail < 1) {  //This does not necessarily mean
            LOG_WARN("Error talking to rotator, might be timeout (especially on network)");
            return -1;
        }
        if (f_scansexa(response, &double_value)) { // 0 = good, thus this is the bad
            GotoRotatorNP.setState(IPS_ALERT);
            GotoRotatorNP.apply();
            return -1;
        }
        GotoRotatorNP[0].setValue(double_value);
        double min_rotator, max_rotator;
        bool changed_minmax = false;
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandDoubleResponse(PortFD, &min_rotator, response, OS_get_rotator_min);
        if (error_or_fail > 1) {
            changed_minmax = true;
            GotoRotatorNP[0].setMin(min_rotator);
        }
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandDoubleResponse(PortFD, &max_rotator, response, OS_get_rotator_max);
        if (error_or_fail > 1) {
            changed_minmax = true;
            GotoRotatorNP[0].setMax(max_rotator);
        }
        if (changed_minmax) {
            GotoRotatorNP.updateMinMax();
            GotoRotatorNP.apply();
        }
        //GotoRotatorN
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_rotator_status);
        if (error_or_fail > 1) {
            if (response[0] == 'S') { /*Stopped normal on EQ mounts */
                GotoRotatorNP.setState(IPS_OK);
                GotoRotatorNP.apply();
            } else if (response[0] == 'M') { /* Moving, including de-rotation */
                GotoRotatorNP.setState(IPS_BUSY);
                GotoRotatorNP.apply();
            } else {
                //INVALID REPLY
                GotoRotatorNP.setState(IPS_ALERT);
                GotoRotatorNP.apply();
            }
        }
        memset(response, 0, RB_MAX_LEN);
        int backlash_value;
        error_or_fail = getCommandIntResponse(PortFD, &backlash_value, response, OS_get_rotator_backlash);
        if (error_or_fail > 1) {
            RotatorBacklashNP[0].setValue(backlash_value);
            RotatorBacklashNP.setState(IPS_OK);
            RotatorBacklashNP.apply();
        }
    }
    return 0;
}

IPState OnStep_Aux::MoveRotator(double angle)
{
    char cmd[CMD_MAX_LEN] = {0};
    char OS_moveRotator_response[RB_MAX_LEN] = {0};
    int d, m, s;
    getSexComponents(angle, &d, &m, &s);

    snprintf(cmd, sizeof(cmd), "%s%.03d:%02d:%02d%s", OS_set_rotator_angle_part, d, m, s, OS_command_terminator);
    LOGF_INFO("Move Rotator: %s", cmd);

    int OS_moveRotator_error_or_fail = getCommandSingleCharResponse(PortFD, OS_moveRotator_response, cmd);

    if (OS_moveRotator_error_or_fail > 1) {
        return IPS_BUSY;
    } else {
        return IPS_ALERT;
    }

    return IPS_BUSY;
}

IPState OnStep_Aux::HomeRotator()
{
    //Not entirely sure if this means attempt to use limit switches and home, or goto home
    //Assuming MOVE to Home
    LOG_INFO("Moving Rotator to Home");
    sendOSCommandBlind(OS_move_rotator_home);
    return IPS_BUSY;
}

bool OnStep_Aux::AbortRotator()
{
    LOG_INFO("Aborting Rotation, de-rotation in same state");
    sendOSCommandBlind(OS_stop_rotator); //Does NOT abort de-rotator
    return true;
}

bool OnStep_Aux::SetRotatorBacklash(int32_t steps)
{
    char cmd[CMD_MAX_LEN] = {0};
    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_set_rotator_backlash_part, steps, OS_command_terminator);
    if(sendOSCommand(cmd)) {
        return true;
    }
    return false;
}

bool OnStep_Aux::SetRotatorBacklashEnabled(bool enabled)
{
    //Nothing required here.
    INDI_UNUSED(enabled);
    return true;
    //     As it's always enabled, which would mean setting it like SetRotatorBacklash to 0, and losing any saved values. So for now, leave it as is (always enabled)
}


/***********************************************************
** Client is asking us to establish connection to the device
************************************************************/
bool OnStep_Aux::Connect()
{
    if (!INDI::DefaultDevice::Connect()) {
        LOG_ERROR("Parent Connect() failed");
        return false;
    }

    if (!Handshake()) {
        LOG_ERROR("Failed to communicate with OnStep Aux");
        return false;
    }

    // Start polling timer (e.g., every 1000ms)
    SetTimer(getCurrentPollingPeriod());

    return true;
}

/***********************************************************
** Client is asking us to terminate connection to the device
************************************************************/
bool OnStep_Aux::Disconnect()
{
    bool status = INDI::DefaultDevice::Disconnect();
    return status;
}

//*******************
// Required overrides
//******************/
void ISPoll(void *p);

//void OnStep_Aux::ISGetProperties(const char *dev)
//{
//    FI::ISGetProperties(dev);
//}

/****************************
* Poll properties for updates
****************************/
void OnStep_Aux::TimerHit()
{

}

/***************************************
* Poll properties for updates per minute
****************************************/
void OnStep_Aux::SlowTimerHit()
{
}

/*****************************************************************
* Poll Weather properties for updates - period set by Weather poll
******************************************************************/
IPState OnStep_Aux::updateWeather() {
    if (hasWeather) {

        LOG_DEBUG("Weather update called");

        for (int measurement = 0; measurement < WEATHER_MEASUREMENTS_COUNT; measurement ++) {
            if (weather_enabled[measurement] == 1) {
                char measurement_reponse[RB_MAX_LEN];
                char measurement_command[CMD_MAX_LEN];

                LOGF_DEBUG("In weather measurements loop, %u", measurement);

                switch (measurement) {
                case WEATHER_TEMPERATURE:
                    indi_strlcpy(measurement_command, OS_get_temperature, sizeof(measurement_command));
                    break;
                case WEATHER_PRESSURE:
                    indi_strlcpy(measurement_command, OS_get_pressure, sizeof(measurement_command));
                    break;
                case WEATHER_HUMIDITY:
                    indi_strlcpy(measurement_command, OS_get_humidity, sizeof(measurement_command));
                    break;
                case WEATHER_DEW_POINT:
                    indi_strlcpy(measurement_command, OS_get_dew_point, sizeof(measurement_command));
                    break;
                default:
                    break;
                }

                double value = conversion_error;
                int measurement_error_or_fail = getCommandDoubleResponse(PortFD, &value, measurement_reponse,
                                                                         measurement_command);
                if ((measurement_error_or_fail >= 0) && (value != conversion_error) &&
                    (weather_enabled[measurement] == 1)) {
                    switch(measurement) {
                    case WEATHER_TEMPERATURE:
                        setParameterValue("WEATHER_TEMPERATURE", value);
                        break;
                    case WEATHER_PRESSURE:
                        setParameterValue("WEATHER_PRESSURE", value);
                        break;
                    case WEATHER_HUMIDITY:
                        setParameterValue("WEATHER_HUMIDITY", value);
                        break;
                    case WEATHER_DEW_POINT:
                        setParameterValue("WEATHER_DEWPOINT", value);
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        if (WI::syncCriticalParameters()) {
            LOG_DEBUG("SyncCriticalParameters = true");
        } else {
            LOG_DEBUG("SyncCriticalParameters = false");
        }
    }

    return IPS_OK;
}
bool OnStep_Aux::saveConfigItems(FILE *fp)
{
    DefaultDevice::saveConfigItems(fp);
    FI::saveConfigItems(fp);
    WI::saveConfigItems(fp);
    RI::saveConfigItems(fp);
    PI::saveConfigItems(fp);
    return true;
}


/*********************************************************************
 * Send command to OCS without checking (intended non-existent) return
 * *******************************************************************/
bool OnStep_Aux::sendOSCommandBlind(const char *cmd)
{
    // No need to block this command as there is no response

    int error_type;
    int nbytes_write = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);
    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(PortFD, TCIFLUSH);
    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK) {
        LOGF_ERROR("CHECK CONNECTION: Error sending command %s", cmd);
        clearBlock();
        return 0; //Fail if we can't write
    }
    return 1;
}

/*********************************************************************
 * Send command to OCS that expects a 0 (sucess) or 1 (failure) return
 * *******************************************************************/
bool OnStep_Aux::sendOSCommand(const char *cmd)
{
    blockUntilClear();

    char response[1] = {0};
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(PortFD, TCIFLUSH);

    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(PortFD, response, 1, OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);

    tcflush(PortFD, TCIFLUSH);
    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%c>", response[0]);
    clearBlock();

    if (nbytes_read < 1) {
        LOG_WARN("Timeout/Error on response. Check connection.");
        return false;
    }

    return (response[0] == '0'); //OCS uses 0 for success and non zero for failure, in *most* cases;
}

/************************************************************
 * Send command to OCS that expects a single character return
 * **********************************************************/
int OnStep_Aux::getCommandSingleCharResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, 1, OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    if (error_type != TTY_OK)
        return error_type;

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //given this function that should always be true, as should nbytes_read always be 1
        data[nbytes_read] = '\0';
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back (which should never happen), last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    clearBlock();

    return nbytes_read;
}

/**************************************************
 * Send command to OCS that expects a double return
 * ************************************************/
int OnStep_Aux::getCommandDoubleResponse(int fd, double *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }

    if (sscanf(data, "%lf", value) != 1) {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }

    return nbytes_read;
}

/************************************************
 * Send command to OCS that expects an int return
 * **********************************************/
int OnStep_Aux::getCommandIntResponse(int fd, int *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, sizeof(char), OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }
    if (sscanf(data, "%i", value) != 1) {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }

    return nbytes_read;
}

/***************************************************************************
 * Send command to OCS that expects a char[] return (could be a single char)
 * *************************************************************************/
int OnStep_Aux::getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        return error_type;
    }

    return nbytes_read;
}

/********************************************************
 * Converts an OCS char[] return of a numeric into an int
 * ******************************************************/
int OnStep_Aux::getCommandIntFromCharResponse(int fd, char *data, int *response, const char *cmd)
{
    int errorOrFail = getCommandSingleCharErrorOrLongResponse(fd, data, cmd);
    if (errorOrFail < 1) {
        waitingForResponse = false;
        return errorOrFail;
    } else {
        int value = conversion_error;
        try {
            value = std::stoi(data);
        } catch (const std::invalid_argument&) {
            LOGF_WARN("Invalid response to %s: %s", cmd, data);
        } catch (const std::out_of_range&) {
            LOGF_WARN("Invalid response to %s: %s", cmd, data);
        }
        *response = value;
        return errorOrFail;
    }
}


/**********************
 * Flush the comms port
 * ********************/
int OnStep_Aux::flushIO(int fd)
{
    tcflush(fd, TCIOFLUSH);
    int error_type = 0;
    int nbytes_read;
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIOFLUSH);
    do {
        char discard_data[RB_MAX_LEN] = {0};
        error_type = tty_read_section_expanded(fd, discard_data, '#', 0, 1000, &nbytes_read);
        if (error_type >= 0) {
            LOGF_DEBUG("flushIO: Information in buffer: Bytes: %u, string: %s", nbytes_read, discard_data);
        }
        //LOGF_DEBUG("flushIO: error_type = %i", error_type);
    }
    while (error_type > 0);

    return 0;
}

int OnStep_Aux::charToInt (char *inString)
{
    int value = conversion_error;
    try {
        value = std::stoi(inString);
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
    return value;
}

/*******************************************************
 * Block outgoing command until previous return is clear
 * *****************************************************/
void OnStep_Aux::blockUntilClear()
{
    // Blocking wait for last command response to clear
    while (waitingForResponse) {
        usleep(((OSTimeoutSeconds * 1000000) + OSTimeoutMicroSeconds) / 10);
        //        usleep(OCSTimeoutMicroSeconds / 10);
    }
    // Grab the response waiting command blocker
    waitingForResponse = true;
}

/*********************************************
 * Flush port and clear command sequence block
 * *******************************************/
void OnStep_Aux::clearBlock()
{
    waitingForResponse = false;
}

