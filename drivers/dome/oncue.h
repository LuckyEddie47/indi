/*******************************************************************************
 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

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

#pragma once

#include "indidome.h"
#include "connectionplugins/connectiontcp.h"
#include "connectionplugins/connectionserial.h"

#define RB_MAX_LEN 64
#define CMD_MAX_LEN 32
enum ResponseErrors {RES_ERR_FORMAT = -1001};

/*******************************************************************************
OnCue OCS lexicon
Extracted from OnCue OCS 3.03i
Note all commands sent and responses returned terminate with a # symbol
These are stripped from returned char* by their retrieving functions
An unterminated 0 is returned from unconfigured items
*******************************************************************************/
// General commands

// Get Product (compatibility)
#define OCS_handshake ":IP#"
// Returns: OCS#

// Get firmware version number
#define OCS_get_firmware ":IN#"
// Returns: firmware_string# for example 3.03i#

// Get safety status
#define OCS_get_safety_status ":Gs#"
// Returns: SAFE#, UNSAFE#

// Set the watchdog reset flag - forces firmware reboot
#define OCS_set_watchdog_flag ":SW#"
// Returns: Rebooting in 8 seconds...# or CE_SLEW_IN_MOTION# for roof/dome in motion blocking error

// Set the UTC Date and Time
// ":SU[MM/DD/YYYY,HH:MM:SS]#"
// Returns: 0# on failure, 1# on success

// Get the power status
#define OCS_get_power_status ":GP#"
// Returns: OK#, OUT#, or N/A#

// Get the internal MCU temperature in deg. C
#define OCS_get_MCU_temperature ":GX9F#"
// Returns: +/-n.n# if supported, 0 if unsupported

// Set USB Baud Rate where n is an ASCII digit (1..9) with the following interpertation
// 0=115.2K, 1=56.7K, 2=38.4K, 3=28.8K, 4=19.2K, 5=14.4K, 6=9600, 7=4800, 8=2400, 9=1200
// ":SB[n]#"
// Returns: 1# (at the current baud rate and then changes to the new rate for further communication)

// Roof/shutter commands
// Roll off roof style observatory or shutter control for dome style observatory

// Command the roof/shutter to close
#define OCS_roof_close ":RC#"
// Returns: nothing

// Command the roof/shutter to open
#define OCS_roof_open ":RO#"
// Returns: nothing

// Command the roof/shutter movement to stop
#define OCS_roof_stop ":RH#"
// Returns: nothing

// Set the roof/shutter safety override - ignore stuck limit switches and timeout
#define OCS_roof_safety_override ":R!#"
// Returns: 1# on success

// Set the roof/shutter high power mode - forces motor pwm to 100%
#define OCS_roof_high_power_mode ":R+#"
// Returns: 1# on success

// Get the roof/shutter status
#define OCS_get_roof_status ":RS#"
// Returns:
// OPEN#, CLOSED#, c,Travel: n%# (for closing), o,Travel: n%# for opening,
// i,No Error# for idle

// Get the roof/shutter last status error
#define OCS_get_roof_last_error ":RSL#"
// Returns:
// RERR_OPEN_SAFETY_INTERLOCK#
// RERR_CLOSE_SAFETY_INTERLOCK#
// RERR_OPEN_UNKNOWN#
// RERR_OPEN_LIMIT_SW#
// RERR_OPEN_MAX_TIME#
// RERR_OPEN_MIN_TIME#
// RERR_CLOSE_UNKNOWN#
// RERR_CLOSE_LIMIT_SW#
// RERR_CLOSE_MAX_TIME#
// RERR_CLOSE_MIN_TIME#
// RERR_LIMIT_SW# (both open and closed limit switches simultaneously)
// nothing if never errored

//Dome commands

// Command the dome to goto the home position
#define OCS_dome_home ":DC#"
// Returns: nothing

// Reset that the dome is at home
#define OCS_reset_dome_home ":DF#"
// Returns: nothing

// Command the dome to goto the park position
#define OCS_dome_park ":DP#"
// Returns: 0# on failure, 1# on success

// Set the dome park position
#define OCS_set_dome_park ":DQ#"
// Returns: 0# on failure, 1# on success

// Restore the dome park position
#define OCS_restore_dome_park ":DR#"
// Returns: 0# on failure, 1# on success

// Command the dome movement to stop
#define OCS_dome_stop ":DH#"
// Returns: nothing

// Get the dome Azimuth (0 to 360 degrees)
#define OCS_get_dome_azimuth ":DZ#"
// Returns: D.DDD#

// Set the dome Azimuth target (0 to 360 degrees)
// ":Dz[D.D]#"
// Returns: nothing

// Get the dome Altitude (0 to 90 degrees)
#define OCS_get_dome_altitude ":DA#"
// Returns: D.D#, NAN# is no second axis

// Set the dome Altitude target (0 to 90 degrees)
// ":Da[D.D]#"
// Returns: nothing

// Set the dome to sync with target (Azimuth only)
#define OCS_dome_sync_target ":DN#"
// Returns: See :DS# command below

// Command the dome to goto target
#define OCS_dome_goto_taget ":DS#"
// Returns:
//  0# = Goto is possible
//  1# = below the horizon limit
//  2# = above overhead limit
//  3# = controller in standby
//  4# = dome is parked
//  5# = Goto in progress
//  6# = outside limits (AXIS2_LIMIT_MAX, AXIS2_LIMIT_MIN, AXIS1_LIMIT_MIN/MAX, MERIDIAN_E/W)
//  7# = hardware fault
//  8# = already in motion
//  9# = unspecified error

// Get dome status
#define OCS_get_dome_status ":DU#"
// Returns: P# if parked, H# if at Home

// Axis commands

// Axis1 is Dome Azimuth - required if dome = true
// Axis2 is Dome Altitude - optional

// Get the axis/driver configuration for axis [n]
// ":GXA[n]#"
// Returns: s,s,s,s#
// where s,s,s,s... comprises:
// parameter [0] = steps per degree,
// parameter [1] = reverse axis
// parameter [2] = minimum limit
// parameter [3] = maximum limit

// Get the stepper driver status for axis [n]
// ":GXU[n]#"
// Returns:
//  ST# = At standstill
//  OA# = Output A open load
//  OB# = Output B open load
//  GA# = Output A short to ground
//  GB# = Output B short to ground
//  OT# = Over temperature (>150 deg. C)
//  PW# = Over temperature warning (>120 deg. C)
//  GF# = Fault

// Not used, superset definition?
// Set the axis/driver configuration for axis [n]
// ":SXA[n]#"

// Revert axis/driver configuration for axis [n] to defaults
// ":SXA[n],R#"

// Set the axis/driver configuration for axis [n]
// :SXA[n],[s,s,s,s...]#
// where s,s,s,s... comprises:
// parameter [0] = steps per degree,
// parameter [1] = reverse axis
// parameter [2] = minimum limit
// parameter [3] = maximum limit

// Weather commands

// Get the outside temperature in deg. C
#define OCS_get_outside_temperature ":G1#"
// Returns: nnn.n#

// Get the sky IR temperature in deg. C
#define OCS_get_sky_IR_temperature ":G2#"
// Returns: nnn.n#

// Get the sky differential temperature
#define OCS_get_sky_diff_temperature ":G3#"
// Returns: nnn.n#
// where <= 21 is cloudy

// Get averaged sky differential temperature
#define OCS_get_av_sky_diff_temperature ":GS#"
// Returns: nnn.n#
// where <= 21 is cloudy

// Get the absolute barometric pressure as Float (mbar, sea-level compensated)
#define OCS_get_pressure ":Gb#"
// Returns: n.nnn#
// where n ranges from about 980.0 to 1050.0

// Get cloud description
#define OCS_get_cloud_description ":GC#"
// Returns: description_string#

// Get relative humidity reading as Float (% Rh)
#define OCS_get_humidity ":Gh#"
// Returns: n.n#
// where n ranges from 0.0 to 100.0

// Get sky quality in mag/arc-sec^2
#define OCS_get_sky_quality ":GQ#"
// Returns: nnn.n#

// Get rain sensor status
#define OCS_get_rain_sensor_status ":GR#"
// Returns: -1000# for invalid, 0# for N/A, 1# for Rain, 2# for Warn, and 3# for Dry

// Get wind status
#define OCS_get_wind_status ":GW#"
// Returns: OK#, HIGH#, or N/A#

// Thermostat commands

// Get thermostat status
#define OCS_get_thermostat_status ":GT#"
// Returns: n.n,m.m#
// where n.n is temperature in deg. C and m.m is % humidity

// Get heat setpoint in deg. C
#define OCS_get_thermostat_setpoint ":GH#"
// Returns: n#, or 0# for invalid

// Set heat setpoint in deg. C
// ":SHnn#"
// Example: ":SH0#" turns heat off
// Example: ":SH21#" heat setpoint 21 deg. C
// Returns: 1# on success

// Get cool/vent setpoint in deg. C
#define OCS_get_vent_setpoint ":GV#"
// Returns: n#, or 0# for invalid

//Set cool/vent setpoint in deg. C
// ":SCnnn#"
// Example: ":SC0#" turns cooling off
// Example: ":SC30#" cool setpoint 30 deg. C
// Returns: 1# on success

// Power/GPIO commands

// Get Relay n state
// ":GRn#"
// Returns: ON#, OFF#, n# (pwm 0-9)

// Set Relay n [state] = ON, OFF, DELAY, n (pwm 0 to 10)
// ":SRn,[state]#"
// Returns: 1# on success

// Get Analog n state
// ":GAn#"
// Returns: n# (0 to 1023, 0 to 5V)

// Get Digital Sense n state
// ":GSn#"
// Returns: ON#, OFF#

/*******************************************************************************
OnCue OCS lexicon end
*******************************************************************************/

class OnCueOCS : public INDI::Dome
{
  public:
    OnCueOCS();
    virtual ~OnCueOCS() override = default;

    virtual bool initProperties();
    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewNumber(const char *dev,const char *name,double values[],char *names[],int n);
    const char *getDefaultName();
    bool updateProperties();
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
//    virtual bool saveConfigItems(FILE *fp);
    virtual bool ISSnoopDevice(XMLEle *root);
    virtual bool Handshake();

  protected:
    bool Connect();
    bool Disconnect();

    void TimerHit();

//    virtual IPState Move(DomeDirection dir, DomeMotionCommand operation);
//    virtual IPState Park();
//    virtual IPState UnPark();
//    virtual bool Abort();
//
//    virtual bool getFullOpenedLimitSwitch(bool*);
//    virtual bool getFullClosedLimitSwitch(bool*);

    bool sendOnCueCommand(const char *cmd);
    bool sendOnCueCommandBlind(const char *cmd);
    int flushIO(int fd);
    int getCommandSingleCharResponse(int fd, char *data, const char *cmd); //Reimplemented from getCommandString
    int getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd); //Reimplemented from getCommandString
    int getCommandDoubleResponse(int fd, double *value, char *data,
                                 const char *cmd); //Reimplemented from getCommandString Will return a double, and raw value.
    int getCommandIntResponse(int fd, int *value, char *data, const char *cmd);

    long int OnCueTimeoutSeconds = 0;
    long int OnCueTimeoutMicroSeconds = 100000;

private:
//    void updateRoofStatus();
//    bool getRoofLockedSwitch(bool*);
//    bool getRoofAuxSwitch(bool*);
//    bool setRoofLock(bool switchOn);
//    bool setRoofAux(bool switchOn);
//    bool readRoofSwitch(const char* roofSwitchId, bool* result);
//    bool roofOpen();
//    bool roofClose();
//    bool roofAbort();
//    bool pushRoofButton(const char*, bool switchOn, bool ignoreLock);
//    bool initialContact();
//    bool evaluateResponse(char*, bool*);
//    bool writeIno(const char*);
//    bool readIno(char*);
//    void msSleep(int);
//
//    bool setupConditions();
//    float CalcTimeLeft(timeval);
//    double MotionRequest { 0 };
//    struct timeval MotionStart { 0, 0 };
//    bool contactEstablished = false;
//    bool roofOpening = false;
//    bool roofClosing = false;
//    ILight RoofStatusL[5];
//    ILightVectorProperty RoofStatusLP;
//    enum { ROOF_STATUS_OPENED, ROOF_STATUS_CLOSED, ROOF_STATUS_MOVING, ROOF_STATUS_LOCKED, ROOF_STATUS_AUXSTATE };
//
//    ISwitch LockS[2];
//    ISwitchVectorProperty LockSP;
//    enum { LOCK_ENABLE, LOCK_DISABLE };
//
//    ISwitch AuxS[2];
//    ISwitchVectorProperty AuxSP;
//    enum { AUX_ENABLE, AUX_DISABLE };
//
//    ISState fullyOpenedLimitSwitch {ISS_OFF};
//    ISState fullyClosedLimitSwitch {ISS_OFF};
//    ISState roofLockedSwitch {ISS_OFF};
//    ISState roofAuxiliarySwitch {ISS_OFF};
//    INumber RoofTimeoutN[1] {};
//    INumberVectorProperty RoofTimeoutNP;
//    enum { EXPIRED_CLEAR, EXPIRED_OPEN, EXPIRED_CLOSE };
//    unsigned int roofTimedOut;
//    bool simRoofOpen = false;
//    bool simRoofClosed = true;
//    unsigned int communicationErrors = 0;

    ITextVectorProperty ThermostatStatusTP;
    IText ThermostatStatusT[10] {};
};

