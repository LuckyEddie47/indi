# OnStep X INDI Driver

INDI driver for the [OnStepX](https://github.com/hjd1964/OnStepX) telescope controller firmware.

## Overview

OnStepX is a general-purpose telescope controller. A mount is one optional feature — the
controller may also run without a mount (focuser-only, rotator-only, or accessory-only
installations). This driver provides two binaries to match:

| Binary | Group | Use for |
|---|---|---|
| `indi_onstepx` | Telescopes | OnStepX with a mount (GEM, Fork, AltAz, AltAlt) |
| `indi_onstepx_aux` | Auxiliary | OnStepX without a mount |

Both binaries share all source files except a single entry-point.

## Hardware Requirements

- OnStepX firmware v10.24c or later (earlier versions may work but are untested)
- Serial (USB) or TCP/IP connection to the controller
- At least one of: mount, focuser, rotator, or aux output configured in the firmware

## Features

### Both Binaries
- Focusers (up to 6 slots, each appears as a separate INDI Focuser device)
- Rotator with optional de-rotator and parallactic tracking (AltAz only)
- Auxiliary outputs: digital switches, PWM analog, dew heaters, intervalometers
- Weather sensors (temperature, pressure, humidity, dew point, MCU temperature)

### Mount Binary Only
- Equatorial and AltAz mounts (GEM, Fork, AltAz, AltAlt)
- Goto, sync, park, abort, home find/set
- Pulse guiding (GuiderInterface)
- Tracking: sidereal, lunar, solar, King; refraction/full compensation; rate nudge
- Meridian flip: auto-flip, preferred pier side
- Horizon and meridian limits
- PEC playback and recording
- N-star geometric alignment
- Polar error display

## Building

### Dependencies

```
libindi-dev >= 2.0
libnova-dev
cmake >= 3.13
googletest (for unit tests; source at /usr/src/googletest on Debian/Ubuntu)
```

Install on Debian/Ubuntu:

```bash
sudo apt install libindi-dev libnova-dev cmake
```

### Build

```bash
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Debug \
    -DINDI_BUILD_UNITTESTS=ON \
    -DGMOCK_LIBRARY=/tmp/gmock_build/lib/libgmock.a \
    -DGMOCK_INCLUDE_DIR=/usr/src/googletest/googlemock/include
cmake --build . --target indi_onstepx --target indi_onstepx_aux
```

If GMock is installed as a system library (not the case on most distributions), omit the
`GMOCK_LIBRARY` and `GMOCK_INCLUDE_DIR` flags. To build GMock from source first:

```bash
cmake -B /tmp/gmock_build /usr/src/googletest
cmake --build /tmp/gmock_build
```

### Run Unit Tests

```bash
ctest -R test-onstepx --output-on-failure
```

## Architecture

```
indi_onstepx (mount)          indi_onstepx_aux (auxiliary)
     |                               |
OnStepXMount                   OnStepXAux
     |                               |
     +---- OnStepXCore (shared, owns OnStepXComm + Capabilities)
     |
     +---- OnStepXAlignment   (N-star alignment)
     +---- OnStepXAuxFeatures  (aux output slots 1-8)
     +---- OnStepXFocuser x6   (child INDI devices)
     +---- OnStepXLimits       (horizon/meridian/home)
     +---- OnStepXPec          (PEC playback/recording)
     +---- OnStepXRotator      (field rotator + de-rotator)
     +---- OnStepXSite         (location/time sync)
     +---- OnStepXStatus       (mount status parser)
     +---- OnStepXTracking     (advanced tracking control)
     +---- OnStepXWeather      (sensor polling)
```

All protocol commands are implemented in the individual helper classes. All I/O goes
through `OnStepXComm`, which serialises access with a mutex and handles LX200 framing.

## Connection

Both drivers support Serial and TCP connections via the standard INDI connection plugins.
The driver verifies the firmware identity (`:GVP#` must return `"OnStepX"`) and probes
capabilities at connect time. No manual configuration of feature flags is required.

## Notes

- The mount binary expects a mount to be configured in OnStepX firmware. If no mount is
  detected, connection is refused with an error message directing the user to
  `indi_onstepx_aux`.
- `INDI::OutputInterface` is not used for aux outputs because it requires the output count
  at `initProperties()` time, before the firmware has been queried. Outputs appear as
  custom properties instead, which function identically in all INDI clients.
- The `:Gu#` binary status path has a provisional bit layout derived from OnStepX source
  analysis. The driver falls back to the ASCII `:GU#` path automatically if parsing fails.
  Hardware validation of the binary layout is pending.
