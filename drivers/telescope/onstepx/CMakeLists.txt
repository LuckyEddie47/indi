
# ########## OnStep X (mount) ##########
add_executable(indi_onstepx
    main_mount.cpp
    OnStepXAlignment.cpp
    OnStepXAuxFeatures.cpp
    OnStepXCore.cpp
    OnStepXComm.cpp
    OnStepXFocuser.cpp
    OnStepXGuide.cpp
    OnStepXInfo.cpp
    OnStepXLimits.cpp
    OnStepXMount.cpp
    OnStepXPec.cpp
    OnStepXRotator.cpp
    OnStepXSite.cpp
    OnStepXStatus.cpp
    OnStepXTracking.cpp
    OnStepXWeather.cpp
    OnStepXUsbPlugin.cpp)

target_link_libraries(indi_onstepx AlignmentDriver indidriver)

install(TARGETS indi_onstepx RUNTIME DESTINATION bin)

# ########## OnStep X Controller (auxiliary, no mount) ##########
add_executable(indi_onstepx_aux
    main_aux.cpp
    OnStepXAuxFeatures.cpp
    OnStepXCore.cpp
    OnStepXComm.cpp
    OnStepXFocuser.cpp
    OnStepXAux.cpp
    OnStepXRotator.cpp
    OnStepXWeather.cpp
    OnStepXUsbPlugin.cpp)

target_link_libraries(indi_onstepx_aux indidriver)

install(TARGETS indi_onstepx_aux RUNTIME DESTINATION bin)
