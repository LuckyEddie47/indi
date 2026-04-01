
include_directories(${INDI_INCLUDE_DIR})
include_directories("../../drivers/telescope/onstepx/")

add_executable(test_onstepxcomm
    test_onstepxcomm.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/onstepx/OnStepXComm.cpp"
)

target_link_libraries(test_onstepxcomm
    indidriver
    ${GTEST_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_test(test-onstepxcomm test_onstepxcomm)

add_executable(test_onstepxprobe
    test_onstepxprobe.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/onstepx/OnStepXCore.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/onstepx/OnStepXComm.cpp"
)

target_link_libraries(test_onstepxprobe
    indidriver
    ${GTEST_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_test(test-onstepxprobe test_onstepxprobe)

add_executable(test_onstepxstatus
    test_onstepxstatus.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/onstepx/OnStepXStatus.cpp"
)

target_link_libraries(test_onstepxstatus
    indidriver
    ${GTEST_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_test(test-onstepxstatus test_onstepxstatus)
