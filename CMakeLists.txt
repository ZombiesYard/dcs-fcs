cmake_minimum_required(VERSION 3.20)

project(ah64d_auto_rudder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(autorudder_core
    src/config.cpp
    src/dcs_bios_protocol.cpp
    src/dcs_bios_refs.cpp
    src/dcs_bios_state.cpp
    src/logger.cpp
    src/yaw_damper.cpp
)

target_include_directories(autorudder_core PUBLIC src)
target_compile_definitions(autorudder_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

if(MSVC)
    target_compile_options(autorudder_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(autorudder_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    add_executable(ah64d_auto_rudder
        src/app.cpp
        src/main.cpp
        src/windows/dcs_bios_udp_client.cpp
        src/windows/directinput_axis.cpp
        src/windows/fast_export_udp_client.cpp
        src/windows/hotkey.cpp
        src/windows/vjoy_device.cpp
    )

    target_include_directories(ah64d_auto_rudder PRIVATE src)
    target_compile_definitions(ah64d_auto_rudder PRIVATE DIRECTINPUT_VERSION=0x0800)
    target_link_libraries(ah64d_auto_rudder PRIVATE autorudder_core ws2_32 dinput8 dxguid)

    if(MSVC)
        target_compile_options(ah64d_auto_rudder PRIVATE /W4 /permissive-)
    endif()
endif()

add_executable(autorudder_tests
    tests/test_main.cpp
)

target_link_libraries(autorudder_tests PRIVATE autorudder_core)

enable_testing()
add_test(NAME autorudder_tests COMMAND autorudder_tests)
