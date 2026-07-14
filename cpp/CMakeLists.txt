cmake_minimum_required(VERSION 3.16)
project(efort_tie_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EFORT_WITH_SDK "Build the real Efort SDK backend" OFF)
set(EFORT_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../埃弗顿机器人/SDK V2.8/V2.8.0"
    CACHE PATH "Path to the Efort SDK V2.8.0 root")

add_library(robot_control
    src/motion_planning.cpp
    src/robot_control.cpp
)
target_include_directories(robot_control PUBLIC include)

if(EFORT_WITH_SDK)
    if(NOT WIN32)
        message(FATAL_ERROR "The configured Efort SDK library is the Windows x64 build")
    endif()
    if(NOT EXISTS "${EFORT_SDK_ROOT}/include/EfortSdk.h")
        message(FATAL_ERROR "EFORT_SDK_ROOT does not contain include/EfortSdk.h")
    endif()
    target_compile_definitions(robot_control PRIVATE EFORT_WITH_SDK)
    target_include_directories(robot_control PRIVATE "${EFORT_SDK_ROOT}/include")
    target_link_libraries(robot_control PRIVATE "${EFORT_SDK_ROOT}/libs/x64/EftSdk.lib")
    file(GLOB EFORT_RUNTIME_DLLS "${EFORT_SDK_ROOT}/libs/x64/*.dll")
    foreach(EFORT_RUNTIME_DLL IN LISTS EFORT_RUNTIME_DLLS)
        add_custom_command(TARGET robot_control POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${EFORT_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:robot_control>"
        )
    endforeach()
endif()

add_executable(run_tie_queue src/main.cpp)
target_link_libraries(run_tie_queue PRIVATE robot_control)

enable_testing()
add_executable(robot_control_tests tests/test_robot_control.cpp)
target_link_libraries(robot_control_tests PRIVATE robot_control)
target_compile_definitions(robot_control_tests PRIVATE
    TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../samples/tie_points.json"
    TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../samples/real_config.json"
    TEST_PGM_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../TIE_QUEUE_SIM.pgm"
    TEST_XPL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../TIE_QUEUE_SIM.XPL"
)
add_test(NAME robot_control_tests COMMAND robot_control_tests)

add_executable(motion_planning_tests tests/test_motion_planning.cpp)
target_link_libraries(motion_planning_tests PRIVATE robot_control)
add_test(NAME motion_planning_tests COMMAND motion_planning_tests)
