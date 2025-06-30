cmake_minimum_required(VERSION 3.10)
project(PitchPlease VERSION 1.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTS "Build test programs" OFF)
option(USE_FIXED_POINT "Use fixed-point math instead of floating-point" OFF)
option(BUILD_DYNAMICS "Build dynamics simulation module" ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuators
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics
)

# Source files
set(CASF_SOURCES
    src/control/control.c
    src/sensors/sensors.c
)

# Add dynamics module if enabled
if(BUILD_DYNAMICS)
    set(CASF_SOURCES ${CASF_SOURCES} src/dynamics/dynamics.c)
    add_definitions(-DBUILD_DYNAMICS)
    message(STATUS "  Dynamics module: Enabled")
else()
    message(STATUS "  Dynamics module: Disabled")
endif()

# Determine if we should use fixed-point math
if(USE_FIXED_POINT)
    add_definitions(-DUSE_FIXED_POINT=1)
else()
    add_definitions(-DUSE_FIXED_POINT=0)
endif()

# Create library
add_library(casf ${CASF_SOURCES})
target_link_libraries(casf m)

# Setup tests if enabled
if(BUILD_TESTS)
    # Add test executable
    add_executable(casf_tests tests/test_main.c)
    target_link_libraries(casf_tests casf)

    # Optionally add a testing framework like CTest
    include(CTest)
    add_test(NAME casf_basic_tests COMMAND casf_tests)

    # Add additional test executables if they exist
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sensors.c)
        add_executable(test_sensors tests/test_sensors.c)
        target_link_libraries(test_sensors casf)
        add_test(NAME SensorTests COMMAND test_sensors)
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_control.c)
        add_executable(test_control tests/test_control.c)
        target_link_libraries(test_control casf)
        add_test(NAME ControlTests COMMAND test_control)
    endif()

    message(STATUS "  Tests: Enabled")
else()
    message(STATUS "  Tests: Disabled")
endif()

# Basic example application
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/simple_adcs.c)
    add_executable(casf_example examples/simple_adcs.c)
    target_link_libraries(casf_example casf)
    message(STATUS "  Basic example: Enabled")
else()
    message(STATUS "  Basic example: Not found")
endif()

# Detumbling example (requires dynamics module)
if(BUILD_DYNAMICS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/detumbling.c)
    add_executable(casf_detumbling examples/detumbling.c)
    target_link_libraries(casf_detumbling casf)
    message(STATUS "  Detumbling example: Enabled")
else()
    message(STATUS "  Detumbling example: Disabled or not found")
endif()

# Main executable (if main.c exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
    add_executable(pitchplease 
        src/main.c
    )
    target_link_libraries(pitchplease casf)
    message(STATUS "  Main application: Enabled")
else()
    message(STATUS "  Main application: Not found")
endif()

# Installation rules
install(TARGETS casf
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
set(CASF_HEADERS
    src/control/control.h
    src/sensors/sensors.h
    src/sensors/sensor_types.h
)

if(BUILD_DYNAMICS)
    set(CASF_HEADERS ${CASF_HEADERS} src/dynamics/dynamics.h)
endif()

install(FILES ${CASF_HEADERS} DESTINATION include/casf)

# Print configuration summary
message(STATUS "Configuration summary:")
message(STATUS "  Build tests: ${BUILD_TESTS}")
message(STATUS "  Use fixed-point math: ${USE_FIXED_POINT}")
message(STATUS "  Build dynamics module: ${BUILD_DYNAMICS}")
