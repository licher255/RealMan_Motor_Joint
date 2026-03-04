cmake_minimum_required(VERSION 3.16)
project(realman_whj_driver VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(BUILD_TESTS "Build test programs" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_ROS2_NODE "Build ROS2 node" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Platform detection
if(WIN32)
    message(STATUS "Building for Windows")
    add_definitions(-DWIN32_LEAN_AND_MEAN -D_CRT_SECURE_NO_WARNINGS)
    set(PLATFORM_LIBS ws2_32)
elseif(UNIX AND NOT APPLE)
    message(STATUS "Building for Linux")
    set(PLATFORM_LIBS pthread)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

# ============================================================================
# Core Library
# ============================================================================

set(CORE_HEADERS
    include/realman_whj/core/types.hpp
    include/realman_whj/core/command.hpp
    include/realman_whj/core/protocol.hpp
)

set(CORE_SOURCES
    src/core/types.cpp
    src/core/driver.cpp
)

set(PLATFORM_HEADERS
    include/realman_whj/platform/can_interface.hpp
)

set(PLATFORM_SOURCES
    src/platform/can_interface.cpp
)

if(WIN32)
    list(APPEND PLATFORM_HEADERS include/realman_whj/platform/windows_can.hpp)
    list(APPEND PLATFORM_SOURCES src/platform/windows_can.cpp)
else()
    list(APPEND PLATFORM_HEADERS include/realman_whj/platform/linux_can.hpp)
    list(APPEND PLATFORM_SOURCES src/platform/linux_can.cpp)
endif()

# Create library
add_library(realman_whj_driver 
    ${CORE_SOURCES}
    ${PLATFORM_SOURCES}
)

add_library(realman_whj::driver ALIAS realman_whj_driver)

target_include_directories(realman_whj_driver
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(realman_whj_driver
    PUBLIC
        ${PLATFORM_LIBS}
)

if(UNIX)
    target_compile_options(realman_whj_driver PRIVATE -Wall -Wextra)
endif()

# ============================================================================
# Tests
# ============================================================================

if(BUILD_TESTS)
    enable_testing()
    
    # Unit tests (no hardware required)
    add_executable(unit_tests
        tests/unit/test_protocol.cpp
        tests/unit/test_types.cpp
    )
    target_link_libraries(unit_tests PRIVATE realman_whj_driver)
    
    # Hardware integration tests
    add_executable(hardware_tests
        tests/hardware/test_basic.cpp
    )
    target_link_libraries(hardware_tests PRIVATE realman_whj_driver)
    
    # Simple test runner
    add_test(NAME unit_tests COMMAND unit_tests)
endif()

# ============================================================================
# Examples
# ============================================================================

if(BUILD_EXAMPLES)
    # C++ examples
    add_executable(example_basic examples/cpp/basic_example.cpp)
    target_link_libraries(example_basic PRIVATE realman_whj_driver)
    
    add_executable(example_position_control examples/cpp/position_control.cpp)
    target_link_libraries(example_position_control PRIVATE realman_whj_driver)
    
    add_executable(example_scan examples/cpp/scan_example.cpp)
    target_link_libraries(example_scan PRIVATE realman_whj_driver)
    
    add_executable(example_basic_read examples/cpp/basic_read_example.cpp)
    target_link_libraries(example_basic_read PRIVATE realman_whj_driver)
endif()

# ============================================================================
# Python Bindings
# ============================================================================

if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
    
    pybind11_add_module(realman_whj_py
        src/python/bindings.cpp
    )
    
    target_link_libraries(realman_whj_py PRIVATE realman_whj_driver)
    
    set_target_properties(realman_whj_py PROPERTIES
        OUTPUT_NAME "realman_whj"
    )
endif()

# ============================================================================
# ROS2 Node
# ============================================================================

if(BUILD_ROS2_NODE)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    
    add_executable(whj_driver_node
        src/ros2/whj_driver_node.cpp
    )
    
    ament_target_dependencies(whj_driver_node
        rclcpp
        std_msgs
        sensor_msgs
    )
    
    target_link_libraries(whj_driver_node PRIVATE realman_whj_driver)
    
    install(TARGETS whj_driver_node
        DESTINATION lib/${PROJECT_NAME}
    )
    
    install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
    )
    
    ament_package()
endif()

# ============================================================================
# Installation
# ============================================================================

include(GNUInstallDirs)

install(TARGETS realman_whj_driver
    EXPORT realman_whj_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT realman_whj_targets
    FILE realman_whj_targets.cmake
    NAMESPACE realman_whj::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/realman_whj
)

# Package configuration
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/realman_whjConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/realman_whjConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/realman_whj
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/realman_whjConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/realman_whjConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/realman_whjConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/realman_whj
)

# ============================================================================
# Summary
# ============================================================================

message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Version:           ${PROJECT_VERSION}")
message(STATUS "  Platform:          ${CMAKE_SYSTEM_NAME}")
message(STATUS "  Build tests:       ${BUILD_TESTS}")
message(STATUS "  Build examples:    ${BUILD_EXAMPLES}")
message(STATUS "  Python bindings:   ${BUILD_PYTHON_BINDINGS}")
message(STATUS "  ROS2 node:         ${BUILD_ROS2_NODE}")
message(STATUS "")
