# Copyright (C) 2023 - 2024 Advanced Micro Devices, Inc.  All rights reserved.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.3)
include(${CMAKE_CURRENT_SOURCE_DIR}/UserConfig.cmake)
set(APP_NAME
sensor_lib
)
project(${APP_NAME})

find_package(common)
collector_create (PROJECT_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}")
collector_create (PROJECT_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../build/include)
include_directories(${CMAKE_INCLUDE_DIR})
string(APPEND CMAKE_C_FLAGS ${USER_COMPILE_OPTIONS})
string(APPEND CMAKE_CXX_FLAGS ${USER_COMPILE_OPTIONS})
string(REPLACE "-O2" "-O0" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
add_library(${APP_NAME} STATIC ${USER_COMPILE_SOURCES})
set_target_properties(${APP_NAME} PROPERTIES LINKER_LANGUAGE C)
install(FILES ${USER_HEADER_SOURCES} DESTINATION ${CMAKE_INCLUDE_DIR})
target_compile_definitions(${APP_NAME} PUBLIC ${USER_COMPILE_DEFINITIONS})
target_include_directories(${APP_NAME} PUBLIC ${USER_INCLUDE_DIRECTORIES})
