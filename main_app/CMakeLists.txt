# Copyright (C) 2023 - 2024 Advanced Micro Devices, Inc.  All rights reserved.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.14.7)


set(APP_NAME versal2_main)
project(${APP_NAME})

include_directories(${CMAKE_BINARY_DIR}/include)
include(${CMAKE_CURRENT_SOURCE_DIR}/UserConfig.cmake)
find_package(common)
enable_language(C ASM)
collect(PROJECT_LIB_DEPS xilstandalone;xiltimer;xilffs;freertos)
collect(PROJECT_LIB_DEPS freertos)
collect(PROJECT_LIB_DEPS xil)
collect(PROJECT_LIB_DEPS xiltimer)
collect(PROJECT_LIB_DEPS gcc)
collect(PROJECT_LIB_DEPS c)
collector_list (_deps PROJECT_LIB_DEPS)
list (APPEND _deps ${USER_LINK_LIBRARIES})

collector_create (PROJECT_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}")
collector_create (PROJECT_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}")



#if("${CMAKE_MACHINE}" STREQUAL "Versal")
#  add_subdirectory(versal)
#elseif("${CMAKE_MACHINE}" STREQUAL "VersalNet")
#  add_subdirectory(versal_net)
#endif()


collector_list (_sources PROJECT_LIB_SOURCES)
collector_list (_headers PROJECT_LIB_HEADERS)
if (CMAKE_EXPORT_COMPILE_COMMANDS AND
    (NOT ${YOCTO}))
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
    set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

file(COPY ${_headers} DESTINATION ${CMAKE_BINARY_DIR}/include)

string(APPEND CMAKE_C_LINK_FLAGS ${USER_LINK_OPTIONS})
string(APPEND CMAKE_CXX_LINK_FLAGS ${USER_LINK_OPTIONS})
string(APPEND CMAKE_C_FLAGS ${USER_COMPILE_OPTIONS})
string(APPEND CMAKE_CXX_FLAGS ${USER_COMPILE_OPTIONS}) 
string(REPLACE "-O2" "-O0" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

message("Build sources are ${CMAKE_CURRENT_SOURCE_DIR}")
message("Build library paths are ${USER_LINK_DIRECTORIES}")
add_executable(${APP_NAME}.elf ${CMAKE_SOURCE_DIR}/main.c)
set_target_properties(${APP_NAME}.elf PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/lscript.ld )

#add_library(isp_fw_lib STATIC IMPORTED)
# Define a root directory variable once
set(LIB_ROOT ../../../m15_isp_fw_lib)

#stripped binary
add_library(isp_fw_lib STATIC IMPORTED)
set_target_properties(isp_fw_lib PROPERTIES  IMPORTED_LOCATION ${LIB_ROOT}/isp_fw_lib/build/libisp_fw_lib.a)

add_library(sensor_lib STATIC IMPORTED)
set_target_properties(sensor_lib PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/sensor_lib/build/libsensor_lib.a)

add_library(freertos STATIC IMPORTED)
set_target_properties(freertos PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/platform/cortexr52_6/freertos_cortexr52_6/bsp/lib/libfreertos.a)

add_library(xilffs STATIC IMPORTED)
set_target_properties(xilffs PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/platform/cortexr52_6/freertos_cortexr52_6/bsp/lib/libxilffs.a)

add_library(xilstandalone STATIC IMPORTED)
set_target_properties(xilstandalone PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/platform/cortexr52_6/freertos_cortexr52_6/bsp/lib/libxilstandalone.a)

add_library(xil STATIC IMPORTED)
set_target_properties(xil PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/platform/export/platform/sw/freertos_cortexr52_6/lib/libxil.a)

add_library(xiltimer STATIC IMPORTED)
set_target_properties(xiltimer PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/platform/export/platform/sw/freertos_cortexr52_6/lib/libxiltimer.a)



target_link_libraries(versal2_main.elf
  PRIVATE
    -Wl,-Map=versal2.map
    -Wl,--build-id=none
    -Wl,--gc-sections
    -T${CMAKE_SOURCE_DIR}/lscript.ld
    -L${CMAKE_LIBRARY_PATH}
    -Wl,--start-group
    isp_fw_lib
    sensor_lib
      xilstandalone
      freertos
      xilffs
      xil
      xiltimer
      m
      ${_deps}
    -Wl,--end-group
)


target_compile_definitions(${APP_NAME}.elf PUBLIC ${USER_COMPILE_DEFINITIONS})
target_include_directories(${APP_NAME}.elf PUBLIC ${USER_INCLUDE_DIRECTORIES})
print_elf_size(CMAKE_SIZE ${APP_NAME})
