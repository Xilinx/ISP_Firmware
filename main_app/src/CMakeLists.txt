# Copyright (C) 2023 - 2024 Advanced Micro Devices, Inc.  All rights reserved.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.14.7)

message(STATUS "*** Building RPU FW - Select 6=>rpu6, 7=>rpu7, 8=>rpu8, 9=>rpu9 0=>all_rpu ***")
set(GEN_RPU_ELF "0")

if ("${GEN_RPU_ELF}" STREQUAL "6")
message(STATUS "*** Selected RPU FW build for Core 6 *** ")
set(RPU_START_ADDR 0xC000000)
set(RPU_SIZE 0x3800000)
set(RPU_FW_DEFINITIONS
"RPU6_FW=1"
)
endif()

if ("${GEN_RPU_ELF}" STREQUAL "7")
message(STATUS "*** Selected RPU FW build for Core 7 *** ")
set(RPU_START_ADDR 0x1082A000)
set(RPU_SIZE 0x3800000)
set(RPU_FW_DEFINITIONS
"RPU7_FW=1"
)
endif()

if ("${GEN_RPU_ELF}" STREQUAL "8")
message(STATUS "*** Selected RPU FW build for Core 8 *** ")
set(RPU_START_ADDR 0x15054000)
set(RPU_SIZE 0x3800000)
set(RPU_FW_DEFINITIONS
"RPU8_FW=1"
)
endif()

if ("${GEN_RPU_ELF}" STREQUAL "9")
message(STATUS "*** Selected RPU FW build for Core 9 *** ")
set(RPU_START_ADDR 0x1987E000)
set(RPU_SIZE 0x3800000)
set(RPU_FW_DEFINITIONS
"RPU9_FW=1"
)
endif()

if ("${GEN_RPU_ELF}" STREQUAL "0")
message(STATUS "*** Selected to build ALL RPU FW cores (6, 7, 8, 9) ***")
set(BUILD_ALL_CORES TRUE)
set(RPU6_START_ADDR 0xC000000)
set(RPU7_START_ADDR 0x1082A000)
set(RPU8_START_ADDR 0x15054000)
set(RPU9_START_ADDR 0x1987E000)
set(RPU_SIZE 0x3800000)
endif()

project(isp_fw_elf)

include_directories(${CMAKE_BINARY_DIR}/include)
include(${CMAKE_CURRENT_SOURCE_DIR}/UserConfig.cmake)
find_package(common)
enable_language(C ASM)
collect(PROJECT_LIB_DEPS xilstandalone;xiltimer;xilffs;freertos)
collect(PROJECT_LIB_DEPS xil)
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

function(build_rpu_fw core_num app_name start_addr size definitions)
message("Build sources are ${CMAKE_CURRENT_SOURCE_DIR}")
#message("Build library paths are ${USER_LINK_DIRECTORIES}")
add_executable(${app_name}.elf ${CMAKE_SOURCE_DIR}/main.c ${CMAKE_SOURCE_DIR}/isp_fw_config.c)
message(STATUS "*** Building RPU FW for Core ${core_num} ***")
set_target_properties(${app_name}.elf PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/lscript.ld)
target_link_libraries(${app_name}.elf
  PRIVATE
    -Wl,-Map=${app_name}.map
    -Wl,--build-id=none
    -Wl,--gc-sections
    -Wl,--defsym=_FW_START_ADDR=${start_addr}
    -Wl,--defsym=_FW_START_SIZE=${size}
    -T${CMAKE_SOURCE_DIR}/lscript.ld
    -Wl,--start-group
    sensor_lib
    isp_fw_lib
    m
    ${_deps}
    -Wl,--end-group
)
target_compile_definitions(${app_name}.elf PUBLIC ${definitions} ${USER_COMPILE_DEFINITIONS})
target_include_directories(${app_name}.elf PUBLIC ${USER_INCLUDE_DIRECTORIES})
endfunction()

#add_library(isp_fw_lib STATIC IMPORTED)
# Define a root directory variable once
set(LIB_ROOT ../../prebuilt_lib/)

#stripped binary
add_library(sensor_lib STATIC IMPORTED)
set_target_properties(sensor_lib PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libsensor_lib.a)

add_library(isp_fw_lib STATIC IMPORTED)
set_target_properties(isp_fw_lib PROPERTIES  IMPORTED_LOCATION ${LIB_ROOT}/libisp_fw_lib.a)

add_library(freertos STATIC IMPORTED)
set_target_properties(freertos PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libfreertos.a)

add_library(xilffs STATIC IMPORTED)
set_target_properties(xilffs PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libxilffs.a)

add_library(xilstandalone STATIC IMPORTED)
set_target_properties(xilstandalone PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libxilstandalone.a)

add_library(xil STATIC IMPORTED)
set_target_properties(xil PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libxil.a)

add_library(xiltimer STATIC IMPORTED)
set_target_properties(xiltimer PROPERTIES IMPORTED_LOCATION ${LIB_ROOT}/libxiltimer.a)

if (BUILD_ALL_CORES)
    # Build all cores when option 0 is selected
    build_rpu_fw(6 "isp-r52-6-firmware" ${RPU6_START_ADDR} ${RPU_SIZE} "RPU6_FW=1")
    build_rpu_fw(7 "isp-r52-7-firmware" ${RPU7_START_ADDR} ${RPU_SIZE} "RPU7_FW=1")
    build_rpu_fw(8 "isp-r52-8-firmware" ${RPU8_START_ADDR} ${RPU_SIZE} "RPU8_FW=1")
    build_rpu_fw(9 "isp-r52-9-firmware" ${RPU9_START_ADDR} ${RPU_SIZE} "RPU9_FW=1")

    # Add a custom target to show ELF sizes
    add_custom_target(all_rpu_build_complete ALL
        COMMAND ${CMAKE_COMMAND} -E echo "ELF Size Information:"
        COMMAND ${CMAKE_SIZE} isp-r52-6-firmware.elf || ${CMAKE_COMMAND} -E echo "Could not get size for isp-r52-6-firmware.elf"
        COMMAND ${CMAKE_SIZE} isp-r52-7-firmware.elf || ${CMAKE_COMMAND} -E echo "Could not get size for isp-r52-7-firmware.elf"
        COMMAND ${CMAKE_SIZE} isp-r52-8-firmware.elf || ${CMAKE_COMMAND} -E echo "Could not get size for isp-r52-8-firmware.elf"
        COMMAND ${CMAKE_SIZE} isp-r52-9-firmware.elf || ${CMAKE_COMMAND} -E echo "Could not get size for isp-r52-9-firmware.elf"
        DEPENDS isp-r52-6-firmware.elf isp-r52-7-firmware.elf isp-r52-8-firmware.elf isp-r52-9-firmware.elf
        COMMENT "All RPU firmwares built successfully"
    )
else()
    # Build single core based on selection
    if ("${GEN_RPU_ELF}" STREQUAL "6")
        build_rpu_fw(6 "isp-r52-6-firmware" ${RPU_START_ADDR} ${RPU_SIZE} "RPU6_FW=1")
        print_elf_size(CMAKE_SIZE "isp-r52-6-firmware")
    endif()

    if ("${GEN_RPU_ELF}" STREQUAL "7")
        build_rpu_fw(7 "isp-r52-7-firmware" ${RPU_START_ADDR} ${RPU_SIZE} "RPU7_FW=1")
        print_elf_size(CMAKE_SIZE "isp-r52-7-firmware")
    endif()

    if ("${GEN_RPU_ELF}" STREQUAL "8")
        build_rpu_fw(8 "isp-r52-8-firmware" ${RPU_START_ADDR} ${RPU_SIZE} "RPU8_FW=1")
        print_elf_size(CMAKE_SIZE "isp-r52-8-firmware")
    endif()

    if ("${GEN_RPU_ELF}" STREQUAL "9")
        build_rpu_fw(9 "isp-r52-9-firmware" ${RPU_START_ADDR} ${RPU_SIZE} "RPU9_FW=1")
        print_elf_size(CMAKE_SIZE "isp-r52-9-firmware")
    endif()
endif()

