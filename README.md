# ISP_Firmware
Versal2 ISP firmware

1. Build the Sensor Library
Open vitis and build Sensor Library

2. Link the Pre-built Libraries to the Main Application
i. Ensure that the pre-built libraries are available and correctly placed. These libraries are typically built in another step or repository and should be accessible for linking.

ii. In the main application’s CMakeLists.txt, link the sensor library and any other pre-built libraries

ex: 
add_library(isp_fw_lib STATIC IMPORTED)
set_target_properties(isp_fw_lib PROPER.TIES IMPORTED_LOCATION <path/libisp_fw_lib.a>)

add_library(sensor_lib STATIC IMPORTED)
set_target_properties(sensor_lib PROPERTIES IMPORTED_LOCATION <path/libsensor_lib.a>)

target_link_libraries(isp_fw_main.elf
  PRIVATE
    isp_fw_lib
    sensor_lib


3. Build main application
