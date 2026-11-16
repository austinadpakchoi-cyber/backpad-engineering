cmake_minimum_required(VERSION 3.20.0)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(backpad_test LANGUAGES C)
# Vendor-generated settings are deliberately not redistributed.
if(NOT EXISTS "${BACKPAD_SENSOR_SETTINGS}")
  message(FATAL_ERROR "Set BACKPAD_SENSOR_SETTINGS to your authorized IQS9151 settings C file")
endif()
if(NOT EXISTS "${BACKPAD_GENERATED_INCLUDE}/build_id.h")
  message(FATAL_ERROR "Use tools/build_firmware.ps1 to generate the public-source build ID")
endif()
set(BP_SENSOR ${CMAKE_CURRENT_SOURCE_DIR}/../sensor9151)
target_sources(app PRIVATE src/main.c src/bp_status.c src/bp_sensor_service.c src/bp_touch_policy.c vendor/v5/src/bp_pointer.c vendor/v5/src/bp_hid.c vendor/v5/src/bp_channel.c
 ${BP_SENSOR}/src/bp_iqs9151.c ${BACKPAD_SENSOR_SETTINGS} ${BP_SENSOR}/src/bp_iqs9151_zephyr.c)
target_include_directories(app PRIVATE ${BACKPAD_GENERATED_INCLUDE} include vendor/v5/include ${BP_SENSOR}/include)
# BackPad V2 touchpad: 13 Rx columns x 12 Tx rows at 4.5 mm -> 54.0 x 49.5 mm between end electrode centres
# IQS9151 V2 pad. Secondary 20 %: bp_touch_policy only forwards contacts with area >= 4 and density >= 850,
# so the IQS550 50 % ghost floor would only drop real unequal-pressure fingers (18385/15 + 8000/8 = 44 %).
target_compile_definitions(app PRIVATE BP_PHYSICAL_X_UM=54000 BP_PHYSICAL_Y_UM=49500 BP_SECONDARY_RELATIVE_PERCENT=20)

