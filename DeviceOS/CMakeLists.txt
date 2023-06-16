# SPDX-License-Identifier: Apache-2.0

# This is a general purpose project created from MotionSenseHRV to be used for data collection.

# Ohio State SENSE LAB

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(external_lib)

set(NO_THREADSAFE_STATICS $<TARGET_PROPERTY:compiler-cpp,no_threadsafe_statics>)


FILE(GLOB app_sources src/*.c)
# this is a defined k-config
# add_subdirectory_ifdef(CONFIG_CMSIS_DSP        src/DSP)
target_sources(app PRIVATE ${app_sources})


if((NOT CONFIG_DISK_DRIVER_FLASH) AND
   (NOT CONFIG_DISK_DRIVER_RAM) AND
   (NOT CONFIG_DISK_DRIVER_SDMMC))
	message( FATAL_ERROR "No disk access settings detected." )
endif()








