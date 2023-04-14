# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(record_realsense)

# Find librealsense2 installed package
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Enable C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

add_executable(short_record short_record.cpp)
include_directories(short_record ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(short_record ${REALSENSE2_FOUND} ${OpenCV_LIBS})

add_executable(long_record long_record.cpp)
include_directories(long_record ${ADDITIONAL_INCLUDE_DIRS})
target_link_libraries(long_record ${REALSENSE2_FOUND} ${OpenCV_LIBS})
