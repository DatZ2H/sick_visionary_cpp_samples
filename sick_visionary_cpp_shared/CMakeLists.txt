#
# Copyright note: Redistribution and use in source, with or without modification, are permitted.
# 
# Updated: January 2020
# 
# SICK AG, Waldkirch
# email: TechSupport0905@sick.de

cmake_minimum_required(VERSION 3.8)

project(sick_visionary_cpp_shared
        VERSION 0.1.0
        LANGUAGES CXX)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

### BUILD ###
aux_source_directory(src SRC_LIST)

add_library(${PROJECT_NAME} STATIC ${SRC_LIST})

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS OFF)

target_include_directories(${PROJECT_NAME} PRIVATE include PUBLIC src)

if(WIN32)
  target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()
