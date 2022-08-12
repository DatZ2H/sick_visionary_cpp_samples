#
# Copyright note: Redistribution and use in source, with or without modification, are permitted.
# 
# Created: November 2018
# 
# SICK AG, Waldkirch
# email: TechSupport0905@sick.de

cmake_minimum_required(VERSION 3.8)

project(SampleVisionary
        VERSION 0.1.0
        LANGUAGES CXX)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(WIN32)
  # We support only LONGHORN and above (WIN7...)
  add_compile_definitions("_WIN32_WINNT=_WIN32_WINNT_LONGHORN")
endif()

### BUILD ###
add_subdirectory(sick_visionary_cpp_shared)

## Visionary-S sample ##
add_executable(SampleVisionaryS SampleVisionaryS/SampleVisionaryS.cpp)
target_link_libraries(SampleVisionaryS sick_visionary_cpp_shared)

## Visionary-T sample ##
add_executable(SampleVisionaryT SampleVisionaryT/SampleVisionaryT.cpp)
target_link_libraries(SampleVisionaryT sick_visionary_cpp_shared)

## Visionary-T VGA samples ##
add_executable(SampleVisionaryTVGA SampleVisionaryTVGA/SampleVisionaryTVGA.cpp)
target_link_libraries(SampleVisionaryTVGA sick_visionary_cpp_shared)

## Visionary-T Mini samples ##
add_executable(SampleVisionaryTMini SampleVisionaryTMini/SampleVisionaryTMini.cpp)
target_link_libraries(SampleVisionaryTMini sick_visionary_cpp_shared)
