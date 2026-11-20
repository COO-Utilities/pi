cmake_minimum_required(VERSION 3.25)

project(pi_cpp
    DESCRIPTION "C++ interfaces to PI controllers"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Timing results are only meaningful from an optimised build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PI_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

add_library(pi_warnings INTERFACE)
target_compile_options(pi_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
    $<$<BOOL:${PI_WARNINGS_AS_ERRORS}>:-Werror>)

include(CTest)

add_subdirectory(e727)
