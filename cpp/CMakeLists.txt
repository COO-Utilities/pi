cmake_minimum_required(VERSION 3.25)

project(pi_cpp
    VERSION 0.1.0
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
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# e727/ turns this on when it installs e727::spi
set(PI_CPP_HAS_SPI OFF)
add_subdirectory(e727)

# Installed as a CMake package, so consumers use find_package(pi_cpp) and link e727::protocol
# or e727::spi
set(PI_CPP_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/pi_cpp")
install(EXPORT pi_cppTargets NAMESPACE e727:: DESTINATION "${PI_CPP_CMAKE_DIR}")
configure_package_config_file(cmake/pi_cppConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/pi_cppConfig.cmake"
    INSTALL_DESTINATION "${PI_CPP_CMAKE_DIR}")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/pi_cppConfigVersion.cmake"
    COMPATIBILITY SameMinorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/pi_cppConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/pi_cppConfigVersion.cmake"
    DESTINATION "${PI_CPP_CMAKE_DIR}")
