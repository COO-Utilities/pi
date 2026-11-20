# PI E-727 fast steering mirror controller over an FT4222 USB-to-SPI bridge
#
# e727::protocol is transport independent. e727::spi and e727_spi_check work only with an FTDI
# FT4222H, through FTDI's LibFT4222, and are skipped when that SDK is not found.

# Protocol: standard library only, always available
add_library(e727_protocol INTERFACE)
add_library(e727::protocol ALIAS e727_protocol)
target_include_directories(e727_protocol INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(e727_protocol INTERFACE cxx_std_23)

if(BUILD_TESTING)
    add_executable(e727_protocol_test tests/E727ProtocolTest.cpp)
    target_link_libraries(e727_protocol_test PRIVATE e727::protocol pi_warnings)
    add_test(NAME e727_protocol COMMAND e727_protocol_test)
endif()

# SPI link and diagnostic tool: need the FTDI LibFT4222 SDK
option(PI_FETCH_FT4222 "Unpack the SHA256-pinned LibFT4222 archive at FT4222_SDK_URL instead of searching FT4222_ROOT" OFF)
set(FT4222_ROOT "/usr/local" CACHE PATH "Install prefix or unpacked directory of the LibFT4222 SDK")

if(PI_FETCH_FT4222)
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchLibFt4222.cmake")
    fetch_libft4222(FT4222_INCLUDE_DIR FT4222_LIBRARY)
else()
    find_path(FT4222_INCLUDE_DIR libft4222.h HINTS "${FT4222_ROOT}/include" "${FT4222_ROOT}")
    find_library(FT4222_LIBRARY ft4222 HINTS "${FT4222_ROOT}/lib" "${FT4222_ROOT}")
endif()

if(NOT FT4222_INCLUDE_DIR OR NOT FT4222_LIBRARY)
    message(STATUS "LibFT4222 not found under FT4222_ROOT=${FT4222_ROOT}; building the E-727 protocol only")
    return()
endif()

find_package(Threads REQUIRED)

add_library(e727_spi INTERFACE)
add_library(e727::spi ALIAS e727_spi)
# SYSTEM keeps vendor header warnings out of our -Werror builds
target_include_directories(e727_spi SYSTEM INTERFACE "${FT4222_INCLUDE_DIR}")
target_link_libraries(e727_spi INTERFACE e727::protocol "${FT4222_LIBRARY}" Threads::Threads)

add_executable(e727_spi_check apps/e727_spi_check.cpp)
target_link_libraries(e727_spi_check PRIVATE e727::spi pi_warnings)
