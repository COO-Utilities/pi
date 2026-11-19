# Unpack a SHA256-pinned LibFT4222 SDK archive into the build tree
#
# FTDI's site answers scripted downloads with a browser challenge page, so FT4222_SDK_URL is
# normally the absolute path of the zip saved from a browser. The hash pin makes the build refuse
# any other file, which is what makes the result reproducible across hosts.

include(FetchContent)

set(FT4222_SDK_VERSION "1.4.4.232")
set(FT4222_SDK_SHA256 "9e7141d252b67c74b585e85652079e801c7e820e7bf271fc0fa061ef849ceff0")
set(FT4222_SDK_URL "" CACHE STRING
    "Absolute path or URL of libft4222-linux-${FT4222_SDK_VERSION}.zip, used when PI_FETCH_FT4222 is ON")

function(_ft4222_check_source)
    if(NOT FT4222_SDK_URL)
        message(FATAL_ERROR
            "PI_FETCH_FT4222 needs FT4222_SDK_URL. Download libft4222-linux-${FT4222_SDK_VERSION}.zip "
            "from https://ftdichip.com/products/ft4222h/ in a browser and pass its absolute path.")
    endif()
    if(FT4222_SDK_URL MATCHES "^[a-z]+://")
        return()
    endif()
    if(NOT IS_ABSOLUTE "${FT4222_SDK_URL}")
        message(FATAL_ERROR "FT4222_SDK_URL must be an absolute path, not '${FT4222_SDK_URL}'")
    endif()
    if(NOT EXISTS "${FT4222_SDK_URL}")
        message(FATAL_ERROR "FT4222_SDK_URL does not exist: ${FT4222_SDK_URL}")
    endif()
endfunction()

# The SDK ships one prebuilt library per architecture, named as install4222.sh expects
function(_ft4222_build_directory out_var)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(${out_var} "build-x86_64" PARENT_SCOPE)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        set(${out_var} "build-arm-v8" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "No prebuilt LibFT4222 is mapped for processor '${CMAKE_SYSTEM_PROCESSOR}'")
    endif()
endfunction()

# Set <include_var> and <library_var> to the unpacked SDK's header directory and shared library
function(fetch_libft4222 include_var library_var)
    _ft4222_check_source()
    _ft4222_build_directory(build_directory)

    FetchContent_Declare(libft4222
        URL "${FT4222_SDK_URL}"
        URL_HASH SHA256=${FT4222_SDK_SHA256})
    FetchContent_MakeAvailable(libft4222)

    # The zip holds a flat .tgz, so a second extraction is needed
    set(tarball "${libft4222_SOURCE_DIR}/libft4222-linux-${FT4222_SDK_VERSION}.tgz")
    set(sdk_directory "${libft4222_BINARY_DIR}/sdk-${FT4222_SDK_VERSION}")
    if(NOT EXISTS "${sdk_directory}/libft4222.h")
        if(NOT EXISTS "${tarball}")
            message(FATAL_ERROR "Expected ${tarball} inside ${FT4222_SDK_URL}")
        endif()
        file(ARCHIVE_EXTRACT INPUT "${tarball}" DESTINATION "${sdk_directory}")
    endif()

    # The library's soname is libft4222.so, so it must sit under that name in the RPATH directory
    set(library "${libft4222_BINARY_DIR}/lib/libft4222.so")
    file(MAKE_DIRECTORY "${libft4222_BINARY_DIR}/lib")
    file(COPY_FILE "${sdk_directory}/${build_directory}/libft4222.so.${FT4222_SDK_VERSION}" "${library}"
        ONLY_IF_DIFFERENT)

    message(STATUS "Using pinned LibFT4222 ${FT4222_SDK_VERSION} from ${FT4222_SDK_URL}")
    set(${include_var} "${sdk_directory}" PARENT_SCOPE)
    set(${library_var} "${library}" PARENT_SCOPE)
endfunction()
