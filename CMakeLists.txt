cmake_minimum_required(VERSION 3.10)
project(lmts VERSION 0.1.0 LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build options
option(BUILD_STATIC_LIBS "Build static libraries" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_APPS "Build TS applications (muxer/demuxer)" ON)
option(INSTALL_TO_USER_LOCAL "Install to ~/.local instead of system-wide" OFF)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Display configuration options for developers
message(STATUS "")
message(STATUS "========== lmts Library Configuration Options ==========")
message(STATUS "")
message(STATUS "Build Type Configuration:")
message(STATUS "  Current build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Available types: Debug, Release, RelWithDebInfo, MinSizeRel")
message(STATUS "  Change with: cmake -DCMAKE_BUILD_TYPE=Debug ..")
message(STATUS "")
message(STATUS "Library Build Options:")
message(STATUS "  BUILD_STATIC_LIBS: Build static libraries (current: ${BUILD_STATIC_LIBS})")
message(STATUS "  BUILD_SHARED_LIBS: Build shared libraries (current: ${BUILD_SHARED_LIBS})")
message(STATUS "  BUILD_EXAMPLES: Build example programs (current: ${BUILD_EXAMPLES})")
message(STATUS "  BUILD_APPS: Build lmts applications (current: ${BUILD_APPS})")
message(STATUS "")
message(STATUS "Installation Options:")
message(STATUS "  CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  INSTALL_TO_USER_LOCAL: Install to ~/.local (current: ${INSTALL_TO_USER_LOCAL})")
message(STATUS "")
message(STATUS "Usage Examples:")
message(STATUS "  Debug build:     cmake -DCMAKE_BUILD_TYPE=Debug ..")
message(STATUS "  User install:    cmake -DINSTALL_TO_USER_LOCAL=ON ..")
message(STATUS "  Custom prefix:   cmake -DCMAKE_INSTALL_PREFIX=/opt/lmts ..")
message(STATUS "  Minimal build:   cmake -DBUILD_EXAMPLES=OFF ..")
message(STATUS "")
message(STATUS "============================================================")

# Handle user-local installation
if(INSTALL_TO_USER_LOCAL AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "User local install path" FORCE)
    message(STATUS "Installing to user directory: ${CMAKE_INSTALL_PREFIX}")
endif()

# Provide install path information to users
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    message(STATUS "Using default install prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "You can customize it with: cmake -DCMAKE_INSTALL_PREFIX=/your/path ..")
endif()

# Enable position independent code for shared library compatibility
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Try to find installed lmcore first, fallback to local source if not found
find_package(lmcore QUIET)
if(lmcore_FOUND)
    message(STATUS "Using system-installed lmcore library")
    set(LMCORE_SOURCE "system")
else()
    # Check if local lmcore directory exists
    set(LMCORE_LOCAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lmcore")
    if(EXISTS "${LMCORE_LOCAL_DIR}/CMakeLists.txt")
        if(NOT TARGET lmcore)
            add_subdirectory(../lmcore lmcore_build)
        endif()
        message(STATUS "Using local lmcore from sibling directory")
        set(LMCORE_SOURCE "local")
    else()
        # Neither system nor local lmcore found - show helpful error
        message(STATUS "")
        message(STATUS "========== DEPENDENCY ERROR ==========")
        message(FATAL_ERROR
            "Cannot find lmcore dependency!\n"
            "\n"
            "Searched locations:\n"
            "  1. System installation (using find_package lmcore)\n"
            "  2. Local directory: ${LMCORE_LOCAL_DIR}\n"
            "\n"
            "Solution - Download lmcore to sibling directory:\n"
            "\n"
            "    cd ${CMAKE_CURRENT_SOURCE_DIR}/..\n"
            "    git clone ssh://git@github.com/lmshao/lmcore.git\n"
            "\n"
            "    # OR if SSH is not available:\n"
            "    git clone https://github.com/lmshao/lmcore.git\n"
            "\n"
            "Repository Information:\n"
            "  📍 GitHub: https://github.com/lmshao/lmcore\n"
            "  🔑 SSH:    ssh://git@github.com/lmshao/lmcore.git\n"
            "\n"
            "After downloading, re-run: cmake .."
        )
    endif()
endif()

# Source files and include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

# Add lmcore include directory
if(LMCORE_SOURCE STREQUAL "local")
    include_directories(${LMCORE_LOCAL_DIR}/include)
endif()

# Collect TS source files
file(GLOB_RECURSE TS_SOURCES "src/*.cpp")
set(SOURCES ${TS_SOURCES})

# Validate build options
if(NOT BUILD_STATIC_LIBS AND NOT BUILD_SHARED_LIBS)
    message(FATAL_ERROR "At least one of BUILD_STATIC_LIBS or BUILD_SHARED_LIBS must be ON")
endif()

# Compiler flags
if(MSVC)
    # MSVC specific flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    set(CMAKE_CXX_FLAGS_DEBUG "/MDd /Zi /Od /DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /DNDEBUG /DRELEASE")
else()
    # GCC/Clang flags for Linux/Unix
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -Wno-format-truncation -Wno-unused-result")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -DRELEASE")
endif()

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "C++ flags for ${CMAKE_BUILD_TYPE}: ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")

# Find threads (required for lmcore operations)
find_package(Threads REQUIRED)

# Print source file summary
list(LENGTH TS_SOURCES TS_COUNT)
message(STATUS "Found ${TS_COUNT} TS source files")

# Static library
if(BUILD_STATIC_LIBS)
    add_library(lmts_static STATIC ${SOURCES})

    target_include_directories(lmts_static PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    # Link dependencies - prefer static to static linking
    if(lmcore_FOUND)
        target_link_libraries(lmts_static PUBLIC lmcore::lmcore)
    else()
        target_link_libraries(lmts_static PUBLIC lmcore)
    endif()

    target_link_libraries(lmts_static PUBLIC Threads::Threads)

    set_target_properties(lmts_static PROPERTIES
        OUTPUT_NAME lmts
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )

    message(STATUS "Static library target: lmts_static")
endif()

# Shared library
if(BUILD_SHARED_LIBS)
    add_library(lmts_shared SHARED ${SOURCES})

    target_include_directories(lmts_shared PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    # Link dependencies
    if(lmcore_FOUND)
        target_link_libraries(lmts_shared PUBLIC lmcore::lmcore)
    else()
        target_link_libraries(lmts_shared PUBLIC lmcore)
    endif()

    target_link_libraries(lmts_shared PUBLIC Threads::Threads)

    set_target_properties(lmts_shared PROPERTIES
        OUTPUT_NAME lmts
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )

    message(STATUS "Shared library target: lmts_shared")
endif()

# Create alias targets for easier usage
if(BUILD_STATIC_LIBS)
    add_library(lmts::lmts_static ALIAS lmts_static)
endif()

if(BUILD_SHARED_LIBS)
    add_library(lmts::lmts_shared ALIAS lmts_shared)
    add_library(lmts::lmts ALIAS lmts_shared)  # Default to shared
elseif(BUILD_STATIC_LIBS)
    add_library(lmts::lmts ALIAS lmts_static)  # Fallback to static
endif()

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Applications
if(BUILD_APPS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps")
    add_subdirectory(apps)
endif()

# Installation
if(NOT WIN32)
    include(GNUInstallDirs)

    # Install headers
    install(DIRECTORY include/lmts
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h"
    )

    # Install libraries
    if(BUILD_STATIC_LIBS)
        install(TARGETS lmts_static
            EXPORT lmtsTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    endif()

    if(BUILD_SHARED_LIBS)
        install(TARGETS lmts_shared
            EXPORT lmtsTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    endif()

    # Install CMake config files
    install(EXPORT lmtsTargets
        FILE lmtsTargets.cmake
        NAMESPACE lmts::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lmts
    )

    # Generate and install config file
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lmtsConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/lmtsConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lmts
    )

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/lmtsConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/lmtsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/lmtsConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lmts
    )

    # Print installation summary
    message(STATUS "")
    message(STATUS "========== Installation Summary ==========")
    message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "Headers will be installed to: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")
    message(STATUS "Libraries will be installed to: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
    message(STATUS "CMake config will be installed to: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/lmts")
    message(STATUS "==========================================")
else()
    message(STATUS "Install/uninstall targets are disabled on Windows platform")
    message(STATUS "Use the library directly from the build directory")
endif()