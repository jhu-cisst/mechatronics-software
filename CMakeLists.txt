#
# (C) Copyright 2011-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (Mechatronics)

cmake_minimum_required (VERSION 2.6)

# Set the version number
set (Amp1394_VERSION_MAJOR "1")
set (Amp1394_VERSION_MINOR "2")
set (Amp1394_VERSION_PATCH "1")
set (Amp1394_VERSION "${Amp1394_VERSION_MAJOR}.${Amp1394_VERSION_MINOR}.${Amp1394_VERSION_PATCH}")

# Add some warnings
include (CheckCXXCompilerFlag)
check_cxx_compiler_flag ("-Wextra" CXX_SUPPORTS_WEXTRA)
if (CXX_SUPPORTS_WEXTRA)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif ()

check_cxx_compiler_flag ("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif ()

check_cxx_compiler_flag ("-fPIC" CXX_SUPPORTS_FPIC)
if (CXX_SUPPORTS_FPIC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif ()

# ... for the build tree
set (CONF_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib" "${CMAKE_CURRENT_BINARY_DIR}")
set (CONF_LIBRARY_DIR "${PROJECT_BINARY_DIR}/lib")
configure_file (Amp1394Config.cmake.in
                "${PROJECT_BINARY_DIR}/Amp1394Config.cmake" @ONLY)

# revision file
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/lib/AmpIORevision.h.in"
                "${CMAKE_CURRENT_BINARY_DIR}/Amp1394/AmpIORevision.h")

# To find Amp1394 library headers
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/lib
                     ${CMAKE_CURRENT_BINARY_DIR})

# To link with build libraries
link_directories (${CMAKE_CURRENT_BINARY_DIR})

# Utilities to test libraw1394
add_subdirectory (util)

# Code for library Amp1394
add_subdirectory (lib)

# Test program using curse for text based display
find_package (Curses)
if (CURSES_FOUND)
  add_subdirectory(tests)
else ()
  message ("Can't compile curses based test programs for Amp1394, make sure you have libcurses dev installed")
endif ()

# Utility to flash/test board
add_subdirectory (programmer)
