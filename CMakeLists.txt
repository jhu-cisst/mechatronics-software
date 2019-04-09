#
# (C) Copyright 2011-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (Amp1394)

cmake_minimum_required (VERSION 2.6)

# Set the version number
set (Amp1394_VERSION_MAJOR "1")
set (Amp1394_VERSION_MINOR "5")
set (Amp1394_VERSION_PATCH "0")
set (Amp1394_VERSION "${Amp1394_VERSION_MAJOR}.${Amp1394_VERSION_MINOR}.${Amp1394_VERSION_PATCH}")

# If LIBRARY_OUTPUT_PATH is not defined, define it here (note that if this
# is built within cisst, then LIBRARY_OUTPUT_PATH is defined by cisst).
if (NOT LIBRARY_OUTPUT_PATH)
  set (LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
endif (NOT LIBRARY_OUTPUT_PATH)

# TODO: Move include files to Amp1394 subdirectory, i.e., lib/Amp1394 or lib/include/Amp1394.
set (Amp1394_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib;${Amp1394_BINARY_DIR}")
set (Amp1394_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH})
set (Amp1394_LIBRARIES   Amp1394)

if (WIN32)

  # On Windows, can build with winpcap (Ethernet support)
  option (Amp1394_HAS_PCAP "Build Amp1394 with Ethernet support (winpcap)" ON)

  # Need to specify where WinPcap Developer's Pack is installed
  find_path (PCAP_INCLUDE_DIR pcap.h)
  # In addition to wpcap, need Packet library for PacketOpenAdapter and PacketReqest
  find_path (PCAP_LIBRARY_DIR "wpcap.lib" DOC "Path to WinPcap wpcap and Packet libraries")
  set (PCAP_LIBRARIES "wpcap" "Packet")

else (WIN32)

  # On other platforms (mostly Linux), can build with pcap and/or libraw1394
  option (Amp1394_HAS_PCAP   "Build Amp1394 with Ethernet support (pcap)"       OFF)
  option (Amp1394_HAS_RAW1394 "Build Amp1394 with FireWire support (libraw1394)" ON)

  # For now, assume pcap is installed somewhere standard
  set (PCAP_INCLUDE_DIR "")
  set (PCAP_LIBRARIES "pcap")

endif (WIN32)

if (NOT Amp1394_HAS_RAW1394 AND NOT Amp1394_HAS_PCAP)
  message (SEND_ERROR "Amp1394 requires at least one of Amp1394_HAS_RAW1394 or Amp1394_HAS_PCAP")
endif (NOT Amp1394_HAS_RAW1394 AND NOT Amp1394_HAS_PCAP)

# TODO: Determine whether it is necessary to have separate EXTRA variables for LIBRARY_DIR
#       and LIBRARIES. Currently, it seems that both are always used together.
#       The Amp1394_EXTRA_INCLUDE_DIR should be separate since it is only needed when
#       building Amp1394.
set (Amp1394_EXTRA_INCLUDE_DIR "")
set (Amp1394_EXTRA_LIBRARY_DIR "")
set (Amp1394_EXTRA_LIBRARIES "")
if (Amp1394_HAS_RAW1394)
  # Assume libraw1394 is installed in standard include/lib directories
  set (Amp1394_EXTRA_LIBRARIES ${Amp1394_EXTRA_LIBRARIES} raw1394)
endif (Amp1394_HAS_RAW1394)
if (Amp1394_HAS_PCAP)
  set (Amp1394_EXTRA_INCLUDE_DIR ${PCAP_INCLUDE_DIR})
  set (Amp1394_EXTRA_LIBRARY_DIR ${PCAP_LIBRARY_DIR})
  set (Amp1394_EXTRA_LIBRARIES ${Amp1394_EXTRA_LIBRARIES} ${PCAP_LIBRARIES})
endif (Amp1394_HAS_PCAP)

# Generate Amp1394Config.cmake
set (CONF_INCLUDE_DIR ${Amp1394_INCLUDE_DIR})
set (CONF_LIBRARY_DIR ${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})
set (CONF_LIBRARIES   ${Amp1394_LIBRARIES}   ${Amp1394_EXTRA_LIBRARIES})
configure_file (Amp1394Config.cmake.in
                "${Amp1394_BINARY_DIR}/Amp1394Config.cmake" @ONLY)

if (UNIX)
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
endif (UNIX)

# Utilities to test libraw1394
if (Amp1394_HAS_RAW1394)
  add_subdirectory (util)
endif (Amp1394_HAS_RAW1394)

# Code for library Amp1394
add_subdirectory (lib)

# Test programs (some use curses for text based display)
add_subdirectory (tests)

# Utility to flash/test board
add_subdirectory (programmer)
