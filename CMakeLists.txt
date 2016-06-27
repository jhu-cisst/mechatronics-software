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
set (Amp1394_VERSION_PATCH "0")
set (Amp1394_VERSION "${Amp1394_VERSION_MAJOR}.${Amp1394_VERSION_MINOR}.${Amp1394_VERSION_PATCH}")

if (WIN32)

  # On Windows, can build with winpcap (Ethernet support)
  option (Amp1394_HAS_PCAP      "Build Amp1394 with Ethernet support (winpcap)" ON)

  # Need to specify where WinPcap Developer's Pack is installed
  find_path(PCAP_INCLUDE_DIR pcap.h)
  # In addition to wpcap, need Packet library for PacketOpenAdapter and PacketReqest
  find_path(PCAP_LIBRARY_DIR "wpcap.lib" DOC "Path to WinPcap wpcap and Packet libraries")
  set(PCAP_LIBRARIES "wpcap" "Packet")
else (WIN32)

  # On other platforms (mostly Linux), can build with pcap and/or libraw1394
  option (Amp1394_HAS_PCAP      "Build Amp1394 with Ethernet support (pcap)" OFF)
  option (Amp1394_HAS_RAW1394   "Build Amp1394 with FireWire support (libraw1394)"   ON)

  # For now, assume pcap is installed somewhere standard
  set(PCAP_INCLUDE_DIR "")
  set(PCAP_LIBRARIES "pcap")

endif (WIN32)

if (NOT Amp1394_HAS_RAW1394 AND NOT Amp1394_HAS_PCAP)
  message (SEND_ERROR "Amp1394 requires at least one of Amp1394_HAS_RAW1394 or Amp1394_HAS_PCAP")
endif (NOT Amp1394_HAS_RAW1394 AND NOT Amp1394_HAS_PCAP)

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

if (UNIX)
  # Add some warnings
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag ("-Wextra" CXX_SUPPORTS_WEXTRA)
  if (CXX_SUPPORTS_WEXTRA)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
  endif (CXX_SUPPORTS_WEXTRA)

  check_cxx_compiler_flag ("-Wall" CXX_SUPPORTS_WALL)
  if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif (CXX_SUPPORTS_WALL)
endif (UNIX)

# To find Amp1394 library headers
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/lib
                     ${CMAKE_CURRENT_BINARY_DIR})

# To link with build libraries
link_directories (${CMAKE_CURRENT_BINARY_DIR})

# Utilities to test libraw1394
if (Amp1394_HAS_RAW1394)
  add_subdirectory (util)
endif (Amp1394_HAS_RAW1394)

# Code for library Amp1394
add_subdirectory (lib)

# Test programs (some use curses for text based display)
add_subdirectory(tests)

# Utility to flash/test board
add_subdirectory (programmer)
