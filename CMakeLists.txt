#
# (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.
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
set (Amp1394_VERSION_MINOR "7")
set (Amp1394_VERSION_PATCH "0")
set (Amp1394_VERSION "${Amp1394_VERSION_MAJOR}.${Amp1394_VERSION_MINOR}.${Amp1394_VERSION_PATCH}")

# If LIBRARY_OUTPUT_PATH is not defined, define it here (note that if this
# is built within cisst, then LIBRARY_OUTPUT_PATH is defined by cisst).
if (NOT LIBRARY_OUTPUT_PATH)
  set (LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
endif (NOT LIBRARY_OUTPUT_PATH)

# If EXECUTABLE_OUTPUT_PATH is not defined, define it here (note that if this
# is built within cisst, then EXECUTABLE_OUTPUT_PATH is defined by cisst).
if (NOT EXECUTABLE_OUTPUT_PATH)
  set (EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/bin")
endif (NOT EXECUTABLE_OUTPUT_PATH)

# TODO: Move include files to Amp1394 subdirectory, i.e., lib/Amp1394 or lib/include/Amp1394.
set (Amp1394_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib;${Amp1394_BINARY_DIR}")
set (Amp1394_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH})
set (Amp1394_LIBRARIES   Amp1394)
set (Amp1394Console_LIBRARIES Amp1394Console)
set (Amp1394Console_EXTRA_LIBRARIES "")

if (WIN32)

  # On Windows, can build with WinPcap or Npcap (Ethernet support)
  #    Use WinPcap for Windows XP and earlier (no longer maintained)
  #    Use Npcap for Windows 7 and higher (use Npcap SDK)
  # OFF by default because UDP interface is preferred
  option (Amp1394_HAS_PCAP "Build Amp1394 with Ethernet support (winpcap or npcap)" OFF)

  if (Amp1394_HAS_PCAP)
    # Need to specify where WinPcap Developer's Pack or Npcap SDK is installed
    find_path (PCAP_INCLUDE_DIR pcap.h)
    # In addition to wpcap, need Packet library for PacketOpenAdapter and PacketReqest
    # Make sure to use correct version (32 or 64 bit)
    find_path (PCAP_LIBRARY_DIR "wpcap.lib" DOC "Path to WinPcap/Npcap wpcap and Packet libraries")
    set (PCAP_LIBRARIES "wpcap" "Packet")
  endif (Amp1394_HAS_PCAP)

else (WIN32)

  # On other platforms (mostly Linux), can build with pcap and/or libraw1394
  option (Amp1394_HAS_PCAP   "Build Amp1394 with Ethernet support (pcap)"       OFF)
  option (Amp1394_HAS_RAW1394 "Build Amp1394 with FireWire support (libraw1394)" ON)

  if (Amp1394_HAS_PCAP)
    # For now, assume pcap is installed somewhere standard
    # To install:  sudo apt-get install libpcap-dev
    set (PCAP_INCLUDE_DIR "")
    set (PCAP_LIBRARIES "pcap")
  endif (Amp1394_HAS_PCAP)

  option (Amp1394Console_HAS_CURSES "Build Amp1394 console library with curses (OFF --> VT100 mode)" ON)

  if (Amp1394Console_HAS_CURSES)
    find_package (Curses)
    if (CURSES_FOUND)
      set (Amp1394Console_EXTRA_LIBRARIES ${Amp1394Console_EXTRA_LIBRARIES} ${CURSES_LIBRARIES})
    else (CURSES_FOUND)
      message ("Cannot find Curses library, changing to VT100 mode")
    endif (CURSES_FOUND)
  endif (Amp1394Console_HAS_CURSES)

endif (WIN32)

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
if (WIN32)
  # for Windows, need WinSock, Iphlpapi (for getting interface info) and Ws2_32 (for WSAIoctl)
  set (Amp1394_EXTRA_LIBRARIES ${Amp1394_EXTRA_LIBRARIES} WSOCK32 Iphlpapi Ws2_32)
endif (WIN32)

# Generate Amp1394Config.cmake
set (CONF_INCLUDE_DIR ${Amp1394_INCLUDE_DIR})
set (CONF_LIBRARY_DIR ${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})
set (CONF_LIBRARIES   ${Amp1394_LIBRARIES}   ${Amp1394_EXTRA_LIBRARIES})
set (CONF_CONSOLE_LIBRARIES ${Amp1394Console_LIBRARIES} ${Amp1394Console_EXTRA_LIBRARIES})
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

# Test programs
add_subdirectory (tests)

# Utility to flash/test board
add_subdirectory (programmer)
