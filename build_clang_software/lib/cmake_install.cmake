# Install script for directory: /mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/mnt/c/Users/kiggz/research/clang+llvm-18.1.8-x86_64-linux-gnu-ubuntu-18.04/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Amp1394" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/mnt/c/Users/kiggz/research/mechatronics-software/build_clang_software/lib/libAmp1394.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Amp1394" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/mnt/c/Users/kiggz/research/mechatronics-software/build_clang_software/lib/libAmp1394Console.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Amp1394-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Amp1394" TYPE FILE FILES
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/BoardIO.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/FpgaIO.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/AmpIO.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/Amp1394Types.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/Amp1394Time.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/Amp1394BSwap.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/EncoderVelocity.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/BasePort.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/EthBasePort.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/EthUdpPort.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/PortFactory.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/FirewirePort.h"
    "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib/Amp1394Console.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/mnt/c/Users/kiggz/research/mechatronics-software/build_clang_software/lib/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
