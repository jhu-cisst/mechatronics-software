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

project (Amp1394Lib)

set (HEADERS
     BoardIO.h
     FpgaIO.h
     AmpIO.h
     Amp1394Types.h
     Amp1394Time.h
     Amp1394BSwap.h
     EncoderVelocity.h
     BasePort.h
     EthBasePort.h
     EthUdpPort.h
     PortFactory.h)

set (SOURCE_FILES
     code/BoardIO.cpp
     code/FpgaIO.cpp
     code/AmpIO.cpp
     code/Amp1394Time.cpp
     code/EncoderVelocity.cpp
     code/BasePort.cpp
     code/EthBasePort.cpp
     code/EthUdpPort.cpp
     code/PortFactory.cpp)


if (Amp1394_HAS_RAW1394)
  set (HEADERS ${HEADERS} FirewirePort.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/FirewirePort.cpp)
endif (Amp1394_HAS_RAW1394)

if (Amp1394_HAS_PCAP)
  set (HEADERS ${HEADERS} EthRawPort.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/EthRawPort.cpp)
endif (Amp1394_HAS_PCAP)

if (Amp1394_HAS_EMIO)
  set (HEADERS ${HEADERS} ZynqEmioPort.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/ZynqEmioPort.cpp)
endif (Amp1394_HAS_EMIO)

include_directories(${Amp1394_INCLUDE_DIR} ${Amp1394_EXTRA_INCLUDE_DIR})
link_directories(${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})

# Create Amp1394 library
add_library(Amp1394 STATIC
            ${HEADERS} ${SOURCE_FILES})

target_link_libraries(Amp1394 ${Amp1394_EXTRA_LIBRARIES})

option (Amp1394_BUILD_SWIG  "Build Amp1394 with Python wrapper" OFF)
if (Amp1394_BUILD_SWIG)
  find_package(SWIG REQUIRED)
  include(${SWIG_USE_FILE})

  find_package(PythonLibs)
  include_directories(${PYTHON_INCLUDE_PATH})

  find_package(PythonInterp)
  if (PYTHON_EXECUTABLE)
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
                    ERROR_QUIET
                    OUTPUT_VARIABLE Amp1394_NUMPY_INCLUDE_DIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    include_directories(${Amp1394_NUMPY_INCLUDE_DIR} "${Amp1394_BINARY_DIR}/Amp1394")
  endif()

  set_source_files_properties(Amp1394.i PROPERTIES CPLUSPLUS ON)
  swig_add_module(Amp1394Python python Amp1394.i ${HEADERS} ${SOURCE_FILES})
  swig_link_libraries(Amp1394Python ${PYTHON_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

  if (WIN32)
    set_target_properties (_Amp1394Python PROPERTIES SUFFIX .pyd)
    set_target_properties (_Amp1394Python PROPERTIES DEBUG_POSTFIX "_d")
  endif (WIN32)

  # post build command
  add_custom_command(TARGET _Amp1394Python POST_BUILD
                     COMMAND ${CMAKE_COMMAND}
                     ARGS -E copy_if_different
                             ${CMAKE_CURRENT_BINARY_DIR}/Amp1394Python.py
                             ${LIBRARY_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/
                     COMMAND ${CMAKE_COMMAND}
                     ARGS -E copy_if_different
                             ${Amp1394_SOURCE_DIR}/python/test_eth1394.py
                             ${LIBRARY_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/
                     COMMAND ${CMAKE_COMMAND}
                     ARGS -E copy_if_different
                             ${Amp1394_SOURCE_DIR}/python/test_amp1394.py
                             ${LIBRARY_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/
                     COMMENT "Copying Python files to ${LIBRARY_OUTPUT_PATH}" )

  # install library and python file
  install (
    TARGETS _Amp1394Python
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    COMPONENT Amp1394)
  install (
    FILES ${CMAKE_CURRENT_BINARY_DIR}/Amp1394Python.py
    DESTINATION lib
    COMPONENT Amp1394)

endif (Amp1394_BUILD_SWIG)

# Create Amp1394 console library

if (NOT WIN32)
  if (CURSES_FOUND)
    add_definitions(-DAmp1394Console_HAS_CURSES)
    include_directories (${CURSES_INCLUDE_DIR})
  endif (CURSES_FOUND)
endif (NOT WIN32)

add_library(Amp1394Console STATIC Amp1394Console.h code/Amp1394Console.cpp)

target_link_libraries(Amp1394Console ${Amp1394Console_EXTRA_LIBRARIES})

# revision file
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/AmpIORevision.h.in"
                "${Amp1394_BINARY_DIR}/Amp1394/AmpIORevision.h")

# Install files
install (TARGETS Amp1394 Amp1394Console COMPONENT Amp1394
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

install (FILES ${HEADERS} Amp1394Console.h
         DESTINATION include/Amp1394
         COMPONENT Amp1394-dev)
