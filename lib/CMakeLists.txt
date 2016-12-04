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

project(Amp1394)

set (HEADERS
     BoardIO.h
     AmpIO.h
     BasePort.h
     Amp1394Time.h)

set (SOURCE_FILES
     code/AmpIO.cpp
     code/Amp1394Time.cpp)

if (Amp1394_HAS_RAW1394)
  set (HEADERS ${HEADERS} FirewirePort.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/FirewirePort.cpp)
endif (Amp1394_HAS_RAW1394)

if (Amp1394_HAS_PCAP)
  set (HEADERS ${HEADERS} Eth1394Port.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/Eth1394Port.cpp)
endif (Amp1394_HAS_PCAP)

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

  set_source_files_properties(amp1394.i PROPERTIES CPLUSPLUS ON)
  swig_add_module(amp1394 python amp1394.i ${HEADERS} ${SOURCE_FILES})
  swig_link_libraries(amp1394 ${PYTHON_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

  # post build command
  add_custom_command(TARGET _amp1394 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/_amp1394.so .
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/amp1394.py .
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/amp1394PYTHON_wrap.cxx .
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/python/
    COMMENT "Copy swig amp1394 module to python folder"
    )
endif (Amp1394_BUILD_SWIG)


# revision file
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/AmpIORevision.h.in"
                "${Mechatronics_BINARY_DIR}/Amp1394/AmpIORevision.h")

# Install files
install (TARGETS Amp1394 COMPONENT Amp1394
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

install (FILES ${HEADERS}
         DESTINATION include/Amp1394
         COMPONENT Amp1394-dev)
