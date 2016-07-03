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
