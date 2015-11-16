#
# (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.
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
     BasePort.h)

set (SOURCE_FILES
     code/AmpIO.cpp)

if (Amp1394_HAS_RAW1394)
  set (HEADERS ${HEADERS} FirewirePort.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/FirewirePort.cpp)
endif (Amp1394_HAS_RAW1394)

if (Amp1394_HAS_PCAP)
  set (HEADERS ${HEADERS} Eth1394Port.h)
  set (SOURCE_FILES ${SOURCE_FILES} code/Eth1394Port.cpp)
  if (WIN32)
    include_directories(${PCAP_INCLUDE_DIR})
  endif (WIN32)
endif (Amp1394_HAS_PCAP)

# Create Amp1394 library
add_library(Amp1394 STATIC
            ${HEADERS} ${SOURCE_FILES})

if (Amp1394_HAS_RAW1394)
  target_link_libraries(Amp1394 raw1394)
endif (Amp1394_HAS_RAW1394)

if (Amp1394_HAS_PCAP)
  target_link_libraries(Amp1394 ${PCAP_LIBRARIES})
endif (Amp1394_HAS_PCAP)

# Install files
install (TARGETS Amp1394 COMPONENT Amp1394
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

install (FILES ${HEADERS}
         DESTINATION include/Amp1394
         COMPONENT Amp1394-dev)
