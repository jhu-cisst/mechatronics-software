#
# $Id$
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.
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
     FirewirePort.h)

set (SOURCE_FILES
     code/AmpIO.cpp
     code/FirewirePort.cpp)
              
# Create Amp1394 library
add_library(Amp1394 STATIC
            ${HEADERS} ${SOURCE_FILES})

target_link_libraries(Amp1394 raw1394)

# Install files
install (TARGETS Amp1394 COMPONENT Amp1394
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

install (FILES ${HEADERS}
         DESTINATION include/Amp1394
         COMPONENT Amp1394-dev)
